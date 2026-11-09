#include "julia/embodiment/BodyCollision.hpp"
#include "julia/animation/Pose.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace julia::embodiment
{
    namespace
    {
        const std::array<ColliderCalibration,9> calibration{{
            {1,SegmentId::LeftUpperArm,JointId::LeftShoulder,JointId::LeftElbow,.16f,.90f,.032f},
            {2,SegmentId::LeftForearm,JointId::LeftElbow,JointId::LeftWrist,.10f,.92f,.027f},
            {3,SegmentId::LeftHand,JointId::LeftWrist,JointId::LeftWrist,0,0,.032f,{0,.025f,0},{0,.105f,0}},
            {4,SegmentId::RightUpperArm,JointId::RightShoulder,JointId::RightElbow,.16f,.90f,.032f},
            {5,SegmentId::RightForearm,JointId::RightElbow,JointId::RightWrist,.10f,.92f,.027f},
            {6,SegmentId::RightHand,JointId::RightWrist,JointId::RightWrist,0,0,.032f,{0,.025f,0},{0,.105f,0}},
            {7,SegmentId::Chest,JointId::Pelvis,JointId::Chest,0,1,.070f},
            {8,SegmentId::Neck,JointId::Neck,JointId::Head,0,1,.035f},
            {9,SegmentId::Head,JointId::Head,JointId::Head,0,0,.075f,{0,.035f,0},{0,.120f,0}}
        }};
    }
    const char* pathStatusName(PathStatus s)
    {
        switch(s) {
        case PathStatus::Disabled:return "disabled"; case PathStatus::Clear:return "clear";
        case PathStatus::Blocked:return "blocked path/clearance"; case PathStatus::InitialIntersection:return "initial intersection";
        case PathStatus::Incomplete:return "verification budget/depth exhausted"; case PathStatus::Unreachable:return "IK target unreachable (local result)";
        case PathStatus::Rejected:return "command rejected";
        case PathStatus::NotChecked:return "no proposed path checked"; case PathStatus::Invalidated:return "certificate invalidated by geometry discontinuity";
        } return "unknown";
    }
    std::span<const ColliderCalibration> bodyColliderCalibration() { return calibration; }
    std::vector<BodyCollider> bodyColliders(const BodyRigBinding& b,std::span<const glm::mat4> globals,const glm::mat4& world)
    {
        std::vector<BodyCollider> result; result.reserve(calibration.size());
        for (const auto& c:calibration)
        {
            const auto f=b.jointFrame(c.first,globals),g=b.jointFrame(c.second,globals);
            glm::vec3 a,bp;
            if (c.first==c.second) { a=glm::vec3{f*glm::vec4{c.localA,1}}; bp=glm::vec3{f*glm::vec4{c.localB,1}}; }
            else { a=glm::mix(glm::vec3{f[3]},glm::vec3{g[3]},c.from); bp=glm::mix(glm::vec3{f[3]},glm::vec3{g[3]},c.to); }
            result.push_back({c.id,c.segment,{glm::dvec3{world*glm::vec4{a,1}},glm::dvec3{world*glm::vec4{bp,1}},c.radius}});
        }
        return result;
    }
    bool checksSelfPair(std::uint32_t a,std::uint32_t b)
    {
        if (a>b) std::swap(a,b);
        if (a==b || a>=7) return false; // Torso/neck/head form a joined fixed assembly.
        if ((a==1 && b==2) || (a==2 && b==3) || (a==4 && b==5) || (a==5 && b==6)) return false;
        return true; // Upper/hand, opposite arms, and every arm vs torso/neck/head.
    }
    void validateCollisionSettings(const CollisionSettings& s)
    {
        if (!std::isfinite(s.margin) || s.margin<0 || s.margin>.05 || !std::isfinite(s.numericSlack) || s.numericSlack<.00003 || s.numericSlack>.001 ||
            !s.maxPoses || s.maxPoses>100000 || !s.maxQueries || s.maxQueries>2000000 || s.maxDepth>24)
            throw std::invalid_argument("Invalid collision tolerance/budget");
    }
    struct ArmPathCheckJob::Impl
    {
        struct Interval
        {
            ArmCoordinates a{}, b{};
            double lo = 0, hi = 0;
            unsigned depth = 0;
        };
        struct Pair { std::size_t first, second; bool sceneObject; };
        enum class Phase { Initial, Intervals, Final, Done };
        const BodyRigBinding& binding;
        DirectBodyControl scratch;
        animation::Pose pose;
        world::CollisionScene scene;
        CollisionSettings settings;
        CollisionContext context;
        glm::mat4 world;
        std::vector<ArmCoordinates> path;
        std::array<JointId,3> ids;
        std::vector<Pair> pairs;
        std::vector<BodyCollider> forms;
        std::vector<Interval> stack;
        PathCheck out;
        Phase phase = Phase::Initial;
        std::size_t side, nextTick = 1, pair = 0;
        bool evaluating = false;
        Interval current;
        ArmCoordinates midpoint{};
        double radius = 0, gap = 0, displacement = 0, traceLastTick = -1e6;

        Impl(const DirectBodyControl& control,std::size_t sideIndex,std::span<const ArmCoordinates> coordinates,
            const glm::mat4& root,const world::CollisionScene& collisionScene,const CollisionSettings& collisionSettings,CollisionContext collisionContext)
            :binding(control.binding()),scratch(binding),pose(binding.asset().skeleton()),
             scene(collisionScene.version(),std::vector<world::SceneObject>(collisionScene.objects().begin(),collisionScene.objects().end())),
             settings(collisionSettings),context(std::move(collisionContext)),world(root),path(coordinates.begin(),coordinates.end()),
             ids(sideIndex==0 ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
                std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist}),side(sideIndex)
        {
            validateCollisionSettings(settings);
            if (side>1 || path.empty() || !control.enabled()) throw std::invalid_argument("Invalid direct arm path");
            // Same proper rigid metric root and numerical model as synchronous 6A.
            const auto rootDistance=glm::length(glm::vec3{world[3]});
            if (!std::isfinite(rootDistance) || rootDistance>100)
                throw std::invalid_argument("Collision root exceeds supported metric range");
            for (int a=0;a<4;++a) for (int b=0;b<4;++b)
                if (!std::isfinite(world[a][b])) throw std::invalid_argument("Nonfinite collision root");
            const glm::mat3 rotation{world}; const auto gram=glm::transpose(rotation)*rotation;
            for (int a=0;a<3;++a) for (int b=0;b<3;++b)
                if (std::abs(gram[a][b]-(a==b ? 1.f:0.f))>1e-5f) throw std::invalid_argument("Collision root must be rigid");
            if (glm::determinant(rotation)<0 || std::abs(world[0][3])+std::abs(world[1][3])+std::abs(world[2][3])+std::abs(world[3][3]-1)>1e-6f)
                throw std::invalid_argument("Collision root must be proper affine rigid");
            scratch.setEnabled(true);
            const auto& profile=binding.profile();
            for (const auto& j:profile.joints) for (std::size_t k=0;k<j.coordinates.size();++k)
                scratch.setCoordinate(j.id,k,control.coordinate(j.id,k));
            const auto frame=[&](JointId id) { return profile.joints[binding.jointIndex(id)].frameInBodyBind; };
            const auto& handCalibration=calibration[side*3+2];
            radius=glm::length(glm::vec3{frame(ids[1])[3]-frame(ids[0])[3]})+
                glm::length(glm::vec3{frame(ids[2])[3]-frame(ids[1])[3]})+
                std::max(glm::length(handCalibration.localA),glm::length(handCalibration.localB))+.000001;
            if (context.contactHand && context.contactHand!=side*3+3)
                throw std::invalid_argument("Contact permission belongs to a different hand");
            if (context.contactObject && !context.contactHand) throw std::invalid_argument("Contact object requires named hand");
            if (context.handEnvelopes[side]) radius+=.10; // Bounds the enlarged proxy, not the full skinned mesh.
            if (context.carried)
            {
                const auto& c=*context.carried;
                const auto e=std::find_if(profile.effectors.begin(),profile.effectors.end(),[&](const auto& f) { return f.id==c.effector; });
                const auto effect=static_cast<unsigned>(c.effector);
                if (e==profile.effectors.end() || effect<side*4 || effect>side*4+2 || !c.object ||
                    context.contactHand!=side*3+3 || !std::isfinite(c.radius) || c.radius<=0 || c.radius>.05 ||
                    glm::length(glm::vec3{c.effectorFromObject[3]})>1)
                    throw std::invalid_argument("Invalid carried sphere");
                const auto gramAttachment=glm::transpose(glm::mat3{c.effectorFromObject})*glm::mat3{c.effectorFromObject};
                for (int a=0;a<4;++a) for (int b=0;b<4;++b)
                    if (!std::isfinite(c.effectorFromObject[a][b])) throw std::invalid_argument("Nonfinite attachment transform");
                for (int a=0;a<3;++a) for (int b=0;b<3;++b)
                    if (std::abs(gramAttachment[a][b]-(a==b ? 1.f:0.f))>1e-5f) throw std::invalid_argument("Attachment must be rigid");
                if (glm::determinant(glm::mat3{c.effectorFromObject})<0 || std::abs(c.effectorFromObject[0][3])+
                    std::abs(c.effectorFromObject[1][3])+std::abs(c.effectorFromObject[2][3])+std::abs(c.effectorFromObject[3][3]-1)>1e-6f)
                    throw std::invalid_argument("Attachment must be proper affine rigid");
                // Shoulder->elbow->wrist + wrist->effector + attachment offset + sphere support.
                // Triangle inequality bounds any payload point under EACH of the six rotations.
                radius=std::max(radius,static_cast<double>(glm::length(glm::vec3{frame(ids[1])[3]-frame(ids[0])[3]})+
                    glm::length(glm::vec3{frame(ids[2])[3]-frame(ids[1])[3]})+
                    glm::length(glm::vec3{e->frameInBodyBind[3]-frame(ids[2])[3]})+
                    glm::length(glm::vec3{c.effectorFromObject[3]}))+c.radius+.000001);
            }
            // Preserve exact pair order and filtering, including all fixed-body pairs.
            for (std::size_t i=0;i<calibration.size();++i)
            {
                for (std::size_t j=0;j<scene.objects().size();++j)
                    if ((scene.objects()[j].id & (1ull<<63)) || world::interacts(settings.bodyFilter,scene.objects()[j].filter)) pairs.push_back({i,j,true});
                for (std::size_t j=i+1;j<calibration.size();++j)
                    if (checksSelfPair(calibration[i].id,calibration[j].id)) pairs.push_back({i,j,false});
            }
            if (context.carried)
            {
                const auto payload=calibration.size();
                for (std::size_t j=0;j<scene.objects().size();++j)
                    if (world::interacts(settings.bodyFilter,scene.objects()[j].filter)) pairs.push_back({payload,j,true});
                for (std::size_t j=0;j<calibration.size();++j)
                    if (calibration[j].id!=context.contactHand) pairs.push_back({payload,j,false});
            }
            out.status=PathStatus::NotChecked; out.detail="verification paused"; out.sceneVersion=scene.version(); out.trace.reserve(128);
            forms.reserve(calibration.size()+(context.carried ? 1:0)); stack.reserve(settings.maxDepth+1);
            out.scratchBytes=sizeof(Impl)+path.capacity()*sizeof(ArmCoordinates)+pose.locals.capacity()*sizeof(glm::mat4)+
                pose.globals.capacity()*sizeof(glm::mat4)+pose.palette.capacity()*sizeof(glm::mat4)+out.trace.capacity()*sizeof(std::array<glm::vec3,3>)+
                forms.capacity()*sizeof(BodyCollider)+pairs.capacity()*sizeof(Pair)+stack.capacity()*sizeof(Interval)+
                scene.objects().capacity()*sizeof(world::SceneObject)+scratch.coordinates().size()*sizeof(glm::vec3);
        }
        void finish(PathStatus status,const char* detail="")
        {
            out.status=status; out.detail=detail; phase=Phase::Done; evaluating=false;
            if (status==PathStatus::Clear) { out.fromTick=0; out.toTick=static_cast<double>(path.size()-1); }
        }
        bool beginPose(const ArmCoordinates& q,double lo,double hi)
        {
            out.fromTick=lo; out.toTick=hi;
            if (out.poses>=settings.maxPoses) { finish(PathStatus::Incomplete,"pose budget exhausted"); return false; }
            ++out.poses;
            for (std::size_t k=0;k<6;++k)
                if (scratch.setCoordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4,q[k]).status!=CommandStatus::Applied)
                    throw std::invalid_argument("Collision path coordinates outside profile limits");
            scratch.apply(pose.locals); animation::computeGlobalTransforms(binding.asset().skeleton(),pose.locals,pose.globals);
            forms=bodyColliders(binding,pose.globals,world);
            for (std::size_t handSide=0;handSide<2;++handSide) if (context.handEnvelopes[handSide])
            {
                const auto f=world*binding.jointFrame(handSide==0 ? JointId::LeftWrist:JointId::RightWrist,pose.globals);
                forms[handSide*3+2].capsule={glm::dvec3{f*glm::vec4{0,.025f,0,1}},glm::dvec3{f*glm::vec4{0,.160f,0,1}},.045};
            }
            if (context.carried)
            {
                forms.reserve(calibration.size()+1);
                const auto& c=*context.carried;
                const auto center=glm::dvec3{(world*binding.effectorFrame(c.effector,pose.globals)*c.effectorFromObject)[3]};
                forms.push_back({10,side==0 ? SegmentId::LeftHand:SegmentId::RightHand,{center,center,c.radius}});
            }
            if (out.trace.size()<128 && lo-traceLastTick>=std::max(1.0,static_cast<double>(path.size())/120.0))
            {
                std::array<glm::vec3,3> trace{};
                for (std::size_t k=0;k<3;++k) { const auto& c=forms[side*3+k].capsule; trace[k]=glm::vec3{(c.a+c.b)*.5}; }
                out.trace.push_back(trace); traceLastTick=lo;
            }
            pair=0; gap=std::numeric_limits<double>::infinity(); evaluating=true;
            return true;
        }
        void completePose()
        {
            evaluating=false;
            if (gap<=settings.margin+settings.numericSlack)
            {
                finish(out.fromTick==0 && out.toTick==0 && gap<=0 ? PathStatus::InitialIntersection:PathStatus::Blocked);
                return;
            }
            if (phase==Phase::Initial) { phase=Phase::Intervals; return; }
            if (phase==Phase::Final) { finish(PathStatus::Clear); return; }
            // DFS keeps the same floating-point subdivision and encounter order as 6A.
            if (gap>settings.margin+settings.numericSlack+2*radius*displacement) return;
            if (current.depth>=settings.maxDepth) { finish(PathStatus::Incomplete,"subdivision depth exhausted"); return; }
            const double t=(current.lo+current.hi)*.5;
            stack.push_back({midpoint,current.b,t,current.hi,current.depth+1});
            stack.push_back({current.a,midpoint,current.lo,t,current.depth+1});
        }
        bool advance(unsigned maxPoses,unsigned maxQueries)
        {
            if (phase==Phase::Done || !maxPoses || !maxQueries) return phase==Phase::Done;
            const auto begin=std::chrono::steady_clock::now();
            const unsigned poseBegin=out.poses, queryBegin=out.queries;
            while (phase!=Phase::Done)
            {
                if (!evaluating)
                {
                    if (out.poses-poseBegin>=maxPoses) break;
                    if (phase==Phase::Initial) { if (!beginPose(path.front(),0,0)) break; }
                    else
                    {
                        if (phase==Phase::Intervals)
                        {
                            if (stack.empty())
                            {
                                if (nextTick<path.size())
                                {
                                    stack.push_back({path[nextTick-1],path[nextTick],static_cast<double>(nextTick-1),static_cast<double>(nextTick),0});
                                    ++nextTick;
                                }
                                else phase=Phase::Final;
                            }
                            if (phase==Phase::Intervals)
                            {
                                current=stack.back(); stack.pop_back(); displacement=0;
                                for (std::size_t k=0;k<6;++k)
                                {
                                    midpoint[k]=(current.a[k]+current.b[k])*.5f;
                                    displacement+=std::abs(static_cast<double>(current.a[k])-current.b[k])*.5;
                                }
                                if (!beginPose(midpoint,current.lo,current.hi)) break;
                            }
                        }
                        if (phase==Phase::Final && !beginPose(path.back(),static_cast<double>(path.size()-1),static_cast<double>(path.size()-1))) break;
                    }
                }
                if (pair==pairs.size()) { completePose(); continue; }
                if (out.queries>=settings.maxQueries) { finish(PathStatus::Incomplete,"query budget exhausted"); break; }
                if (out.queries-queryBegin>=maxQueries) break;
                const auto& p=pairs[pair++]; const auto& a=forms[p.first];
                // Spatially bounded allowance: the selected sphere must be within 12 cm
                // of this hand's grip. It cannot mask another form or scene object.
                if (p.sceneObject && a.id==context.contactHand && scene.objects()[p.second].id==context.contactObject)
                {
                    const auto e=side==0 ? EffectorId::LeftGrip:EffectorId::RightGrip;
                    const auto center=glm::dvec3{(world*binding.effectorFrame(e,pose.globals))[3]};
                    if (glm::length(center-scene.objects()[p.second].position)<=.12) { ++out.queries; continue; }
                }
                const auto c=p.sceneObject ? world::clearance(a.capsule,scene.objects()[p.second]):
                    world::clearance(a.capsule,forms[p.second].capsule);
                ++out.queries;
                if (c.gap<gap)
                {
                    gap=c.gap; out.contact=c; out.bodyForm=a.id;
                    out.otherBodyForm=p.sceneObject ? 0:forms[p.second].id;
                    out.object=p.sceneObject ? scene.objects()[p.second].id:(a.id==10 && context.carried ? context.carried->object:0);
                }
            }
            out.microseconds+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
            return phase==Phase::Done;
        }
    };
    ArmPathCheckJob::ArmPathCheckJob(const DirectBodyControl& control,std::size_t side,std::span<const ArmCoordinates> path,
        const glm::mat4& world,const world::CollisionScene& scene,const CollisionSettings& settings,CollisionContext context)
        :impl_(std::make_unique<Impl>(control,side,path,world,scene,settings,std::move(context))) {}
    ArmPathCheckJob::~ArmPathCheckJob()=default;
    ArmPathCheckJob::ArmPathCheckJob(ArmPathCheckJob&&) noexcept=default;
    ArmPathCheckJob& ArmPathCheckJob::operator=(ArmPathCheckJob&&) noexcept=default;
    bool ArmPathCheckJob::advance(unsigned maxPoses,unsigned maxQueries) { return impl_->advance(maxPoses,maxQueries); }
    bool ArmPathCheckJob::done() const { return impl_->phase==Impl::Phase::Done; }
    const PathCheck& ArmPathCheckJob::result() const { return impl_->out; }
    PathCheck checkArmPath(const DirectBodyControl& control,std::size_t side,std::span<const ArmCoordinates> path,
        const glm::mat4& world,const world::CollisionScene& scene,const CollisionSettings& settings,CollisionContext context)
    {
        ArmPathCheckJob job(control,side,path,world,scene,settings,std::move(context));
        job.advance(std::numeric_limits<unsigned>::max(),std::numeric_limits<unsigned>::max());
        return job.result();
    }
}
