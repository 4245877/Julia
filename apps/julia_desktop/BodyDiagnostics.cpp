#include "BodyDiagnostics.hpp"
#include "../common/CollisionDemo.hpp"
#include "../common/SpatialDemo.hpp"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <array>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace julia::desktop
{
    using namespace embodiment;
    namespace
    {
        const DirectBodyControl& requireControl(const BodyInstance& body)
        { if (!body.control()) throw std::invalid_argument("Body diagnostics require calibration"); return *body.control(); }
    }
    BodyDiagnostics::BodyDiagnostics(BodyInstance& body):control_(requireControl(body)),body_(body)
    { soleContacts_=calibrateSoleContacts(control_.binding()); select(JointId::LeftElbow); }
    void BodyDiagnostics::select(JointId id,std::size_t coordinate)
    {
        (void)control_.coordinate(id,coordinate);
        joint_=control_.binding().jointIndex(id); coordinate_=coordinate;
    }
    std::string BodyDiagnostics::title() const
    {
        const auto& j=control_.binding().profile().joints[joint_]; const auto& c=j.coordinates[coordinate_];
        std::ostringstream s;
        const auto& support=body_.snapshot().support;
        if (support.enabled) {
            s << "SUPPORT tick=" << body_.snapshot().tick << ' ' << supportPhaseName(support.phase)
                << (support.kind==SupportKind::KinematicSupport ? " KinematicSupport":" Unbound") << " L/R=";
            for (const auto& a:support.anchors) s << a.positionError*1000 << "mm/" << glm::degrees(a.angularError) << "deg ";
            s << support.result << ' ' << support.reason << " | ";
        }
        const auto& interaction=body_.snapshot().interaction;
        if (body_.snapshot().object) s << "OBJECT " << body_.snapshot().object->handle.id << ' '
            << interactionPhaseName(interaction.phase) << " closure=" << body_.snapshot().hands[interaction.arm==ArmSide::Left ? 0:1].closure
            << (interaction.hold==HoldType::AssistedAttachment ? " AssistedAttachment":" Free") << " " << interaction.result << " " << interaction.reason << " | ";
        if (interaction.preparation) s << "PREP owner=" << static_cast<int>(body_.snapshot().armOwners[interaction.arm==ArmSide::Left ? 0:1])
            << " source=#" << interaction.sourceReach << " handle=" << interaction.object.id << ':' << interaction.object.generation
            << " err=" << interaction.frameError*1000 << "mm/" << glm::degrees(interaction.angularError)
            << "deg geometry=" << interaction.geometryCurrent << " | ";
        const auto& check=body_.snapshot().pathCheck;
        if (body_.snapshot().collisionScene) s << "COLLISION v" << body_.snapshot().collisionScene->version() << ' ' << pathStatusName(check.status)
            << " [" << check.bodyForm << '/' << check.otherBodyForm << " obj=" << check.object << "] | ";
        const auto& reach=body_.snapshot().reaches[reachArm_==ArmSide::Left ? 0:1];
        const auto& geometry=body_.snapshot().reachGeometry[reachArm_==ArmSide::Left ? 0:1];
        if (geometry.certificate.fixedStance) s << "ARM/FIXED-STANCE " << (geometry.executable ? "current":geometry.dependenciesMatch ? "historical":"stale") << " | ";
        const auto shownTask=reach.action ? reach.target.task:reachTask_;
        const unsigned shownEffector=reach.action ? static_cast<unsigned>(reach.target.effector)%4:reachEffector_;
        const auto shownStrategy=reach.action ? reach.target.strategy:reachStrategy_;
        const auto shownSelection=reach.action ? reach.target.selection:reachSelection_;
        s << "IK " << (reachArm_==ArmSide::Left ? "L":"R");
        s << ' ' << reachTaskName(shownTask) << (shownEffector==0 ? " wrist":shownEffector==1 ? " palm":" grip");
        s << (shownStrategy==MovementStrategy::AllowDetour ? " detour":" direct");
        s << (shownSelection==RouteSelection::Ranked ? " Ranked":" FirstValid");
        if (shownSelection==RouteSelection::Ranked) s << (elbowPositive_ ? " elbow +X":" elbow -X");
        if (reach.action) s << " #" << reach.action << ' ' << (reach.cancellationPending && (reach.state==ReachState::Moving || reach.state==ReachState::Settling) ? "stopping":reachStateName(reach.state))
            << " err=" << std::fixed << std::setprecision(1) << reach.error*1000 << "mm / " << glm::degrees(reach.angularError) << "deg " << reachReasonName(reach.reason);
        if (reach.action && (reach.target.strategy==MovementStrategy::AllowDetour || reach.planning.state!=PlanningState::Idle || reach.route.segmentCount)) {
            const auto& p=reach.planning;
            s << " plan=" << planningStateName(p.state) << " direct=" << pathStatusName(p.directCheck.status)
                << " stage=" << planningStateName(p.stage) << " seeds=" << p.seedsProcessed << '/' << p.seedAttempts
                << " IK=" << p.ikIterations << " unique=" << p.configurations << " selected=" << reach.route.selectedConfiguration
                << " cand=" << p.candidates << " q=" << p.queries << " tick=" << p.tickIterations << '/' << p.tickPoses << '/' << p.tickQueries;
            if (reach.route.segmentCount) s << " seg=" << reach.route.currentSegment+1 << '/' << reach.route.segmentCount;
            if (p.selection==RouteSelection::Ranked) s << " Clear=" << p.scoreEvaluations << " E/J/T=" << std::setprecision(4)
                << p.score.elbow.penalty << '/' << p.score.travel << '/' << p.score.duration << " score=" << p.score.total
                << " end=" << selectionCompletionName(p.completion);
        }
        s << " next target " << std::fixed << std::setprecision(2) << reachPosition_.x << ',' << reachPosition_.y << ',' << reachPosition_.z;
        s << " | BODY " << (control_.enabled() ? "ON ":"OFF ") << j.name << '/' << c.name
            << ' ' << std::setprecision(0) << glm::degrees(control_.coordinate(j.id,coordinate_))
            << "deg [" << glm::degrees(c.lower) << ',' << glm::degrees(c.upper) << "] " << result_;
        return s.str();
    }
    bool BodyDiagnostics::key(int key,int mods)
    {
        const auto& joints=control_.binding().profile().joints;
        if (pendingMode_ && body_.snapshot().tick>=pendingMode_->first) pendingMode_.reset();
        const bool enabled=pendingMode_ ? pendingMode_->second:control_.enabled();
        if (key==GLFW_KEY_K && (mods&GLFW_MOD_CONTROL)) {
            if (mods&GLFW_MOD_SHIFT) { preparationDemo_.stage=0; body_.enqueue(FinishGraspPreparation{body_.snapshot().interaction.action}); }
            else { preparationDemo_.arm=reachArm_; preparationDemo_.start(body_); }
            result_="fixed stance preparation queued";
        }
        else if (key==GLFW_KEY_J && (mods&GLFW_MOD_CONTROL)) {
            if (!body_.snapshot().direct) { result_="F2: select controlled pose before support demo"; return true; }
            if (mods&GLFW_MOD_SHIFT) { stanceDemo_.arm=reachArm_; stanceDemo_.start(body_); result_="fixed stance Reach demo queued"; }
            else { supportDemo_.start(body_); result_="support demo queued: explicit alignment then root motion"; }
        }
        else if (key==GLFW_KEY_H && (mods&GLFW_MOD_CONTROL)) {
            supportDemo_.stage=0;
            if (mods&GLFW_MOD_SHIFT) body_.enqueue(DisableSupport{});
            else if (!body_.snapshot().support.enabled) body_.enqueue(EnableSupport{});
            else body_.enqueue(AlignSupport{});
            result_="support enable/align/disable queued";
        }
        else if ((key==GLFW_KEY_DELETE && (mods&GLFW_MOD_ALT)) || (key==GLFW_KEY_BACKSPACE && (mods&GLFW_MOD_CONTROL))) {
            supportDemo_.stage=0; body_.enqueue(CancelSupportMotion{}); result_="finish support segment and rest";
        }
        else if ((mods&GLFW_MOD_ALT) && (key==GLFW_KEY_Z || key==GLFW_KEY_C || key==GLFW_KEY_V || key==GLFW_KEY_N || key==GLFW_KEY_O || key==GLFW_KEY_P)) {
            supportDemo_.stage=0; auto target=body_.snapshot().root.position;
            const int axis=(key==GLFW_KEY_Z || key==GLFW_KEY_C) ? 0:(key==GLFW_KEY_V || key==GLFW_KEY_N) ? 1:2;
            target[axis]+=(key==GLFW_KEY_C || key==GLFW_KEY_N || key==GLFW_KEY_P) ? .005f:-.005f;
            body_.enqueue(MoveSupportRoot{target}); result_="5 mm root move queued";
        }
        else if (key==GLFW_KEY_G && (mods&GLFW_MOD_CONTROL)) {
            if (!body_.snapshot().object) { demo_.arm=reachArm_; demo_.root=body_.snapshot().root; demo_.start(body_); }
            else if (mods&GLFW_MOD_SHIFT) { body_.enqueue(ReleaseObject{body_.snapshot().object->handle}); demo_.stage=0; }
            else { body_.enqueue(GraspObject{body_.snapshot().object->handle,reachArm_}); demo_.stage=2; }
            result_="interaction queued";
        }
        else if (key==GLFW_KEY_F2)
        {
            body_.enqueue(SetDirectMode{!enabled}); pendingMode_=std::pair{body_.snapshot().tick+1,!enabled};
            result_="queued";
        }
        else if (key==GLFW_KEY_F11 && (mods&GLFW_MOD_SHIFT))
        {
            const auto& scene=body_.snapshot().collisionScene;
            if (scene) sceneRevision_=std::max(sceneRevision_,scene->version());
            const auto root=body_.snapshot().root;
            const auto demo=app::multiConfigurationDemo(control_.binding(),++sceneRevision_,
                glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation));
            body_.enqueue(SetCollisionScene{demo.scene,{}}); reachPosition_=demo.target.position;
            result_="multi-IK demo scene queued";
        }
        else if (key==GLFW_KEY_F11)
        {
            const auto& scene=body_.snapshot().collisionScene;
            const bool blocked=scene && std::any_of(scene->objects().begin(),scene->objects().end(),[](const auto& o) { return o.id==101; });
            if (blocked) body_.enqueue(SetCollisionScene{nullptr,{}});
            else {
                if (scene) sceneRevision_=std::max(sceneRevision_,scene->version());
                const auto root=body_.snapshot().root;
                const auto demo=app::collisionDemo(control_.binding(),scene!=nullptr,++sceneRevision_,
                    glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation));
                body_.enqueue(SetCollisionScene{demo.scene,{}}); reachPosition_=demo.target.position;
            }
            result_="scene change queued (idle only)";
        }
        else if (key==GLFW_KEY_F12 && (mods&GLFW_MOD_CONTROL))
        {
            reachSelection_=reachSelection_==RouteSelection::FirstValid ? RouteSelection::Ranked:RouteSelection::FirstValid;
            result_="selection edited; F7 to commit";
        }
        else if (key==GLFW_KEY_F12 && (mods&GLFW_MOD_ALT))
        {
            elbowPositive_=!elbowPositive_; result_="elbow direction edited; F7 to commit";
        }
        else if (key==GLFW_KEY_F12 && (mods&GLFW_MOD_SHIFT))
        {
            reachConfigurations_=reachConfigurations_==1 ? 4:1;
            result_="IK configuration limit="+std::to_string(reachConfigurations_);
        }
        else if (key==GLFW_KEY_F12)
        {
            reachStrategy_=reachStrategy_==MovementStrategy::DirectOnly ? MovementStrategy::AllowDetour:MovementStrategy::DirectOnly;
            result_="strategy selected; F7 to commit";
        }
        else if (!enabled) return false;
        else if (key==GLFW_KEY_F7)
        {
            if ((mods&GLFW_MOD_CONTROL) && (mods&GLFW_MOD_SHIFT)) {
                reachEffector_=(reachEffector_+1)%3; result_="effector edited; F7 commits";
            }
            else if (mods&GLFW_MOD_CONTROL) {
                reachTask_=reachTask_==ReachTask::PositionOnly ? ReachTask::PositionAndOrientation:ReachTask::PositionOnly;
                if (reachTask_==ReachTask::PositionAndOrientation) reachOrientation_=normalizedReachOrientation(glm::quat_cast(glm::mat3{
                    control_.binding().effectorFrame(static_cast<EffectorId>((reachArm_==ArmSide::Left ? 0:4)+reachEffector_),body_.snapshot().pose.globals)}));
                result_="task edited, current hand orientation captured; F7 commits";
            }
            else if (mods&GLFW_MOD_ALT) {
                spatialSample_=(spatialSample_+1)%3;
                const auto demo=app::spatialDemo(control_.binding(),spatialSample_,reachArm_,reachEffector_);
                reachPosition_=demo.position; reachOrientation_=demo.orientation; reachTask_=demo.task;
                result_="spatial demo "+std::to_string(spatialSample_)+"; F7 commits";
            }
            else if (mods&GLFW_MOD_SHIFT)
            {
                reachArm_=reachArm_==ArmSide::Left ? ArmSide::Right:ArmSide::Left;
                reachPosition_.x=-reachPosition_.x;
                result_="arm selected";
            }
            else
            {
                ReachTarget target; target.arm=reachArm_; target.position=reachPosition_; target.strategy=reachStrategy_; target.detour.maxConfigurations=reachConfigurations_;
                target.task=reachTask_; target.orientation=reachOrientation_;
                target.selection=reachSelection_;
                if (target.selection==RouteSelection::Ranked) target.elbowPreference=ElbowPreference{{elbowPositive_ ? 1.0f:-1.0f,0,0},TargetSpace::Body};
                target.effector=static_cast<EffectorId>((reachArm_==ArmSide::Left ? 0:4)+reachEffector_);
                const auto& current=body_.snapshot().reaches[reachArm_==ArmSide::Left ? 0:1];
                if (current.state==ReachState::Moving || current.state==ReachState::Settling || current.state==ReachState::Planning)
                    body_.enqueue(RetargetReach{current.action,target});
                else body_.enqueue(StartReach{target});
                result_="reach queued";
            }
        }
        else if (key==GLFW_KEY_DELETE)
        {
            if (body_.snapshot().object && (mods&GLFW_MOD_CONTROL)) {
                body_.enqueue(CancelInteraction{body_.snapshot().interaction.action}); demo_.stage=0; preparationDemo_.stage=0;
            } else {
                const auto& current=body_.snapshot().reaches[reachArm_==ArmSide::Left ? 0:1];
                body_.enqueue(CancelReach{reachArm_,current.action});
            }
            result_="smooth stop queued";
        }
        // Six keys edit an arbitrary body-space position; F7 commits/retargets it.
        else if (key==GLFW_KEY_Z || key==GLFW_KEY_C || key==GLFW_KEY_V || key==GLFW_KEY_N || key==GLFW_KEY_O || key==GLFW_KEY_P)
        {
            const int component=key==GLFW_KEY_Z || key==GLFW_KEY_C ? 0:key==GLFW_KEY_V || key==GLFW_KEY_N ? 1:2;
            const float direction=key==GLFW_KEY_Z || key==GLFW_KEY_V || key==GLFW_KEY_O ? -1.0f:1.0f;
            if (mods&GLFW_MOD_CONTROL) {
                glm::vec3 axis{0}; axis[component]=1;
                reachOrientation_=normalizedReachOrientation(reachOrientation_*glm::angleAxis(glm::radians(direction*((mods&GLFW_MOD_SHIFT) ? 1.0f:5.0f)),axis));
            } else reachPosition_[component]+=direction*((mods&GLFW_MOD_SHIFT) ? .005f:.025f);
            result_="target edited; F7 to commit";
        }
        else if (key==GLFW_KEY_F3)
        { joint_=(joint_+((mods&GLFW_MOD_SHIFT) ? joints.size()-1:1))%joints.size(); coordinate_=0; result_="selected"; }
        else if (key==GLFW_KEY_F4) { coordinate_=(coordinate_+1)%joints[joint_].coordinates.size(); result_="selected"; }
        else if (key==GLFW_KEY_F5) overlay=!overlay;
        else if (key==GLFW_KEY_BACKSPACE)
        { body_.enqueue(ResetCoordinates{}); result_="queued"; }
        else if (key==GLFW_KEY_EQUAL || key==GLFW_KEY_MINUS || key==GLFW_KEY_KP_ADD || key==GLFW_KEY_KP_SUBTRACT)
        {
            const bool positive=key==GLFW_KEY_EQUAL || key==GLFW_KEY_KP_ADD;
            const auto id=joints[joint_].id;
            body_.enqueue(SetCoordinate{id,coordinate_,glm::radians(positive ? 5.0f:-5.0f),LimitPolicy::Clamp,true});
            result_="queued";
        }
        else return false;
        std::cout << title() << '\n';
        return true;
    }
    void BodyDiagnostics::draw(render_opengl::OpenGLRenderer& renderer,const glm::mat4& viewerFromResource,
        const BodyDisplay& display) const
    {
        if (!display.direct && !display.object) return;
        const auto& globals=display.pose.globals;
        const auto shownReach=[&](std::size_t side) -> const ReachObservation& {
            if (display.interaction.preparation && display.preparationReach.action==display.interaction.action &&
                side==(display.interaction.arm==ArmSide::Left ? 0u:1u)) return display.preparationReach;
            return display.reaches[side];
        };
        const auto& b=control_.binding(); const auto& p=b.profile();
        const auto coordinate=[&](JointId id,std::size_t k) { return display.coordinates[b.jointIndex(id)][static_cast<int>(k)]; };
        const auto displayFromBody=viewerFromResource*glm::inverse(p.bodyFromResource);
        std::vector<render_opengl::DebugLineVertex> lines;
        const auto line=[&](glm::vec3 a,glm::vec3 end,glm::vec3 color)
        { lines.push_back({glm::vec3{displayFromBody*glm::vec4{a,1}},color}); lines.push_back({glm::vec3{displayFromBody*glm::vec4{end,1}},color}); };
        const auto axes=[&](const glm::mat4& f,float length)
        {
            const auto origin=glm::vec3{f[3]};
            for (int k=0;k<3;++k) { glm::vec3 color{.05f}; color[k]=1; line(origin,origin+glm::vec3{f[k]}*length,color); }
        };
        if (display.object) {
            const auto worldToBody=glm::inverse(rigidMatrix({display.root.position,display.root.rotation}));
            const auto& o=*display.object;
            if (display.interaction.preparation) axes(worldToBody*rigidMatrix(o.worldPose)*rigidMatrix(o.localGrip),.075f);
            for (int axis=0;axis<3;++axis) for (int k=0;k<48;++k) {
                glm::vec3 a{0},end{0}; const float t=k*6.28318530718f/48,u=(k+1)*6.28318530718f/48;
                a[(axis+1)%3]=std::cos(t)*o.radius; a[(axis+2)%3]=std::sin(t)*o.radius;
                end[(axis+1)%3]=std::cos(u)*o.radius; end[(axis+2)%3]=std::sin(u)*o.radius;
                line(glm::vec3{worldToBody*glm::vec4{o.worldPose.position+o.worldPose.rotation*a,1}},
                    glm::vec3{worldToBody*glm::vec4{o.worldPose.position+o.worldPose.rotation*end,1}},{1,.2f,.05f});
            }
        }
        if (!overlay || !display.direct) { renderer.renderDebugLines(lines); return; }
        if (display.support.enabled) {
            const auto worldToBody=glm::inverse(rigidMatrix({display.root.position,display.root.rotation}));
            const auto worldLine=[&](glm::vec3 a,glm::vec3 b,glm::vec3 color) {
                line(glm::vec3{worldToBody*glm::vec4{a,1}},glm::vec3{worldToBody*glm::vec4{b,1}},color);
            };
            const auto& o=display.support; const auto center=(glm::vec3{o.anchors[0].desiredWorld[3]}+glm::vec3{o.anchors[1].desiredWorld[3]})*.5f;
            for (int k=-5;k<=5;++k) {
                const float t=k*.1f, y=o.surface.height;
                worldLine({center.x+t,y,center.z-.5f},{center.x+t,y,center.z+.5f},{.3f,.4f,.3f});
                worldLine({center.x-.5f,y,center.z+t},{center.x+.5f,y,center.z+t},{.3f,.4f,.3f});
            }
            const auto& contacts=soleContacts_;
            for (unsigned side=0;side<2;++side) {
                const auto& a=o.anchors[side]; axes(worldToBody*a.desiredWorld,.05f); axes(worldToBody*a.actualWorld,.035f);
                for (unsigned k=0;k<4;++k) {
                    const auto goal0=glm::vec3{a.desiredWorld*glm::vec4{contacts[side].points[k],1}};
                    const auto goal1=glm::vec3{a.desiredWorld*glm::vec4{contacts[side].points[(k+1)%4],1}};
                    worldLine(goal0,goal1,{1,.15f,1}); worldLine(a.worldPoints[k],a.worldPoints[(k+1)%4],{.1f,1,.2f});
                    worldLine(goal0,a.worldPoints[k],{1,.1f,.1f});
                }
                for (unsigned k=0;k<2;++k) worldLine(a.chain[k],a.chain[k+1],{.1f,.7f,1});
                axes(glm::translate(glm::mat4{1},glm::vec3{worldToBody*glm::vec4{a.chain[1],1}}),.025f);
            }
            axes(glm::mat4{1},.08f);
        }
        for (const auto& j:p.joints)
        {
            std::uint32_t end=animation::NoNode;
            for (std::uint32_t n=0;n<b.asset().nodes().size();++n) if (b.asset().nodes()[n].name==j.segmentEnd.name) { end=n; break; }
            if (end!=animation::NoNode) line(glm::vec3{b.jointFrame(j.id,globals)[3]},glm::vec3{p.bodyFromResource*globals[end][3]},{.65f,.5f,.08f});
        }
        for (const auto& e:p.effectors) axes(b.effectorFrame(e.id,globals),.035f);
        const auto cross=[&](glm::vec3 point,glm::vec3 color,float radius)
        { for (int k=0;k<3;++k) { glm::vec3 d{0}; d[k]=radius; line(point-d,point+d,color); } };
        if (display.collisionScene)
        {
            const auto worldFromBody=glm::translate(glm::mat4{1},display.root.position)*glm::mat4_cast(display.root.rotation);
            const auto worldToBody=glm::inverse(worldFromBody);
            const auto worldLine=[&](glm::dvec3 a,glm::dvec3 b,glm::vec3 color) {
                line(glm::vec3{worldToBody*glm::vec4{glm::vec3{a},1}},glm::vec3{worldToBody*glm::vec4{glm::vec3{b},1}},color);
            };
            const auto sphere=[&](glm::dvec3 center,double radius,glm::vec3 color) {
                for (int axis=0;axis<3;++axis) for (int k=0;k<24;++k) {
                    glm::dvec3 a{0},b{0}; const double x=k*6.283185307179586/24,y=(k+1)*6.283185307179586/24;
                    a[(axis+1)%3]=std::cos(x)*radius; a[(axis+2)%3]=std::sin(x)*radius;
                    b[(axis+1)%3]=std::cos(y)*radius; b[(axis+2)%3]=std::sin(y)*radius;
                    worldLine(center+a,center+b,color);
                }
            };
            for (const auto& f:display.colliders) {
                const auto& c=f.capsule; const glm::vec3 color{.2f,.9f,.8f}; sphere(c.a,c.radius,color); sphere(c.b,c.radius,color);
                if (glm::length(c.b-c.a)<1e-9) continue;
                const auto d=glm::normalize(c.b-c.a); auto u=glm::cross(d,glm::dvec3{1,0,0});
                if (glm::length(u)<.01) u=glm::cross(d,glm::dvec3{0,1,0});
                u=glm::normalize(u); const auto v=glm::cross(d,u);
                for (const auto& offset:std::array{u*c.radius,-u*c.radius,v*c.radius,-v*c.radius}) worldLine(c.a+offset,c.b+offset,color);
            }
            for (const auto& o:display.collisionScene->objects()) {
                if (display.object && o.id==((1ull<<63)|display.object->handle.id)) continue;
                const glm::vec3 color{1,.55f,.1f};
                if (const auto* s=std::get_if<world::Sphere>(&o.shape)) sphere(o.position,s->radius,color);
                else if (const auto* box=std::get_if<world::Box>(&o.shape)) {
                    for (int corner=0;corner<8;++corner) for (int axis=0;axis<3;++axis) if (!(corner&(1<<axis))) {
                        glm::dvec3 cornerA,cornerB;
                        for (int k=0;k<3;++k) { cornerA[k]=(corner&(1<<k) ? 1:-1)*box->halfExtent[k]; cornerB[k]=cornerA[k]; }
                        cornerB[axis]=-cornerB[axis]; worldLine(o.position+o.rotation*cornerA,o.position+o.rotation*cornerB,color);
                    }
                } else for (int k=-4;k<=4;++k) {
                    const double t=k*.2;
                    worldLine(o.position+o.rotation*glm::dvec3{t,0,-.8},o.position+o.rotation*glm::dvec3{t,0,.8},color);
                    worldLine(o.position+o.rotation*glm::dvec3{-.8,0,t},o.position+o.rotation*glm::dvec3{.8,0,t},color);
                }
            }
            const auto& check=display.pathCheck;
            for (std::size_t t=1;t<check.trace.size();++t) for (std::size_t k=0;k<3;++k)
                worldLine(check.trace[t-1][k],check.trace[t][k],k==0 ? glm::vec3{.4f,.5f,1}:k==1 ? glm::vec3{1,1,.1f}:glm::vec3{.7f,.3f,1});
            if (check.status==PathStatus::Blocked || check.status==PathStatus::InitialIntersection) {
                const auto point=glm::vec3{worldToBody*glm::vec4{glm::vec3{check.contact.otherPoint},1}};
                cross(point,{1,.05f,.05f},.03f); worldLine(check.contact.otherPoint,check.contact.bodyPoint,{1,0,0});
            }
            // Both previews use the validator's bounded traces. They never authorize motion.
            for (const auto* entry:std::array{&shownReach(0),&shownReach(1)}) if (const auto& reach=*entry; reach.action && (reach.target.strategy==MovementStrategy::AllowDetour || reach.route.segmentCount)) {
                const auto& route=reach.route.segmentCount ? reach.route:reach.planning;
                const auto drawTrace=[&](const PathCheck& path,glm::vec3 color) {
                    for (std::size_t t=1;t<path.trace.size();++t) for (std::size_t k=0;k<3;++k)
                        worldLine(path.trace[t-1][k],path.trace[t][k],color);
                };
                drawTrace(route.directCheck,{1,.15f,.1f});
                for (unsigned segment=0;segment<route.segmentCount;++segment)
                    drawTrace(route.segmentChecks[segment],segment==0 ? glm::vec3{.2f,1,.25f}:glm::vec3{.15f,.7f,1});
                const auto drawPose=[&](const ArmCoordinates& q,glm::vec3 color) {
                    auto coordinates=display.coordinates;
                    const auto ids=reach.target.arm==ArmSide::Left ?
                        std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
                        std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist};
                    for (std::size_t k=0;k<6;++k)
                        coordinates[b.jointIndex(ids[k<3 ? 0:k==3 ? 1:2])][static_cast<int>(k<3 ? k:k==3 ? 0:k-4)]=q[k];
                    animation::Pose waypoint(b.asset().skeleton());
                    DirectBodyControl preview(control_);
                    for (std::size_t j=0;j<p.joints.size();++j)
                        for (std::size_t k=0;k<p.joints[j].coordinates.size();++k)
                            preview.setCoordinate(p.joints[j].id,k,coordinates[j][static_cast<int>(k)]);
                    preview.apply(waypoint.locals);
                    animation::computeGlobalTransforms(b.asset().skeleton(),waypoint.locals,waypoint.globals);
                    const auto a=glm::vec3{b.jointFrame(ids[0],waypoint.globals)[3]},
                        e=glm::vec3{b.jointFrame(ids[1],waypoint.globals)[3]}, w=glm::vec3{b.jointFrame(ids[2],waypoint.globals)[3]};
                    line(a,e,color); line(e,w,color); axes(b.effectorFrame(reach.target.effector,waypoint.globals),.045f);
                    cross(glm::vec3{b.effectorFrame(reach.target.effector,waypoint.globals)[3]},color,.025f);
                };
                if (route.segmentCount==2) drawPose(route.waypoint,{1,1,.1f});
                if (route.selectedConfiguration) drawPose(route.finalCoordinates,{1,.4f,1});
            }
        }
        cross(reachPosition_,{1,.2f,1},.018f);
        if (reachTask_==ReachTask::PositionAndOrientation) {
            auto goal=glm::mat4_cast(reachOrientation_); goal[3]=glm::vec4{reachPosition_,1}; axes(goal,.075f);
        }
        for (std::size_t side=0;side<2;++side)
        {
            const auto& r=shownReach(side); if (!r.action) continue;
            if (r.elbowDirectionBody) line(r.shoulderBody,r.shoulderBody+*r.elbowDirectionBody*.18f,{1,.6f,.05f});
            const auto shoulder=side==0 ? JointId::LeftShoulder:JointId::RightShoulder,
                elbow=side==0 ? JointId::LeftElbow:JointId::RightElbow;
            const auto hand=glm::vec3{b.effectorFrame(r.target.effector,globals)[3]}, e=glm::vec3{b.jointFrame(elbow,globals)[3]},
                sh=glm::vec3{b.jointFrame(shoulder,globals)[3]};
            cross(r.goalBody,{1,0,1},.02f); cross(r.referenceBody,{0,1,1},.012f); cross(hand,{.1f,1,.2f},.012f);
            if (r.target.task==ReachTask::PositionAndOrientation) {
                auto goal=glm::mat4_cast(r.goalOrientationBody); goal[3]=glm::vec4{r.goalBody,1}; axes(goal,.075f);
                axes(b.effectorFrame(r.target.effector,globals),.05f);
            }
            line(hand,r.goalBody,{1,.2f,.2f}); line(sh,e,{0,1,1}); line(e,hand,{0,1,1});
            line(e,e+r.bendNormalBody*.10f,{.8f,.3f,1});
            line(sh,sh+r.bendNormalBody*.04f,{.8f,.3f,1});
        }
        const auto& j=p.joints[joint_]; const auto& c=j.coordinates[coordinate_];
        const auto current=b.jointFrame(j.id,globals); axes(current,.09f);
        glm::mat4 total{1},previous{1};
        for (std::size_t k=0;k<j.coordinates.size();++k)
        {
            if (k<coordinate_) previous=glm::rotate(previous,coordinate(j.id,k),j.coordinates[k].axis);
            total=glm::rotate(total,coordinate(j.id,k),j.coordinates[k].axis);
        }
        const auto active=current*glm::inverse(total)*previous;
        const auto origin=glm::vec3{active[3]},axis=glm::vec3{active*glm::vec4{c.axis,0}};
        line(origin-axis*.1f,origin+axis*.1f,{1,1,0});
        auto tangent=glm::cross(c.axis,glm::vec3{0,1,0});
        if (glm::length(tangent)<.01f) tangent=glm::cross(c.axis,glm::vec3{1,0,0});
        tangent=glm::normalize(tangent)*.075f;
        const auto arc=[&](float angle) { return glm::vec3{active*glm::rotate(glm::mat4{1},angle,c.axis)*glm::vec4{tangent,1}}; };
        for (int k=0;k<32;++k)
            line(arc(c.lower+(c.upper-c.lower)*static_cast<float>(k)/32),arc(c.lower+(c.upper-c.lower)*static_cast<float>(k+1)/32),{1,.35f,.05f});
        line(origin,arc(c.lower),{1,.1f,.1f}); line(origin,arc(c.upper),{.1f,1,.1f});
        line(origin,arc(coordinate(j.id,coordinate_)),{1,1,0});
        renderer.renderDebugLines(lines);
    }

    void verifyDesktopSpatial(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output); std::ofstream report(output/"desktop_spatial.txt"),ticks(output/"ticks.csv");
        report << std::setprecision(17) << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        ticks << std::setprecision(17) << "sample,tick,state,error_m,angular_error_rad,iterations,evaluations,poses,queries,configuration,candidate,q0,q1,q2,q3,q4,q5\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        unsigned captures=0;
        const auto capture=[&](const std::string& name) {
            const auto& binding=body.control()->binding(); render_opengl::CameraConfig camera;
            camera.target=glm::vec3{fit*glm::inverse(binding.profile().bodyFromResource)*glm::vec4{.22f,1.18f,.2f,1}};
            camera.position=camera.target+glm::vec3{.8f,.15f,1.5f}; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto before=body.snapshot().tick; const auto display=body.display(1);
            renderer.beginFrame(); renderer.renderModel(model,fit*body.resourceRoot(display.root),display.pose.palette);
            diagnostics.draw(renderer,fit*body.resourceRoot(display.root),display);
            require(body.display(1).pose.palette==display.pose.palette && body.snapshot().tick==before,"Spatial rendering mutated body");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3); glPixelStorei(GL_PACK_ALIGNMENT,1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+".ppm"),std::ios::binary); image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image) && glGetError()==GL_NO_ERROR,"Spatial capture/GL failure");
            glfwSetWindowTitle(window,diagnostics.title().c_str()); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}});
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step();
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,0); input(window,GLFW_KEY_F12,0,GLFW_PRESS,GLFW_MOD_SHIFT);
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        glm::vec3 wristPosition{0};
        for (unsigned sample=0;sample<3;++sample) {
            input(window,GLFW_KEY_F7,0,GLFW_PRESS,GLFW_MOD_ALT);
            const auto initial=body.snapshot().coordinates;
            input(window,GLFW_KEY_F7,0,GLFW_PRESS,0);
            require(body.snapshot().coordinates==initial,"Spatial callback mutated q before tick"); body.step();
            require(body.results().back().status==BodyCommandStatus::Applied,"Spatial callback command rejected");
            capture(std::to_string(sample)+"_target");
            bool capturedMotion=false;
            for (unsigned t=0;t<16000;++t) {
                body.step(); const auto& o=body.snapshot().reaches[0]; const auto& p=o.planning;
                ticks << sample << ',' << body.snapshot().tick << ',' << static_cast<int>(o.state) << ',' << o.error << ',' << o.angularError << ','
                    << p.ikIterations << ',' << p.ikEvaluations << ',' << p.poses << ',' << p.queries << ',' << o.route.selectedConfiguration << ',' << o.route.selectedCandidate;
                for (unsigned k=0;k<6;++k) ticks << ',' << body.control()->coordinate(k<3 ? JointId::LeftShoulder:k==3 ? JointId::LeftElbow:JointId::LeftWrist,k<3 ? k:k==3 ? 0:k-4);
                ticks << '\n';
                if (o.state==ReachState::Planning) require(body.snapshot().coordinates==initial,"Spatial planning moved body");
                if (!capturedMotion && o.state==ReachState::Moving) { capture(std::to_string(sample)+"_route"); capturedMotion=true; }
                if (o.state==ReachState::Succeeded) break;
                require(o.state!=ReachState::Failed && t<15999,"Spatial desktop goal failed");
            }
            const auto& o=body.snapshot().reaches[0];
            require(o.error<=o.target.tolerance && o.angularError<=o.target.angularTolerance,"Spatial desktop false success");
            if (sample) require(glm::length(o.goalBody-wristPosition)<1e-6f,"Spatial demo changed wrist position");
            wristPosition=o.goalBody; capture(std::to_string(sample)+"_complete");
            report << "sample=" << sample << " error_m=" << o.error << " angular_error_rad=" << o.angularError
                << " iterations=" << o.planning.ikIterations << " evaluations=" << o.planning.ikEvaluations
                << " poses=" << o.planning.poses << " queries=" << o.planning.queries << " planning_ticks=" << o.planning.ticks
                << " configurations=" << o.planning.configurations << " selected=" << o.route.selectedConfiguration
                << " candidate=" << o.route.selectedCandidate << " score=" << o.route.score.total << '\n';
        }
        // Exercise local target rotation edits and task changes without advancing q.
        const auto held=body.snapshot().coordinates;
        input(window,GLFW_KEY_C,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        input(window,GLFW_KEY_Z,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        require(body.snapshot().coordinates==held,"Editing orientation changed actual body");
        report << "captures=" << captures << "\ncallbacks=F2,F11,F12,ShiftF12,CtrlF12,AltF7,F7,CtrlC,CtrlZ,CtrlF7\n"
            << "same_wrist_position=3_orientations\nrender=read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopCollision(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output); std::ofstream report(output/"desktop_collision.txt");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        unsigned captures=0;
        const auto capture=[&](const std::string& name) {
            render_opengl::CameraConfig camera;
            const auto& b=body.control()->binding();
            camera.target=glm::vec3{fit*glm::inverse(b.profile().bodyFromResource)*glm::vec4{.13f,1.10f,.18f,1}};
            camera.position=camera.target+glm::vec3{1,.25f,2.1f}; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick; const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit*body.resourceRoot(display.root),display.pose.palette); diagnostics.draw(renderer,fit*body.resourceRoot(display.root),display);
            const auto again=body.display(.5);
            require(again.pose.palette==display.pose.palette && tick==body.snapshot().tick,"Collision rendering mutated body");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3); glPixelStorei(GL_PACK_ALIGNMENT,1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+".ppm"),std::ios::binary); image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Cannot save collision frame"); require(glGetError()==GL_NO_ERROR,"Collision GL error");
            glfwSetWindowTitle(window,diagnostics.title().c_str()); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step(); // Off -> free.
        require(body.snapshot().collisionScene!=nullptr,"F11 free scene failed");
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step(); // Free -> blocked.
        const auto before=body.snapshot().coordinates;
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        require(body.results().back().status==BodyCommandStatus::Rejected && before==body.snapshot().coordinates,"Blocked desktop path moved");
        const auto hit=body.snapshot().pathCheck;
        require(hit.status==PathStatus::Blocked && hit.object==101 && hit.bodyForm==2,"Desktop collision diagnosis failed");
        capture("blocked_forearm");
        report << "blocked_form=" << hit.bodyForm << "\nobject=" << hit.object << "\ninterval=" << hit.fromTick << ',' << hit.toTick << "\ngap_m=" << hit.contact.gap << '\n';
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step(); // Blocked -> off.
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step(); // Off -> free, higher revision.
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        require(body.snapshot().pathCheck.status==PathStatus::Clear,"Desktop free path rejected"); capture("free_start");
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step();
        require(body.results().back().status==BodyCommandStatus::Rejected,"Scene modified during active certified segment");
        for (int t=0;t<1200;++t) {
            body.step(); const auto& r=body.snapshot().reaches[0];
            if (t==45 || t==90) capture("free_"+std::to_string(t));
            if (r.state==ReachState::Succeeded) break;
            require(t<1199,"Desktop collision-free reach did not complete");
        }
        require(body.snapshot().reaches[0].error<=.002f,"Collision mode changed IK accuracy"); capture("free_complete");
        report << "error_m=" << body.snapshot().reaches[0].error << "\ncaptures=" << captures << "\ncallbacks=F2,F11,F7\nrender=single_display_read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopDetour(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_detour.txt"),csv(output/"ticks.csv");
        if (!report || !csv) throw std::runtime_error("Cannot write detour evidence");
        report << std::setprecision(12) << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        csv << "tick,state,planning_state,candidates,queries,poses,planning_ticks,segments,current_segment,error_m\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        unsigned captures=0;
        const auto capture=[&](const std::string& name) {
            render_opengl::CameraConfig camera;
            const auto& b=body.control()->binding();
            camera.target=glm::vec3{fit*glm::inverse(b.profile().bodyFromResource)*glm::vec4{.13f,1.10f,.18f,1}};
            camera.position=camera.target+glm::vec3{1,.25f,2.1f}; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick; const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit*body.resourceRoot(display.root),display.pose.palette);
            diagnostics.draw(renderer,fit*body.resourceRoot(display.root),display);
            const auto again=body.display(.5);
            require(again.pose.palette==display.pose.palette && tick==body.snapshot().tick,"Detour rendering mutated body");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3); glPixelStorei(GL_PACK_ALIGNMENT,1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+".ppm"),std::ios::binary); image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Cannot save detour frame"); require(glGetError()==GL_NO_ERROR,"Detour GL error");
            glfwSetWindowTitle(window,diagnostics.title().c_str()); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step();
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step();
        const auto initial=body.snapshot().coordinates;
        const auto originalScene=body.snapshot().collisionScene;
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        require(body.results().back().status==BodyCommandStatus::Rejected && body.snapshot().pathCheck.status==PathStatus::Blocked,
            "DirectOnly demo must remain blocked");
        require(initial==body.snapshot().coordinates,"Rejected direct route changed start");
        const auto direct=body.snapshot().pathCheck; capture("01_direct_blocked");
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        require(body.results().back().status==BodyCommandStatus::Applied,"AllowDetour callback rejected");
        require(body.snapshot().reaches[0].state==ReachState::Planning,"AllowDetour did not enter planning");
        capture("02_planning");
        bool sawRoute=false,sawSecond=false;
        for (unsigned t=0;t<12000;++t) {
            body.step(); const auto& r=body.snapshot().reaches[0]; const auto& p=r.planning;
            csv << body.snapshot().tick << ',' << static_cast<int>(r.state) << ',' << static_cast<int>(p.state) << ','
                << p.candidates << ',' << p.queries << ',' << p.poses << ',' << p.ticks << ',' << r.route.segmentCount << ',' << r.route.currentSegment << ',' << r.error << '\n';
            require(body.snapshot().collisionScene==originalScene,"Detour changed collision scene");
            if (r.state==ReachState::Planning) require(body.snapshot().coordinates==initial,"Planning moved body");
            if (!sawRoute && r.route.segmentCount==2 && r.state==ReachState::Moving) {
                require(r.route.directCheck.status==PathStatus::Blocked,"Detour lost direct refusal reason");
                require(r.route.segmentChecks[0].status==PathStatus::Clear && r.route.segmentChecks[1].status==PathStatus::Clear,"Unverified route accepted");
                capture("03_route_selected"); sawRoute=true;
            }
            if (!sawSecond && r.route.segmentCount==2 && r.route.currentSegment==1) { capture("04_second_segment"); sawSecond=true; }
            if (r.state==ReachState::Succeeded) break;
            require(r.state!=ReachState::Failed && r.state!=ReachState::Cancelled,"Desktop detour failed");
            require(t<11999,"Desktop detour did not finish");
        }
        const auto& r=body.snapshot().reaches[0]; const auto& p=r.planning;
        require(sawRoute && sawSecond && r.state==ReachState::Succeeded && r.error<=r.target.tolerance,"Desktop did not execute real waypoint detour");
        capture("05_complete");
        report << "direct=" << pathStatusName(direct.status) << "\ndirect_object=" << direct.object << "\ndirect_scene=" << direct.sceneVersion
            << "\nroute_scene=" << r.collisionSceneVersion << "\nplanning_ticks=" << p.ticks << "\ncandidates=" << p.candidates
            << "\nqueries=" << p.queries << "\nposes=" << p.poses << "\nselected_candidate=" << p.selectedCandidate
            << "\nsegments=" << r.route.segmentCount << "\nsegment0=" << pathStatusName(r.route.segmentChecks[0].status)
            << "\nsegment1=" << pathStatusName(r.route.segmentChecks[1].status)
            << "\nsegment0_scene=" << r.route.segmentChecks[0].sceneVersion << "\nsegment1_scene=" << r.route.segmentChecks[1].sceneVersion
            << "\nsegment0_start_tick=" << r.route.segmentChecks[0].segmentStartTick
            << "\nsegment1_start_tick=" << r.route.segmentChecks[1].segmentStartTick << "\nwaypoint=";
        for (const auto q:r.route.waypoint) report << q << ',';
        report << "\nerror_m=" << r.error << "\ncaptures=" << captures
            << "\ncallbacks=F2,F11,F7,F12\npreview=red_blocked_direct,green_first,blue_second,yellow_waypoint\n"
            << "render=single_display_read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopMultiIK(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_multik.txt"),csv(output/"ticks.csv");
        if (!report || !csv) throw std::runtime_error("Cannot write detour evidence");
        report << std::setprecision(12) << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        csv << "tick,state,planning_state,candidates,queries,poses,planning_ticks,segments,current_segment,error_m,stage,seeds,iterations,unique,selected_configuration,tick_iterations,tick_poses,tick_queries\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        unsigned captures=0;
        const auto capture=[&](const std::string& name) {
            render_opengl::CameraConfig camera;
            const auto& b=body.control()->binding();
            camera.target=glm::vec3{fit*glm::inverse(b.profile().bodyFromResource)*glm::vec4{.13f,1.10f,.18f,1}};
            camera.position=camera.target+glm::vec3{1,.25f,2.1f}; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick; const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit*body.resourceRoot(display.root),display.pose.palette);
            diagnostics.draw(renderer,fit*body.resourceRoot(display.root),display);
            const auto again=body.display(.5);
            require(again.pose.palette==display.pose.palette && tick==body.snapshot().tick,"Detour rendering mutated body");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3); glPixelStorei(GL_PACK_ALIGNMENT,1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+".ppm"),std::ios::binary); image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Cannot save detour frame"); require(glGetError()==GL_NO_ERROR,"Detour GL error");
            glfwSetWindowTitle(window,diagnostics.title().c_str()); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,GLFW_MOD_SHIFT); body.step();
        const auto initial=body.snapshot().coordinates;
        const auto originalScene=body.snapshot().collisionScene;
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        require(body.results().back().status==BodyCommandStatus::Rejected && body.snapshot().pathCheck.status==PathStatus::Blocked,
            "DirectOnly demo must remain blocked");
        require(initial==body.snapshot().coordinates,"Rejected direct route changed start");
        const auto direct=body.snapshot().pathCheck; capture("01_direct_blocked");
        // First exercise the complete 6B candidate search on this identical scene.
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        for (unsigned t=0;t<12000 && body.snapshot().reaches[0].state==ReachState::Planning;++t) body.step();
        require(body.snapshot().reaches[0].state==ReachState::Failed && body.snapshot().reaches[0].planning.candidates==32,"One-configuration witness must exhaust 6B");
        require(body.snapshot().coordinates==initial,"Failed 6B witness moved body");
        report << "single_configuration=Exhausted_32_candidates\n";
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,GLFW_MOD_SHIFT);
        input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
        require(body.results().back().status==BodyCommandStatus::Applied,"AllowDetour callback rejected");
        require(body.snapshot().reaches[0].state==ReachState::Planning,"AllowDetour did not enter planning");
        capture("02_planning");
        bool sawRoute=false;
        for (unsigned t=0;t<12000;++t) {
            body.step(); const auto& r=body.snapshot().reaches[0]; const auto& p=r.planning;
            csv << body.snapshot().tick << ',' << static_cast<int>(r.state) << ',' << static_cast<int>(p.state) << ','
                << p.candidates << ',' << p.queries << ',' << p.poses << ',' << p.ticks << ',' << r.route.segmentCount << ',' << r.route.currentSegment << ',' << r.error << ',' << planningStateName(p.stage) << ',' << p.seedAttempts << ',' << p.ikIterations << ',' << p.configurations << ',' << r.route.selectedConfiguration << ',' << p.tickIterations << ',' << p.tickPoses << ',' << p.tickQueries << '\n';
            require(body.snapshot().collisionScene==originalScene,"Detour changed collision scene");
            if (r.state==ReachState::Planning) require(body.snapshot().coordinates==initial,"Planning moved body");
            if (!sawRoute && r.route.segmentCount==1 && r.state==ReachState::Moving) {
                require(r.route.directCheck.status==PathStatus::Blocked,"Detour lost direct refusal reason");
                require(r.route.segmentChecks[0].status==PathStatus::Clear && r.route.selectedConfiguration>1,"Unverified alternative route accepted");
                capture("03_route_selected"); sawRoute=true;
            }

            if (r.state==ReachState::Succeeded) break;
            require(r.state!=ReachState::Failed && r.state!=ReachState::Cancelled,"Desktop detour failed");
            require(t<11999,"Desktop detour did not finish");
        }
        const auto& r=body.snapshot().reaches[0]; const auto& p=r.planning;
        require(sawRoute && r.route.selectedConfiguration>1 && r.state==ReachState::Succeeded && r.error<=r.target.tolerance,"Desktop did not execute alternative configuration route");
        capture("05_complete");
        report << "direct=" << pathStatusName(direct.status) << "\ndirect_object=" << direct.object << "\ndirect_scene=" << direct.sceneVersion
            << "\nroute_scene=" << r.collisionSceneVersion << "\nplanning_ticks=" << p.ticks << "\ncandidates=" << p.candidates
            << "\nqueries=" << p.queries << "\nposes=" << p.poses << "\nselected_candidate=" << p.selectedCandidate
            << "\nselected_configuration=" << r.route.selectedConfiguration << "\nseeds=" << p.seedAttempts << "\niterations=" << p.ikIterations << "\nduplicates=" << p.duplicates
            << "\nsegments=" << r.route.segmentCount << "\nsegment0=" << pathStatusName(r.route.segmentChecks[0].status)
            << "\nsegment1=" << pathStatusName(r.route.segmentChecks[1].status)
            << "\nsegment0_scene=" << r.route.segmentChecks[0].sceneVersion << "\nsegment1_scene=" << r.route.segmentChecks[1].sceneVersion
            << "\nsegment0_start_tick=" << r.route.segmentChecks[0].segmentStartTick
            << "\nsegment1_start_tick=" << r.route.segmentChecks[1].segmentStartTick << "\nwaypoint=";
        for (const auto q:r.route.waypoint) report << q << ',';
        report << "\nerror_m=" << r.error << "\ncaptures=" << captures
            << "\ncallbacks=F2,Shift+F11,F7,F12,Shift+F12\npreview=red_first_blocked_direct,green_certified_route,magenta_selected_final_pose\n"
            << "render=single_display_read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopRanked(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_ranked.txt"),csv(output/"ticks.csv");
        if (!report || !csv) throw std::runtime_error("Cannot write ranked evidence");
        report << std::setprecision(12) << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        csv << "case,tick,state,planning_ticks,scores,configuration,candidate,elbow,travel,duration,total,completion,error_m\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        unsigned captures=0;
        const auto capture=[&](const std::string& name) {
            render_opengl::CameraConfig camera; const auto& b=body.control()->binding();
            camera.target=glm::vec3{fit*glm::inverse(b.profile().bodyFromResource)*glm::vec4{.2f,1.14f,.18f,1}};
            camera.position=camera.target+glm::vec3{.7f,.2f,1.8f}; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick; const auto display=body.display(1);
            renderer.beginFrame(); renderer.renderModel(model,fit*body.resourceRoot(display.root),display.pose.palette);
            diagnostics.draw(renderer,fit*body.resourceRoot(display.root),display);
            require(body.display(1).pose.palette==display.pose.palette && body.snapshot().tick==tick,"Ranked draw mutated body");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3); glPixelStorei(GL_PACK_ALIGNMENT,1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+".ppm"),std::ios::binary); image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image) && glGetError()==GL_NO_ERROR,"Ranked GPU capture failed");
            glfwSetWindowTitle(window,diagnostics.title().c_str()); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F11,0,GLFW_PRESS,0); body.step(); // Unchanged free demo scene and goal.
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,GLFW_MOD_SHIFT);
        input(window,GLFW_KEY_F12,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        std::array<glm::vec3,2> elbows{}; std::array<unsigned,2> configurations{};
        for (unsigned sign=0;sign<2;++sign) {
            if (sign) {
                input(window,GLFW_KEY_BACKSPACE,0,GLFW_PRESS,0); body.step();
                input(window,GLFW_KEY_F12,0,GLFW_PRESS,GLFW_MOD_ALT);
            }
            const auto held=body.snapshot().coordinates;
            input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
            require(body.results().back().status==BodyCommandStatus::Applied,"Ranked F7 rejected");
            capture(std::to_string(sign)+"_planning");
            bool savedBest=false,admitted=false;
            for (unsigned t=0;t<16000;++t) {
                const auto& r=body.snapshot().reaches[0]; const auto& p=r.planning;
                csv << sign << ',' << body.snapshot().tick << ',' << reachStateName(r.state) << ',' << p.ticks << ',' << p.scoreEvaluations
                    << ',' << r.route.selectedConfiguration << ',' << r.route.selectedCandidate << ',' << p.score.elbow.penalty << ',' << p.score.travel
                    << ',' << p.score.duration << ',' << p.score.total << ',' << selectionCompletionName(p.completion) << ',' << r.error << '\n';
                if (r.state==ReachState::Planning) {
                    require(body.snapshot().coordinates==held && r.route.segmentCount==0,"Ranked moved before selection ended");
                    if (!savedBest && p.scoreEvaluations) { capture(std::to_string(sign)+"_best_pending"); savedBest=true; }
                }
                if (!admitted && r.state==ReachState::Moving) {
                    require(r.route.scoreEvaluations>=2 && r.route.segmentChecks[0].status==PathStatus::Clear,"Ranked did not compare verified routes");
                    capture(std::to_string(sign)+"_selected"); admitted=true;
                }
                if (r.state==ReachState::Succeeded) break;
                require(r.state!=ReachState::Failed && r.state!=ReachState::Cancelled && t<15999,"Ranked desktop action failed");
                body.step();
            }
            const auto& r=body.snapshot().reaches[0];
            require(savedBest && admitted && r.state==ReachState::Succeeded && r.error<=r.target.tolerance,"Ranked goal not reached");
            elbows[sign]=r.elbowBody; configurations[sign]=r.route.selectedConfiguration; capture(std::to_string(sign)+"_complete");
            report << "case=" << sign << " configuration=" << configurations[sign] << " scores=" << r.route.scoreEvaluations << " ticks=" << r.route.ticks
                << " queries=" << r.route.queries << " poses=" << r.route.poses << " seeds=" << r.route.seedAttempts << " iterations=" << r.route.ikIterations
                << " completion=" << selectionCompletionName(r.route.completion) << " E=" << r.route.score.elbow.penalty << " J=" << r.route.score.travel
                << " T=" << r.route.score.duration << " total=" << r.route.score.total << " error_m=" << r.error
                << " elbow=" << r.elbowBody.x << ',' << r.elbowBody.y << ',' << r.elbowBody.z << '\n';
        }
        require(configurations[0]!=configurations[1] && glm::length(elbows[0]-elbows[1])>.02f,"Elbow direction did not change selected pose");
        report << "callbacks=F2,F11,F12,Shift+F12,Ctrl+F12,Alt+F12,F7,Backspace\ncaptures=" << captures
            << "\nrender=single_display_read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopReach(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_ik.txt"),csv(output/"ticks.csv");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        csv << "side,tick,action,state,error_m,actual_x,actual_y,actual_z\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step();
        require(body.snapshot().direct,"IK F2 callback failed");
        unsigned captures=0;
        const auto capture=[&](int side,int frame)
        {
            render_opengl::CameraConfig camera;
            camera.target=glm::vec3{fit*glm::vec4{model.asset().bounds().center(),1}};
            camera.position=camera.target+glm::vec3{side==0 ? .8f:-.8f,.1f,3.2f};
            renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick; const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit,display.pose.palette); diagnostics.draw(renderer,fit,display);
            const auto again=body.display(.5);
            require(again.pose.palette==display.pose.palette && body.snapshot().tick==tick,"IK draw mutated simulation");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
            glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(std::to_string(side)+"_"+std::to_string(frame)+".ppm"),std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Cannot save IK screenshot");
            require(glGetError()==GL_NO_ERROR,"IK GL error"); ++captures;
            glfwSwapBuffers(window); glfwPollEvents();
        };
        for (int side=0;side<2;++side)
        {
            if (side) input(window,GLFW_KEY_F7,0,GLFW_PRESS,GLFW_MOD_SHIFT);
            DirectBodyControl source(body.control()->binding()); source.setEnabled(true);
            const auto shoulder=side==0 ? JointId::LeftShoulder:JointId::RightShoulder,
                elbow=side==0 ? JointId::LeftElbow:JointId::RightElbow;
            source.setCoordinate(shoulder,0,.45f); source.setCoordinate(shoulder,1,.25f); source.setCoordinate(elbow,0,.8f);
            animation::Pose targetPose(model.asset().skeleton()); source.apply(targetPose.locals);
            animation::computeGlobalTransforms(model.asset().skeleton(),targetPose.locals,targetPose.globals);
            const auto effector=side==0 ? EffectorId::LeftWrist:EffectorId::RightWrist;
            const auto goal=glm::vec3{body.control()->binding().effectorFrame(effector,targetPose.globals)[3]};
            diagnostics.setReachPosition(goal); input(window,GLFW_KEY_F7,0,GLFW_PRESS,0);
            const auto before=body.snapshot().coordinates;
            require(body.snapshot().coordinates==before,"F7 changed q outside tick");
            for (int tick=0;tick<800;++tick)
            {
                body.step(); const auto& r=body.snapshot().reaches[static_cast<std::size_t>(side)];
                csv << side << ',' << body.snapshot().tick << ',' << r.action << ',' << static_cast<int>(r.state) << ',' << r.error << ',' << r.actualBody.x << ',' << r.actualBody.y << ',' << r.actualBody.z << '\n';
                if (tick%45==0) capture(side,tick);
                if (r.state==ReachState::Succeeded) { capture(side,999); break; }
                require(tick<799,"Desktop IK did not complete");
            }
            require(body.snapshot().reaches[static_cast<std::size_t>(side)].error<=.002f,"Desktop IK accuracy failed");
            // Real keyboard editing, retarget during motion, then smooth cancellation.
            input(window,GLFW_KEY_P,0,GLFW_PRESS,0); input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
            for (int k=0;k<20;++k) body.step();
            input(window,GLFW_KEY_O,0,GLFW_PRESS,0); input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
            require(body.results().back().status==BodyCommandStatus::Applied,"Desktop retarget rejected");
            input(window,GLFW_KEY_DELETE,0,GLFW_PRESS,0); body.step();
            for (int k=0;k<800 && body.snapshot().reaches[static_cast<std::size_t>(side)].state!=ReachState::Cancelled;++k) body.step();
            require(body.snapshot().reaches[static_cast<std::size_t>(side)].state==ReachState::Cancelled,"Desktop cancel did not settle");
            capture(side,1000);
        }
        report << "captures=" << captures << "\ncallbacks=F2,F7,ShiftF7,O,P,Delete\nwrists=left,right\n"
            << "render=interpolated_read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopStancePreparation(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output); std::ofstream report(output/"desktop_stance_preparation.txt"),trace(output/"stance-preparation.csv");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        const auto require=[](bool ok,const std::string& why) { if (!ok) throw std::runtime_error(why); };
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}}); input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step();
        input(window,GLFW_KEY_K,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        bool ready=false; unsigned captures=0; int previous=-1; float maxP=0,maxA=0,maxGap=0;
        for (unsigned t=0;t<8000;++t) {
            diagnostics.updateInteractionDemo(); body.step(); app::stancePreparationTrace(trace,body.snapshot());
            const auto& s=body.snapshot();
            for (const auto& r:body.results()) require(r.status!=BodyCommandStatus::Rejected,r.reason);
            require(s.support.result!="PlanRejected",s.support.reason);
            const int state=static_cast<int>(s.support.phase)*100+static_cast<int>(s.interaction.phase);
            if (s.interaction.phase==InteractionPhase::ReadyToGrasp) {
                ready=true; require(s.interaction.geometryCurrent && s.armOwners[0]==ArmOwner::GraspPreparation,"Ready lacks current certificate/owner");
                require(s.object && s.object->state==ObjectState::Free && s.object->owner==0,"Ready attached object");
            }
            require(s.interaction.phase!=InteractionPhase::Closing && s.interaction.hold==HoldType::None,"Preparation closed/attached");
            if (s.interaction.preparation) require(s.root.position==glm::vec3{0},"Preparation moved root");
            for (double alpha:{0.,.23,.5,.77,1.}) {
                const auto d=body.display(alpha);
                if (d.support.kind==SupportKind::KinematicSupport) for (const auto& a:d.support.anchors) {
                    maxP=std::max(maxP,a.positionError); maxA=std::max(maxA,a.angularError); maxGap=std::max({maxGap,std::abs(a.minGap),std::abs(a.maxGap)});
                    require(a.bound && a.positionError<=SupportPositionTolerance && a.angularError<=SupportAngularTolerance &&
                        a.minGap>=-SupportSurfaceTolerance && a.maxGap<=SupportSurfaceTolerance,"Desktop soles outside tolerances");
                }
            }
            if ((state!=previous || (s.interaction.preparation && t%120==0) || t==7999) && captures<96) {
                report << "tick=" << s.tick << " support=" << supportPhaseName(s.support.phase) << " reach=" << reachStateName(s.reaches[0].state)
                    << " preparation=" << interactionPhaseName(s.interaction.phase) << " scope=preparation_vs_fixed_stance\n";
                for (unsigned view=0;view<3;++view) {
                    const auto d=body.display(.37); const auto again=body.display(.37);
                    require(d.pose.palette==again.pose.palette && s.tick==body.snapshot().tick,"Display mutated simulation");
                    const auto transform=fit*body.resourceRoot(d.root);
                    const auto viewerFromWorld=fit*glm::inverse(body.control()->binding().profile().bodyFromResource);
                    const auto target=glm::vec3{viewerFromWorld*glm::vec4{d.root.position+glm::vec3{0,.78f,0},1}};
                    render_opengl::CameraConfig camera; camera.target=target;
                    camera.position=target+(view==0 ? glm::vec3{0,.2f,2.5f}:view==1 ? glm::vec3{2.5f,.2f,0}:glm::vec3{-1.7f,.25f,-2.f});
                    camera.nearPlane=.005f; renderer.setCamera(camera);
                    int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
                    diagnostics.overlay=view!=0; renderer.beginFrame(); renderer.renderModel(model,transform,d.pose.palette); diagnostics.draw(renderer,transform,d);
                    std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
                    glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
                    std::ofstream image(output/(std::to_string(s.tick)+"_"+std::to_string(view)+".ppm"),std::ios::binary);
                    image << "P6\n" << width << ' ' << height << "\n255\n";
                    for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
                    require(static_cast<bool>(image) && glGetError()==GL_NO_ERROR,"Preparation GPU capture failed"); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
                }
            }
            previous=state;
        }
        const auto& s=body.snapshot(); require(ready && s.reaches[0].state==ReachState::Succeeded && !s.support.enabled && !s.object,"Desktop preparation/independent Reach demo incomplete");
        report << "captures=" << captures << "\nmax_display_p_angle_gap=" << maxP << ',' << maxA << ',' << maxGap
            << "\ncallbacks=F2,CtrlK\nrender=one_read_only_BodyDisplay\nGL_errors=0\nresult=PASS\n";
    }
    void verifyDesktopStanceReach(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output); std::ofstream report(output/"desktop_stance_reach.txt"),trace(output/"stance-reach.csv");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        const auto require=[](bool ok,const std::string& why) { if (!ok) throw std::runtime_error(why); };
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}}); input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step();
        input(window,GLFW_KEY_J,0,GLFW_PRESS,GLFW_MOD_CONTROL|GLFW_MOD_SHIFT);
        unsigned captures=0; int previous=-1; float maxP=0,maxA=0,maxGap=0;
        for (unsigned t=0;t<12000;++t) {
            diagnostics.updateInteractionDemo(); body.step(); app::stanceReachTrace(trace,body.snapshot());
            const auto& s=body.snapshot();
            for (const auto& r:body.results()) require(r.status!=BodyCommandStatus::Rejected,r.reason);
            require(s.support.result!="PlanRejected",s.support.reason);
            const int state=static_cast<int>(s.support.phase)*10+static_cast<int>(s.reaches[0].state);
            for (double alpha:{0.,.23,.5,.77,1.}) {
                const auto d=body.display(alpha);
                if (d.support.kind==SupportKind::KinematicSupport) for (const auto& a:d.support.anchors) {
                    maxP=std::max(maxP,a.positionError); maxA=std::max(maxA,a.angularError); maxGap=std::max({maxGap,std::abs(a.minGap),std::abs(a.maxGap)});
                    require(a.bound && a.positionError<=SupportPositionTolerance && a.angularError<=SupportAngularTolerance &&
                        a.minGap>=-SupportSurfaceTolerance && a.maxGap<=SupportSurfaceTolerance,"Desktop soles outside tolerances");
                }
            }
            if ((state!=previous || (s.reaches[0].state==ReachState::Moving && t%120==0) || t==11999) && captures<96) {
                report << "tick=" << s.tick << " support=" << supportPhaseName(s.support.phase) << " reach=" << reachStateName(s.reaches[0].state)
                    << " current=" << s.reachGeometry[0].executable << " scope=arm_vs_fixed_stance\n";
                for (unsigned view=0;view<3;++view) {
                    const auto d=body.display(.37); const auto again=body.display(.37);
                    require(d.pose.palette==again.pose.palette && s.tick==body.snapshot().tick,"Display mutated simulation");
                    const auto transform=fit*body.resourceRoot(d.root);
                    const auto viewerFromWorld=fit*glm::inverse(body.control()->binding().profile().bodyFromResource);
                    const auto target=glm::vec3{viewerFromWorld*glm::vec4{d.root.position+glm::vec3{0,.78f,0},1}};
                    render_opengl::CameraConfig camera; camera.target=target;
                    camera.position=target+(view==0 ? glm::vec3{0,.2f,2.5f}:view==1 ? glm::vec3{2.5f,.2f,0}:glm::vec3{-1.7f,.25f,-2.f});
                    camera.nearPlane=.005f; renderer.setCamera(camera);
                    int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
                    diagnostics.overlay=view!=0; renderer.beginFrame(); renderer.renderModel(model,transform,d.pose.palette); diagnostics.draw(renderer,transform,d);
                    std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
                    glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
                    std::ofstream image(output/(std::to_string(s.tick)+"_"+std::to_string(view)+".ppm"),std::ios::binary);
                    image << "P6\n" << width << ' ' << height << "\n255\n";
                    for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
                    require(static_cast<bool>(image) && glGetError()==GL_NO_ERROR,"Stance Reach GPU capture failed"); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
                }
            }
            previous=state;
        }
        const auto& s=body.snapshot(); require(s.reaches[0].state==ReachState::Succeeded && s.support.phase==SupportPhase::Holding && s.root.position.x==.008f,"Desktop coordinated demo incomplete");
        require(diagnostics.title().find("PositionAndOrientation grip")!=std::string::npos,"Desktop title describes selector instead of admitted action");
        report << "captures=" << captures << "\nmax_display_p_angle_gap=" << maxP << ',' << maxA << ',' << maxGap
            << "\ncallbacks=F2,CtrlShiftJ\nrender=one_read_only_BodyDisplay\nGL_errors=0\nresult=PASS\n";
    }
    void verifyDesktopSupport(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output); std::ofstream report(output/"desktop_support.txt"), trace(output/"support.csv");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        const auto require=[](bool ok,const std::string& why) { if (!ok) throw std::runtime_error(why); };
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}}); input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step();
        unsigned captures=0;
        const auto capture=[&](const std::string& name,int view,bool overlay) {
            const auto tick=body.snapshot().tick; const auto d=body.display(.37); const auto again=body.display(.37);
            require(d.pose.palette==again.pose.palette && body.snapshot().tick==tick,"Support display mutated simulation");
            const auto transform=fit*body.resourceRoot(d.root); const auto& binding=body.control()->binding();
            const auto viewerFromWorld=fit*glm::inverse(binding.profile().bodyFromResource);
            const auto centerWorld=(d.support.anchors[0].chain[0]+d.support.anchors[1].chain[0]+d.support.anchors[0].chain[2]+d.support.anchors[1].chain[2])*.25f-glm::vec3{0,.08f,0};
            auto target=glm::vec3{viewerFromWorld*glm::vec4{centerWorld,1}};
            render_opengl::CameraConfig camera; camera.target=target;
            camera.position=target+(view==0 ? glm::vec3{0,.03f,1.95f}:view==1 ? glm::vec3{1.95f,.03f,0}:glm::vec3{-1.1f,.12f,-1.7f});
            camera.nearPlane=.005f; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            diagnostics.overlay=overlay; renderer.beginFrame(); renderer.renderModel(model,transform,d.pose.palette); diagnostics.draw(renderer,transform,d);
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
            glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+"_"+std::to_string(view)+(overlay ? "_overlay":"")+".ppm"),std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image) && glGetError()==GL_NO_ERROR,"Support GPU capture failed"); glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        input(window,GLFW_KEY_J,0,GLFW_PRESS,GLFW_MOD_CONTROL);
        auto previous=SupportPhase::Disabled; std::array<float,2> displayP{},displayA{},displayGap{};
        for (unsigned t=0;t<12000;++t) {
            diagnostics.updateInteractionDemo(); body.step(); app::supportTrace(trace,body.snapshot());
            const auto& s=body.snapshot(); const auto& o=s.support;
            for (const auto& r:body.results()) require(r.status!=BodyCommandStatus::Rejected,r.reason);
            require(o.result!="PlanRejected",o.reason);
            if (o.phase!=previous || (o.phase==SupportPhase::Moving && o.executedStates==180)) {
                const auto name=std::to_string(s.tick)+"_"+supportPhaseName(o.phase);
                report << "tick=" << s.tick << " phase=" << supportPhaseName(o.phase) << " p_L=" << o.anchors[0].positionError << " p_R=" << o.anchors[1].positionError << '\n';
                for (int view=0;view<3;++view) capture(name,view,false);
                capture(name,0,true); previous=o.phase;
            }
            if (o.kind==SupportKind::KinematicSupport) for (double alpha:{0.,.23,.5,.77,1.}) {
                const auto d=body.display(alpha);
                for (unsigned side=0;side<2;++side) if (d.support.anchors[side].bound) {
                    const auto& f=d.support.anchors[side]; displayP[side]=std::max(displayP[side],f.positionError); displayA[side]=std::max(displayA[side],f.angularError);
                    displayGap[side]=std::max({displayGap[side],std::abs(f.minGap),std::abs(f.maxGap)});
                }
            }
        }
        require(body.snapshot().support.result=="Succeeded" && body.snapshot().root.position==glm::vec3{0},"Desktop support cycle did not return");
        for (unsigned side=0;side<2;++side) {
            require(displayP[side]<=SupportPositionTolerance && displayA[side]<=SupportAngularTolerance && displayGap[side]<=SupportSurfaceTolerance,"Interpolated desktop soles slipped");
            report << "display_side=" << side << " p=" << displayP[side] << " angle=" << displayA[side] << " gap=" << displayGap[side] << '\n';
        }
        // Independent keyboard handlers use the same limited root command and cancellation.
        input(window,GLFW_KEY_V,0,GLFW_PRESS,GLFW_MOD_ALT); body.step();
        for (unsigned t=0;t<4000 && body.snapshot().support.phase==SupportPhase::Planning;++t) body.step();
        require(body.snapshot().support.phase==SupportPhase::Moving,body.snapshot().support.reason);
        for (int t=0;t<50;++t) body.step(); input(window,GLFW_KEY_BACKSPACE,0,GLFW_PRESS,GLFW_MOD_CONTROL); body.step();
        require(body.snapshot().support.cancellationPending,"Desktop cancellation did not preserve moving segment");
        for (unsigned t=0;t<500 && body.snapshot().support.phase==SupportPhase::Moving;++t) body.step();
        require(body.snapshot().support.result=="Cancelled","Desktop cancellation did not reach rest");
        body.enqueue(MoveSupportRoot{{0,1,0}}); body.step(); require(body.results().back().status==BodyCommandStatus::Rejected,"High root was not rejected");
        capture("cancelled_rejected",0,true);
        input(window,GLFW_KEY_H,0,GLFW_PRESS,GLFW_MOD_CONTROL|GLFW_MOD_SHIFT); body.step(); require(!body.snapshot().support.enabled,"Desktop disable failed");
        report << "captures=" << captures << "\ncallbacks=CtrlJ,AltV,CtrlBackspace,CtrlShiftH\nrender=read_only_interpolated\nsupport=KinematicSupport\nforces=unavailable\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopInteraction(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_interaction.txt");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        unsigned captures=0;
        const auto capture=[&](const std::string& name,int view) {
            const auto d=body.display(.5); const auto again=body.display(.5);
            require(d.pose.palette==again.pose.palette,"Interaction render changed simulation");
            const auto transform=fit*body.resourceRoot(d.root);
            const auto& b=body.control()->binding();
            const auto e=d.interaction.arm==ArmSide::Left ? EffectorId::LeftGrip:EffectorId::RightGrip;
            const auto center=glm::vec3{transform*glm::inverse(b.profile().bodyFromResource)*b.effectorFrame(e,d.pose.globals)[3]};
            render_opengl::CameraConfig camera; camera.target=center;
            camera.position=center+(view==0 ? glm::vec3{.18f,.08f,.42f}:view==1 ? glm::vec3{-.32f,.08f,.12f}:glm::vec3{.05f,-.24f,-.35f});
            camera.nearPlane=.005f; renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            diagnostics.overlay=false; renderer.beginFrame(); renderer.renderModel(model,transform,d.pose.palette); diagnostics.draw(renderer,transform,d);
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
            glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+"_"+std::to_string(view)+".ppm"),std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image) && glGetError()==GL_NO_ERROR,"Interaction GPU capture failed");
            glfwSwapBuffers(window); glfwPollEvents(); ++captures;
        };
        for (unsigned side=0;side<2;++side) {
            if (side) {
                body.enqueue(DeleteInteractionObject{body.snapshot().object->handle}); body.enqueue(ResetBody{}); body.step();
                input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step();
                input(window,GLFW_KEY_F7,0,GLFW_PRESS,GLFW_MOD_SHIFT); // Select right via real callback.
            }
            std::ofstream trace(output/(std::string{side==0 ? "left":"right"}+".csv"));
            input(window,GLFW_KEY_G,0,GLFW_PRESS,GLFW_MOD_CONTROL);
            auto previous=InteractionPhase::Idle;
            for (unsigned t=0;t<10000;++t) {
                diagnostics.updateInteractionDemo(); body.step(); app::interactionTrace(trace,body.snapshot());
                for (const auto& r:body.results()) require(r.status!=BodyCommandStatus::Rejected,r.reason.c_str());
                const auto& s=body.snapshot();
                if (s.interaction.phase!=previous) {
                    report << "side=" << side << " tick=" << s.tick << " phase=" << interactionPhaseName(s.interaction.phase)
                        << " result=" << s.interaction.result << " reason=" << s.interaction.reason << '\n'; report.flush();
                    for (int view=0;view<3;++view) capture(std::to_string(side)+"_"+interactionPhaseName(s.interaction.phase)+(s.interaction.result.empty() ? "":"_"+s.interaction.result),view);
                    previous=s.interaction.phase;
                }
                if (s.object && s.object->state==ObjectState::Attached) {
                    const auto& b=body.control()->binding(); const auto d=body.display(.37);
                    const auto f=rigidMatrix({d.root.position,d.root.rotation})*b.effectorFrame(side==0 ? EffectorId::LeftGrip:EffectorId::RightGrip,d.pose.globals);
                    require(glm::length(glm::vec3{(f*d.object->effectorFromObject)[3]}-d.object->worldPose.position)<1e-6f,"Display attachment slipped");
                }
                require(s.interaction.phase!=InteractionPhase::Failed && s.interaction.phase!=InteractionPhase::ReleaseBlocked,"Desktop interaction failed");
                if (s.interaction.phase==InteractionPhase::Succeeded) break;
                require(t<9999,"Desktop interaction timed out");
            }
            require(body.snapshot().interaction.result=="Released","Desktop cycle did not release");
        }
        report << "captures=" << captures << "\ncallbacks=CtrlG,ShiftF7\nhold=AssistedAttachment\nforces=unavailable\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopBody(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,const std::filesystem::path& output,
        void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_body.txt"),samples(output/"poses.csv");
        if (!report || !samples) throw std::runtime_error("Cannot write body diagnostic evidence");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        samples << "pose,view,colored_pixels,changed_pixels\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        const auto& control=*body.control();
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}}); body.step();
        std::size_t captures=0;
        std::array<std::vector<unsigned char>,3> zeroPixels;
        const auto capture=[&](const std::string& name,int view)
        {
            render_opengl::CameraConfig camera;
            const auto center=glm::vec3{fit*glm::vec4{model.asset().bounds().center(),1}};
            camera.target=center;
            camera.position=center+(view==0 ? glm::vec3{0,.1f,4}:glm::vec3{view==1 ? 4.0f:-4.0f,.1f,0});
            renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick;
            const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit,display.pose.palette);
            diagnostics.draw(renderer,fit,display);
            for (int repeat=0;repeat<3;++repeat)
            {
                const auto same=body.display(.5);
                renderer.beginFrame(); renderer.renderModel(model,fit,same.pose.palette);
                diagnostics.draw(renderer,fit,same);
                require(body.snapshot().tick==tick,"Render advanced simulation tick");
                require(same.pose.palette==display.pose.palette,"Repeated display changed palette");
            }
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
            glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+(view==0 ? "_front.ppm":view==1 ? "_side.ppm":"_right_side.ppm")),std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Body capture failed");
            require(glGetError()==GL_NO_ERROR,"OpenGL body diagnostic error");
            glfwSwapBuffers(window); glfwPollEvents();
            std::size_t colored=0; for (std::size_t i=0;i<pixels.size();i+=3) if (std::max({pixels[i],pixels[i+1],pixels[i+2]})-std::min({pixels[i],pixels[i+1],pixels[i+2]})>25) ++colored;
            require(colored>1000,"Body capture contains no model/overlay");
            auto& base=zeroPixels[static_cast<std::size_t>(view)];
            if (name=="zero") base=pixels;
            std::size_t changed=0;
            if (!base.empty())
                for (std::size_t i=0;i<pixels.size();i+=3)
                    if (std::abs(static_cast<int>(pixels[i])-base[i])+std::abs(static_cast<int>(pixels[i+1])-base[i+1])+
                        std::abs(static_cast<int>(pixels[i+2])-base[i+2])>15) ++changed;
            if (view==0 && name!="zero" && !base.empty()) require(changed>10,"Body command did not change rendered pixels");
            ++captures;
            samples << name << ',' << view << ',' << colored << ',' << changed << '\n';
        };
        // Exercise the actual application callbacks for toggle, increments, reset and limits.
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0);
        require(!control.enabled(),"F2 mutated body before tick"); body.step();
        require(control.enabled(),"F2 body activation failed");
        diagnostics.select(JointId::LeftElbow);
        for (int k=0;k<4;++k) input(window,GLFW_KEY_EQUAL,0,GLFW_PRESS,0);
        require(control.coordinate(JointId::LeftElbow,0)==0,"Keyboard mutated q before tick"); body.step();
        require(std::abs(control.coordinate(JointId::LeftElbow,0)-glm::radians(20.0f))<1e-6f,"Elbow +20 command failed");
        for (int v=0;v<3;++v) capture("left_elbow20_overlay",v);
        input(window,GLFW_KEY_BACKSPACE,0,GLFW_PRESS,0);
        body.step();
        require(control.coordinate(JointId::LeftElbow,0)==0,"Body reset callback failed");
        for (int k=0;k<30;++k) input(window,GLFW_KEY_EQUAL,0,GLFW_PRESS,0);
        body.step();
        require(std::abs(control.coordinate(JointId::LeftElbow,0)-glm::radians(100.0f))<1e-6f,"Body clamp failed");
        body.enqueue(ResetCoordinates{}); body.step(); diagnostics.overlay=false;
        for (int v=0;v<3;++v) capture("zero",v);
        const auto pose=[&](const char* name,JointId id,std::size_t coordinate,float degrees)
        {
            body.enqueue(ResetCoordinates{}); diagnostics.select(id,coordinate);
            body.enqueue(SetCoordinate{id,coordinate,glm::radians(degrees)}); body.step();
            require(body.results().back().status==BodyCommandStatus::Applied,"Body pose rejected");
            for (int v=0;v<3;++v) capture(name,v);
        };
        pose("left_elbow20",JointId::LeftElbow,0,20); pose("right_elbow20",JointId::RightElbow,0,20);
        pose("left_elbow60",JointId::LeftElbow,0,60); pose("right_elbow60",JointId::RightElbow,0,60);
        pose("left_elbow100",JointId::LeftElbow,0,100); pose("right_elbow100",JointId::RightElbow,0,100);
        pose("left_knee20",JointId::LeftKnee,0,20); pose("right_knee20",JointId::RightKnee,0,20);
        pose("left_knee60",JointId::LeftKnee,0,60); pose("right_knee60",JointId::RightKnee,0,60);
        pose("left_shoulder45",JointId::LeftShoulder,1,45); pose("right_shoulder45",JointId::RightShoulder,1,45);
        pose("left_roll30",JointId::LeftShoulder,2,30); pose("right_roll30",JointId::RightShoulder,2,30);
        pose("left_wrist25",JointId::LeftWrist,0,25); pose("right_wrist25",JointId::RightWrist,0,25);
        pose("left_ankle20",JointId::LeftAnkle,0,20); pose("right_ankle20",JointId::RightAnkle,0,20);
        body.enqueue(ResetCoordinates{}); body.step(); diagnostics.overlay=true; diagnostics.select(JointId::LeftElbow);
        for (int v=0;v<3;++v) capture("zero_overlay",v);
        input(window,GLFW_KEY_F3,0,GLFW_PRESS,0); input(window,GLFW_KEY_F4,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step(); require(!control.enabled(),"Body disable callback failed");
        report << "captures=" << captures << "\ncallbacks=queued_toggle,relative_increment,clamp,reset,select_joint,select_coordinate\n"
            << "snapshots=interpolated\nrepeat_draw=immutable\nGL_errors=0\nresult=PASS\n";
    }
}
