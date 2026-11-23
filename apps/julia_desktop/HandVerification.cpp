#include <glad/glad.h>
#include "HandVerification.hpp"
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>

namespace julia::desktop
{
    using namespace embodiment;

    namespace
    {
        constexpr std::array<const char*,5> fingerNames{"index","middle","ring","pinky","thumb"};
        constexpr std::array<const char*,2> sideNames{"left","right"};

        void refresh(const BodyInstance& body,BodyDisplay& d)
        {
            animation::computeGlobalTransforms(body.asset().skeleton(),d.pose.locals,d.pose.globals);
            animation::buildSkinningPalette(body.asset().skeleton(),d.pose.globals,d.pose.palette);
        }

        // Calibrated wrist frame in world metres; the profile's +Z points to the back of the hand.
        glm::mat4 worldHand(const BodyInstance& body,const BodyDisplay& d,unsigned side)
        {
            return rigidMatrix({d.root.position,d.root.rotation})*
                body.control()->binding().jointFrame(side==0 ? JointId::LeftWrist:JointId::RightWrist,d.pose.globals);
        }

        std::vector<HandLine> skeletonOverlay(const BodyInstance& body,const HandLayer& layer,const BodyDisplay& d,unsigned side)
        {
            std::vector<HandLine> lines;
            const auto& b=body.control()->binding();
            const auto world=rigidMatrix({d.root.position,d.root.rotation})*b.profile().bodyFromResource;
            const auto point=[&](std::uint32_t n) { return glm::vec3{world*d.pose.globals[n][3]}; };
            const auto wristNode=b.jointNode(side==0 ? JointId::LeftWrist:JointId::RightWrist);
            const glm::vec3 colors[5]{{1,.9f,.1f},{.1f,1,.3f},{.1f,.8f,1},{1,.3f,1},{1,.45f,.1f}};
            std::array<std::array<std::uint32_t,3>,5> node{};
            for (const auto& j:layer.joints()) if (j.side==side) node[j.finger][j.phalanx]=j.node;
            for (unsigned f=0;f<5;++f) {
                lines.push_back({point(wristNode),point(node[f][0]),{.6f,.6f,.6f}});
                lines.push_back({point(node[f][0]),point(node[f][1]),colors[f]});
                lines.push_back({point(node[f][1]),point(node[f][2]),colors[f]});
                const auto g=world*d.pose.globals[node[f][2]];
                lines.push_back({point(node[f][2]),point(node[f][2])+glm::normalize(glm::vec3{g[1]})*.016f,colors[f]});
                // Calibrated flexion axes carried by the parent of each joint (short white ticks).
                for (unsigned k=0;k<3;++k) {
                    const auto parent=world*d.pose.globals[k ? node[f][k-1]:wristNode];
                    const auto& a=layer.anatomy(side);
                    glm::vec3 axis=f<4 ? a.flexion[f]:(k==0 ? a.thumbPalmar:k==1 ? a.thumbMcp:a.thumbIp);
                    const auto bindParent=b.profile().bodyFromResource*body.asset().nodes()[k ? node[f][k-1]:wristNode].globalBindTransform;
                    const auto local=glm::inverse(glm::mat3{bindParent})*axis;
                    const auto posed=glm::normalize(glm::mat3{parent}*local);
                    lines.push_back({point(node[f][k])-posed*.006f,point(node[f][k])+posed*.006f,{1,1,1}});
                }
            }
            const auto hand=worldHand(body,d,side);
            const auto origin=glm::vec3{hand[3]};
            for (int k=0;k<3;++k) { glm::vec3 color{.05f}; color[k]=1; lines.push_back({origin,origin+glm::vec3{hand[k]}*.04f,color}); }
            return lines;
        }

        // Linear path in anatomical joint angles.
        HandPose mixPose(const HandPose& a,const HandPose& b,float t)
        {
            HandPose p;
            const auto lerp=[t](float x,float y) { return x+(y-x)*t; };
            for (unsigned f=0;f<4;++f) p.fingers[f]={lerp(a.fingers[f].mcpFlexion,b.fingers[f].mcpFlexion),lerp(a.fingers[f].mcpSpread,b.fingers[f].mcpSpread),
                lerp(a.fingers[f].pipFlexion,b.fingers[f].pipFlexion),lerp(a.fingers[f].dipFlexion,b.fingers[f].dipFlexion)};
            p.thumb={lerp(a.thumb.palmarAbduction,b.thumb.palmarAbduction),lerp(a.thumb.radialAbduction,b.thumb.radialAbduction),
                lerp(a.thumb.pronation,b.thumb.pronation),lerp(a.thumb.mcpFlexion,b.thumb.mcpFlexion),lerp(a.thumb.ipFlexion,b.thumb.ipFlexion)};
            return p;
        }

        struct View { const char* name; glm::vec3 direction; };
        // Anatomical views; directions in (radial, distal, palmar) of each hand.
        constexpr View views[]{{"dorsal",{0,.08f,-1}},{"palmar",{0,.08f,1}},{"radial",{1,.08f,0}},{"ulnar",{-1,.08f,0}}};

        HandCamera camera(const BodyInstance& body,const BodyDisplay& d,unsigned side,glm::vec3 direction,float distance=.27f)
        {
            const auto hand=worldHand(body,d,side);
            const auto radial=glm::vec3{hand[0]}*(side==0 ? 1.f:-1.f), distal=glm::vec3{hand[1]}, palmar=-glm::vec3{hand[2]};
            HandCamera c;
            c.target=glm::vec3{hand*glm::vec4{0,.075f,0,1}};
            c.eye=c.target+glm::normalize(direction.x*radial+direction.y*distal+direction.z*palmar)*distance;
            c.up=distal;
            return c;
        }
    }

    void verifyDesktopHands(BodyInstance& body,const std::filesystem::path& output,const CaptureHandFrame& capture,
        const std::filesystem::path& extraPoses)
    {
        if (!body.control()) throw std::runtime_error("Hand verification requires calibrated Raphtalia");
        std::filesystem::create_directories(output);
        std::ofstream report(output/"hands.txt");
        report << std::setprecision(8) << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        body.enqueue(StartIdle{});
        body.setIdleView(IdleView{{0,0,1},true});
        for (unsigned t=0;t<400 && !(body.snapshot().idle.phase==IdlePhase::Idle && body.snapshot().tick>300);++t) body.step();
        if (body.snapshot().idle.phase!=IdlePhase::Idle) throw std::runtime_error("Idle was not reached");
        report << "idle_tick=" << body.snapshot().tick << '\n';
        const auto idle=body.display(1);
        const HandLayer layer(body.control()->binding());
        const auto shot=[&](const std::string& stem,const BodyDisplay& d,unsigned side,glm::vec3 direction,bool bones=false,float distance=.27f,bool avatar=false) {
            const auto c=camera(body,d,side,direction,distance);
            saveSceneFrame(output/(stem+".pam"),capture(d,avatar,&c,{}));
            if (bones) {
                const auto overlay=skeletonOverlay(body,layer,d,side);
                saveSceneFrame(output/(stem+"-bones.pam"),capture(d,false,&c,overlay));
            }
        };
        // Presentation arms for joint sweeps: hands in front of the body, clear of the skirt.
        auto presented=idle;
        {
            const auto& b=body.control()->binding();
            const auto set=[&](JointId id,glm::vec3 degrees) { presented.coordinates[b.jointIndex(id)]=glm::radians(degrees); };
            set(JointId::LeftShoulder,{50,15,0}); set(JointId::RightShoulder,{50,15,0});
            set(JointId::LeftElbow,{80,0,0}); set(JointId::RightElbow,{80,0,0});
            set(JointId::LeftWrist,{0,0,0}); set(JointId::RightWrist,{0,0,0});
            b.applyMeasuredCoordinates(presented.pose.locals,presented.coordinates);
        }
        const auto withPose=[&](const std::function<HandPose(unsigned)>& pose,const BodyDisplay& base) {
            auto d=base;
            for (unsigned side=0;side<2;++side) layer.applyPose(d.pose.locals,side,pose(side));
            refresh(body,d);
            return d;
        };
        // Static poses: application rest (Idle), modelled bind, open.
        const std::pair<const char*,BodyDisplay> poses[]{{"rest",idle},
            {"bind",withPose([&](unsigned side) { return layer.anatomy(side).bind; },idle)},
            {"open",withPose([](unsigned) { return openHandPose(); },idle)},
            {"front-rest",withPose([](unsigned) { return relaxedHandPose(); },presented)},
            {"front-open",withPose([](unsigned) { return openHandPose(); },presented)},
            {"front-bind",withPose([&](unsigned side) { return layer.anatomy(side).bind; },presented)}};
        for (const auto& [name,d]:poses) for (unsigned side=0;side<2;++side) for (const auto& v:views)
            shot(std::string{name}+"-"+sideNames[side]+"-"+v.name,d,side,v.direction,true);
        // The Desktop Avatar portrait frames the body from the waist up; these close-ups use its
        // offscreen surface and premultiplied composition with the hanging Idle hands in view.
        for (const auto& [name,d]:{std::pair{"avatar-rest",poses[0].second},{"avatar-open",poses[2].second}})
            for (unsigned side=0;side<2;++side) for (const auto& v:{views[0],views[2]})
                shot(std::string{name}+"-"+sideNames[side]+"-"+v.name,d,side,v.direction,false,.27f,true);
        if (!extraPoses.empty()) {
            std::ifstream in(extraPoses);
            if (!in) throw std::runtime_error("Cannot read hand pose file");
            std::string name;
            while (in >> name) {
                std::array<float,21> v{};
                for (auto& x:v) if (!(in >> x)) throw std::runtime_error("Malformed hand pose file");
                HandPose p;
                for (unsigned f=0;f<4;++f) p.fingers[f]={glm::radians(v[f*4]),glm::radians(v[f*4+1]),glm::radians(v[f*4+2]),glm::radians(v[f*4+3])};
                p.thumb={glm::radians(v[16]),glm::radians(v[17]),glm::radians(v[18]),glm::radians(v[19]),glm::radians(v[20])};
                const auto hanging=withPose([&](unsigned) { return p; },idle), front=withPose([&](unsigned) { return p; },presented);
                for (unsigned side=0;side<2;++side) for (const auto& v2:views) {
                    shot("pose-"+name+"-"+sideNames[side]+"-"+v2.name,hanging,side,v2.direction);
                    shot("pose-"+name+"-front-"+sideNames[side]+"-"+v2.name,front,side,v2.direction);
                }
            }
            report << "extra_poses=PASS\n";
            return;
        }
        // Isolated sweeps from the open pose; all four fingers bend the same joint, each in its own chain.
        const auto sweep=[&](const std::string& stem,const std::function<void(HandPose&,float)>& set,std::initializer_list<float> degrees,
            std::initializer_list<View> sweepViews) {
            for (const float deg:degrees) {
                const auto d=withPose([&](unsigned) { auto p=openHandPose(); set(p,glm::radians(deg)); return p; },presented);
                for (unsigned side=0;side<2;++side) for (const auto& v:sweepViews) {
                    std::ostringstream name; name << stem << '-' << sideNames[side] << '-' << v.name << '-' << std::showpos << static_cast<int>(deg);
                    shot(name.str(),d,side,v.direction,false,.25f);
                }
            }
        };
        const View side{"radial",{1,.08f,.15f}}, ulnar{"ulnar",{-1,.08f,.15f}}, palm{"palmar",{0,.08f,1}};
        sweep("bend-mcp",[](HandPose& p,float a) { for (auto& f:p.fingers) f.mcpFlexion=a; },{0,30,60,90},{ulnar,palm});
        sweep("bend-pip",[](HandPose& p,float a) { for (auto& f:p.fingers) f.pipFlexion=a; },{0,30,60,90},{ulnar,palm});
        sweep("bend-dip",[](HandPose& p,float a) { for (auto& f:p.fingers) f.dipFlexion=a; },{0,25,50,70},{ulnar,palm});
        sweep("thumb-palmar",[](HandPose& p,float a) { p.thumb.palmarAbduction=a; },{-10,10,30,50},{side,palm});
        sweep("thumb-radial",[](HandPose& p,float a) { p.thumb.radialAbduction=a; },{-10,10,30,45},{side,palm});
        sweep("thumb-pronation",[](HandPose& p,float a) { p.thumb.pronation=a; },{-30,0,25,50},{side,palm});
        sweep("thumb-mcp",[](HandPose& p,float a) { p.thumb.mcpFlexion=a; },{-10,15,35,55},{side,palm});
        sweep("thumb-ip",[](HandPose& p,float a) { p.thumb.ipFlexion=a; },{-15,15,45,75},{side,palm});
        // Opposition: open -> opposition onset along the anatomical angle path.
        for (const float t:{0.f,1.f/3,2.f/3,1.f}) {
            const auto d=withPose([&](unsigned) { return mixPose(openHandPose(),oppositionHandPose(),t); },presented);
            for (unsigned hand=0;hand<2;++hand) for (const auto& v:{side,palm,ulnar}) {
                std::ostringstream name; name << "opposition-" << sideNames[hand] << '-' << v.name << '-' << static_cast<int>(std::lround(t*100));
                shot(name.str(),d,hand,v.direction,false,.25f);
            }
        }
        // Application sequence: rest -> open -> rest through SetIdleHands, including intermediate display fractions.
        const auto start=body.snapshot().tick;
        body.enqueue(SetIdleHands{true});
        float maxSpeed=0;
        auto previous=body.snapshot().pose.locals;
        for (unsigned t=1;t<=300;++t) {
            if (t==151) body.enqueue(SetIdleHands{false});
            body.step();
            for (const auto& r:body.results()) if (r.status!=BodyCommandStatus::Applied) throw std::runtime_error("Idle hands command rejected: "+r.reason);
            const auto& s=body.snapshot();
            for (const auto& j:layer.joints()) {
                const auto qa=normalizedReachOrientation(glm::quat_cast(glm::mat3{previous[j.node]}));
                const auto qb=normalizedReachOrientation(glm::quat_cast(glm::mat3{s.pose.locals[j.node]}));
                maxSpeed=std::max(maxSpeed,static_cast<float>(glm::length(orientationResidual(qb,qa))*120));
            }
            previous=s.pose.locals;
            if (t%20==0 || t==1 || t==151) {
                const auto d=body.display(1);
                for (unsigned hand=0;hand<2;++hand) {
                    std::ostringstream name; name << "sequence-" << sideNames[hand] << "-t" << std::setw(3) << std::setfill('0') << t;
                    shot(name.str()+"-radial",d,hand,{1,.08f,.15f});
                    shot(name.str()+"-dorsal",d,hand,{0,.08f,-1});
                }
            }
            if (t==60 || t==210) {
                const auto d=body.display(.5);
                for (unsigned hand=0;hand<2;++hand) {
                    std::ostringstream name; name << "sequence-" << sideNames[hand] << "-t" << std::setw(3) << std::setfill('0') << t-1 << "h";
                    shot(name.str()+"-radial",d,hand,{1,.08f,.15f});
                    shot(name.str()+"-dorsal",d,hand,{0,.08f,-1});
                }
            }
        }
        const auto& s=body.snapshot();
        if (s.hands[0].enabled || s.hands[1].enabled || !s.hands[0].relaxed || !s.hands[1].relaxed)
            throw std::runtime_error("Hands did not return to the relaxed rest pose");
        report << "sequence_ticks=" << s.tick-start << "\nmax_finger_speed_rad_s=" << maxSpeed << '\n';
        // Existing grasp shapes for reference (unchanged calibration).
        for (const auto grip:{HandGrip::Sphere,HandGrip::Cube}) {
            auto d=idle;
            std::array<HandState,2> states{};
            for (auto& st:states) { st.enabled=true; st.blend=1; st.closure=1; st.grip=grip; st.relaxed=true; }
            layer.apply(d.pose.locals,states);
            refresh(body,d);
            for (unsigned hand=0;hand<2;++hand) for (const auto& v:{views[2],views[1]})
                shot(std::string{grip==HandGrip::Sphere ? "sphere":"cube"}+"-"+sideNames[hand]+"-"+v.name,d,hand,v.direction);
        }
        report << "result=PASS\n";
    }
}
