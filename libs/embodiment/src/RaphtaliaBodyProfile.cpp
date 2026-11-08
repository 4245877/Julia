#include "julia/embodiment/BodyProfile.hpp"
#include <glm/gtc/matrix_transform.hpp>

namespace julia::embodiment
{
    BodyProfile raphtaliaBodyProfile()
    {
        BodyProfile p;
        p.name="Raphtalia_prepared/body-v1";
        p.rigFingerprint=8613155383294238335ull; // Exact CPU-imported calibration, see stage3 report.
        // Resource +X is character left, +Y up, +Z front. 0.4 m/resource unit.
        // Origin: projection of root.x_01 onto the calibrated lowest shoe plane.
        p.bodyFromResource=glm::scale(glm::mat4{1},glm::vec3{.4f})*
            glm::translate(glm::mat4{1},glm::vec3{0,.00904308166f,-.0065985275f});
        const auto point=[&](glm::vec3 resource) { return glm::vec3{p.bodyFromResource*glm::vec4{resource,1}}; };
        const auto frame=[&](glm::vec3 resource) { return glm::translate(glm::mat4{1},point(resource)); };
        const auto dof=[](const char* name,glm::vec3 axis,float low,float high,const char* positive)
        { return JointCoordinate{name,positive,axis,glm::radians(low),glm::radians(high)}; };
        const auto add=[&](JointId id,SegmentId segment,const char* name,NodeLink node,
            glm::mat4 f,NodeLink end,std::vector<JointCoordinate> coordinates)
        { p.joints.push_back({id,segment,name,std::move(node),f,std::move(coordinates),std::move(end)}); };
        const auto torso=[&](float flex,float bend,float yaw)
        { return std::vector<JointCoordinate>{dof("flexion",{1,0,0},-flex,flex,"top towards body +Z"),
            dof("left_bend",{0,0,-1},-bend,bend,"top towards body +X"),
            dof("left_turn",{0,1,0},-yaw,yaw,"front towards body +X")}; };
        add(JointId::Pelvis,SegmentId::Pelvis,"Pelvis",{"root.x_01","Skeleton_4_rootJoint"},frame({0.0000000000f,2.1320497990f,0.0065985275f}),{"spine_01.x_014","root.x_01"},torso(15,10,15));
        add(JointId::Lumbar,SegmentId::Lumbar,"Lumbar",{"spine_01.x_014","root.x_01"},frame({-0.0000000004f,2.4625689983f,0.0005658921f}),{"spine_02.x_015","spine_01.x_014"},torso(15,10,15));
        add(JointId::Thorax,SegmentId::Thorax,"Thorax",{"spine_02.x_015","spine_01.x_014"},frame({0.0000000018f,2.7930841446f,-0.0054655322f}),{"spine_03.x_016","spine_02.x_015"},torso(15,10,15));
        add(JointId::Chest,SegmentId::Chest,"Chest",{"spine_03.x_016","spine_02.x_015"},frame({0.0000000039f,3.1235990524f,-0.0114969527f}),{"neck.x_017","spine_03.x_016"},torso(15,10,15));
        add(JointId::Neck,SegmentId::Neck,"Neck",{"neck.x_017","spine_03.x_016"},frame({-0.0000000014f,3.4541177750f,-0.0175284520f}),{"head.x_018","neck.x_017"},torso(15,15,25));
        add(JointId::Head,SegmentId::Head,"Head",{"head.x_018","neck.x_017"},frame({-0.0000000048f,3.5992615223f,-0.0081821941f}),{"eye.L","head.x_018"},torso(20,15,35));
        {
            constexpr float side=1.0f;
            const glm::vec3 shoulder={0.2702773213f,3.3219347000f,-0.0016252846f}, elbow={0.3783898950f,2.6803998947f,0.1168715283f}, wrist={0.4983181059f,2.0953183174f,0.2614150047f};
            add(JointId::LeftClavicle,SegmentId::LeftClavicle,"LeftClavicle",{"shoulder.l_0157","spine_03.x_016"},frame({0.0675695240f,3.3219349384f,0.0550715551f}),{"arm_stretch.l_0158","shoulder.l_0157"},
                {dof("elevation",{0,0,side},-10,15,"shoulder moves up from bind")});
            add(JointId::LeftShoulder,SegmentId::LeftUpperArm,"LeftShoulder",{"arm_stretch.l_0158","shoulder.l_0157"},frame(shoulder),{"forearm_stretch.l_0159","arm_stretch.l_0158"},
                {dof("flexion",{-1,0,0},-20,65,"arm towards body +Z"),
                  dof("abduction",{0,0,side},-10,45,"arm away from torso"),
                  dof("axial_roll",glm::normalize(elbow-shoulder),-30,30,"right-hand rotation along upper arm")});
            auto elbowFrame=frame(elbow);
            const auto y=glm::normalize(wrist-elbow), x=glm::normalize(glm::cross(y,glm::vec3{0,0,1}));
            elbowFrame[0]=glm::vec4{x,0}; elbowFrame[1]=glm::vec4{y,0}; elbowFrame[2]=glm::vec4{glm::cross(x,y),0};
            add(JointId::LeftElbow,SegmentId::LeftForearm,"LeftElbow",{"forearm_stretch.l_0159","arm_stretch.l_0158"},elbowFrame,{"hand.l_0160","forearm_stretch.l_0159"},
                {dof("flexion",{1,0,0},-5,100,"wrist towards projected body +Z")});
            auto handFrame=frame(wrist);
            const auto distal=glm::normalize(glm::vec3{0.5904711485f,1.9114799499f,0.3594261408f}-wrist);
            const auto across=glm::vec3{0.5705143213f,1.9227365255f,0.3914998770f}-glm::vec3{0.6101165414f,1.9113278389f,0.2895886898f};
            const auto palm=side*glm::normalize(glm::cross(across,distal));
            handFrame[0]=glm::vec4{glm::cross(distal,palm),0};
            handFrame[1]=glm::vec4{distal,0}; handFrame[2]=glm::vec4{palm,0};
            add(JointId::LeftWrist,SegmentId::LeftHand,"LeftWrist",{"hand.l_0160","forearm_stretch.l_0159"},handFrame,{"c_middle1.l_0170","hand.l_0160"},
                {dof("palm_flexion",{1,0,0},-25,30,"fingers towards calibrated palm +Z"),
                  dof("radial_deviation",{0,0,-side},-15,15,"fingers towards thumb side")});
            p.followers.push_back({JointId::LeftShoulder,{"arm_twist.l_0177","arm_stretch.l_0158"},{1,1,.5f}});
            p.followers.push_back({JointId::LeftElbow,{"forearm_twist.l_0176","forearm_stretch.l_0159"},{1,1,1}});
            p.effectors.push_back({EffectorId::LeftWrist,"LeftWrist",{"hand.l_0160","forearm_stretch.l_0159"},handFrame});
            auto palmFrame=handFrame;
            palmFrame[3]=glm::vec4{point(wrist+(glm::vec3{0.5904711485f,1.9114799499f,0.3594261408f}-wrist)*.55f),1};
            p.effectors.push_back({EffectorId::LeftPalm,"LeftPalm",{"hand.l_0160","forearm_stretch.l_0159"},palmFrame});
            auto gripFrame=palmFrame;
            gripFrame[3]+=glm::vec4{palm*.025f,0}; // Engineering grasp centre, 25 mm off palm.
            p.effectors.push_back({EffectorId::LeftGrip,"LeftGrip",{"hand.l_0160","forearm_stretch.l_0159"},gripFrame});
        }
        {
            constexpr float side=-1.0f;
            const glm::vec3 shoulder={-0.2702773809f,3.3219344616f,-0.0016253926f}, elbow={-0.4960764050f,2.7364754677f,-0.0946736634f}, wrist={-0.6970762014f,2.1600625515f,-0.1555995196f};
            add(JointId::RightClavicle,SegmentId::RightClavicle,"RightClavicle",{"shoulder.r_0136","spine_03.x_016"},frame({-0.0675695688f,3.3219354153f,0.0550717115f}),{"arm_stretch.r_0137","shoulder.r_0136"},
                {dof("elevation",{0,0,side},-10,15,"shoulder moves up from bind")});
            add(JointId::RightShoulder,SegmentId::RightUpperArm,"RightShoulder",{"arm_stretch.r_0137","shoulder.r_0136"},frame(shoulder),{"forearm_stretch.r_0138","arm_stretch.r_0137"},
                {dof("flexion",{-1,0,0},-20,65,"arm towards body +Z"),
                  dof("abduction",{0,0,side},-10,45,"arm away from torso"),
                  dof("axial_roll",glm::normalize(elbow-shoulder),-30,30,"right-hand rotation along upper arm")});
            auto elbowFrame=frame(elbow);
            const auto y=glm::normalize(wrist-elbow), x=glm::normalize(glm::cross(y,glm::vec3{0,0,1}));
            elbowFrame[0]=glm::vec4{x,0}; elbowFrame[1]=glm::vec4{y,0}; elbowFrame[2]=glm::vec4{glm::cross(x,y),0};
            add(JointId::RightElbow,SegmentId::RightForearm,"RightElbow",{"forearm_stretch.r_0138","arm_stretch.r_0137"},elbowFrame,{"hand.r_0139","forearm_stretch.r_0138"},
                {dof("flexion",{1,0,0},-5,100,"wrist towards projected body +Z")});
            auto handFrame=frame(wrist);
            const auto distal=glm::normalize(glm::vec3{-0.8173342943f,1.9688788652f,-0.1692916304f}-wrist);
            const auto across=glm::vec3{-0.7982736826f,1.9573199749f,-0.1367871165f}-glm::vec3{-0.8321745992f,2.0105659962f,-0.2267904282f};
            const auto palm=side*glm::normalize(glm::cross(across,distal));
            handFrame[0]=glm::vec4{glm::cross(distal,palm),0};
            handFrame[1]=glm::vec4{distal,0}; handFrame[2]=glm::vec4{palm,0};
            add(JointId::RightWrist,SegmentId::RightHand,"RightWrist",{"hand.r_0139","forearm_stretch.r_0138"},handFrame,{"c_middle1.r_0149","hand.r_0139"},
                {dof("palm_flexion",{1,0,0},-25,30,"fingers towards calibrated palm +Z"),
                  dof("radial_deviation",{0,0,-side},-15,15,"fingers towards thumb side")});
            p.followers.push_back({JointId::RightShoulder,{"arm_twist.r_0156","arm_stretch.r_0137"},{1,1,.5f}});
            p.followers.push_back({JointId::RightElbow,{"forearm_twist.r_0135","forearm_stretch.r_0138"},{1,1,1}});
            p.effectors.push_back({EffectorId::RightWrist,"RightWrist",{"hand.r_0139","forearm_stretch.r_0138"},handFrame});
            auto palmFrame=handFrame;
            palmFrame[3]=glm::vec4{point(wrist+(glm::vec3{-0.8173342943f,1.9688788652f,-0.1692916304f}-wrist)*.55f),1};
            p.effectors.push_back({EffectorId::RightPalm,"RightPalm",{"hand.r_0139","forearm_stretch.r_0138"},palmFrame});
            auto gripFrame=palmFrame;
            gripFrame[3]+=glm::vec4{palm*.025f,0}; // Engineering grasp centre, 25 mm off palm.
            p.effectors.push_back({EffectorId::RightGrip,"RightGrip",{"hand.r_0139","forearm_stretch.r_0138"},gripFrame});
        }
        {
            constexpr float side=1.0f;
            const glm::vec3 hip={0.1064245775f,2.1166470051f,0.0048588580f}, knee={0.2004249096f,1.1559050083f,-0.1047509983f}, ankle={0.2471179217f,0.3453735113f,-0.6124744415f}, toe={0.3077883720f,0.0286859870f,-0.4703016877f};
            add(JointId::LeftHip,SegmentId::LeftThigh,"LeftHip",{"thigh_stretch.l_02","root.x_01"},frame(hip),{"leg_stretch.l_03","thigh_stretch.l_02"},
                {dof("flexion",{-1,0,0},-15,50,"thigh towards body +Z"),
                  dof("abduction",{0,0,side},-10,30,"thigh away from midline"),
                  dof("axial_roll",glm::normalize(knee-hip),-20,20,"right-hand rotation along thigh")});
            auto kneeFrame=frame(knee);
            const auto y=glm::normalize(ankle-knee), x=glm::normalize(glm::cross(glm::vec3{0,0,1},y));
            kneeFrame[0]=glm::vec4{x,0}; kneeFrame[1]=glm::vec4{y,0}; kneeFrame[2]=glm::vec4{glm::cross(x,y),0};
            add(JointId::LeftKnee,SegmentId::LeftShank,"LeftKnee",{"leg_stretch.l_03","thigh_stretch.l_02"},kneeFrame,{"foot.l_04","leg_stretch.l_03"},
                {dof("flexion",{1,0,0},-5,80,"ankle towards projected body -Z")});
            auto ankleFrame=frame(ankle);
            const auto forward=glm::normalize(glm::vec3{toe.x-ankle.x,0,toe.z-ankle.z});
            ankleFrame[0]=glm::vec4{glm::cross(glm::vec3{0,1,0},forward),0};
            ankleFrame[2]=glm::vec4{forward,0};
            add(JointId::LeftAnkle,SegmentId::LeftFoot,"LeftAnkle",{"foot.l_04","leg_stretch.l_03"},ankleFrame,{"toes_01.l_05","foot.l_04"},
                {dof("dorsiflexion",{-1,0,0},-20,25,"toes rise from bind"),
                  dof("inversion",{0,0,-side},-10,10,"inner sole edge rises")});
            p.followers.push_back({JointId::LeftHip,{"thigh_twist.l_07","thigh_stretch.l_02"},{1,1,.5f}});
            p.followers.push_back({JointId::LeftKnee,{"leg_twist.l_06","leg_stretch.l_03"},{1,1,1}});
            // Mean of the audited low shoe-surface samples (within 0.015 resource units).
            // Provisional sole reference, ~3 mm above the lowest vertex, not a solved contact.
            auto sole=frame(side>0 ? glm::vec3{.322252035f,.007377806f,-.460035741f}:
                glm::vec3{-.334952459f,.000303040f,.588397652f});
            sole[0]=glm::vec4{glm::cross(glm::vec3{0,1,0},forward),0};
            sole[1]=glm::vec4{0,1,0,0}; sole[2]=glm::vec4{forward,0};
            p.effectors.push_back({EffectorId::LeftSole,"LeftSole",{"foot.l_04","leg_stretch.l_03"},sole});
        }
        {
            constexpr float side=-1.0f;
            const glm::vec3 hip={-0.2760468125f,2.1474530697f,0.0087755714f}, knee={-0.3113237619f,1.1961774826f,0.1946547925f}, ankle={-0.2674751580f,0.2523869276f,0.3045876622f}, toe={-0.3128719330f,0.0237026811f,0.5809189081f};
            add(JointId::RightHip,SegmentId::RightThigh,"RightHip",{"thigh_stretch.r_08","root.x_01"},frame(hip),{"leg_stretch.r_09","thigh_stretch.r_08"},
                {dof("flexion",{-1,0,0},-15,50,"thigh towards body +Z"),
                  dof("abduction",{0,0,side},-10,30,"thigh away from midline"),
                  dof("axial_roll",glm::normalize(knee-hip),-20,20,"right-hand rotation along thigh")});
            auto kneeFrame=frame(knee);
            const auto y=glm::normalize(ankle-knee), x=glm::normalize(glm::cross(glm::vec3{0,0,1},y));
            kneeFrame[0]=glm::vec4{x,0}; kneeFrame[1]=glm::vec4{y,0}; kneeFrame[2]=glm::vec4{glm::cross(x,y),0};
            add(JointId::RightKnee,SegmentId::RightShank,"RightKnee",{"leg_stretch.r_09","thigh_stretch.r_08"},kneeFrame,{"foot.r_010","leg_stretch.r_09"},
                {dof("flexion",{1,0,0},-5,80,"ankle towards projected body -Z")});
            auto ankleFrame=frame(ankle);
            const auto forward=glm::normalize(glm::vec3{toe.x-ankle.x,0,toe.z-ankle.z});
            ankleFrame[0]=glm::vec4{glm::cross(glm::vec3{0,1,0},forward),0};
            ankleFrame[2]=glm::vec4{forward,0};
            add(JointId::RightAnkle,SegmentId::RightFoot,"RightAnkle",{"foot.r_010","leg_stretch.r_09"},ankleFrame,{"toes_01.r_011","foot.r_010"},
                {dof("dorsiflexion",{-1,0,0},-20,25,"toes rise from bind"),
                  dof("inversion",{0,0,-side},-10,10,"inner sole edge rises")});
            p.followers.push_back({JointId::RightHip,{"thigh_twist.r_013","thigh_stretch.r_08"},{1,1,.5f}});
            p.followers.push_back({JointId::RightKnee,{"leg_twist.r_012","leg_stretch.r_09"},{1,1,1}});
            // Mean of the audited low shoe-surface samples (within 0.015 resource units).
            // Provisional sole reference, ~3 mm above the lowest vertex, not a solved contact.
            auto sole=frame(side>0 ? glm::vec3{.322252035f,.007377806f,-.460035741f}:
                glm::vec3{-.334952459f,.000303040f,.588397652f});
            sole[0]=glm::vec4{glm::cross(glm::vec3{0,1,0},forward),0};
            sole[1]=glm::vec4{0,1,0,0}; sole[2]=glm::vec4{forward,0};
            p.effectors.push_back({EffectorId::RightSole,"RightSole",{"foot.r_010","leg_stretch.r_09"},sole});
        }
        return p;
    }
}
