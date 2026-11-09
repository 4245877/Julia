#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "../common/CollisionDemo.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <glm/gtc/quaternion.hpp>

namespace
{
    template<class T> T number(std::string_view value)
    {
        T result{};
        const auto parsed=std::from_chars(value.data(),value.data()+value.size(),result);
        if (parsed.ec!=std::errc{} || parsed.ptr!=value.data()+value.size())
            throw std::invalid_argument("Invalid numeric argument");
        return result;
    }
}
int main(int argc,char** argv)
{
    using namespace julia;
    using namespace julia::embodiment;
    try
    {
        std::uint64_t ticks=1200;
        std::optional<std::size_t> clip;
        std::optional<float> elbow;
        std::optional<glm::vec3> reachPosition,retargetPosition;
        std::uint64_t retargetTick=0,cancelTick=0,action=0;
        ReachTarget reachTarget;
        int effectorOffset=0;
        RootPose root;
        bool eyes=true;
        std::optional<bool> collisionDemo;
        std::filesystem::path source=std::filesystem::path{JULIA_ASSET_DIR}/"models/Raphtalia_prepared.fbx", output;
        for (int i=1;i<argc;++i)
        {
            const std::string_view argument{argv[i]};
            if (argument=="--help")
            {
                std::cout << "julia_body_sim [--ticks N] [--clip INDEX] [--elbow RADIANS] [--no-eyes] [--model PATH] [--output CSV]\n"
                    << "  --reach X Y Z [--right] [--effector wrist|palm|grip] [--world] [--root X Y Z YAW_RADIANS]\n"
                    << "  [--retarget X Y Z --retarget-tick N] [--cancel-tick N] [--speed RAD_S] [--acceleration RAD_S2]\n"
                    << "  --collision-demo free|blocked (static CPU scene; default demo target)\n"
                    << "  --allow-detour [--detour-candidates N] [--planning-queries-per-tick N] [--planning-query-budget N]\n";
                return 0;
            }
            if (argument=="--allow-detour") { reachTarget.strategy=MovementStrategy::AllowDetour; continue; }
            if (argument=="--no-eyes") { eyes=false; continue; }
            if (argument=="--right") { reachTarget.arm=ArmSide::Right; continue; }
            if (argument=="--world") { reachTarget.space=TargetSpace::World; continue; }
            if (argument=="--reach" || argument=="--retarget" || argument=="--root")
            {
                const int count=argument=="--root" ? 4:3;
                if (i+count>=argc) throw std::invalid_argument("Position requires X Y Z; root also requires yaw radians");
                glm::vec3 position{0};
                for (int k=0;k<3;++k) position[k]=number<float>(argv[++i]);
                if (argument=="--reach") reachPosition=position;
                else if (argument=="--retarget") retargetPosition=position;
                else { root.position=position; root.rotation=glm::angleAxis(number<float>(argv[++i]),glm::vec3{0,1,0}); }
                continue;
            }
            if (i+1>=argc) throw std::invalid_argument("Missing option value");
            const std::string_view value{argv[++i]};
            if (argument=="--ticks") ticks=number<std::uint64_t>(value);
            else if (argument=="--clip") clip=number<std::size_t>(value);
            else if (argument=="--elbow") elbow=number<float>(value);
            else if (argument=="--model") source=std::filesystem::path{value};
            else if (argument=="--output") output=std::filesystem::path{value};
            else if (argument=="--retarget-tick") retargetTick=number<std::uint64_t>(value);
            else if (argument=="--cancel-tick") cancelTick=number<std::uint64_t>(value);
            else if (argument=="--collision-demo") {
                if (value!="free" && value!="blocked") throw std::invalid_argument("collision-demo requires free|blocked");
                collisionDemo=value=="blocked";
            }
            else if (argument=="--detour-candidates") reachTarget.detour.maxCandidates=number<unsigned>(value);
            else if (argument=="--planning-queries-per-tick") reachTarget.detour.queriesPerTick=number<unsigned>(value);
            else if (argument=="--planning-query-budget") reachTarget.detour.maxTotalQueries=number<unsigned>(value);
            else if (argument=="--speed") reachTarget.maxSpeed=number<float>(value);
            else if (argument=="--acceleration") reachTarget.maxAcceleration=number<float>(value);
            else if (argument=="--effector")
            {
                if (value=="wrist") effectorOffset=0; else if (value=="palm") effectorOffset=1;
                else if (value=="grip") effectorOffset=2; else throw std::invalid_argument("Unknown hand effector");
            }
            else throw std::invalid_argument("Unknown option");
        }
        const auto asset=model_import::ModelImporter::loadFromFile(source);
        const auto profile=raphtaliaBodyProfile();
        const bool calibrated=validateBodyProfile(profile,*asset).empty();
        BodyInstance body(1,asset,calibrated ? &profile:nullptr,eyes);
        if (collisionDemo) {
            if (!body.control()) throw std::invalid_argument("Collision demo requires Raphtalia calibration");
            const auto worldFromBody=glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation);
            const auto demo=app::collisionDemo(body.control()->binding(),*collisionDemo,1,worldFromBody,reachTarget.arm,static_cast<unsigned>(effectorOffset));
            body.enqueue(SetCollisionScene{demo.scene,{}});
            if (!reachPosition) reachPosition=reachTarget.space==TargetSpace::World ? glm::vec3{worldFromBody*glm::vec4{demo.target.position,1}}:demo.target.position;
        }
        if (clip)
        { body.enqueue(ClipCommand{ClipOperation::Select,static_cast<double>(*clip)}); body.enqueue(ClipCommand{ClipOperation::Play}); }
        if (elbow)
        { body.enqueue(SetDirectMode{true}); body.enqueue(SetCoordinate{JointId::LeftElbow,0,*elbow}); }
        if (retargetPosition.has_value()!=(retargetTick!=0) || ((retargetTick || cancelTick) && !reachPosition))
            throw std::invalid_argument("Retarget needs a position and tick; reach is required for retarget/cancel");
        if (reachPosition)
        {
            reachTarget.position=*reachPosition;
            reachTarget.effector=static_cast<EffectorId>((reachTarget.arm==ArmSide::Left ? 0:4)+effectorOffset);
            body.enqueue(SetDirectMode{true}); body.enqueue(SetRoot{root}); action=body.enqueue(StartReach{reachTarget}).id;
        }
        std::ofstream csv;
        if (!output.empty())
        {
            if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path());
            csv.open(output); if (!csv) throw std::runtime_error("Cannot open output CSV");
            csv << "tick,simulation_seconds,clip_seconds,left_elbow_radians,gaze_yaw_degrees,gaze_pitch_degrees,blink_left,blink_right,action,state,error_m,goal_x,goal_y,goal_z,reference_x,reference_y,reference_z,actual_x,actual_y,actual_z,reason,path_status,scene_version,body_form,other_body_form,object,from_tick,to_tick,gap_m,queries,check_us,strategy,planning_state,direct_status,candidates,total_queries,total_poses,planning_ticks,selected_candidate,route_segments,current_segment,segment0_status,segment1_status,segment0_ticks,segment1_ticks,waypoint0,waypoint1,waypoint2,waypoint3,waypoint4,waypoint5,planning_scratch_bytes,accepted_route_segments,accepted_current_segment,accepted_selected_candidate,route_segment0_status,route_segment1_status,route_segment0_scene_version,route_segment1_scene_version,route_segment0_start_tick,route_segment1_start_tick\n" << std::setprecision(17);
        }
        runtime::FixedStepScheduler scheduler;
        scheduler.runTicks(ticks,[&] {
            if (retargetTick==body.snapshot().tick+1) { reachTarget.position=*retargetPosition; body.enqueue(RetargetReach{action,reachTarget}); }
            if (cancelTick==body.snapshot().tick+1) body.enqueue(CancelReach{reachTarget.arm,action});
            body.step();
            for (const auto& result:body.results())
                if (result.status==BodyCommandStatus::Rejected) {
                    if (!collisionDemo) throw std::runtime_error("Command rejected: "+result.reason);
                    std::cout << "rejected_command=" << result.id << " reason=" << result.reason << '\n';
                }
            if (csv.is_open())
            {
                const auto& s=body.snapshot();
                const auto& r=s.reaches[reachTarget.arm==ArmSide::Left ? 0:1];
                csv << s.tick << ',' << s.time << ',' << s.clipTime << ','
                    << (body.control() ? body.control()->coordinate(JointId::LeftElbow,0):0) << ','
                    << s.eyes.gazeDegrees.x << ',' << s.eyes.gazeDegrees.y << ',' << s.eyes.blinkLeft << ',' << s.eyes.blinkRight << ','
                    << r.action << ',' << static_cast<int>(r.state) << ',' << r.error << ',' << r.goalBody.x << ',' << r.goalBody.y << ',' << r.goalBody.z << ','
                    << r.referenceBody.x << ',' << r.referenceBody.y << ',' << r.referenceBody.z << ',' << r.actualBody.x << ',' << r.actualBody.y << ',' << r.actualBody.z << ',' << reachReasonName(r.reason) << ',' << pathStatusName(s.pathCheck.status) << ',' << s.pathCheck.sceneVersion << ',' << s.pathCheck.bodyForm << ',' << s.pathCheck.otherBodyForm << ',' << s.pathCheck.object << ',' << s.pathCheck.fromTick << ',' << s.pathCheck.toTick << ',' << s.pathCheck.contact.gap << ',' << s.pathCheck.queries << ',' << s.pathCheck.microseconds << ',' << static_cast<int>(r.target.strategy) << ','
                    << static_cast<int>(r.planning.state) << ',' << pathStatusName(r.planning.directCheck.status) << ',' << r.planning.candidates << ','
                    << r.planning.queries << ',' << r.planning.poses << ',' << r.planning.ticks << ',' << r.planning.selectedCandidate << ','
                    << r.planning.segmentCount << ',' << r.planning.currentSegment << ',' << pathStatusName(r.planning.segmentChecks[0].status) << ','
                    << pathStatusName(r.planning.segmentChecks[1].status) << ',' << r.planning.segmentDurations[0] << ',' << r.planning.segmentDurations[1];
                for (const auto q:r.planning.waypoint) csv << ',' << q;
                csv << ',' << r.planning.scratchBytes << ',' << r.route.segmentCount << ',' << r.route.currentSegment << ',' << r.route.selectedCandidate << ','
                    << pathStatusName(r.route.segmentChecks[0].status) << ',' << pathStatusName(r.route.segmentChecks[1].status) << ','
                    << r.route.segmentChecks[0].sceneVersion << ',' << r.route.segmentChecks[1].sceneVersion << ','
                    << r.route.segmentChecks[0].segmentStartTick << ',' << r.route.segmentChecks[1].segmentStartTick << '\n';
            }
        });
        if (csv.is_open()) { csv.flush(); if (!csv) throw std::runtime_error("Cannot write output CSV"); }
        std::cout << "completed_ticks=" << body.snapshot().tick << " simulation_seconds=" << std::setprecision(17)
            << body.snapshot().time << " discarded_seconds=" << scheduler.diagnostics().discardedSeconds << '\n';
        if (reachPosition)
        {
            const auto& r=body.snapshot().reaches[reachTarget.arm==ArmSide::Left ? 0:1];
            std::cout << "action=" << r.action << " state=" << static_cast<int>(r.state) << " error_m=" << r.error << " reason=" << reachReasonName(r.reason) << '\n';
            std::cout << "planning=" << planningStateName(r.planning.state) << " direct=" << pathStatusName(r.planning.directCheck.status)
                << " candidates=" << r.planning.candidates << " queries=" << r.planning.queries << " planning_ticks=" << r.planning.ticks
                << " segments=" << r.route.segmentCount << " current_segment=" << r.route.currentSegment
                << " selected=" << r.route.selectedCandidate << " scratch_bytes=" << r.planning.scratchBytes << '\n';
        }
        if (collisionDemo) { const auto& p=body.snapshot().pathCheck;
            std::cout << "path=" << pathStatusName(p.status) << " scene=" << p.sceneVersion << " forms=" << p.bodyForm << ',' << p.otherBodyForm << " object=" << p.object
                << " interval=" << p.fromTick << ',' << p.toTick << " gap=" << p.contact.gap << " queries=" << p.queries << " check_us=" << p.microseconds << '\n'; }
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
