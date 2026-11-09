#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "../common/CollisionDemo.hpp"
#include "../common/SpatialDemo.hpp"
#include "../common/InteractionDemo.hpp"
#include "../common/SupportDemo.hpp"
#include "../common/StanceReachDemo.hpp"
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
        std::optional<glm::quat> retargetOrientation;
        std::optional<unsigned> spatialSample;
        std::uint64_t retargetTick=0,cancelTick=0,action=0;
        ReachTarget reachTarget;
        int effectorOffset=0;
        RootPose root;
        bool eyes=true, multiDemo=false, interactionDemo=false, supportDemo=false, stanceDemo=false, explicitTicks=false;
        std::optional<bool> collisionDemo;
        std::filesystem::path source=std::filesystem::path{JULIA_ASSET_DIR}/"models/Raphtalia_prepared.fbx", output;
        for (int i=1;i<argc;++i)
        {
            const std::string_view argument{argv[i]};
            if (argument=="--help")
            {
                std::cout << "julia_body_sim [--ticks N] [--clip INDEX] [--elbow RADIANS] [--no-eyes] [--model PATH] [--output CSV]\n"
                    << "  --support-demo (explicit alignment, four small root moves; default 12000 ticks)\n"
                    << "  --stance-reach-demo (align, Reach, Retarget/Cancel, root move, new Reach; 12000 ticks)\n"
                    << "  --interaction-demo (one sphere; grasp, move, release; --right optional)\n  --reach X Y Z [--right] [--effector wrist|palm|grip] [--world] [--root X Y Z YAW_RADIANS]\n"
                    << "  --orientation W X Y Z (PositionAndOrientation); --position-only; --spatial-demo 0|1|2\n"
                    << "  --tolerance METRES --angular-tolerance RADIANS; --retarget-orientation W X Y Z\n"
                    << "  [--retarget X Y Z --retarget-tick N] [--cancel-tick N] [--speed RAD_S] [--acceleration RAD_S2]\n"
                    << "  --collision-demo free|blocked|multik (static CPU scene; default demo target)\n"
                    << "  --allow-detour [--ik-configurations 1..4] [--detour-candidates N]\n"
                    << "  --ranked --elbow-direction X Y Z [--elbow-world] --ranking-extra-ticks N --ranking-evaluations N\n"
                    << "  --elbow-weight W --travel-weight W --duration-weight W (finite nonnegative weights)\n"
                    << "  --ik-iterations-per-tick N --ik-iteration-budget N --ik-seed-budget N\n"
                    << "  --planning-poses-per-tick N --planning-pose-budget N --planning-queries-per-tick N --planning-query-budget N\n";
                return 0;
            }
            if (argument=="--support-demo") { supportDemo=true; continue; }
            if (argument=="--stance-reach-demo") { stanceDemo=true; continue; }
            if (argument=="--interaction-demo") { interactionDemo=true; continue; }
            if (argument=="--allow-detour") { reachTarget.strategy=MovementStrategy::AllowDetour; continue; }
            if (argument=="--position-only") { reachTarget.task=ReachTask::PositionOnly; continue; }
            if (argument=="--orientation" || argument=="--retarget-orientation") {
                if (i+4>=argc) throw std::invalid_argument("Orientation requires W X Y Z");
                const float w=number<float>(argv[++i]),x=number<float>(argv[++i]),y=number<float>(argv[++i]),z=number<float>(argv[++i]);
                if (argument=="--orientation") { reachTarget.orientation={w,x,y,z}; reachTarget.task=ReachTask::PositionAndOrientation; }
                else retargetOrientation=glm::quat{w,x,y,z};
                continue;
            }
            if (argument=="--ranked") { reachTarget.selection=RouteSelection::Ranked; continue; }
            if (argument=="--elbow-world") {
                if (!reachTarget.elbowPreference) reachTarget.elbowPreference=ElbowPreference{};
                reachTarget.elbowPreference->space=TargetSpace::World; continue;
            }
            if (argument=="--no-eyes") { eyes=false; continue; }
            if (argument=="--right") { reachTarget.arm=ArmSide::Right; continue; }
            if (argument=="--world") { reachTarget.space=TargetSpace::World; continue; }
            if (argument=="--reach" || argument=="--retarget" || argument=="--root" || argument=="--elbow-direction")
            {
                const int count=argument=="--root" ? 4:3;
                if (i+count>=argc) throw std::invalid_argument("Position requires X Y Z; root also requires yaw radians");
                glm::vec3 position{0};
                for (int k=0;k<3;++k) position[k]=number<float>(argv[++i]);
                if (argument=="--reach") reachPosition=position;
                else if (argument=="--retarget") retargetPosition=position;
                else if (argument=="--elbow-direction") {
                    if (!reachTarget.elbowPreference) reachTarget.elbowPreference=ElbowPreference{};
                    reachTarget.elbowPreference->direction=position;
                }
                else { root.position=position; root.rotation=glm::angleAxis(number<float>(argv[++i]),glm::vec3{0,1,0}); }
                continue;
            }
            if (i+1>=argc) throw std::invalid_argument("Missing option value");
            const std::string_view value{argv[++i]};
            if (argument=="--ticks") { ticks=number<std::uint64_t>(value); explicitTicks=true; }
            else if (argument=="--spatial-demo") spatialSample=number<unsigned>(value);
            else if (argument=="--tolerance") reachTarget.tolerance=number<float>(value);
            else if (argument=="--angular-tolerance") reachTarget.angularTolerance=number<float>(value);
            else if (argument=="--clip") clip=number<std::size_t>(value);
            else if (argument=="--elbow") elbow=number<float>(value);
            else if (argument=="--model") source=std::filesystem::path{value};
            else if (argument=="--output") output=std::filesystem::path{value};
            else if (argument=="--retarget-tick") retargetTick=number<std::uint64_t>(value);
            else if (argument=="--cancel-tick") cancelTick=number<std::uint64_t>(value);
            else if (argument=="--collision-demo") {
                if (value!="free" && value!="blocked" && value!="multik") throw std::invalid_argument("collision-demo requires free|blocked|multik");
                collisionDemo=value=="blocked"; multiDemo=value=="multik";
            }
            else if (argument=="--ik-configurations") reachTarget.detour.maxConfigurations=number<unsigned>(value);
            else if (argument=="--ranking-extra-ticks") reachTarget.ranking.additionalTicks=number<unsigned>(value);
            else if (argument=="--ranking-evaluations") reachTarget.ranking.maxEvaluations=number<unsigned>(value);
            else if (argument=="--elbow-weight") reachTarget.ranking.elbowWeight=number<double>(value);
            else if (argument=="--travel-weight") reachTarget.ranking.travelWeight=number<double>(value);
            else if (argument=="--duration-weight") reachTarget.ranking.durationWeight=number<double>(value);
            else if (argument=="--ik-iterations-per-tick") reachTarget.detour.iterationsPerTick=number<unsigned>(value);
            else if (argument=="--ik-iteration-budget") reachTarget.detour.maxIkIterations=number<unsigned>(value);
            else if (argument=="--ik-seed-budget") reachTarget.detour.maxSeedAttempts=number<unsigned>(value);
            else if (argument=="--planning-pose-budget") reachTarget.detour.maxTotalPoses=number<unsigned>(value);
            else if (argument=="--planning-poses-per-tick") reachTarget.detour.posesPerTick=number<unsigned>(value);
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
        if (stanceDemo) {
            if (!calibrated) throw std::invalid_argument("Stance Reach demo requires Raphtalia");
            if (!explicitTicks) ticks=12000;
            body.enqueue(SetRoot{root}); body.enqueue(SetDirectMode{true}); body.step();
            app::StanceReachDemo demo; demo.arm=reachTarget.arm; demo.start(body);
            std::ofstream trace;
            if (!output.empty()) { if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path()); trace.open(output); }
            for (std::uint64_t t=0;t<ticks;++t) {
                demo.drive(body); body.step(); if (trace) app::stanceReachTrace(trace,body.snapshot());
                for (const auto& r:body.results()) if (r.status==BodyCommandStatus::Rejected) throw std::runtime_error(r.reason);
            }
            std::cout << "stance_reach_stage=" << demo.stage << " support=" << supportPhaseName(body.snapshot().support.phase) << '\n';
            return demo.done() ? 0:1;
        }
        if (supportDemo) {
            if (!calibrated) throw std::invalid_argument("Support demo requires Raphtalia");
            if (!explicitTicks) ticks=12000;
            body.enqueue(SetRoot{root}); body.enqueue(SetDirectMode{true}); body.step();
            app::SupportDemo demo; demo.start(body);
            std::ofstream trace;
            if (!output.empty()) { if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path()); trace.open(output); if (!trace) throw std::runtime_error("Cannot open support trace"); }
            for (std::uint64_t t=0;t<ticks;++t) {
                demo.drive(body); body.step(); if (trace) app::supportTrace(trace,body.snapshot());
                for (const auto& r:body.results()) if (r.status==BodyCommandStatus::Rejected) throw std::runtime_error(r.reason);
                if (body.snapshot().support.result=="PlanRejected") throw std::runtime_error(body.snapshot().support.reason);
            }
            const auto& o=body.snapshot().support;
            std::cout << "support=" << supportPhaseName(o.phase) << " stage=" << demo.stage << " result=" << o.result << " reason=" << o.reason << "\n";
            for (unsigned s=0;s<2;++s) std::cout << "foot=" << s << " p=" << o.anchors[s].positionError << " angle=" << o.anchors[s].angularError << " gap=" << o.anchors[s].minGap << ':' << o.anchors[s].maxGap << "\n";
            return demo.done(body.snapshot()) ? 0:1;
        }
        if (interactionDemo) {
            if (!calibrated) throw std::invalid_argument("Interaction demo requires Raphtalia");
            app::InteractionDemo demo; demo.arm=reachTarget.arm; demo.root=root; demo.start(body);
            std::ofstream trace;
            if (!output.empty()) { if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path()); trace.open(output); if (!trace) throw std::runtime_error("Cannot open interaction trace"); }
            for (std::uint64_t t=0;t<ticks;++t) {
                demo.drive(body); body.step();
                if (trace) app::interactionTrace(trace,body.snapshot());
                for (const auto& r:body.results()) if (r.status==BodyCommandStatus::Rejected) throw std::runtime_error(r.reason);
            }
            const auto& i=body.snapshot().interaction;
            std::cout << "interaction=" << interactionPhaseName(i.phase) << " result=" << i.result << " reason=" << i.reason
                << " hold=" << (i.hold==HoldType::AssistedAttachment ? "AssistedAttachment":"None") << "\n";
            return i.phase==InteractionPhase::Succeeded ? 0:1;
        }
        if (collisionDemo) {
            if (!body.control()) throw std::invalid_argument("Collision demo requires Raphtalia calibration");
            const auto worldFromBody=glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation);
            const auto demo=multiDemo ? app::multiConfigurationDemo(body.control()->binding(),1,worldFromBody,reachTarget.arm,static_cast<unsigned>(effectorOffset)):
                app::collisionDemo(body.control()->binding(),*collisionDemo,1,worldFromBody,reachTarget.arm,static_cast<unsigned>(effectorOffset));
            body.enqueue(SetCollisionScene{demo.scene,{}});
            if (!reachPosition) reachPosition=reachTarget.space==TargetSpace::World ? glm::vec3{worldFromBody*glm::vec4{demo.target.position,1}}:demo.target.position;
        }
        if (spatialSample) {
            if (!body.control()) throw std::invalid_argument("Spatial demo requires calibrated body");
            const auto demo=app::spatialDemo(body.control()->binding(),*spatialSample,reachTarget.arm,static_cast<unsigned>(effectorOffset));
            const auto world=glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation);
            reachPosition=reachTarget.space==TargetSpace::World ? glm::vec3{world*glm::vec4{demo.position,1}}:demo.position;
            reachTarget.orientation=reachTarget.space==TargetSpace::World ? root.rotation*demo.orientation:demo.orientation;
            reachTarget.task=ReachTask::PositionAndOrientation;
        }
        if (clip)
        { body.enqueue(ClipCommand{ClipOperation::Select,static_cast<double>(*clip)}); body.enqueue(ClipCommand{ClipOperation::Play}); }
        if (elbow)
        { body.enqueue(SetDirectMode{true}); body.enqueue(SetCoordinate{JointId::LeftElbow,0,*elbow}); }
        if ((retargetPosition.has_value() || retargetOrientation.has_value())!=(retargetTick!=0) || ((retargetTick || cancelTick) && !reachPosition))
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
            csv << "tick,simulation_seconds,clip_seconds,left_elbow_radians,gaze_yaw_degrees,gaze_pitch_degrees,blink_left,blink_right,action,state,error_m,goal_x,goal_y,goal_z,reference_x,reference_y,reference_z,actual_x,actual_y,actual_z,reason,path_status,scene_version,body_form,other_body_form,object,from_tick,to_tick,gap_m,queries,check_us,strategy,planning_state,direct_status,candidates,total_queries,total_poses,planning_ticks,selected_candidate,route_segments,current_segment,segment0_status,segment1_status,segment0_ticks,segment1_ticks,waypoint0,waypoint1,waypoint2,waypoint3,waypoint4,waypoint5,planning_scratch_bytes,accepted_route_segments,accepted_current_segment,accepted_selected_candidate,route_segment0_status,route_segment1_status,route_segment0_scene_version,route_segment1_scene_version,route_segment0_start_tick,route_segment1_start_tick,planning_stage,ik_seeds,ik_processed,ik_unique,ik_duplicates,ik_failed,ik_iterations,ik_fk_checks,current_configuration,selected_configuration,tick_iterations,tick_seeds,tick_poses,tick_queries,tick_candidates,budget_reason,final0,final1,final2,final3,final4,final5,selection,selection_completion,score_evaluations,tick_score_evaluations,first_clear_tick,elbow_penalty,joint_travel,duration_s,score,score_key,elbow_status,elbow_x,elbow_y,elbow_z,score_us,elbow_direction_x,elbow_direction_y,elbow_direction_z,actual_elbow_penalty,actual_elbow_status,task,angular_error_rad,angular_tolerance_rad,goal_qw,goal_qx,goal_qy,goal_qz,actual_qw,actual_qx,actual_qy,actual_qz,world_qw,world_qx,world_qy,world_qz,ik_evaluations,tick_ik_evaluations,ik_us,segment_us,path_us,q0,q1,q2,q3,q4,q5\n" << std::setprecision(17);
        }
        runtime::FixedStepScheduler scheduler;
        scheduler.runTicks(ticks,[&] {
            if (retargetTick==body.snapshot().tick+1) {
                if (retargetPosition) reachTarget.position=*retargetPosition;
                if (retargetOrientation) { reachTarget.orientation=*retargetOrientation; reachTarget.task=ReachTask::PositionAndOrientation; }
                body.enqueue(RetargetReach{action,reachTarget});
            }
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
                    << r.route.segmentChecks[0].segmentStartTick << ',' << r.route.segmentChecks[1].segmentStartTick << ','
                    << planningStateName(r.planning.stage) << ',' << r.planning.seedAttempts << ',' << r.planning.seedsProcessed << ','
                    << r.planning.configurations << ',' << r.planning.duplicates << ',' << r.planning.failedSeeds << ',' << r.planning.ikIterations << ',' << r.planning.fkChecks << ','
                    << r.planning.currentConfiguration << ',' << r.route.selectedConfiguration << ',' << r.planning.tickIterations << ',' << r.planning.tickSeeds << ','
                    << r.planning.tickPoses << ',' << r.planning.tickQueries << ',' << r.planning.tickCandidates << ',' << r.planning.budgetReason;
                for (const auto q:r.route.finalCoordinates) csv << ',' << q;
                csv << ',' << static_cast<int>(r.target.selection) << ',' << selectionCompletionName(r.planning.completion)
                    << ',' << r.planning.scoreEvaluations << ',' << r.planning.tickScoreEvaluations << ',' << r.planning.firstClearTick
                    << ',' << r.planning.score.elbow.penalty << ',' << r.planning.score.travel << ',' << r.planning.score.duration << ',' << r.planning.score.total << ',' << r.planning.score.key
                    << ',' << elbowPreferenceStatusName(r.planning.score.elbow.status) << ',' << r.elbowBody.x << ',' << r.elbowBody.y << ',' << r.elbowBody.z
                    << ',' << r.planning.scoreMicroseconds << ',' << (r.elbowDirectionBody ? r.elbowDirectionBody->x:0)
                    << ',' << (r.elbowDirectionBody ? r.elbowDirectionBody->y:0) << ',' << (r.elbowDirectionBody ? r.elbowDirectionBody->z:0)
                    << ',' << r.actualElbow.penalty << ',' << elbowPreferenceStatusName(r.actualElbow.status)
                    << ',' << reachTaskName(r.target.task) << ',' << r.angularError << ',' << r.target.angularTolerance
                    << ',' << r.goalOrientationBody.w << ',' << r.goalOrientationBody.x << ',' << r.goalOrientationBody.y << ',' << r.goalOrientationBody.z
                    << ',' << r.actualOrientationBody.w << ',' << r.actualOrientationBody.x << ',' << r.actualOrientationBody.y << ',' << r.actualOrientationBody.z
                    << ',' << r.actualOrientationWorld.w << ',' << r.actualOrientationWorld.x << ',' << r.actualOrientationWorld.y << ',' << r.actualOrientationWorld.z
                    << ',' << r.evaluations << ',' << r.planning.tickIkEvaluations
                    << ',' << r.planning.ikMicroseconds << ',' << r.planning.segmentMicroseconds << ',' << r.planning.pathMicroseconds;
                const auto ids=reachTarget.arm==ArmSide::Left ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
                    std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist};
                for (unsigned k=0;k<6;++k) csv << ',' << (body.control() ? body.control()->coordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4):0);
                csv << '\n';
            }
        });
        if (csv.is_open()) { csv.flush(); if (!csv) throw std::runtime_error("Cannot write output CSV"); }
        std::cout << "completed_ticks=" << body.snapshot().tick << " simulation_seconds=" << std::setprecision(17)
            << body.snapshot().time << " discarded_seconds=" << scheduler.diagnostics().discardedSeconds << '\n';
        if (reachPosition)
        {
            const auto& r=body.snapshot().reaches[reachTarget.arm==ArmSide::Left ? 0:1];
            std::cout << "action=" << r.action << " state=" << static_cast<int>(r.state) << " error_m=" << r.error << " reason=" << reachReasonName(r.reason) << '\n';
            std::cout << "task=" << reachTaskName(r.target.task) << " angular_error_rad=" << r.angularError << " angular_tolerance_rad=" << r.target.angularTolerance
                << " goal_orientation_body=" << r.goalOrientationBody.w << ',' << r.goalOrientationBody.x << ',' << r.goalOrientationBody.y << ',' << r.goalOrientationBody.z
                << " ik_evaluations=" << r.evaluations << '\n';
            std::cout << "planning=" << planningStateName(r.planning.state) << " direct=" << pathStatusName(r.planning.directCheck.status)
                << " candidates=" << r.planning.candidates << " queries=" << r.planning.queries << " planning_ticks=" << r.planning.ticks
                << " segments=" << r.route.segmentCount << " current_segment=" << r.route.currentSegment
                << " configuration=" << r.route.selectedConfiguration << " unique=" << r.planning.configurations << " seeds=" << r.planning.seedAttempts << " duplicates=" << r.planning.duplicates << " iterations=" << r.planning.ikIterations
                << " selected=" << r.route.selectedCandidate << " scratch_bytes=" << r.planning.scratchBytes
                << " last_seed=" << ikSeedOutcomeName(r.planning.lastSeedOutcome) << " budget_reason=" << r.planning.budgetReason << '\n';
            std::cout << "selection=" << (r.target.selection==RouteSelection::Ranked ? "Ranked":"FirstValid")
                << " completion=" << selectionCompletionName(r.planning.completion) << " evaluations=" << r.planning.scoreEvaluations
                << " elbow=" << r.planning.score.elbow.penalty << " travel=" << r.planning.score.travel << " duration=" << r.planning.score.duration
                << " score=" << r.planning.score.total << " key=" << r.planning.score.key << " score_us=" << r.planning.scoreMicroseconds
                << " actual_elbow=" << r.elbowBody.x << ',' << r.elbowBody.y << ',' << r.elbowBody.z
                << " preference_status=" << elbowPreferenceStatusName(r.actualElbow.status) << '\n';
            for (unsigned i=0;i<r.planning.scoreEvaluations;++i) {
                const auto& e=r.planning.evaluations[i];
                std::cout << "verified_route=" << i+1 << " configuration=" << e.configuration << " candidate=" << e.candidate << " segments=" << e.segments
                    << " elbow=" << e.score.elbow.penalty << " travel=" << e.score.travel << " duration=" << e.score.duration << " score=" << e.score.total << '\n';
            }
        }
        if (reachPosition) for (unsigned i=0;i<body.snapshot().reaches[reachTarget.arm==ArmSide::Left ? 0:1].planning.configurations;++i) {
            const auto& c=body.snapshot().reaches[reachTarget.arm==ArmSide::Left ? 0:1].planning.solutions[i];
            std::cout << "configuration=" << i+1 << " error=" << c.error << " direct=" << pathStatusName(c.directStatus) << " outcome=" << configurationOutcomeName(c.outcome)
                << " forms=" << c.bodyForm << ',' << c.otherBodyForm << " object=" << c.object << " gap=" << c.gap << " detail=" << c.detail << " q=";
            for (float q:c.coordinates) std::cout << q << ',';
            std::cout << '\n';
        }
        if (collisionDemo) { const auto& p=body.snapshot().pathCheck;
            std::cout << "path=" << pathStatusName(p.status) << " scene=" << p.sceneVersion << " forms=" << p.bodyForm << ',' << p.otherBodyForm << " object=" << p.object
                << " interval=" << p.fromTick << ',' << p.toTick << " gap=" << p.contact.gap << " queries=" << p.queries << " check_us=" << p.microseconds << '\n'; }
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
