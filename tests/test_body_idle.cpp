#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "../apps/common/StancePreparationDemo.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace
{
    using namespace julia;
    using namespace julia::embodiment;
    constexpr auto dt = runtime::BodyStepSeconds;
    constexpr std::uint64_t ticks = 2880;
    const RootPose initialRoot{{.76f,.11f,-.24f}, glm::angleAxis(.61f, glm::vec3{0,1,0})};
    constexpr std::pair<std::uint64_t,bool> changes[]{
        {420,true}, {650,false}, {980,true}, {1240,false}, {1730,true}, {1940,false},
        {2250,true}, {2264,false}, {2278,true}, {2475,false},
    };

    void require(bool condition, const std::string& reason)
    {
        if (!condition) throw std::runtime_error(reason);
    }

    IdleView replayView(std::uint64_t tick)
    {
        if (tick >= 700 && tick < 1400)
            return {initialRoot.rotation * glm::normalize(glm::vec3{-.33f,.1f,1}), true};
        return {initialRoot.position + initialRoot.rotation * glm::vec3{1.3f,1.64f,3}};
    }

    struct FootErrors
    {
        float position=0, angle=0, gap=0;
        void record(const SupportObservation& support)
        {
            for (const auto& foot : support.anchors) {
                require(foot.bound && foot.positionError <= SupportPositionTolerance &&
                    foot.angularError <= SupportAngularTolerance && foot.minGap >= -SupportSurfaceTolerance &&
                    foot.maxGap <= SupportSurfaceTolerance, "Support deviated from an anchor");
                position = std::max(position, foot.positionError);
                angle = std::max(angle, foot.angularError);
                gap = std::max({gap, -foot.minGap, foot.maxGap});
            }
        }
        void report(std::ostream& out, const char* label) const
        {
            out << label << "_max_position_m=" << position << '\n' << label << "_max_angle_rad=" << angle
                << '\n' << label << "_max_gap_m=" << gap << '\n';
        }
    };

    std::uint64_t fingerprint(const BodySnapshot& s)
    {
        std::uint64_t h=14695981039346656037ull;
        const auto add = [&](auto bits) { h = (h ^ bits) * 1099511628211ull; };
        add(static_cast<unsigned>(s.idle.phase));
        add(std::bit_cast<std::uint64_t>(s.idle.phaseSeconds));
        const auto scalar = [&](float f) { add(std::bit_cast<std::uint32_t>(f)); };
        for (int k=0; k<3; ++k) scalar(s.root.position[k]);
        for (const auto& q : s.coordinates)
            for (int k=0; k<3; ++k) scalar(q[k]);
        scalar(s.eyes.gazeDegrees.x); scalar(s.eyes.gazeDegrees.y);
        scalar(s.eyes.blinkLeft); scalar(s.eyes.blinkRight);
        for (const auto& m : s.pose.palette)
            for (int col=0; col<4; ++col)
                for (int row=0; row<4; ++row) scalar(m[col][row]);
        return h;
    }

    float gazeError(const BodyInstance& body, IdleView view)
    {
        const auto& snapshot = body.snapshot();
        const auto& nodes = body.asset().nodes();
        const auto bodyFromResource = body.control()->binding().profile().bodyFromResource;
        glm::vec3 center{0}, gaze{0};
        for (std::size_t i=0; i<nodes.size(); ++i) {
            if (nodes[i].name != "eye.L" && nodes[i].name != "eye.R") continue;
            const auto& eye = snapshot.pose.globals[i];
            const auto delta = eye * glm::inverse(nodes[i].globalBindTransform);
            gaze += glm::normalize(glm::vec3{delta * glm::vec4{0,0,1,0}});
            center += glm::vec3{bodyFromResource * eye[3]};
        }
        const auto& root = snapshot.root;
        const auto direction = glm::normalize(view.directional ? view.value :
            view.value - (root.position + root.rotation * center * .5f));
        return glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(root.rotation * gaze), direction), -1.0f, 1.0f)));
    }

    std::uint64_t applied(BodyInstance& body, BodyOperation op, bool expected=true)
    {
        const auto id=body.enqueue(std::move(op)).id;
        body.step();
        const auto& result=body.results().front();
        require(result.status==(expected ? BodyCommandStatus::Applied : BodyCommandStatus::Rejected), result.reason);
        return id;
    }

    void reachFromReady(BodyInstance& body, std::ostream& report)
    {
        applied(body, SetIdleStance{true});
        for (unsigned i=0; i<200 && body.snapshot().idle.phase!=IdlePhase::Ready; ++i) body.step();
        require(body.snapshot().idle.phase==IdlePhase::Ready, "Missing Ready before Reach");
        const auto stance = body.snapshot();
        constexpr auto arm = ArmSide::Left;
        const auto& binding = body.control()->binding();
        applied(body, CreateInteractionSphere{1, app::interactionDemoPose(binding, arm, 0, stance.root)});
        const auto object = body.snapshot().object->handle;
        const auto action = applied(body, StartReach{app::preparationPregrip(arm, body.snapshot().object->worldPose)});
        applied(body, SetIdleStance{false}, false);
        for (unsigned i=0; i<2400 && app::reachActive(body.snapshot().reaches[0].state); ++i) body.step();
        const auto& result = body.snapshot().reaches[0];
        require(result.state==ReachState::Succeeded && result.collisionChecked && body.snapshot().reachGeometry[0].dependenciesMatch,
            std::string("Reach from Idle Ready: ")+reachReasonName(result.reason)+" "+body.snapshot().pathCheck.detail);
        applied(body, PrepareGrasp{object, arm, action, stance.epoch});
        for (unsigned i=0; i<2400 && body.snapshot().interaction.phase!=InteractionPhase::ReadyToGrasp; ++i) body.step();
        require(body.snapshot().interaction.phase==InteractionPhase::ReadyToGrasp, "Preparation from Idle Ready: "+body.snapshot().interaction.reason);
        require(body.snapshot().idle.phase==IdlePhase::Ready && body.snapshot().support.phase==SupportPhase::Holding &&
            body.snapshot().root.position==stance.root.position && body.snapshot().object->state==ObjectState::Free,
            "Handoff changed stance or attached the object");
        report << "ready_handoff=ReadyToGrasp\n";
    }

    void replay(const std::shared_ptr<const model::ModelAsset>& asset, const BodyProfile& profile,
        std::span<const double> frameTimes, std::vector<std::uint64_t>& reference, std::ostream& report)
    {
        BodyInstance body{1, asset, &profile};
        body.enqueue(SetRoot{initialRoot});
        body.enqueue(SetSupportSurface{{1,2,initialRoot.position.y,true}});
        body.enqueue(StartIdle{});
        const bool exact = frameTimes.empty();
        FootErrors atTicks, atDisplay;
        float boundaryStep=0, boundarySpeedChange=0, readySpeed=0, perspectiveError=0, parallelError=0;
        std::vector<glm::vec3> previousVelocity(profile.joints.size(), glm::vec3{0}), readyCoordinates;
        glm::vec3 readyRoot{0};
        std::array<glm::mat4,2> anchors{};
        unsigned readyCount=0, resumed=0, reversals=0;
        std::size_t maxPending=body.pendingCommands();
        runtime::FixedStepScheduler scheduler;
        const auto step = [&] {
            if (body.snapshot().tick==ticks) return;
            const auto tick = body.snapshot().tick + 1;
            const auto view = replayView(tick);
            body.setIdleView(view);
            for (const auto& [at, ready] : changes)
                if (tick==at) body.enqueue(SetIdleStance{ready});
            maxPending=std::max(maxPending,body.pendingCommands());
            body.step();
            for (const auto& r : body.results()) require(r.status==BodyCommandStatus::Applied, r.reason);
            const auto& s = body.snapshot();
            const auto& p = body.previousSnapshot();
            require(s.idle.phase!=IdlePhase::Fault && s.direct && body.pendingCommands()==0, "Idle fault or queued camera input");
            atTicks.record(s.support);
            for (std::size_t i=0; i<anchors.size(); ++i) {
                if (tick==1) anchors[i]=s.support.anchors[i].desiredWorld;
                require(s.support.anchors[i].desiredWorld==anchors[i], "Idle recaptured a planted anchor");
            }
            if (tick>1) {
                require(p.tick+1==tick, "Idle/Ready collapsed interpolation history");
                const bool boundary=s.idle.phase!=p.idle.phase;
                for (std::size_t j=0; j<s.coordinates.size(); ++j) {
                    const auto delta=s.coordinates[j]-p.coordinates[j];
                    const auto velocity=delta/static_cast<float>(dt);
                    for (int k=0; k<3; ++k) {
                        if (boundary) {
                            boundaryStep=std::max(boundaryStep,std::abs(delta[k]));
                            boundarySpeedChange=std::max(boundarySpeedChange,std::abs(velocity[k]-previousVelocity[j][k]));
                        }
                        if (s.idle.phase==IdlePhase::Ready) readySpeed=std::max(readySpeed,std::abs(velocity[k]));
                    }
                    previousVelocity[j]=velocity;
                }
                if (s.idle.phase==IdlePhase::Ready) {
                    require(s.support.phase==SupportPhase::Holding && s.idle.phaseSeconds==p.idle.phaseSeconds, "Ready did not hold");
                    if (boundary) {
                        if (readyCount==0) { readyCoordinates=s.coordinates; readyRoot=s.root.position; }
                        require(s.coordinates==readyCoordinates && s.root.position==readyRoot, "Ready differs by Idle phase");
                        ++readyCount;
                    } else require(s.coordinates==p.coordinates && s.root.position==p.root.position, "Ready drifted");
                }
                if (s.idle.phase==IdlePhase::Resuming && boundary) {
                    require(s.idle.phaseSeconds==p.idle.phaseSeconds+dt, "Idle phase restarted on resume");
                    if (p.idle.phase==IdlePhase::Settling) ++reversals;
                    else ++resumed;
                }
                if (tick%12==0 || boundary) {
                    for (double alpha : {.125,.5,.875}) {
                        const auto display = body.display(alpha);
                        atDisplay.record(display.support);
                        require(display.fromTick==p.tick && display.toTick==s.tick, "Display skipped the enclosing states");
                    }
                }
            }
            if (tick==320) perspectiveError=gazeError(body,view);
            if (tick==930) parallelError=gazeError(body,view);
            if (exact) reference.push_back(fingerprint(s));
            else require(reference.at(static_cast<std::size_t>(tick-1))==fingerprint(s), "Frame replay diverged at tick "+std::to_string(tick));
        };
        if (exact) scheduler.runTicks(ticks,step);
        else for (std::size_t frame=0; body.snapshot().tick<ticks; ++frame) {
            if (body.snapshot().tick>=1050 && !scheduler.diagnostics().suspendedFrames) {
                const auto saved=fingerprint(body.snapshot());
                const auto tick=body.snapshot().tick;
                scheduler.setPaused(true);
                body.collapseHistory();
                require(scheduler.advance(2.0,step)==0 && fingerprint(body.snapshot())==saved &&
                    body.display(1).fromTick==tick, "Pause advanced or changed Idle");
                scheduler.setPaused(false);
                require(scheduler.advance(.6,step)==0 && fingerprint(body.snapshot())==saved, "Suspension advanced Idle");
            }
            scheduler.advance(frameTimes[frame%frameTimes.size()],step);
            if (body.snapshot().tick) atDisplay.record(body.display(scheduler.alpha()).support);
        }
        require(readyCount==4 && resumed==4 && reversals==1, "Missing transition coverage");
        require(boundaryStep<.005f && boundarySpeedChange<.04f && readySpeed<.004f, "Idle transition discontinuity");
        require(perspectiveError<2.5f && parallelError<2.5f && body.eyes().behavior().stats().blinkCount>2, "Gaze or blink failed");
        require(body.commandHistorySize()==3+std::size(changes), "Per-tick input accumulated in command history");
        if (!exact) require(scheduler.diagnostics().overloadFrames==0, "Unexpected overload in replay");
        report << "replay=" << (exact ? "exact" : frameTimes.size()==1 ? std::to_string(1/frameTimes.front())+"fps":"irregular")
            << " ticks=" << ticks << " pause=" << (!exact) << " ready_count=" << readyCount << " resume_count=" << resumed
            << " reversal_count=" << reversals << " max_pending=" << maxPending << " history=" << body.commandHistorySize() << '\n';
        atTicks.report(report,"tick");
        atDisplay.report(report,"display");
        report << "boundary_max_delta_rad=" << boundaryStep << "\nboundary_max_velocity_change_rad_s=" << boundarySpeedChange
            << "\nready_max_speed_rad_s=" << readySpeed << "\nperspective_gaze_error_deg=" << perspectiveError
            << "\northographic_gaze_error_deg=" << parallelError << "\nblinks=" << body.eyes().behavior().stats().blinkCount << '\n';
        if (exact) reachFromReady(body,report);
    }
}

int main(int argc, char** argv)
{
    try {
        if (argc!=2) throw std::invalid_argument("Usage: julia_test_body_idle output_directory");
        std::filesystem::create_directories(argv[1]);
        std::ofstream report{std::filesystem::path{argv[1]}/"idle.txt"};
        require(static_cast<bool>(report), "Cannot open Idle report");
        const auto asset = model_import::ModelImporter::loadFromFile(
            std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile();
        std::vector<std::uint64_t> reference;
        replay(asset, profile, {}, reference, report);
        replay(asset, profile, std::array{1.0/30}, reference, report);
        replay(asset, profile, std::array{1.0/144}, reference, report);
        replay(asset, profile, std::array{.004,.019,.051,.007,.025}, reference, report);
        report << "result=PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
