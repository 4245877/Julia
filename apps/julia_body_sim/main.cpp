#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>

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
        bool eyes=true;
        std::filesystem::path source=std::filesystem::path{JULIA_ASSET_DIR}/"models/Raphtalia_prepared.fbx", output;
        for (int i=1;i<argc;++i)
        {
            const std::string_view argument{argv[i]};
            if (argument=="--help")
            {
                std::cout << "julia_body_sim [--ticks N] [--clip INDEX] [--elbow RADIANS] [--no-eyes] [--model PATH] [--output CSV]\n";
                return 0;
            }
            if (argument=="--no-eyes") { eyes=false; continue; }
            if (i+1>=argc) throw std::invalid_argument("Missing option value");
            const std::string_view value{argv[++i]};
            if (argument=="--ticks") ticks=number<std::uint64_t>(value);
            else if (argument=="--clip") clip=number<std::size_t>(value);
            else if (argument=="--elbow") elbow=number<float>(value);
            else if (argument=="--model") source=std::filesystem::path{value};
            else if (argument=="--output") output=std::filesystem::path{value};
            else throw std::invalid_argument("Unknown option");
        }
        const auto asset=model_import::ModelImporter::loadFromFile(source);
        const auto profile=raphtaliaBodyProfile();
        const bool calibrated=validateBodyProfile(profile,*asset).empty();
        BodyInstance body(1,asset,calibrated ? &profile:nullptr,eyes);
        if (clip)
        { body.enqueue(ClipCommand{ClipOperation::Select,static_cast<double>(*clip)}); body.enqueue(ClipCommand{ClipOperation::Play}); }
        if (elbow)
        { body.enqueue(SetDirectMode{true}); body.enqueue(SetCoordinate{JointId::LeftElbow,0,*elbow}); }
        std::ofstream csv;
        if (!output.empty())
        {
            if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path());
            csv.open(output); if (!csv) throw std::runtime_error("Cannot open output CSV");
            csv << "tick,simulation_seconds,clip_seconds,left_elbow_radians,gaze_yaw_degrees,gaze_pitch_degrees,blink_left,blink_right\n" << std::setprecision(17);
        }
        runtime::FixedStepScheduler scheduler;
        scheduler.runTicks(ticks,[&] {
            body.step();
            for (const auto& result:body.results())
                if (result.status==BodyCommandStatus::Rejected) throw std::runtime_error("Command rejected: "+result.reason);
            if (csv.is_open())
            {
                const auto& s=body.snapshot();
                csv << s.tick << ',' << s.time << ',' << s.clipTime << ','
                    << (body.control() ? body.control()->coordinate(JointId::LeftElbow,0):0) << ','
                    << s.eyes.gazeDegrees.x << ',' << s.eyes.gazeDegrees.y << ',' << s.eyes.blinkLeft << ',' << s.eyes.blinkRight << '\n';
            }
        });
        if (csv.is_open()) { csv.flush(); if (!csv) throw std::runtime_error("Cannot write output CSV"); }
        std::cout << "completed_ticks=" << body.snapshot().tick << " simulation_seconds=" << std::setprecision(17)
            << body.snapshot().time << " discarded_seconds=" << scheduler.diagnostics().discardedSeconds << '\n';
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
