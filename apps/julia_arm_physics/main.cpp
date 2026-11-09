#include "BenchIO.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"

int main(int argc,char** argv)
{
    try
    {
        const auto options=bench::parse(argc,argv);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(
            std::filesystem::path{JULIA_PHYSICS_ASSETS}/"models/Raphtalia_prepared.fbx");
        const julia::physics::ArmRig rig{*asset,options.side,options.chain};
        julia::physics::PhysicsWorld world{julia::physics::benchDefinition(rig,options.scenario)};
        const auto parity=julia::physics::verifyFramesBeforeDynamics(rig,world);
        bench::reset(world,options.scenario,options);
        bench::Log log{options.output}; bench::metadata(options.output,rig,world);
        std::cout<<"FK position_m="<<parity.position<<" rotation_element="<<parity.rotationElement<<'\n';
        julia::runtime::FixedStepScheduler scheduler;
        scheduler.runTicks(options.ticks,[&] {
            julia::physics::stepBench(world,options.scenario);
            const auto snapshot=julia::physics::publish(rig,world.snapshot());
            log.row(snapshot,world.definition());
        });
        const auto& s=world.snapshot();
        if(s.tick!=options.ticks||s.worldSteps!=options.ticks*julia::physics::Substeps)
            throw std::runtime_error("Exact tick count failed");
        std::cout<<"PASS MuJoCo "<<world.backendVersion()<<" fixed-base "<<julia::physics::benchCaseName(options.scenario)
            <<" ticks="<<s.tick<<" substeps="<<s.worldSteps<<" q="<<s.q[0]<<','<<s.q[1]<<"; CSV: "<<options.output.string()<<'\n';
        return 0;
    }
    catch(const std::exception& e) { std::cerr<<"PHYSICS FAILURE: "<<e.what()<<'\n'; return 1; }
}
