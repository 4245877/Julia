#include "BodyBenchIO.hpp"
int main(int argc,char** argv)
{
    try
    {
        const auto o=bodybench::parse(argc,argv);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_PHYSICS_ASSETS}/"models/Raphtalia_prepared.fbx");
        const bodybench::BodyRig rig{*asset};
        bodybench::PhysicsWorld world{bodybench::bodyBenchDefinition(rig,o.scenario)};
        const auto parity=bodybench::verifyFramesBeforeDynamics(rig,world);
        bodybench::Log log{o.output}; bodybench::metadata(o.output,rig,world);
        bodybench::resetBodyBench(rig,world,o.scenario); log.row(bodybench::publish(rig,world.snapshot()));
        for(std::uint64_t n=0;n<o.ticks;++n) { bodybench::stepBodyBench(world,o.scenario); log.row(bodybench::publish(rig,world.snapshot())); }
        std::cout<<"PASS free body case="<<bodybench::bodyCaseName(o.scenario)<<" ticks="<<world.snapshot().tick<<" steps="<<world.snapshot().worldSteps
            <<" FK_m="<<parity.position<<" rotation="<<parity.rotationElement<<'\n'; return 0;
    }
    catch(const std::exception& e) { std::cerr<<"BODY PHYSICS FAILURE: "<<e.what()<<'\n'; return 1; }
}
