#include "ContactBench.hpp"
#include "julia/model_import/ModelImporter.hpp"

int main(int argc,char** argv)
{
    try
    {
        const auto options=contact_bench::parse(argc,argv);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_PHYSICS_ASSETS}/"models/Raphtalia_prepared.fbx");
        const julia::physics::ArmRig rig{*asset,options.side,julia::physics::ArmChain::Full};
        julia::physics::PhysicsWorld world{contact_bench::description(rig,options),contact_bench::initial(options.pose)};
        julia::physics::verifyFramesBeforeDynamics(rig,world);
        contact_bench::reset(world,options);
        contact_bench::Log log{options.output}; contact_bench::metadata(options.output,rig,world,options);
        for(std::uint64_t tick=0;tick<options.ticks;++tick)
        {
            contact_bench::step(world,options);
            log.row(julia::physics::publish(rig,world.snapshot()),world.definition());
        }
        const auto& c=world.snapshot().contact;
        std::cout<<"PASS execution ticks="<<world.snapshot().tick<<" phase="<<julia::physics::contactPhaseName(c.phase)
            <<" failure="<<julia::physics::contactFailureName(c.failure)<<" hold="<<c.holdSucceeded
            <<" peak_N="<<c.peakForce<<" penetration_m="<<c.peakPenetration<<'\n';
        return 0;
    }
    catch(const std::exception& e) { std::cerr<<"CONTACT FAILURE: "<<e.what()<<'\n'; return 1; }
}
