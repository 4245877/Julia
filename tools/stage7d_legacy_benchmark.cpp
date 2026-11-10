// Deliberately uses only the stage7c public API, so it can also be linked to
// the archived baseline backend for an equal-workload overhead measurement.
#include "julia/physics/BodyRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>

using namespace julia::physics;
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path dir=argc>1?argv[1]:"build/diagnostics/stage7d/legacy-benchmark";
        std::filesystem::create_directories(dir); std::ofstream report{dir/"performance.txt"}; report<<std::setprecision(17);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const ArmRig arm{*asset,ArmSide::Left,ArmChain::Full}; const BodyRig body{*asset};
        for(bool full:{false,true})
        {
            PhysicsWorld w{full?bodyBenchDefinition(body,BodyCase::Internal):arm.description(1,1,.5)};
            if(full) resetBodyBench(body,w,BodyCase::Internal); else resetBench(w,BenchCase::Hold);
            std::ofstream csv{dir/(full?"body.csv":"arm.csv")}; csv<<"tick,tick_us,control_us,step_us,read_us\n";
            std::vector<double> samples;
            for(unsigned n=0;n<10000;++n)
            {
                const auto start=std::chrono::steady_clock::now();
                if(full) stepBodyBench(w,BodyCase::Internal); else w.step();
                const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count(); samples.push_back(elapsed);
                const auto& t=w.snapshot().timing; csv<<n+1<<','<<elapsed<<','<<t.controlUs<<','<<t.stepUs<<','<<t.readUs<<'\n';
            }
            const double mean=std::accumulate(samples.begin(),samples.end(),0.)/static_cast<double>(samples.size()); std::sort(samples.begin(),samples.end());
            report<<(full?"body":"arm")<<" mean_us="<<mean<<" p95_us="<<samples[9500]<<" max_us="<<samples.back()<<'\n';
            report<<"final_q="; for(unsigned k=0;k<w.snapshot().dofs;++k) report<<w.snapshot().q[k]<<','; report<<'\n';
        }
        return 0;
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
