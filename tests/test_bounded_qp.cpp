#include "../libs/physics/src/BoundedQp.hpp"
#include "julia_test.hpp"
#include <fstream>
#include <filesystem>
#include <limits>
#include <memory>
#include <iomanip>

using namespace julia::physics::qp;
int main(int argc,char** argv)
{
    const auto path=std::filesystem::path{argc>1?argv[1]:"build/diagnostics/stage7h/qp"}; std::filesystem::create_directories(path);
    std::ofstream log{path/"analytic.txt"};
    for(const auto implementation:{Implementation::Reference7h,Implementation::Optimized7i})
    {
    log<<(implementation==Implementation::Reference7h?"reference7h":"optimized7i")<<'\n';
    auto solver=std::make_unique<Solver>(); auto p=std::make_unique<Problem>();
    p->n=2; p->e=1; p->m=4; p->h[0]=p->h[Variables+1]=1;
    p->g[0]=8; p->g[1]=2; p->equality[0]=p->equality[1]=1; p->target[0]=10;
    p->inequality[0]=1; p->inequality[Variables+1]=1;
    p->inequality[2*Variables]=-1; p->lower[2]=-4;
    p->inequality[3*Variables+1]=-1; p->lower[3]=-8;
    const auto a=solver->solve(*p,128,implementation);
    JULIA_CHECK(a.code==Code::Feasible&&std::abs(a.x[0]-4)<1e-9&&std::abs(a.x[1]-6)<1e-9&&a.active==1);
    JULIA_CHECK(a.multipliers[2]>0&&a.equalityResidual<1e-9&&a.violation<1e-9&&a.projectedStationarity<1e-9);
    log<<"Equality optimum (8,2) violates cap4. Bounded optimum "<<a.x[0]<<','<<a.x[1]<<" active="<<a.active<<" dual="<<a.multipliers[2]<<" PASS\n";
    const auto b=solver->solve(*p,128,implementation); JULIA_CHECK(a.x==b.x&&a.multipliers==b.multipliers&&a.iterations==b.iterations);
    const auto exhausted=solver->solve(*p,0,implementation); JULIA_CHECK(exhausted.code==Code::Budget);
    p->lower[2]=p->lower[3]=-1; const auto no=solver->solve(*p,128,implementation);
    JULIA_CHECK(no.code==Code::Infeasible&&no.certificateMargin>0);
    for(int j=0;j<2;++j) { double v=no.equalityCertificate[0]; for(int r=0;r<4;++r) v+=p->inequality[r*Variables+j]*no.certificate[r]; JULIA_CHECK(std::abs(v)<1e-9); }
    for(int r=0;r<4;++r) JULIA_CHECK(no.certificate[r]>=-1e-10);
    log<<"Infeasible load10 with caps1+1: independently verified Farkas margin="<<no.certificateMargin<<"; budget0="<<name(exhausted.code)<<" PASS\n";
    {
        std::ofstream certificate{path/(implementation==Implementation::Reference7h?"farkas-reference.json":"farkas-optimized.json")};
        certificate<<std::setprecision(17)<<"{\"inequality\":[";
        for(int r=0;r<4;++r) { if(r) certificate<<',';certificate<<no.certificate[r]; }
        certificate<<"],\"equality\":"<<no.equalityCertificate[0]<<",\"margin\":"<<no.certificateMargin<<"}\n";
    }
    p->n=0; JULIA_CHECK(solver->solve(*p,128,implementation).code==Code::Invalid); p->n=2;
    JULIA_CHECK(solver->solve(*p,1025,implementation).code==Code::Invalid);
    p->g[0]=std::numeric_limits<double>::quiet_NaN(); JULIA_CHECK(solver->solve(*p,128,implementation).code==Code::Invalid);
    p->g[0]=std::numeric_limits<double>::infinity(); JULIA_CHECK(solver->solve(*p,128,implementation).code==Code::Invalid);
    *p={}; p->n=1; p->h[0]=1; p->m=1; p->inequality[0]=1; p->lower[0]=2;
    auto reset=solver->solve(*p,128,implementation); JULIA_CHECK(reset.code==Code::Feasible&&std::abs(reset.x[0]-2)<1e-9);
    *p={}; p->n=3; p->e=1; p->m=2;
    p->h[0]=100; p->h[Variables+1]=p->h[2*Variables+2]=.01;
    p->equality[0]=-1; p->equality[1]=p->equality[2]=1; p->target[0]=10;
    p->inequality[1]=-1; p->lower[0]=-4; p->inequality[Variables+2]=-1; p->lower[1]=-4;
    const auto relaxed=solver->solve(*p,128,implementation);
    JULIA_CHECK(relaxed.code==Code::Feasible&&std::abs(relaxed.x[0]+2)<1e-8&&std::abs(relaxed.x[1]-4)<1e-8&&std::abs(relaxed.x[2]-4)<1e-8);
    log<<"Joint acceleration/force QP revises soft requested acceleration0 to -2 to satisfy force caps4+4 and full balance10 PASS\n";
    *p={}; p->n=3; p->e=1; p->m=5;
    for(int i=0;i<3;++i) { p->h[i*Variables+i]=1; p->g[i]=10; }
    p->equality[1]=1; p->target[0]=10; p->inequality[1]=1;
    int row=1;
    for(int sx:{-1,1}) for(int sz:{-1,1}) { p->inequality[row*Variables]=-sx; p->inequality[row*Variables+1]=.6; p->inequality[row*Variables+2]=-sz; ++row; }
    const auto friction=solver->solve(*p,128,implementation);
    JULIA_CHECK(friction.code==Code::Feasible&&std::abs(friction.x[0]-3)<1e-9&&std::abs(friction.x[1]-10)<1e-9&&std::abs(friction.x[2]-3)<1e-9);
    JULIA_CHECK(std::hypot(friction.x[0],friction.x[2])<=.6*friction.x[1]);
    log<<"Known active floor-normal friction pyramid solution(3,10,3), circular cone certificate PASS\n";
    log<<"Cold-start after failures, zero dimensions, NaN/Inf, repeatability PASS\n";
    *p={};p->n=1;JULIA_CHECK(solver->solve(*p,128,implementation).code==Code::Numerical);
    *p={};p->n=1;p->h[0]=1;JULIA_CHECK(solver->solve(*p,128,static_cast<Implementation>(99)).code==Code::Invalid);
    }
    return JULIA_TEST_RESULT();
}
