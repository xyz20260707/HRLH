#include "hrlh.hpp"
#include "solver_io.hpp"
#include "solution_witness.hpp"
#include "loading_witness.hpp"
#include "original_validator_adapter.hpp"
#include "independent_checker.hpp"
#ifdef HRLH_ROUTING_COMPARISON
#include "routing_search.hpp"
#include "alns.hpp"
#include "hgs.hpp"
#endif
#include <chrono>
#include <iostream>
using namespace hrlh;
int main(int argc,char** argv) try {
    if(argc!=4)throw std::runtime_error("Usage: hrlh_backend INPUT.dat CONFIG.ini OUTPUT.json");
    std::ifstream input(argv[1]);if(!input)throw std::runtime_error("Cannot read instance");
    auto instance=io::read_case(input);
    auto config=io::read_config(argv[2]);
    const auto begin=std::chrono::steady_clock::now();
    HrlhResult result;
#ifdef HRLH_ROUTING_COMPARISON
    auto start=freeze_common_start(instance,config);
    BackendRunControl control;control.algorithm=HRLH_BACKEND;
#ifdef HRLH_BACKEND_ALNS
    if(control.algorithm=="ALNS")control.search=[](const auto& p,const auto& w,const auto& s,const auto& cb,auto seed){alns::run(p,w,s,cb,seed);};
#else
    if(control.algorithm=="HGS")control.search=[](const auto& p,const auto& w,const auto& s,const auto& cb,auto seed){hgs::run(p,w,s,cb,seed);};
#endif
    result=resume_common_start(start,control);
    const std::string algorithm=HRLH_BACKEND;
#else
    result=solve_hrlh(instance,config);
    const std::string algorithm="ILS";
#endif
    const auto runtime=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    if(result.has_solution){
        recover_preprocessed_material_witness(instance,result.preprocess,result.best_solution);
        complete_physical_witness(result.best_solution);
        auto a=independent::check(instance,result.best_solution);
        auto b=physical_validation::check_original(instance,result.best_solution);
        if(!a.ok()||!b.ok)throw std::runtime_error("Physical solution validation failed");
    }
    std::ofstream out(argv[3]);if(!out)throw std::runtime_error("Cannot write output");
    out<<std::setprecision(17)<<"{\"algorithm\":"<<io::quote(algorithm)
       <<",\"seed\":"<<config.seed<<",\"feasible\":"<<(result.has_solution?"true":"false")
       <<",\"runtime\":"<<runtime<<",\"stop_reason\":"<<io::quote(result.has_solution?"solver_returned":"no_feasible_solution_found")
       <<",\"objective\":";
    if(result.has_solution)out<<"{\"C\":"<<result.best_solution.transport_cost<<",\"P\":"<<result.best_solution.optional_profit<<",\"F\":"<<result.best_solution.latest_departure_sum<<"}";
    else out<<"null";
    out<<",\"vehicle_count\":"<<(result.has_solution?result.best_solution.routes.size():0)<<",\"cost_components\":";
    if(result.has_solution){const double fixed=instance.fixed_vehicle_cost*result.best_solution.routes.size();out<<"{\"fixed_vehicles\":"<<fixed<<",\"distance\":"<<result.best_solution.transport_cost-fixed<<"}";}else out<<"null";
    out<<",\"validation_ok\":"<<(result.has_solution?"true":"false")<<",\"solution\":";
    if(result.has_solution)out<<solution_witness_json(result.best_solution);else out<<"null";
    out<<"}";out.close();if(!out)throw std::runtime_error("Incomplete output");
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
