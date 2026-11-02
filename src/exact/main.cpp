#include "solver_io.hpp"
#include <iostream>
int main(int argc,char** argv)try{
    if(argc!=4)throw std::runtime_error("Usage: hrlh_exact INPUT.dat SECONDS OUTPUT.json");
    std::ifstream input(argv[1]);if(!input)throw std::runtime_error("Cannot read instance");
    auto instance=hrlh::io::read_case(input);
    const double seconds=std::stod(argv[2]);if(!std::isfinite(seconds)||seconds<=0)throw std::runtime_error("Time limit must be finite and positive");
    auto result=hrlh_model::solve_exact_model(instance,seconds,0.0,false,seconds,false,1,1,false,false);
    std::ofstream out(argv[3]);if(!out)throw std::runtime_error("Cannot write output");
    const bool feasible=result.transport_cost.has_value();
    out<<std::setprecision(17)<<"{\"feasible\":"<<(feasible?"true":"false")<<",\"objective\":";
    if(feasible)out<<"{\"C\":"<<*result.transport_cost<<",\"P\":"<<*result.optional_profit<<",\"F\":"<<*result.latest_departure_sum<<"}";else out<<"null";
    int vehicles=0;for(const auto& route:result.route_arcs)if(!route.second.empty())++vehicles;
    const bool valid=feasible&&hrlh_model::validate_solution_against_model(instance,result,1e-4,false).ok;
    out<<",\"runtime\":"<<result.runtime<<",\"stop_reason\":"<<hrlh::io::quote(result.status_name)<<",\"optimal\":"<<(result.status_name=="OPTIMAL"?"true":"false")<<",\"vehicle_count\":"<<vehicles<<",\"validation_ok\":"<<(valid?"true":"false")<<",\"cost_components\":";
    if(feasible){const double fixed=vehicles*instance.fixed_vehicle_cost;out<<"{\"fixed_vehicles\":"<<fixed<<",\"distance\":"<<*result.transport_cost-fixed<<"}";}else out<<"null";
    out<<",\"mip_gap\":";
    if(result.mip_gap&&std::isfinite(*result.mip_gap))out<<*result.mip_gap;else out<<"null";
    out<<",\"solution\":"<<hrlh_model::solution_detail_json(instance,result)<<"}";
    out.close();if(!out)throw std::runtime_error("Incomplete output");return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
