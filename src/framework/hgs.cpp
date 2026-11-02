#include "hgs.hpp"
#include "Genetic.h"
#include <stdexcept>
namespace hrlh::hgs {
void run(const routing::Problem& problem,const RoutePlan& warm,
         const std::vector<RoutePlan>& initial,const routing::SearchCallbacks& cb,
         std::uint64_t seed,const RunControl& control){
    if(control.initial_population<1||control.iterations<0||control.no_improvement_restart<1)
        throw std::invalid_argument("invalid HGS fixed-work control");
    Params params(problem,cb,seed);params.iterationLimit=control.iterations;
    params.initialPopulationLimit=control.initial_population;
    auto convert=[&](const RoutePlan& plan,bool auxiliary){
        const auto check=problem.evaluate(plan);
        if(check.duplicate_or_unknown||check.missing_suppliers||(!auxiliary&&check.used_vehicles>params.nbVehicles))
            throw std::invalid_argument("invalid common initial HGS plan");

        if(check.used_vehicles>params.nbVehicles)return std::vector<std::vector<int>>{};
        std::vector<std::vector<int>> out;
        for(const auto& r:plan)if(!r.empty()){
            std::vector<int> row;for(int supplier:r){auto found=std::find(params.supplierIds.begin(),params.supplierIds.end(),supplier);
                row.push_back(static_cast<int>(found-params.supplierIds.begin())+1);}out.push_back(std::move(row));}
        return out;
    };
    params.commonInitialPlans.push_back(convert(warm,false));
    for(const auto& plan:initial){auto converted=convert(plan,true);
        if(converted.empty())continue;
        if(std::find(params.commonInitialPlans.begin(),params.commonInitialPlans.end(),converted)==params.commonInitialPlans.end())params.commonInitialPlans.push_back(std::move(converted));}
    Split split(&params);LocalSearch local(&params);Population population(&params,&split,&local);
    population.generatePopulation();Genetic genetic(&params,&split,&population,&local);
    genetic.run(control.no_improvement_restart);
}
}
