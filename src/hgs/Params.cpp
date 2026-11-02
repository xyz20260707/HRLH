#include "Params.h"
#include <limits>
#include <numeric>

Params::Params(const hrlh::routing::Problem& problem,
               const hrlh::routing::SearchCallbacks& cb, std::uint64_t seed)
 : routingProblem(&problem), callbacks(&cb), rng(seed) {
    supplierIds=problem.instance.suppliers;
    nbClients=static_cast<int>(supplierIds.size());
    nbVehicles=std::min(nbClients,static_cast<int>(problem.instance.vehicles.size()));
    if(nbClients<1||nbVehicles<1)throw std::invalid_argument("HGS needs suppliers and vehicles");
    const double maxDemand=problem.max_normalized_demand;
    const double maxDist=problem.max_arc_cost;
    correlatedVertices.resize(nbClients+1);std::vector<std::set<int>> sym(nbClients+1);
    for(int i=1;i<=nbClients;++i){
        std::vector<std::pair<double,int>> order;
        for(int j=1;j<=nbClients;++j)if(i!=j)order.emplace_back(problem.matrix_proximity(supplierIds[i-1],supplierIds[j-1]),j);
        std::sort(order.begin(),order.end());
        for(int k=0;k<std::min(nbGranular,nbClients-1);++k){
            sym[i].insert(order[k].second);sym[order[k].second].insert(i);
        }
    }
    for(int i=1;i<=nbClients;++i)correlatedVertices[i].assign(sym[i].begin(),sym[i].end());
    penaltyDuration=1.;
    penaltyCapacity=std::max(.1,std::min(1000.,maxDist/std::max(maxDemand,1.e-12)));
}
void Params::checkBudget()const {if(callbacks->stop)callbacks->stop();}
std::size_t Params::randomIndex(std::size_t n){return hrlh::routing::uniform_index(rng,n);}
std::vector<int> Params::externalRoute(const std::vector<int>& route)const {
    std::vector<int> out;out.reserve(route.size());
    for(int client:route){if(client<1||client>nbClients)throw std::invalid_argument("invalid HGS client");out.push_back(supplierIds[client-1]);}
    return out;
}
hrlh::routing::RouteMetrics Params::metrics(const std::vector<int>& route)const {
    return routingProblem->evaluate_route(externalRoute(route));
}
double Params::routeObjective(const std::vector<int>& route,double pc,double pt)const {
    const auto m=metrics(route);return m.cost+pc*m.capacity_excess+pt*m.time_excess;
}

hrlh::routing::PackingAssessment Params::submitAssessment(
    const std::vector<std::vector<int>>& routes){
    using A=hrlh::routing::PackingAssessment;
    checkBudget();hrlh::RoutePlan external;
    for(const auto& r:routes)if(!r.empty())external.push_back(externalRoute(r));
    const auto m=routingProblem->evaluate(external);
    if(!m.routing_feasible)return A::RoutingInfeasible;

    if(!callbacks->evaluate_integrated)return A::NotEvaluated;
    const auto outcome=callbacks->evaluate_integrated(external);


    return outcome;
}
