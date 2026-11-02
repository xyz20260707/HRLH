#include "Split.h"
#include <limits>

void Split::generalSplit(Individual* indiv,int nbMaxVehicles){
    params->checkBudget();
    const int n=params->nbClients;

    int capacityVehicleLowerBound=1;
    const auto& problem=*params->routingProblem;
    if(!problem.integer_capacities.empty()){
        const auto capacity=problem.integer_capacities.front();
        if(capacity<=0)throw std::invalid_argument("invalid HGS Split capacity");
        std::int64_t total=0;
        for(const auto& demand:problem.integer_demands){
            const auto quantity=demand.at(0);
            if(quantity<0||quantity>std::numeric_limits<std::int64_t>::max()-total)
                throw std::overflow_error("HGS Split total demand overflow");
            total+=quantity;
        }
        const auto required=total/capacity+(total%capacity!=0);
        capacityVehicleLowerBound=static_cast<int>(std::min<std::int64_t>(params->nbVehicles,required));
    }
    maxVehicles=std::min(params->nbVehicles,std::max({1,nbMaxVehicles,capacityVehicleLowerBound}));
    if(static_cast<int>(indiv->chromT.size())!=n)throw std::invalid_argument("HGS giant tour size");
    std::vector<int> seen(n+1,0);
    for(int v:indiv->chromT)if(v<1||v>n||++seen[v]!=1)throw std::invalid_argument("HGS giant tour duplicate");
    const double inf=std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> interval(n,std::vector<double>(n+1,inf));
    for(int i=0;i<n;++i){
        params->checkBudget();std::vector<int> route;
        for(int j=i+1;j<=n;++j){
            route.push_back(indiv->chromT[j-1]);
            interval[i][j]=params->routeObjective(route,params->penaltyCapacity,params->penaltyDuration);
        }
    }
    potential.assign(maxVehicles+1,std::vector<double>(n+1,inf));
    pred.assign(maxVehicles+1,std::vector<int>(n+1,-1));potential[0][0]=0;
    for(int k=1;k<=maxVehicles;++k){
        params->checkBudget();
        for(int j=k;j<=n;++j)for(int i=k-1;i<j;++i){
            double cost=potential[k-1][i]+interval[i][j];
            if(cost<potential[k][j]){potential[k][j]=cost;pred[k][j]=i;}
        }
    }
    int count=1;
    for(int k=2;k<=maxVehicles;++k)if(potential[k][n]<potential[count][n])count=k;
    if(!std::isfinite(potential[count][n]))throw std::runtime_error("no finite HGS split");
    indiv->chromR.assign(params->nbVehicles,{});
    int end=n;
    for(int k=count;k>0;--k){
        int begin=pred[k][end];
        if(begin<0||begin>=end)throw std::runtime_error("invalid HGS split predecessor");
        indiv->chromR[k-1].assign(indiv->chromT.begin()+begin,indiv->chromT.begin()+end);
        end=begin;
    }
    if(end)throw std::runtime_error("incomplete HGS split backtrace");
    indiv->evaluateCompleteCost();
}
Split::Split(Params* p):params(p),maxVehicles(p->nbVehicles){}
