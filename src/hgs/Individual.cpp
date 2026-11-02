#include "Individual.h"

void Individual::evaluateCompleteCost()
{
    myCostSol=CostSol();std::vector<int> visits(params->nbClients+1,0);
    for(const auto& r:chromR)if(!r.empty()){
        const auto m=params->metrics(r);
        myCostSol.transportCost+=m.cost;
        myCostSol.capacityExcess+=m.capacity_excess;myCostSol.durationExcess+=m.time_excess;
        ++myCostSol.nbRoutes;
        for(std::size_t k=0;k<r.size();++k){
            if(r[k]<1||r[k]>params->nbClients||++visits[r[k]]!=1)throw std::invalid_argument("HGS duplicate/unknown client");
            predecessors[r[k]]=k?r[k-1]:0;successors[r[k]]=k+1<r.size()?r[k+1]:0;
        }
    }
    for(int i=1;i<=params->nbClients;++i)if(visits[i]!=1)throw std::invalid_argument("HGS missing client");
    if(myCostSol.nbRoutes>params->nbVehicles)throw std::invalid_argument("HGS fleet exceeded");
    myCostSol.penalizedCost=myCostSol.transportCost+myCostSol.capacityExcess*params->penaltyCapacity+myCostSol.durationExcess*params->penaltyDuration;
    isFeasible=myCostSol.capacityExcess==0&&myCostSol.durationExcess==0;
}

void Individual::removeProximity(Individual * indiv)
{
	auto it = indivsPerProximity.begin();
	while (it->second != indiv) ++it;
	indivsPerProximity.erase(it);
}

double Individual::brokenPairsDistance(Individual * indiv2)
{
	int differences = 0;
	for (int j = 1; j <= params->nbClients; j++)
	{
		if (successors[j] != indiv2->successors[j] && successors[j] != indiv2->predecessors[j]) differences++;
		if (predecessors[j] == 0 && indiv2->predecessors[j] != 0 && indiv2->successors[j] != 0) differences++;
	}
	return (double)differences/(double)params->nbClients;
}

double Individual::averageBrokenPairsDistanceClosest(int nbClosest)
{
	double result = 0 ;
	int maxSize = std::min<int>(nbClosest, indivsPerProximity.size());
	auto it = indivsPerProximity.begin();
	for (int i=0 ; i < maxSize; i++)
	{
		result += it->first ;
		++it ;
	}
	return maxSize ? result/(double)maxSize : 0.;
}

Individual::Individual(Params * params) : params(params)
{
	successors = std::vector <int>(params->nbClients + 1);
	predecessors = std::vector <int>(params->nbClients + 1);
	chromR = std::vector < std::vector <int> >(params->nbVehicles);
	chromT = std::vector <int>(params->nbClients);
	for (int i = 0; i < params->nbClients; i++) chromT[i] = i + 1;
	std::shuffle(chromT.begin(), chromT.end(), params->rng);
}

Individual::Individual()
{
	myCostSol.penalizedCost = 1.e30;
}
