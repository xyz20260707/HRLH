#include "Population.h"
#include <memory>

void Population::generatePopulation()
{
    int count=0;
    if(!params->commonSeedsConsumed){
        params->commonSeedsConsumed=true;
        for(const auto& routes:params->commonInitialPlans){
            params->checkBudget();Individual seed(params);seed.chromR=routes;
            seed.chromR.resize(params->nbVehicles);seed.chromT.clear();
            for(const auto& route:routes)seed.chromT.insert(seed.chromT.end(),route.begin(),route.end());
            seed.evaluateCompleteCost();addIndividual(&seed,false);++count;
        }
    }
    for(int i=count;i<params->initialPopulationLimit;++i){
        params->checkBudget();auto randomIndiv=std::make_unique<Individual>(params);
        split->generalSplit(randomIndiv.get(),params->nbVehicles);
        localSearch->run(randomIndiv.get(),params->penaltyCapacity,params->penaltyDuration);
        addIndividual(randomIndiv.get(),true);
        if(!randomIndiv->isFeasible&&params->randomIndex(2)==0){
            localSearch->run(randomIndiv.get(),params->penaltyCapacity*10.,params->penaltyDuration*10.);
            if(randomIndiv->isFeasible)addIndividual(randomIndiv.get(),false);
        }
    }
}

bool Population::addIndividual(const Individual * indiv, bool updateFeasible)
{
    params->checkBudget();

    using A=hrlh::routing::PackingAssessment;
    const auto assessment=indiv->isFeasible?params->submitAssessment(indiv->chromR):A::NotEvaluated;
	if (updateFeasible)
	{
		listFeasibilityLoad.push_back(indiv->myCostSol.capacityExcess == 0);
		listFeasibilityDuration.push_back(indiv->myCostSol.durationExcess == 0);
		listFeasibilityLoad.pop_front();
		listFeasibilityDuration.pop_front();
	}

    if(assessment==A::CertifiedInfeasible||assessment==A::RoutingInfeasible)return false;

	SubPopulation & subpop = (indiv->isFeasible) ? feasibleSubpopulation : infeasibleSubpopulation;

	auto pending = std::make_unique<Individual>(*indiv);
    Individual * myIndividual = pending.get();
    myIndividual->indivsPerProximity.clear();
	for (Individual * myIndividual2 : subpop)
	{
		double myDistance = myIndividual->brokenPairsDistance(myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	int place = (int)subpop.size();
	while (place > 0 && subpop[place - 1]->myCostSol.penalizedCost > indiv->myCostSol.penalizedCost - MY_EPSILON) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);
    pending.release();

	if ((int)subpop.size() > params->mu + params->lambda)
		while ((int)subpop.size() > params->mu)
			removeWorstBiasedFitness(subpop);

	if (indiv->isFeasible && indiv->myCostSol.penalizedCost < bestSolutionRestart.myCostSol.penalizedCost - MY_EPSILON)
	{
		bestSolutionRestart = *indiv;

		return true;
	}
	else
		return false;
}

void Population::updateBiasedFitnesses(SubPopulation & pop)
{

	std::vector <std::pair <double, int> > ranking;
	for (int i = 0 ; i < (int)pop.size(); i++)
		ranking.push_back({-pop[i]->averageBrokenPairsDistanceClosest(params->nbClose),i});
	std::sort(ranking.begin(), ranking.end());

	if (pop.size() == 1)
		pop[0]->biasedFitness = 0;
	else
	{
		for (int i = 0; i < (int)pop.size(); i++)
		{
			double divRank = (double)i / (double)(pop.size() - 1);
			double fitRank = (double)ranking[i].second / (double)(pop.size() - 1);
			if ((int)pop.size() <= params->nbElite)
				pop[ranking[i].second]->biasedFitness = fitRank;
			else
				pop[ranking[i].second]->biasedFitness = fitRank + (1.0 - (double)params->nbElite / (double)pop.size()) * divRank;
		}
	}
}

void Population::removeWorstBiasedFitness(SubPopulation & pop)
{
	updateBiasedFitnesses(pop);
	if (pop.size() <= 1) throw std::string("Eliminating the best individual: this should not occur in HGS");

	Individual * worstIndividual = NULL;
	int worstIndividualPosition = -1;
	bool isWorstIndividualClone = false;
	double worstIndividualBiasedFitness = -1.e30;
	for (int i = 1; i < (int)pop.size(); i++)
	{
		bool isClone = (pop[i]->averageBrokenPairsDistanceClosest(1) < MY_EPSILON);
		if ((isClone && !isWorstIndividualClone) || (isClone == isWorstIndividualClone && pop[i]->biasedFitness > worstIndividualBiasedFitness))
		{
			worstIndividualBiasedFitness = pop[i]->biasedFitness;
			isWorstIndividualClone = isClone;
			worstIndividualPosition = i;
			worstIndividual = pop[i];
		}
	}

	pop.erase(pop.begin() + worstIndividualPosition);
	for (Individual * myIndividual2 : pop) myIndividual2->removeProximity(worstIndividual);
	delete worstIndividual;
}

void Population::restart()
{
	params->checkBudget();

	std::unique_ptr<Individual> fallback;
	if (!empty()) {
		const Individual * parent = !feasibleSubpopulation.empty()
			? feasibleSubpopulation.front() : infeasibleSubpopulation.front();
		fallback = std::make_unique<Individual>(*parent);
		fallback->indivsPerProximity.clear();
	}
	for (Individual * indiv : feasibleSubpopulation) delete indiv ;
	for (Individual * indiv : infeasibleSubpopulation) delete indiv;
	feasibleSubpopulation.clear();
	infeasibleSubpopulation.clear();
	bestSolutionRestart = Individual();
	generatePopulation();
	if (empty() && fallback) {
		fallback->evaluateCompleteCost();
		addIndividual(fallback.get(), false);
	}
}

void Population::managePenalties()
{

	double fractionFeasibleLoad = (double)std::count(listFeasibilityLoad.begin(), listFeasibilityLoad.end(), true) / (double)listFeasibilityLoad.size();
	if (fractionFeasibleLoad < params->targetFeasible - 0.05 && params->penaltyCapacity < 100000.) params->penaltyCapacity = std::min<double>(params->penaltyCapacity * 1.2,100000.);
	else if (fractionFeasibleLoad > params->targetFeasible + 0.05 && params->penaltyCapacity > 0.1) params->penaltyCapacity = std::max<double>(params->penaltyCapacity * 0.85, 0.1);

	double fractionFeasibleDuration = (double)std::count(listFeasibilityDuration.begin(), listFeasibilityDuration.end(), true) / (double)listFeasibilityDuration.size();
	if (fractionFeasibleDuration < params->targetFeasible - 0.05 && params->penaltyDuration < 100000.)	params->penaltyDuration = std::min<double>(params->penaltyDuration * 1.2,100000.);
	else if (fractionFeasibleDuration > params->targetFeasible + 0.05 && params->penaltyDuration > 0.1) params->penaltyDuration = std::max<double>(params->penaltyDuration * 0.85, 0.1);

	for (int i = 0; i < (int)infeasibleSubpopulation.size(); i++)
		infeasibleSubpopulation[i]->myCostSol.penalizedCost = infeasibleSubpopulation[i]->myCostSol.transportCost
		+ params->penaltyCapacity * infeasibleSubpopulation[i]->myCostSol.capacityExcess
		+ params->penaltyDuration * infeasibleSubpopulation[i]->myCostSol.durationExcess;

	for (int i = 0; i < (int)infeasibleSubpopulation.size(); i++)
	{
		for (int j = 0; j < (int)infeasibleSubpopulation.size() - i - 1; j++)
		{
			if (infeasibleSubpopulation[j]->myCostSol.penalizedCost > infeasibleSubpopulation[j + 1]->myCostSol.penalizedCost + MY_EPSILON)
			{
				Individual * indiv = infeasibleSubpopulation[j];
				infeasibleSubpopulation[j] = infeasibleSubpopulation[j + 1];
				infeasibleSubpopulation[j + 1] = indiv;
			}
		}
	}
}

Individual * Population::getBinaryTournament ()
{
	Individual * individual1 ;
	Individual * individual2 ;

	updateBiasedFitnesses(feasibleSubpopulation);
	updateBiasedFitnesses(infeasibleSubpopulation);

	int place1 = params->randomIndex(feasibleSubpopulation.size() + infeasibleSubpopulation.size()) ;
	if (place1 >= (int)feasibleSubpopulation.size()) individual1 = infeasibleSubpopulation[place1 - feasibleSubpopulation.size()] ;
	else individual1 = feasibleSubpopulation[place1] ;

	int place2 = params->randomIndex(feasibleSubpopulation.size() + infeasibleSubpopulation.size()) ;
	if (place2 >= (int)feasibleSubpopulation.size()) individual2 = infeasibleSubpopulation[place2 - feasibleSubpopulation.size()] ;
	else individual2 = feasibleSubpopulation[place2] ;

	if (individual1->biasedFitness < individual2->biasedFitness) return individual1 ;
	else return individual2 ;
}



Population::Population(Params * params, Split * split, LocalSearch * localSearch) : params(params), split(split), localSearch(localSearch)
{
	listFeasibilityLoad = std::list<bool>(100, true);
	listFeasibilityDuration = std::list<bool>(100, true);

}

Population::~Population()
{
	for (int i = 0; i < (int)feasibleSubpopulation.size(); i++) delete feasibleSubpopulation[i];
	for (int i = 0; i < (int)infeasibleSubpopulation.size(); i++) delete infeasibleSubpopulation[i];
}
