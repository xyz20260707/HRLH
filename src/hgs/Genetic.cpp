#include "Genetic.h"

void Genetic::run(int maxIterNonProd){
    int noImprovement=1;
    for(long long iteration=0;iteration<params->iterationLimit;++iteration){
        params->checkBudget();

        if(population->empty()) {
            population->generatePopulation();

            continue;
        }
        crossoverOX(offspring,population->getBinaryTournament(),population->getBinaryTournament());
        localSearch->run(offspring,params->penaltyCapacity,params->penaltyDuration);
        bool newBest=population->addIndividual(offspring,true);
        if(!offspring->isFeasible&&params->randomIndex(2)==0){
            localSearch->run(offspring,params->penaltyCapacity*10.,params->penaltyDuration*10.);
            if(offspring->isFeasible)newBest=population->addIndividual(offspring,false)||newBest;
        }

        noImprovement=newBest?1:noImprovement+1;
        if(iteration%100==0)population->managePenalties();

        if(noImprovement==maxIterNonProd){population->restart();noImprovement=1;}
    }
}

void Genetic::crossoverOX(Individual * result, const Individual * parent1, const Individual * parent2)
{
    params->checkBudget();
    if(params->nbClients==1){result->chromT=parent1->chromT;split->generalSplit(result,params->nbVehicles);return;}

	std::vector <bool> freqClient = std::vector <bool> (params->nbClients + 1, false);

	int start = static_cast<int>(params->randomIndex(params->nbClients));
	int end = static_cast<int>(params->randomIndex(params->nbClients));
	while (end == start) end = static_cast<int>(params->randomIndex(params->nbClients));

	int j = start;
	while (j % params->nbClients != (end + 1) % params->nbClients)
	{
		result->chromT[j % params->nbClients] = parent1->chromT[j % params->nbClients];
		freqClient[result->chromT[j % params->nbClients]] = true;
		j++;
	}

	for (int i = 1; i <= params->nbClients; i++)
	{
		int temp = parent2->chromT[(end + i) % params->nbClients];
		if (freqClient[temp] == false)
		{
			result->chromT[j % params->nbClients] = temp;
			j++;
		}
	}

	split->generalSplit(result, parent1->myCostSol.nbRoutes);
}

Genetic::Genetic(Params * params, Split * split, Population * population, LocalSearch * localSearch) : params(params), split(split), population(population), localSearch(localSearch)
{
	offspring = new Individual(params);
}

Genetic::~Genetic(void)
{
	delete offspring ;
}
