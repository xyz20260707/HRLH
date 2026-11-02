#ifndef GENETIC_H
#define GENETIC_H

#include "Population.h"
#include "Individual.h"

class Genetic
{
private:

	Params * params;
	Split * split;
	Population * population;
	LocalSearch * localSearch;
	Individual * offspring;

	void crossoverOX(Individual * result, const Individual * parent1, const Individual * parent2);

public:

    void run(int maxIterNonProd) ;

	Genetic(Params * params, Split * split, Population * population, LocalSearch * localSearch);

	~Genetic(void);
};

#endif
