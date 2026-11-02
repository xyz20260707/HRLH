#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"
#include "LocalSearch.h"
#include "Split.h"

typedef std::vector <Individual*> SubPopulation ;

class Population
{
   private:

   Params * params ;
   Split * split;
   LocalSearch * localSearch;
   SubPopulation feasibleSubpopulation;
   SubPopulation infeasibleSubpopulation;
   std::list <bool> listFeasibilityLoad ;
   std::list <bool> listFeasibilityDuration ;
   Individual bestSolutionRestart;

   void updateBiasedFitnesses(SubPopulation & pop);

   void removeWorstBiasedFitness(SubPopulation & subpop);

   public:

   void generatePopulation();

   bool addIndividual (const Individual * indiv, bool updateFeasible);

   void restart();

   bool empty() const { return feasibleSubpopulation.empty() && infeasibleSubpopulation.empty(); }

   void managePenalties();

   Individual * getBinaryTournament();



   Population(Params * params, Split * split, LocalSearch * localSearch);

   ~Population();
};

#endif
