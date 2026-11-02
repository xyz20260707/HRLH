#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H

#include "Params.h"

class Individual;

struct CostSol
{
	double penalizedCost;
	int nbRoutes;
	double transportCost;
	double capacityExcess;
	double durationExcess;
	CostSol() { penalizedCost = 0.; nbRoutes = 0; transportCost = 0.; capacityExcess = 0.; durationExcess = 0.; }
};

class Individual
{
public:

  Params * params ;
  CostSol myCostSol;
  std::vector < int > chromT ;
  std::vector < std::vector <int> > chromR ;
  std::vector < int > successors ;
  std::vector < int > predecessors ;
  std::multiset < std::pair < double, Individual* > > indivsPerProximity ;
  bool isFeasible;
  double biasedFitness;

  void evaluateCompleteCost();

  void removeProximity(Individual * indiv);

  double brokenPairsDistance(Individual * indiv2);

  double averageBrokenPairsDistanceClosest(int nbClosest) ;

  Individual(Params * params);

  Individual();
};
#endif
