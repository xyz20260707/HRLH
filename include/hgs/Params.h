#ifndef PARAMS_H
#define PARAMS_H

#include "routing_problem.hpp"
#include <string>
#include <vector>
#include <list>
#include <set>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <time.h>
#include <climits>
#include <algorithm>
#include <unordered_set>
#include <limits>
#define MY_EPSILON 0.00001

class Params
{
public:

	int nbGranular			= 20;
	int mu					= 25;
	int lambda				= 40;
	int nbElite				= 4;
	int nbClose				= 5;
	double targetFeasible   = 0.2;

	double penaltyCapacity;
	double penaltyDuration;

	int nbClients ;
	int nbVehicles ;
	std::vector < std::vector < int > > correlatedVertices;

	const hrlh::routing::Problem* routingProblem;
	const hrlh::routing::SearchCallbacks* callbacks;
	std::mt19937_64 rng;
	std::vector<int> supplierIds;
	std::vector<std::vector<std::vector<int>>> commonInitialPlans;
	long long iterationLimit = std::numeric_limits<long long>::max();
	int initialPopulationLimit = 100;
	bool commonSeedsConsumed = false;
	void checkBudget() const;
	std::size_t randomIndex(std::size_t n);
	std::vector<int> externalRoute(const std::vector<int>& route) const;
	hrlh::routing::RouteMetrics metrics(const std::vector<int>& route) const;
	double routeObjective(const std::vector<int>& route, double pc, double pt) const;

    hrlh::routing::PackingAssessment submitAssessment(
        const std::vector<std::vector<int>>& routes);

	Params(const hrlh::routing::Problem& problem,
	       const hrlh::routing::SearchCallbacks& cb, std::uint64_t seed);
};
#endif
