#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "Individual.h"

struct Node ;

struct Route
{
	int cour;
	int nbCustomers;
	int whenLastModified;
	int whenLastTestedSWAPStar;
	Node * depot;
	double route_sort_key;
};

struct Node
{
	bool isDepot;
	int cour;
	int position;
	int whenLastTestedRI;
	Node * next;
	Node * prev;
	Route * route;
};

class LocalSearch
{

private:

	Params * params ;
	bool searchCompleted;
	int nbMoves;
	std::vector < int > orderNodes;
	std::vector < int > orderRoutes;
	std::set < int > emptyRoutes;
	int loopID;

	std::vector < Node > clients;
	std::vector < Node > depots;
	std::vector < Node > depotsEnd;
	std::vector < Route > routes;

	Node * nodeU ;
	Node * nodeX ;
    Node * nodeV ;
	Node * nodeY ;
	Route * routeU ;
	Route * routeV ;
	int nodeUIndex, nodeXIndex ;
	int nodeVPrevIndex, nodeVIndex, nodeYIndex ;
	double penaltyCapacityLS, penaltyDurationLS ;

	void setLocalVariablesRouteU();
	void setLocalVariablesRouteV();

	bool move1 ();
	bool move2 ();
	bool move3 ();

	bool move4 ();
	bool move5 ();
	bool move6 ();

	bool move7 ();
	bool move8 ();
	bool move9 ();

	bool swapStar();

	static void insertNode(Node * U, Node * V);
	static void swapNode(Node * U, Node * V) ;
	void updateRouteData(Route * myRoute);
	std::vector<int> routeVector(Route* r) const;
	void installRoute(Route* r,const std::vector<int>& clients);
	double moveDelta(int kind) const;

	public:

	void run(Individual * indiv, double penaltyCapacityLS, double penaltyDurationLS);

	void loadIndividual(Individual * indiv);

	void exportIndividual(Individual * indiv);

	LocalSearch(Params * params);
};

#endif
