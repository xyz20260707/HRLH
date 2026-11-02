#include "LocalSearch.h"
#include "PackingAwareMoves.h"
#include <limits>

void LocalSearch::run(Individual * indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
	loadIndividual(indiv);
	this->penaltyCapacityLS = penaltyCapacityLS;
	this->penaltyDurationLS = penaltyDurationLS;
    for(auto& r:routes)updateRouteData(&r);

	std::shuffle(orderNodes.begin(), orderNodes.end(), params->rng);
	std::shuffle(orderRoutes.begin(), orderRoutes.end(), params->rng);
	for (int i = 1; i <= params->nbClients; i++)
		if (params->randomIndex(params->nbGranular) == 0)
			std::shuffle(params->correlatedVertices[i].begin(), params->correlatedVertices[i].end(), params->rng);

	searchCompleted = false;
	for (loopID = 0; !searchCompleted; loopID++)
	{
		if (loopID > 1)
			searchCompleted = true;

		for (int posU = 0; posU < params->nbClients; posU++)
		{
			params->checkBudget();
			nodeU = &clients[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params->correlatedVertices[nodeU->cour].size(); posV++)
			{
				nodeV = &clients[params->correlatedVertices[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)
				{

					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (move1()) continue;
					if (move2()) continue;
					if (move3()) continue;
					if (nodeUIndex <= nodeVIndex && move4()) continue;
					if (move5()) continue;
					if (nodeUIndex <= nodeVIndex && move6()) continue;
					if (routeU == routeV && move7()) continue;
					if (routeU != routeV && move8()) continue;
					if (routeU != routeV && move9()) continue;

					if (nodeV->prev->isDepot)
					{
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						if (move1()) continue;
						if (move2()) continue;
						if (move3()) continue;
						if (routeU != routeV && move8()) continue;
						if (routeU != routeV && move9()) continue;
					}
				}
			}

			if (loopID > 0 && !emptyRoutes.empty())
			{
				nodeV = routes[*emptyRoutes.begin()].depot;
				setLocalVariablesRouteU();
				setLocalVariablesRouteV();
				if (move1()) continue;
				if (move2()) continue;
				if (move3()) continue;
				if (move9()) continue;
			}
		}

		for (int rU = 0; rU < params->nbVehicles; rU++)
		{
			routeU = &routes[orderRoutes[rU]];
			int lastTestSWAPStarRouteU = routeU->whenLastTestedSWAPStar;
			routeU->whenLastTestedSWAPStar = nbMoves;
			for (int rV = 0; rV < params->nbVehicles; rV++)
			{
				routeV = &routes[orderRoutes[rV]];
				if (routeU->nbCustomers > 0 && routeV->nbCustomers > 0 && routeU->cour < routeV->cour && (loopID == 0 || std::max<int>(routeU->whenLastModified, routeV->whenLastModified) > lastTestSWAPStarRouteU))
					swapStar();
			}
		}
	}

	exportIndividual(indiv);
}

void LocalSearch::setLocalVariablesRouteU()
{
	routeU = nodeU->route;
	nodeX = nodeU->next;
	nodeUIndex = nodeU->cour;
	nodeXIndex = nodeX->cour;
}

void LocalSearch::setLocalVariablesRouteV()
{
	routeV = nodeV->route;
	nodeY = nodeV->next;
	nodeVIndex = nodeV->cour;
	nodeVPrevIndex = nodeV->prev->cour;
	nodeYIndex = nodeY->cour;
}

bool LocalSearch::move1()
{
	const double cost = moveDelta(1);
	if (cost > -MY_EPSILON) return false;
	if (nodeUIndex == nodeYIndex) return false;

	insertNode(nodeU, nodeV);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move2()
{
	const double cost = moveDelta(2);
	if (cost > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot) return false;

	insertNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move3()
{
	const double cost = moveDelta(3);
	if (cost > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot) return false;

	insertNode(nodeX, nodeV);
	insertNode(nodeU, nodeX);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move4()
{
	const double cost = moveDelta(4);
	if (cost > -MY_EPSILON) return false;
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;

	swapNode(nodeU, nodeV);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move5()
{
	const double cost = moveDelta(5);
	if (cost > -MY_EPSILON) return false;
	if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY || nodeX->isDepot) return false;

	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move6()
{
	const double cost = moveDelta(6);
	if (cost > -MY_EPSILON) return false;
	if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

	swapNode(nodeU, nodeV);
	swapNode(nodeX, nodeY);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move7()
{
	const double cost = moveDelta(7);
	if (cost > -MY_EPSILON) return false;
	if (nodeU->next == nodeV) return false;

	Node * nodeNum = nodeX->next;
	nodeX->prev = nodeNum;
	nodeX->next = nodeY;

	while (nodeNum != nodeV)
	{
		Node * temp = nodeNum->next;
		nodeNum->next = nodeNum->prev;
		nodeNum->prev = temp;
		nodeNum = temp;
	}

	nodeV->next = nodeV->prev;
	nodeV->prev = nodeU;
	nodeU->next = nodeV;
	nodeY->prev = nodeX;

	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	return true;
}

bool LocalSearch::move8()
{
	const double cost = moveDelta(8);
	if (cost > -MY_EPSILON) return false;

	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = routeU->depot->prev;
	Node * depotVFin = routeV->depot->prev;
	Node * depotVSuiv = depotV->next;

	Node * temp;
	Node * xx = nodeX;
	Node * vv = nodeV;

	while (!xx->isDepot)
	{
		temp = xx->next;
		xx->next = xx->prev;
		xx->prev = temp;
		xx->route = routeV;
		xx = temp;
	}

	while (!vv->isDepot)
	{
		temp = vv->prev;
		vv->prev = vv->next;
		vv->next = temp;
		vv->route = routeU;
		vv = temp;
	}

	nodeU->next = nodeV;
	nodeV->prev = nodeU;
	nodeX->next = nodeY;
	nodeY->prev = nodeX;

	if (nodeX->isDepot)
	{
		depotUFin->next = depotU;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;
		depotV->next = nodeY;
		nodeY->prev = depotV;
	}
	else if (nodeV->isDepot)
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;
		depotV->prev = depotVFin;
		depotUFin->prev = nodeU;
		nodeU->next = depotUFin;
	}
	else
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;
	}

	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

bool LocalSearch::move9()
{
	const double cost = moveDelta(9);
	if (cost > -MY_EPSILON) return false;

	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = depotU->prev;
	Node * depotVFin = depotV->prev;
	Node * depotUpred = depotUFin->prev;

	Node * count = nodeY;
	while (!count->isDepot)
	{
		count->route = routeU;
		count = count->next;
	}

	count = nodeX;
	while (!count->isDepot)
	{
		count->route = routeV;
		count = count->next;
	}

	nodeU->next = nodeY;
	nodeY->prev = nodeU;
	nodeV->next = nodeX;
	nodeX->prev = nodeV;

	if (nodeX->isDepot)
	{
		depotUFin->prev = depotVFin->prev;
		depotUFin->prev->next = depotUFin;
		nodeV->next = depotVFin;
		depotVFin->prev = nodeV;
	}
	else
	{
		depotUFin->prev = depotVFin->prev;
		depotUFin->prev->next = depotUFin;
		depotVFin->prev = depotUpred;
		depotVFin->prev->next = depotVFin;
	}

	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

bool LocalSearch::swapStar()
{
    params->checkBudget();
    const auto best=hrlh_hgs_moves::best_swap_star(*params,routeVector(routeU),routeVector(routeV),
                                             penaltyCapacityLS,penaltyDurationLS);
    if(best.delta>-MY_EPSILON)return false;
    installRoute(routeU,best.first);installRoute(routeV,best.second);
    ++nbMoves;searchCompleted=false;updateRouteData(routeU);updateRouteData(routeV);
    return true;
}

std::vector<int> LocalSearch::routeVector(Route* r) const {
    std::vector<int> v;Node* n=r->depot->next;
    for(int visited=0;!n->isDepot;n=n->next){
        if(++visited>params->nbClients)throw std::runtime_error("HGS linked route cycle");
        v.push_back(n->cour);
    }return v;
}
void LocalSearch::installRoute(Route* r,const std::vector<int>& v){
    Node* previous=r->depot;
    for(int id:v){Node* n=&clients[id];previous->next=n;n->prev=previous;n->route=r;previous=n;}
    Node* end=&depotsEnd[r->cour];previous->next=end;end->prev=previous;end->next=r->depot;r->depot->prev=end;
}
double LocalSearch::moveDelta(int kind) const {
    const auto a=routeVector(routeU),b=routeVector(routeV);const bool same=routeU==routeV;
    const auto preview=hrlh_hgs_moves::preview(kind,a,b,nodeU->position-1,nodeV->isDepot?-1:nodeV->position-1,same);
    if(!preview.valid)return std::numeric_limits<double>::infinity();
    return params->routeObjective(preview.first,penaltyCapacityLS,penaltyDurationLS)
       +(same?0:params->routeObjective(preview.second,penaltyCapacityLS,penaltyDurationLS))
       -params->routeObjective(a,penaltyCapacityLS,penaltyDurationLS)
       -(same?0:params->routeObjective(b,penaltyCapacityLS,penaltyDurationLS));
}

void LocalSearch::insertNode(Node * U, Node * V)
{
	U->prev->next = U->next;
	U->next->prev = U->prev;
	V->next->prev = U;
	U->prev = V;
	U->next = V->next;
	V->next = U;
	U->route = V->route;
}

void LocalSearch::swapNode(Node * U, Node * V)
{
	Node * myVPred = V->prev;
	Node * myVSuiv = V->next;
	Node * myUPred = U->prev;
	Node * myUSuiv = U->next;
	Route * myRouteU = U->route;
	Route * myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}

void LocalSearch::updateRouteData(Route* r){
    const auto v=routeVector(r);
    r->nbCustomers=static_cast<int>(v.size());
    r->whenLastModified=nbMoves;

    r->route_sort_key=v.empty()?1.e30:static_cast<double>(r->cour);
    int position=0;r->depot->position=0;
    for(Node* n=r->depot->next;!n->isDepot;n=n->next)n->position=++position;
    if(v.empty())emptyRoutes.insert(r->cour);else emptyRoutes.erase(r->cour);
}

void LocalSearch::loadIndividual(Individual * indiv)
{
    penaltyCapacityLS=params->penaltyCapacity;penaltyDurationLS=params->penaltyDuration;
	emptyRoutes.clear();
	nbMoves = 0;
	for (int r = 0; r < params->nbVehicles; r++)
	{
		Node * myDepot = &depots[r];
		Node * myDepotFin = &depotsEnd[r];
		Route * myRoute = &routes[r];
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;
		if (!indiv->chromR[r].empty())
		{
			Node * myClient = &clients[indiv->chromR[r][0]];
			myClient->route = myRoute;
			myClient->prev = myDepot;
			myDepot->next = myClient;
			for (int i = 1; i < (int)indiv->chromR[r].size(); i++)
			{
				Node * myClientPred = myClient;
				myClient = &clients[indiv->chromR[r][i]];
				myClient->prev = myClientPred;
				myClientPred->next = myClient;
				myClient->route = myRoute;
			}
			myClient->next = myDepotFin;
			myDepotFin->prev = myClient;
		}
		else
		{
			myDepot->next = myDepotFin;
			myDepotFin->prev = myDepot;
		}
		updateRouteData(&routes[r]);
		routes[r].whenLastTestedSWAPStar = -1;
	}

	for (int i = 1; i <= params->nbClients; i++)
		clients[i].whenLastTestedRI = -1;
}

void LocalSearch::exportIndividual(Individual * indiv)
{
	std::vector < std::pair <double, int> > route_order_keys ;
	for (int r = 0; r < params->nbVehicles; r++)
		route_order_keys.push_back(std::pair <double, int>(routes[r].route_sort_key, r));
	std::sort(route_order_keys.begin(), route_order_keys.end());

	int pos = 0;
	for (int r = 0; r < params->nbVehicles; r++)
	{
		indiv->chromR[r].clear();
		Node * node = depots[route_order_keys[r].second].next;
		while (!node->isDepot)
		{
			indiv->chromT[pos] = node->cour;
			indiv->chromR[r].push_back(node->cour);
			node = node->next;
			pos++;
		}
	}

	indiv->evaluateCompleteCost();
}

LocalSearch::LocalSearch(Params * params) : params (params)
{
	clients = std::vector < Node >(params->nbClients + 1);
	routes = std::vector < Route >(params->nbVehicles);
	depots = std::vector < Node >(params->nbVehicles);
	depotsEnd = std::vector < Node >(params->nbVehicles);

	for (int i = 0; i <= params->nbClients; i++)
	{
		clients[i].cour = i;
		clients[i].isDepot = false;
	}
	for (int i = 0; i < params->nbVehicles; i++)
	{
		routes[i].cour = i;
		routes[i].depot = &depots[i];
		depots[i].cour = 0;
		depots[i].isDepot = true;
		depots[i].route = &routes[i];
		depotsEnd[i].cour = 0;
		depotsEnd[i].isDepot = true;
		depotsEnd[i].route = &routes[i];
	}
	for (int i = 1 ; i <= params->nbClients ; i++) orderNodes.push_back(i);
	for (int r = 0 ; r < params->nbVehicles ; r++) orderRoutes.push_back(r);
}

