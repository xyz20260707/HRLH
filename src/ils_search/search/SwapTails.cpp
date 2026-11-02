#include "search/SwapTails.h"

#include "search/Route.h"

#include <cassert>

using pyvrp::search::SwapTails;

namespace
{
bool onLastTrip(pyvrp::search::Route::Node *node)
{
    auto const *route = node->route();
    return node->trip() + 1 == route->numTrips();
}
}

std::pair<pyvrp::Cost, bool> SwapTails::evaluate(
    Route::Node *U, Route::Node *V, CostEvaluator const &costEvaluator)
{
    assert(!U->isEndDepot() && !U->isReloadDepot());
    assert(!V->isEndDepot() && !V->isReloadDepot());

    auto const *uRoute = U->route();
    auto const *vRoute = V->route();

    if (!uRoute || uRoute == vRoute)
        return std::make_pair(0, false);

    if (uRoute > vRoute && !uRoute->empty() && !vRoute->empty())
        return std::make_pair(0, false);

    if (!onLastTrip(U) || !onLastTrip(V))

        return std::make_pair(0, false);

    Cost deltaCost = 0;

    if (uRoute->empty() && !n(V)->isEndDepot())
        deltaCost += uRoute->fixedVehicleCost();

    if (vRoute->empty() && !n(U)->isEndDepot())
        deltaCost += vRoute->fixedVehicleCost();

    if (!uRoute->empty() && U->isStartDepot() && n(V)->isEndDepot())
        deltaCost -= uRoute->fixedVehicleCost();

    if (!vRoute->empty() && V->isStartDepot() && n(U)->isEndDepot())
        deltaCost -= vRoute->fixedVehicleCost();

    if (!n(U)->isEndDepot() && !n(V)->isEndDepot())
    {
        auto const uProposal
            = Route::Proposal(uRoute->before(U->pos()),
                              vRoute->between(V->pos() + 1, vRoute->size() - 2),
                              uRoute->at(uRoute->size() - 1));

        auto const vProposal
            = Route::Proposal(vRoute->before(V->pos()),
                              uRoute->between(U->pos() + 1, uRoute->size() - 2),
                              vRoute->at(vRoute->size() - 1));

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }
    else if (!n(U)->isEndDepot() && n(V)->isEndDepot())
    {
        auto const uProposal = Route::Proposal(uRoute->before(U->pos()),
                                               uRoute->at(uRoute->size() - 1));

        auto const vProposal
            = Route::Proposal(vRoute->before(V->pos()),
                              uRoute->between(U->pos() + 1, uRoute->size() - 2),
                              vRoute->at(vRoute->size() - 1));

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }
    else if (n(U)->isEndDepot() && !n(V)->isEndDepot())
    {
        auto const uProposal
            = Route::Proposal(uRoute->before(U->pos()),
                              vRoute->between(V->pos() + 1, vRoute->size() - 2),
                              uRoute->at(uRoute->size() - 1));

        auto const vProposal = Route::Proposal(vRoute->before(V->pos()),
                                               vRoute->at(vRoute->size() - 1));

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }

    return std::make_pair(deltaCost, deltaCost < 0);
}

void SwapTails::apply(Route::Node *U, Route::Node *V) const
{
    auto *nU = n(U);
    auto *nV = n(V);

    auto insertIdx = U->pos() + 1;
    while (!nV->isEndDepot())
    {
        auto *node = nV;
        nV = n(nV);
        V->route()->remove(node->pos());
        U->route()->insert(insertIdx++, node);
    }

    insertIdx = V->pos() + 1;
    while (!nU->isEndDepot())
    {
        auto *node = nU;
        nU = n(nU);
        U->route()->remove(node->pos());
        V->route()->insert(insertIdx++, node);
    }
}

template <> bool pyvrp::search::supports<SwapTails>(ProblemData const &data)
{

    return data.numVehicles() > 1;
}
