#include "search/Solution.h"

#include "search/ClientSegment.h"

#include <algorithm>
#include <cassert>
#include <iterator>

using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;

using pyvrp::search::Solution;

namespace
{
Cost insertCost(pyvrp::search::Route::Node *U,
                pyvrp::search::Route::Node *V,
                pyvrp::ProblemData const &data,
                pyvrp::CostEvaluator const &costEvaluator)
{
    assert(V->route() && U->isClient());

    auto *route = V->route();
    auto const &client = data.client(U->idx());

    Cost deltaCost
        = Cost(route->empty()) * route->fixedVehicleCost() - client.prize;

    costEvaluator.deltaCost<true>(
        deltaCost,
        pyvrp::search::Route::Proposal(
            route->before(V->pos()),
            pyvrp::search::ClientSegment(data, U->idx()),
            route->after(V->pos() + 1)));

    return deltaCost;
}

bool operator==(pyvrp::Route const &pyvrp, pyvrp::search::Route const &search)
{

    bool const simpleChecks = pyvrp.distance() == search.distance()
                              && pyvrp.duration() == search.duration()
                              && pyvrp.timeWarp() == search.timeWarp()
                              && pyvrp.vehicleType() == search.vehicleType()
                              && pyvrp.size() == search.size();

    if (!simpleChecks)
        return false;

    size_t idx = 0;
    for (auto const &activity : pyvrp)
        if (search[idx++]->activity() != activity)
            return false;

    return true;
}
}

Solution::Solution(ProblemData const &data) : data_(data)
{
    nodes.reserve(data.numClients());
    for (size_t loc = 0; loc != data.numClients(); ++loc)
        nodes.emplace_back(Activity::ActivityType::CLIENT, loc);

    routes.reserve(data.numVehicles());
    for (size_t vehType = 0; vehType != data.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data.vehicleType(vehType).numAvailable;
        for (size_t vehicle = 0; vehicle != numAvailable; ++vehicle)
            routes.emplace_back(data, vehType);
    }
}

void Solution::load(pyvrp::Solution const &solution)
{

    std::vector<size_t> vehicleOffset(data_.numVehicleTypes(), 0);
    for (size_t vehType = 1; vehType < data_.numVehicleTypes(); vehType++)
    {
        auto const prevAvail = data_.vehicleType(vehType - 1).numAvailable;
        vehicleOffset[vehType] = vehicleOffset[vehType - 1] + prevAvail;
    }

    for (auto const &solRoute : solution.routes())
    {

        auto const idx = vehicleOffset[solRoute.vehicleType()]++;
        auto &route = routes[idx];

        if (route == solRoute)
            continue;

        route.clear();

        route.reserve(solRoute.size());
        for (size_t idx = 1; idx != solRoute.size() - 1; ++idx)
        {
            auto const &activity = solRoute[idx];
            if (activity.isDepot())
            {
                Route::Node depot = activity;
                route.push_back(&depot);
            }
            else
            {
                assert(activity.isClient());
                route.push_back(&nodes[activity.idx()]);
            }
        }

        route.update();
    }

    size_t firstOfType = 0;
    for (size_t vehType = 0; vehType != data_.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data_.vehicleType(vehType).numAvailable;
        auto const firstOfNextType = firstOfType + numAvailable;
        for (size_t idx = vehicleOffset[vehType]; idx != firstOfNextType; ++idx)
            routes[idx].clear();

        firstOfType = firstOfNextType;
    }
}

pyvrp::Solution Solution::unload() const
{
    std::vector<pyvrp::Route> solRoutes;
    solRoutes.reserve(data_.numVehicles());

    for (auto const &route : routes)
    {
        if (route.empty())
            continue;

        std::vector<Activity> activities;
        activities.reserve(route.size());

        for (size_t idx = 1; idx != route.size() - 1; ++idx)
            activities.emplace_back(route[idx]->activity());

        solRoutes.emplace_back(
            data_, std::move(activities), route.vehicleType());
    }

    return {data_, std::move(solRoutes)};
}

bool Solution::insert(Route::Node *U,
                      SearchSpace const &searchSpace,
                      CostEvaluator const &costEvaluator,
                      bool required)
{
    assert(U->isClient());
    assert(size_t(std::distance(nodes.data(), U)) < nodes.size());

    Route::Node *UAfter = routes[0][0];
    auto bestCost = insertCost(U, UAfter, data_, costEvaluator);

    for (auto const vClient : searchSpace.neighboursOf(U->idx()))
    {
        auto *V = &nodes[vClient];

        if (!V->route())
            continue;

        auto const cost = insertCost(U, V, data_, costEvaluator);
        if (cost < bestCost)
        {
            bestCost = cost;
            UAfter = V;
        }
    }

    for (auto const &[vehType, offset] : searchSpace.vehTypeOrder())
    {
        auto const begin = routes.begin() + offset;
        auto const end = begin + data_.vehicleType(vehType).numAvailable;
        auto const pred = [](auto const &route) { return route.empty(); };
        auto empty = std::find_if(begin, end, pred);

        if (empty == end)
            continue;

        auto const cost = insertCost(U, (*empty)[0], data_, costEvaluator);
        if (cost < bestCost)
        {
            bestCost = cost;
            UAfter = (*empty)[0];
            break;
        }
    }

    if (required || bestCost < 0)
    {
        auto *route = UAfter->route();
        route->insert(UAfter->pos() + 1, U);
        return true;
    }

    return false;
}

template <>
pyvrp::Cost pyvrp::CostEvaluator::penalisedCost(
    pyvrp::search::Solution const &solution) const
{
    auto const &data = solution.data_;

    Cost cost = 0;
    for (size_t idx = 0; idx != data.numClients(); ++idx)
        if (!solution.nodes[idx].route())
            cost += data.client(idx).prize;

    for (auto const &route : solution.routes)
        cost += penalisedCost(route);

    return cost;
}
