#include "case_protocol.h"

#include "Client.h"
#include "ClientGroup.h"
#include "Depot.h"
#include "Location.h"
#include "Matrix.h"
#include "Measure.h"
#include "VehicleType.h"

#include <limits>
#include <stdexcept>
#include <utility>

using namespace pyvrp;

namespace hrlh_ils_search
{
namespace
{

template <typename Measure>
std::vector<Measure> measures(std::vector<Integer> const &values)
{
    std::vector<Measure> result;
    result.reserve(values.size());
    for (auto const value : values)
        result.emplace_back(value);
    return result;
}

template <typename Measure>
std::vector<Matrix<Measure>> matrices(std::vector<MatrixSpec> const &specs,
                                     size_t numLocations)
{
    std::vector<Matrix<Measure>> result;
    result.reserve(specs.size());
    for (auto const &spec : specs)
    {
        if (spec.values.size() != numLocations * numLocations)
            throw std::runtime_error("Matrix does not match location count.");
        result.emplace_back(measures<Measure>(spec.values),
                            numLocations,
                            numLocations);
    }
    return result;
}
}

std::vector<std::string> defaultOperators()
{
    return {"Exchange10",
            "Exchange20",
            "Exchange11",
            "Exchange21",
            "Exchange22",
            "SwapTails"};
}

ProblemData buildProblem(CaseSpec const &spec)
{
    std::vector<Location> locations;
    locations.reserve(spec.locations.size());
    for (auto const &location : spec.locations)
        locations.emplace_back(Coordinate(location.x), Coordinate(location.y));

    std::vector<Client> clients;
    clients.reserve(spec.clients.size());
    for (auto const &client : spec.clients)
        clients.emplace_back(client.location,
                             measures<Load>(client.delivery),
                             measures<Load>(client.pickup),
                             Duration(client.serviceDuration),
                             Duration(client.twEarly),
                             Duration(client.twLate),
                             Duration(client.releaseTime),
                             Duration(client.latestReturn),
                             Cost(client.prize),
                             client.required,
                             client.group);

    std::vector<Depot> depots;
    depots.reserve(spec.depots.size());
    for (auto const &depot : spec.depots)
        depots.emplace_back(depot.location,
                            Duration(depot.twEarly),
                            Duration(depot.twLate),
                            Duration(depot.serviceDuration));

    std::vector<ClientGroup> groups;
    groups.reserve(spec.groups.size());
    for (auto const &group : spec.groups)
        groups.emplace_back(group.clients, group.required);

    std::vector<VehicleType> vehicleTypes;
    vehicleTypes.reserve(spec.vehicleTypes.size());
    for (auto const &vehicle : spec.vehicleTypes)
        vehicleTypes.emplace_back(vehicle.numAvailable,
                                  measures<Load>(vehicle.capacity),
                                  vehicle.startDepot,
                                  vehicle.endDepot,
                                  Cost(vehicle.fixedCost),
                                  Duration(vehicle.twEarly),
                                  Duration(vehicle.twLate),
                                  Duration(vehicle.shiftDuration),
                                  Distance(vehicle.maxDistance),
                                  Cost(vehicle.unitDistanceCost),
                                  Cost(vehicle.unitDurationCost),
                                  vehicle.profile,
                                  vehicle.startLate
                                      ? std::optional<Duration>(
                                            Duration(*vehicle.startLate))
                                      : std::nullopt,
                                  measures<Load>(vehicle.initialLoad),
                                  vehicle.reloadDepots,
                                  vehicle.maxReloads,
                                  Duration(vehicle.maxOvertime),
                                  Cost(vehicle.unitOvertimeCost));

    return ProblemData(locations,
                       clients,
                       depots,
                       vehicleTypes,
                       matrices<Distance>(spec.distanceMatrices,
                                          locations.size()),
                       matrices<Duration>(spec.durationMatrices,
                                          locations.size()),
                       groups);
}

std::vector<Route> buildRoutes(ProblemData const &data,
                               std::vector<RouteSpec> const &specs)
{
    std::vector<Route> routes;
    routes.reserve(specs.size());
    for (auto const &spec : specs)
    {
        std::vector<Activity> activities;
        activities.reserve(spec.activities.size());
        for (auto const &activity : spec.activities)
            activities.emplace_back(activity.type, activity.idx);
        routes.emplace_back(data, activities, spec.vehicleType);
    }
    return routes;
}
}
