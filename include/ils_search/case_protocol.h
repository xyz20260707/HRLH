#ifndef HRLH_ILS_SEARCH_CASE_PROTOCOL_H
#define HRLH_ILS_SEARCH_CASE_PROTOCOL_H

#include "Activity.h"
#include "ProblemData.h"
#include "Route.h"

#include <cstdint>
#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace hrlh_ils_search
{
using Integer = std::int64_t;

struct LocationSpec
{
    double x = 0;
    double y = 0;
};

struct ClientSpec
{
    size_t location = 0;
    std::vector<Integer> delivery;
    std::vector<Integer> pickup;
    Integer serviceDuration = 0;
    Integer twEarly = 0;
    Integer twLate = 0;
    Integer releaseTime = 0;
    Integer latestReturn = 0;
    Integer prize = 0;
    bool required = true;
    std::optional<size_t> group;
};

struct DepotSpec
{
    size_t location = 0;
    Integer twEarly = 0;
    Integer twLate = 0;
    Integer serviceDuration = 0;
};

struct GroupSpec
{
    bool required = true;
    std::vector<size_t> clients;
};

struct VehicleTypeSpec
{
    size_t numAvailable = 1;
    std::vector<Integer> capacity;
    size_t startDepot = 0;
    size_t endDepot = 0;
    Integer fixedCost = 0;
    Integer twEarly = 0;
    Integer twLate = 0;
    Integer shiftDuration = 0;
    Integer maxDistance = 0;
    Integer unitDistanceCost = 1;
    Integer unitDurationCost = 0;
    size_t profile = 0;
    std::optional<Integer> startLate;
    std::vector<Integer> initialLoad;
    std::vector<size_t> reloadDepots;
    size_t maxReloads = 0;
    Integer maxOvertime = 0;
    Integer unitOvertimeCost = 0;
};

struct MatrixSpec
{
    std::vector<Integer> values;
};

struct ActivitySpec
{
    pyvrp::Activity::ActivityType type = pyvrp::Activity::ActivityType::CLIENT;
    size_t idx = 0;
};

struct RouteSpec
{
    size_t vehicleType = 0;
    std::vector<ActivitySpec> activities;
};

struct PenaltyParamsSpec
{
    size_t solutionsBetweenUpdates = 500;
    double penaltyIncrease = 1.50;
    double penaltyDecrease = 0.90;
    double targetFeasible = 0.65;
    double feasTolerance = 0.05;
    double minPenalty = 0.1;
    double maxPenalty = 100000.0;
};

struct ILSParamsSpec
{
    size_t numItersNoImprovement = 150000;
    size_t historyLength = 300;
    bool exhaustiveOnBest = true;
};

struct NeighbourhoodParamsSpec
{
    double weightWaitTime = 0.2;
    size_t numNeighbours = 50;
    bool symmetricProximity = true;
};

struct PerturbationParamsSpec
{
    size_t minPerturbations = 1;
    size_t maxPerturbations = 25;
};

struct SolverParamsSpec
{
    std::uint32_t seed = 0;
    size_t maxIterations = 0;
    bool useSingletonWarmStart = true;
    PenaltyParamsSpec penalty;
    ILSParamsSpec ils;
    NeighbourhoodParamsSpec neighbourhood;
    PerturbationParamsSpec perturbation;
    std::vector<std::string> operators;
    std::optional<std::vector<RouteSpec>> initialRoutes;
};

struct CaseSpec
{
    std::vector<LocationSpec> locations;
    std::vector<ClientSpec> clients;
    std::vector<DepotSpec> depots;
    std::vector<GroupSpec> groups;
    std::vector<VehicleTypeSpec> vehicleTypes;
    std::vector<MatrixSpec> distanceMatrices;
    std::vector<MatrixSpec> durationMatrices;
    SolverParamsSpec solver;
};

std::vector<std::string> defaultOperators();

pyvrp::ProblemData buildProblem(CaseSpec const &spec);
std::vector<pyvrp::Route> buildRoutes(pyvrp::ProblemData const &data,
                                      std::vector<RouteSpec> const &routes);
}

#endif
