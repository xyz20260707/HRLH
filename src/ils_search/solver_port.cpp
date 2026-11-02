#include "solver_port.h"

#include "CostEvaluator.h"
#include "RandomNumberGenerator.h"
#include "Route.h"
#include "search/Exchange.h"
#include "search/LocalSearch.h"
#include "search/PerturbationManager.h"
#include "search/SwapTails.h"
#include "search/neighbourhood.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace pyvrp;
namespace ps = pyvrp::search;

namespace hrlh_ils_search
{
namespace
{
struct PenaltyManager
{
    PenaltyParamsSpec params;
    std::vector<double> penalties;
    std::vector<std::vector<bool>> feasLists;

    PenaltyManager(ProblemData const &data, PenaltyParamsSpec params)
        : params(params)
    {
        if (params.solutionsBetweenUpdates < 1)
            throw std::invalid_argument("Expected solutions_between_updates >= 1.");
        if (params.penaltyIncrease < 1)
            throw std::invalid_argument("Expected penalty_increase >= 1.");
        if (params.penaltyDecrease < 0 || params.penaltyDecrease > 1)
            throw std::invalid_argument("Expected penalty_decrease in [0, 1].");
        if (params.targetFeasible < 0 || params.targetFeasible > 1)
            throw std::invalid_argument("Expected target_feasible in [0, 1].");
        if (params.feasTolerance < 0 || params.feasTolerance > 1)
            throw std::invalid_argument("Expected feas_tolerance in [0, 1].");
        if (params.minPenalty < 0)
            throw std::invalid_argument("Expected min_penalty >= 0.");
        if (params.maxPenalty < params.minPenalty)
            throw std::invalid_argument("Expected max_penalty >= min_penalty.");

        auto const midpoint
            = params.minPenalty + (params.maxPenalty - params.minPenalty) / 2;
        penalties.assign(data.numLoadDimensions() + 2, midpoint);
        feasLists.assign(penalties.size(), {});
    }

    double compute(double penalty, double feasiblePercentage) const
    {
        auto const diff = params.targetFeasible - feasiblePercentage;
        if (std::abs(diff) < params.feasTolerance)
            return penalty;
        auto const updated = diff > 0 ? params.penaltyIncrease * penalty
                                      : params.penaltyDecrease * penalty;
        return std::clamp(updated, params.minPenalty, params.maxPenalty);
    }

    double registerOne(std::vector<bool> &values, double penalty, bool feasible)
    {
        values.push_back(feasible);
        if (values.size() != params.solutionsBetweenUpdates)
            return penalty;
        auto const sum = std::accumulate(values.begin(), values.end(), 0);
        auto const average = static_cast<double>(sum) / values.size();
        values.clear();
        return compute(penalty, average);
    }

    void registerSolution(Solution const &solution)
    {
        std::vector<bool> feasible;
        for (auto const excess : solution.excessLoad())
            feasible.push_back(excess == 0);
        feasible.push_back(!solution.hasTimeWarp());
        feasible.push_back(!solution.hasExcessDistance());
        for (size_t idx = 0; idx != feasible.size(); ++idx)
            penalties[idx]
                = registerOne(feasLists[idx], penalties[idx], feasible[idx]);
    }

    CostEvaluator costEvaluator() const
    {
        return CostEvaluator({penalties.begin(), penalties.end() - 2},
                             penalties[penalties.size() - 2],
                             penalties[penalties.size() - 1]);
    }

    CostEvaluator maxCostEvaluator() const
    {
        return CostEvaluator(
            std::vector<double>(penalties.size() - 2, params.maxPenalty),
            params.maxPenalty,
            params.maxPenalty);
    }
};

bool singletonRouteIsFeasible(ProblemData const &data,
                              size_t client,
                              size_t vehicleType)
{
    try
    {
        return Route(data, std::vector<size_t>{client}, vehicleType).isFeasible();
    }
    catch (...)
    {
        return false;
    }
}

void assertSingletonFeasibility(ProblemData const &data)
{
    for (size_t client = 0; client != data.numClients(); ++client)
    {
        auto const &item = data.client(client);
        if (item.group || !item.required)
            continue;
        bool feasible = false;
        for (size_t type = 0; type != data.numVehicleTypes(); ++type)
            feasible |= singletonRouteIsFeasible(data, client, type);
        if (!feasible)
            throw std::runtime_error("Instance infeasible: required client "
                                     + std::to_string(client)
                                     + " cannot be served even as a singleton route.");
    }

    for (size_t group = 0; group != data.numGroups(); ++group)
    {
        auto const &item = data.group(group);
        if (!item.required)
            continue;
        bool feasible = false;
        for (auto const client : item)
            for (size_t type = 0; type != data.numVehicleTypes(); ++type)
                feasible |= singletonRouteIsFeasible(data, client, type);
        if (!feasible)
            throw std::runtime_error("Instance infeasible: required client group "
                                     + std::to_string(group)
                                     + " has no member that can be served as a singleton route.");
    }
}

std::vector<size_t> requiredSingletonClients(ProblemData const &data)
{
    std::vector<size_t> clients;
    for (size_t client = 0; client != data.numClients(); ++client)
    {
        auto const &item = data.client(client);
        if (!item.group && item.required)
            clients.push_back(client);
    }

    for (auto const &group : data.groups())
    {
        if (!group.required)
            continue;
        for (auto const client : group)
        {
            bool feasible = false;
            for (size_t type = 0; type != data.numVehicleTypes(); ++type)
                feasible |= singletonRouteIsFeasible(data, client, type);
            if (feasible)
            {
                clients.push_back(client);
                break;
            }
        }
    }
    return clients;
}

struct FlowEdge
{
    size_t to;
    int capacity;
    size_t reverse;
};

std::vector<size_t> assignSingletonVehicleTypes(ProblemData const &data,
                                                 std::vector<size_t> const &clients)
{
    if (clients.empty())
        return {};

    auto const source = size_t{0};
    auto const clientOffset = size_t{1};
    auto const typeOffset = clientOffset + clients.size();
    auto const sink = typeOffset + data.numVehicleTypes();
    std::vector<std::vector<FlowEdge>> graph(sink + 1);
    auto addEdge = [&](size_t from, size_t to, int capacity)
    {
        graph[from].push_back({to, capacity, graph[to].size()});
        graph[to].push_back({from, 0, graph[from].size() - 1});
    };

    for (size_t idx = 0; idx != clients.size(); ++idx)
    {
        addEdge(source, clientOffset + idx, 1);
        bool feasible = false;
        for (size_t type = 0; type != data.numVehicleTypes(); ++type)
            if (singletonRouteIsFeasible(data, clients[idx], type))
            {
                addEdge(clientOffset + idx, typeOffset + type, 1);
                feasible = true;
            }
        if (!feasible)
            throw std::runtime_error("Instance infeasible: required client "
                                     + std::to_string(clients[idx])
                                     + " cannot be served even as a singleton route.");
    }

    for (size_t type = 0; type != data.numVehicleTypes(); ++type)
        addEdge(typeOffset + type,
                sink,
                static_cast<int>(data.vehicleType(type).numAvailable));

    size_t flow = 0;
    while (flow != clients.size())
    {
        std::vector<std::optional<std::pair<size_t, size_t>>> parent(
            graph.size());
        std::queue<size_t> queue;
        parent[source] = {source, source};
        queue.push(source);
        while (!queue.empty() && !parent[sink])
        {
            auto const from = queue.front();
            queue.pop();
            for (size_t edge = 0; edge != graph[from].size(); ++edge)
            {
                auto const &item = graph[from][edge];
                if (item.capacity > 0 && !parent[item.to])
                {
                    parent[item.to] = {from, edge};
                    queue.push(item.to);
                }
            }
        }
        if (!parent[sink])
            break;

        for (auto node = sink; node != source;)
        {
            auto const [from, edge] = *parent[node];
            auto &item = graph[from][edge];
            --item.capacity;
            ++graph[node][item.reverse].capacity;
            node = from;
        }
        ++flow;
    }

    if (flow != clients.size())
        throw std::runtime_error(
            "Cannot construct a feasible singleton warm start: not enough vehicles of feasible vehicle types are available. This does not certify instance infeasibility: clients may still be combinable in fewer routes, or a custom initial_solution can be provided.");

    std::vector<size_t> assignments(clients.size(), data.numVehicleTypes());
    for (size_t idx = 0; idx != clients.size(); ++idx)
        for (auto const &edge : graph[clientOffset + idx])
            if (edge.to >= typeOffset
                && edge.to < typeOffset + data.numVehicleTypes()
                && edge.capacity == 0)
            {
                assignments[idx] = edge.to - typeOffset;
                break;
            }

    if (std::any_of(assignments.begin(),
                    assignments.end(),
                    [&](size_t type) { return type == data.numVehicleTypes(); }))
        throw std::runtime_error("Could not recover singleton vehicle assignment.");
    return assignments;
}

std::unique_ptr<Solution> makeSingletonWarmStart(ProblemData const &data)
{
    assertSingletonFeasibility(data);
    auto const clients = requiredSingletonClients(data);
    if (clients.empty())
        return nullptr;
    auto const assignments = assignSingletonVehicleTypes(data, clients);
    std::vector<Route> routes;
    routes.reserve(clients.size());
    for (size_t idx = 0; idx != clients.size(); ++idx)
        routes.emplace_back(data,
                            std::vector<size_t>{clients[idx]},
                            assignments[idx]);
    auto solution = std::make_unique<Solution>(data, routes);
    if (!solution->isFeasible())
        throw std::runtime_error("Constructed singleton warm start is not feasible.");
    return solution;
}


struct Operators
{
    std::vector<std::unique_ptr<ps::BinaryOperator>> binary;
};

template <typename T>
void addBinary(ProblemData const &data, ps::LocalSearch &search, Operators &ops)
{
    if (ps::supports<T>(data))
    {
        auto op = std::make_unique<T>(data);
        search.addOperator(*op);
        ops.binary.push_back(std::move(op));
    }
}

Operators addOperators(ProblemData const &data,
                       ps::LocalSearch &search,
                       std::vector<std::string> const &names)
{
    Operators ops;
    for (auto const &name : names)
    {
        if (name == "Exchange10") addBinary<ps::Exchange<1, 0>>(data, search, ops);
        else if (name == "Exchange20") addBinary<ps::Exchange<2, 0>>(data, search, ops);
        else if (name == "Exchange30") addBinary<ps::Exchange<3, 0>>(data, search, ops);
        else if (name == "Exchange11") addBinary<ps::Exchange<1, 1>>(data, search, ops);
        else if (name == "Exchange21") addBinary<ps::Exchange<2, 1>>(data, search, ops);
        else if (name == "Exchange31") addBinary<ps::Exchange<3, 1>>(data, search, ops);
        else if (name == "Exchange22") addBinary<ps::Exchange<2, 2>>(data, search, ops);
        else if (name == "Exchange32") addBinary<ps::Exchange<3, 2>>(data, search, ops);
        else if (name == "Exchange33") addBinary<ps::Exchange<3, 3>>(data, search, ops);
        else if (name == "SwapTails") addBinary<ps::SwapTails>(data, search, ops);
        else throw std::invalid_argument("Unknown operator: " + name);
    }
    return ops;
}
}

SolveResult solveWithFeedback(ProblemData const &data,
                              SolverParamsSpec const &params,
                              CandidateFeedbackCallback feedbackCallback,
                              StopCallback stopCallback)
{
    if (params.ils.historyLength == 0)
        throw std::invalid_argument("history_length must be positive.");
    if (params.perturbation.minPerturbations > params.perturbation.maxPerturbations)
        throw std::invalid_argument("min_perturbations must be <= max_perturbations.");

    auto checkStop = [&]() { if (stopCallback) stopCallback(); };
    checkStop();

    RandomNumberGenerator rng(params.seed);
    auto neighbours = ps::computeNeighbours(
        data,
        ps::NeighbourhoodParams(params.neighbourhood.weightWaitTime,
                                params.neighbourhood.numNeighbours,
                                params.neighbourhood.symmetricProximity));
    ps::PerturbationManager perturbation(
        ps::PerturbationParams(params.perturbation.minPerturbations,
                               params.perturbation.maxPerturbations));
    ps::LocalSearch localSearch(data, neighbours, perturbation);
    localSearch.setInterruptCheck(checkStop);
    auto operators = addOperators(data, localSearch, params.operators);
    PenaltyManager penalties(data, params.penalty);

    auto improve = [&](Solution const &solution,
                       CostEvaluator const &evaluator,
                       bool exhaustive)
    {
        localSearch.shuffle(rng);
        return localSearch(solution, evaluator, exhaustive);
    };

    std::shared_ptr<Solution> initial;
    if (params.initialRoutes)
        initial = std::make_shared<Solution>(
            data, buildRoutes(data, *params.initialRoutes));
    else if (params.useSingletonWarmStart)
        if (auto warm = makeSingletonWarmStart(data))
            initial = std::make_shared<Solution>(*warm);

    if (!initial)
    {
        Solution random(data, rng);
        initial = std::make_shared<Solution>(
            improve(random, penalties.maxCostEvaluator(), true));
    }

    auto current = std::make_shared<Solution>(*initial);
    checkStop();
    const bool initialExternallyFeasible
        = !feedbackCallback || feedbackCallback(*initial);
    std::shared_ptr<Solution> best = initialExternallyFeasible
        ? std::make_shared<Solution>(*initial) : nullptr;
    std::vector<std::shared_ptr<Solution>> history(params.ils.historyLength);
    size_t historyIndex = 0;
    size_t iterationsNoImprovement = 0;

    for (size_t iterations = 0; iterations != params.maxIterations; ++iterations)
    {
        checkStop();
        if (iterationsNoImprovement == params.ils.numItersNoImprovement)
        {
            std::fill(history.begin(), history.end(), nullptr);
            historyIndex = 0;
            current = best
                ? std::make_shared<Solution>(*best)
                : std::make_shared<Solution>(*initial);
            iterationsNoImprovement = 0;
        }

        auto evaluator = penalties.costEvaluator();
        checkStop();
        auto candidate
            = std::make_shared<Solution>(improve(*current, evaluator, false));
        penalties.registerSolution(*candidate);
        checkStop();
        const bool externallyFeasible = !feedbackCallback || feedbackCallback(*candidate);
        if (!externallyFeasible)
        {
            ++iterationsNoImprovement;
            continue;
        }

        ++iterationsNoImprovement;
        if (!best || evaluator.cost(*candidate) < evaluator.cost(*best))
        {
            best = std::make_shared<Solution>(*candidate);
            iterationsNoImprovement = 0;
            if (params.ils.exhaustiveOnBest)
            {
                candidate = std::make_shared<Solution>(
                    improve(*candidate, evaluator, true));
                checkStop();
                const bool exhaustiveExternallyFeasible
                    = !feedbackCallback || feedbackCallback(*candidate);
                if (candidate->isFeasible() && exhaustiveExternallyFeasible)
                    best = std::make_shared<Solution>(*candidate);
            }
        }

        auto candidateCost = evaluator.penalisedCost(*candidate);
        auto currentCost = evaluator.penalisedCost(*current);
        auto lateCost = evaluator.penalisedCost(*initial);
        auto const late = history[historyIndex % history.size()];
        if (late)
            lateCost = evaluator.penalisedCost(*late);

        if (candidateCost < lateCost || candidateCost < currentCost)
        {
            current = std::make_shared<Solution>(*candidate);
            currentCost = candidateCost;
        }
        if (currentCost < lateCost || !late)
            history[historyIndex % history.size()]
                = std::make_shared<Solution>(*current);
        ++historyIndex;
    }
    SolveResult result(best ? *best : *initial);
    return result;
}

}
