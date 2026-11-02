#ifndef PYVRP_SOLUTION_H
#define PYVRP_SOLUTION_H

#include "Activity.h"
#include "CostEvaluator.h"
#include "Measure.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"

#include <functional>
#include <iosfwd>
#include <vector>

namespace pyvrp
{

class Solution
{
    using Client = size_t;
    using Depot = size_t;
    using VehicleType = size_t;

    using Routes = std::vector<Route>;

    size_t numClients_ = 0;
    size_t numMissingClients_ = 0;
    size_t numMissingGroups_ = 0;
    Distance distance_ = 0;
    Cost distanceCost_ = 0;
    Duration duration_ = 0;
    Duration overtime_ = 0;
    Cost durationCost_ = 0;
    Distance excessDistance_ = 0;
    std::vector<Load> excessLoad_;
    Cost fixedVehicleCost_ = 0;
    Cost prizes_ = 0;
    Cost uncollectedPrizes_ = 0;
    Duration timeWarp_ = 0;

    Routes routes_;

    void evaluate(ProblemData const &data);

    Solution &operator=(Solution const &other) = default;
    Solution &operator=(Solution &&other) = default;

public:

    [[nodiscard]] bool empty() const;

    [[nodiscard]] size_t numRoutes() const;

    [[nodiscard]] size_t numClients() const;

    [[nodiscard]] Routes const &routes() const;

    [[nodiscard]] bool isFeasible() const;

    [[nodiscard]] bool isComplete() const;

    [[nodiscard]] bool hasExcessLoad() const;

    [[nodiscard]] bool hasExcessDistance() const;

    [[nodiscard]] bool hasTimeWarp() const;

    [[nodiscard]] Distance distance() const;

    [[nodiscard]] Cost distanceCost() const;

    [[nodiscard]] Duration duration() const;

    [[nodiscard]] Duration overtime() const;

    [[nodiscard]] Cost durationCost() const;

    [[nodiscard]] std::vector<Load> const &excessLoad() const;

    [[nodiscard]] Distance excessDistance() const;

    [[nodiscard]] Cost fixedVehicleCost() const;

    [[nodiscard]] Cost prizes() const;

    [[nodiscard]] Cost uncollectedPrizes() const;

    [[nodiscard]] Duration timeWarp() const;

    bool operator==(Solution const &other) const;

    Solution(Solution const &other) = default;
    Solution(Solution &&other) = default;

    Solution(ProblemData const &data, RandomNumberGenerator &rng);

    Solution(ProblemData const &data, Routes routes);
};

template <>
Cost CostEvaluator::penalisedCost(Solution const &solution) const;
}

#endif
