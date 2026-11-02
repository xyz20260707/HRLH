#ifndef PYVRP_SEARCH_SOLUTION_H
#define PYVRP_SEARCH_SOLUTION_H

#include "../Solution.h"
#include "CostEvaluator.h"
#include "ProblemData.h"
#include "Route.h"
#include "SearchSpace.h"

#include <vector>

namespace pyvrp::search
{

class Solution
{
    ProblemData const &data_;

    friend class pyvrp::CostEvaluator;

public:
    std::vector<Route::Node> nodes;
    std::vector<Route> routes;

    Solution(ProblemData const &data);

    void load(pyvrp::Solution const &solution);

    pyvrp::Solution unload() const;

    bool insert(Route::Node *node,
                SearchSpace const &searchSpace,
                CostEvaluator const &costEvaluator,
                bool required);
};
}

template <>
pyvrp::Cost pyvrp::CostEvaluator::penalisedCost(
    pyvrp::search::Solution const &solution) const;

#endif
