#ifndef PYVRP_SEARCH_LOCALSEARCHOPERATOR_H
#define PYVRP_SEARCH_LOCALSEARCHOPERATOR_H

#include "CostEvaluator.h"
#include "Measure.h"
#include "ProblemData.h"
#include "Route.h"
#include "Solution.h"

#include <utility>

namespace pyvrp::search
{

template <std::same_as<Route::Node *>... Args> class LocalSearchOperator
{
protected:
    ProblemData const &data;

public:

    virtual std::pair<Cost, bool> evaluate(Args... args,
                                           CostEvaluator const &costEvaluator)
        = 0;

    virtual void apply(Args... args) const = 0;

    LocalSearchOperator(ProblemData const &data) : data(data){};
    virtual ~LocalSearchOperator() = default;
};

using BinaryOperator = LocalSearchOperator<Route::Node *, Route::Node *>;

template <typename Op> bool supports([[maybe_unused]] ProblemData const &data)
{
    return true;
}
}

#endif
