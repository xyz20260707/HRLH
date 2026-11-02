#ifndef PYVRP_SEARCH_SWAPTAILS_H
#define PYVRP_SEARCH_SWAPTAILS_H

#include "LocalSearchOperator.h"

namespace pyvrp::search
{

class SwapTails : public BinaryOperator
{
    using BinaryOperator::BinaryOperator;

public:
    std::pair<Cost, bool> evaluate(Route::Node *U,
                                   Route::Node *V,
                                   CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U, Route::Node *V) const override;
};

template <> bool supports<SwapTails>(ProblemData const &data);
}

#endif
