#ifndef PYVRP_SEARCH_LOCALSEARCH_H
#define PYVRP_SEARCH_LOCALSEARCH_H

#include "CostEvaluator.h"
#include "LocalSearchOperator.h"
#include "PerturbationManager.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"
#include "SearchSpace.h"
#include "Solution.h"

#include <functional>
#include <stdexcept>
#include <vector>

namespace pyvrp::search
{
class LocalSearch
{
    ProblemData const &data;

    Solution solution_;

    SearchSpace searchSpace_;

    PerturbationManager &perturbationManager_;

    std::vector<BinaryOperator *> binaryOps_;

    std::vector<int> lastTest_;
    std::vector<int> lastUpdate_;

    size_t numUpdates_ = 0;
    bool searchCompleted_ = false;

    std::function<void()> interruptCheck_;

    void checkInterrupt() const;

    bool applyBinaryOps(Route::Node *U,
                        Route::Node *V,
                        CostEvaluator const &costEvaluator);

    void applyEmptyRouteMoves(Route::Node *U,
                              CostEvaluator const &costEvaluator);

    void ensureStructuralFeasibility(CostEvaluator const &costEvaluator);

    void update(Route *U, Route *V);

    void search(CostEvaluator const &costEvaluator);

public:

    void addOperator(BinaryOperator &op);


    pyvrp::Solution operator()(pyvrp::Solution const &solution,
                               CostEvaluator const &costEvaluator,
                               bool exhaustive = false);

    void shuffle(RandomNumberGenerator &rng);

    void setInterruptCheck(std::function<void()> interruptCheck);

    LocalSearch(ProblemData const &data,
                SearchSpace::Neighbours neighbours,
                PerturbationManager &perturbationManager);
};
}

#endif
