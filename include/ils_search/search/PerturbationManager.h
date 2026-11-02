#ifndef PYVRP_SEARCH_PERTURBATIONMANAGER_H
#define PYVRP_SEARCH_PERTURBATIONMANAGER_H

#include "CostEvaluator.h"
#include "RandomNumberGenerator.h"
#include "SearchSpace.h"
#include "Solution.h"

#include <iosfwd>

namespace pyvrp::search
{

struct PerturbationParams
{
    size_t const minPerturbations;
    size_t const maxPerturbations;

    PerturbationParams(size_t minPerturbations = 1,
                       size_t maxPerturbations = 25);

    bool operator==(PerturbationParams const &other) const = default;
};

class PerturbationManager
{
    PerturbationParams const params_;
    size_t numPerturbations_;

public:
    PerturbationManager(PerturbationParams params = PerturbationParams());

    void shuffle(RandomNumberGenerator &rng);

    void perturb(Solution &solution,
                 SearchSpace &searchSpace,
                 CostEvaluator const &costEvaluator) const;
};
}

#endif
