#include "search/PerturbationManager.h"

#include <cassert>
#include <stdexcept>

using pyvrp::search::PerturbationManager;
using pyvrp::search::PerturbationParams;
using pyvrp::search::Route;

namespace
{
enum class PerturbType
{
    REMOVE,
    INSERT
};
}

PerturbationParams::PerturbationParams(size_t minPerturbations,
                                       size_t maxPerturbations)
    : minPerturbations(minPerturbations), maxPerturbations(maxPerturbations)
{
    if (minPerturbations > maxPerturbations)
        throw std::invalid_argument(
            "min_perturbations must be <= max_perturbations.");
}

PerturbationManager::PerturbationManager(PerturbationParams params)
    : params_(params), numPerturbations_(params_.minPerturbations)
{
}

void PerturbationManager::shuffle(RandomNumberGenerator &rng)
{
    auto const range = params_.maxPerturbations - params_.minPerturbations;
    numPerturbations_ = params_.minPerturbations + rng.randint(range + 1);
}

void PerturbationManager::perturb(Solution &solution,
                                  SearchSpace &searchSpace,
                                  CostEvaluator const &costEvaluator) const
{
    size_t movesLeft = numPerturbations_;

    if (!movesLeft)
        return;

    searchSpace.unmarkAllPromising();

    DynamicBitset perturbed = {solution.nodes.size()};
    auto const perturb = [&](auto *node, PerturbType action)
    {

        if (perturbed[node->idx()])
            return;

        auto *route = node->route();
        if (route && action == PerturbType::REMOVE)
        {
            searchSpace.markPromising(node);
            route->remove(node->pos());
            route->update();
        }

        else if (!route && action == PerturbType::INSERT)
        {
            solution.insert(node, searchSpace, costEvaluator, true);
            node->route()->update();
            searchSpace.markPromising(node);
        }
        else
            return;

        perturbed[node->idx()] = true;
        movesLeft--;
    };

    for (auto const uClient : searchSpace.clientOrder())
    {
        auto *U = &solution.nodes[uClient];
        auto action = U->route() ? PerturbType::REMOVE : PerturbType::INSERT;
        perturb(U, action);

        if (!movesLeft)
            return;

        for (auto const vClient : searchSpace.neighboursOf(uClient))
        {
            auto *V = &solution.nodes[vClient];
            perturb(V, action);

            if (!movesLeft)
                return;
        }
    }
}
