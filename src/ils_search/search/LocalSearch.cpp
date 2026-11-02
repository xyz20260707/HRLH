#include "search/LocalSearch.h"
#include "DynamicBitset.h"
#include "Measure.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <numeric>

using pyvrp::Solution;
using pyvrp::search::BinaryOperator;
using pyvrp::search::LocalSearch;
using pyvrp::search::SearchSpace;

pyvrp::Solution LocalSearch::operator()(pyvrp::Solution const &solution,
                                        CostEvaluator const &costEvaluator,
                                        bool exhaustive)
{
    checkInterrupt();

    std::fill(lastTest_.begin(), lastTest_.end(), -1);
    std::fill(lastUpdate_.begin(), lastUpdate_.end(), 0);
    numUpdates_ = 0;

    solution_.load(solution);

    if (exhaustive)
        searchSpace_.markAllPromising();
    else
        perturbationManager_.perturb(solution_, searchSpace_, costEvaluator);

    checkInterrupt();
    ensureStructuralFeasibility(costEvaluator);
    checkInterrupt();
    search(costEvaluator);
    checkInterrupt();

    return solution_.unload();
}

void LocalSearch::search(CostEvaluator const &costEvaluator)
{
    if (binaryOps_.empty())
        return;

    searchCompleted_ = false;
    for (int step = 0; !searchCompleted_; ++step)
    {
        checkInterrupt();
        searchCompleted_ = true;

        for (auto const uClient : searchSpace_.clientOrder())
        {
            checkInterrupt();
            auto *U = &solution_.nodes[uClient];
            if (!searchSpace_.isPromising(uClient))
                continue;

            auto const lastTest = lastTest_[uClient];
            lastTest_[uClient] = numUpdates_;

            for (auto const vClient : searchSpace_.neighboursOf(uClient))
            {
                checkInterrupt();
                auto *V = &solution_.nodes[vClient];

                if (!V->route())
                    continue;

                auto *routes = solution_.routes.data();
                auto uUpdate = 0;
                if (U->route())
                    uUpdate = lastUpdate_[std::distance(routes, U->route())];
                auto vUpdate = lastUpdate_[std::distance(routes, V->route())];
                if (uUpdate > lastTest || vUpdate > lastTest)
                {
                    if (applyBinaryOps(U, V, costEvaluator))
                        continue;

                    if (p(V)->isStartDepot()
                        && applyBinaryOps(U, p(V), costEvaluator))
                        continue;
                }
            }

            if (step > 0 || !U->route())
                applyEmptyRouteMoves(U, costEvaluator);
        }
    }
}

void LocalSearch::checkInterrupt() const
{
    if (interruptCheck_)
        interruptCheck_();
}

void LocalSearch::setInterruptCheck(std::function<void()> interruptCheck)
{
    interruptCheck_ = std::move(interruptCheck);
}

void LocalSearch::shuffle(RandomNumberGenerator &rng)
{
    perturbationManager_.shuffle(rng);
    searchSpace_.shuffle(rng);

    rng.shuffle(binaryOps_.begin(), binaryOps_.end());
}


bool LocalSearch::applyBinaryOps(Route::Node *U,
                                 Route::Node *V,
                                 CostEvaluator const &costEvaluator)
{
    for (auto *op : binaryOps_)
    {
        checkInterrupt();
        auto const [deltaCost, shouldApply] = op->evaluate(U, V, costEvaluator);
        if (shouldApply)
        {

            auto *rU = U->route();
            auto *rV = V->route();
            assert(rV);

            if (rU)
                searchSpace_.markPromising(U);
            searchSpace_.markPromising(V);

            [[maybe_unused]] auto const costBefore
                = costEvaluator.penalisedCost(solution_);

            op->apply(U, V);
            update(rU, rV);

            [[maybe_unused]] auto const costAfter
                = costEvaluator.penalisedCost(solution_);

            assert(costAfter == costBefore + deltaCost);

            return true;
        }
    }

    return false;
}

void LocalSearch::applyEmptyRouteMoves(Route::Node *U,
                                       CostEvaluator const &costEvaluator)
{

    for (auto const &[vehType, offset] : searchSpace_.vehTypeOrder())
    {
        checkInterrupt();
        auto const begin = solution_.routes.begin() + offset;
        auto const end = begin + data.vehicleType(vehType).numAvailable;
        auto const pred = [](auto const &route) { return route.empty(); };
        auto empty = std::find_if(begin, end, pred);

        if (empty != end && applyBinaryOps(U, (*empty)[0], costEvaluator))
            break;
    }
}

void LocalSearch::ensureStructuralFeasibility(
    CostEvaluator const &costEvaluator)
{
    std::vector<size_t> groupCount(data.numGroups(), 0);
    for (size_t idx = 0; idx != data.numGroups(); ++idx)
    {
        auto const &group = data.group(idx);
        for (auto const client : group)
            if (solution_.nodes[client].route())
                groupCount[idx]++;
    }

    for (auto const client : searchSpace_.clientOrder())
    {
        auto &node = solution_.nodes[client];
        auto const &clientData = data.client(client);

        if (!node.route() && clientData.required)
        {
            solution_.insert(&node, searchSpace_, costEvaluator, true);
            update(node.route(), node.route());
            searchSpace_.markPromising(&node);
            continue;
        }

        if (clientData.group)
        {
            auto const idx = *clientData.group;
            auto const &group = data.group(idx);

            if (group.required && groupCount[idx] == 0)
            {
                assert(!node.route());
                solution_.insert(&node, searchSpace_, costEvaluator, true);
                update(node.route(), node.route());
                searchSpace_.markPromising(&node);
                groupCount[idx]++;
                continue;
            }

            if (node.route() && groupCount[idx] > 1)
            {
                searchSpace_.markPromising(&node);
                auto *route = node.route();
                route->remove(node.pos());
                update(route, route);
                groupCount[idx]--;
            }
        }
    }

#ifndef NDEBUG

    for (size_t idx = 0; idx != data.numClients(); ++idx)
    {
        auto const &node = solution_.nodes[idx];
        auto const &clientData = data.client(idx);
        assert(node.route() || !clientData.required);
    }

    for (size_t idx = 0; idx != data.numGroups(); ++idx)
    {
        auto const &group = data.group(idx);
        assert(group.required ? groupCount[idx] == 1 : groupCount[idx] <= 1);
    }
#endif
}

void LocalSearch::update(Route *U, Route *V)
{
    assert(V);
    numUpdates_++;
    searchCompleted_ = false;

    auto const update = [&](Route *route)
    {
        route->update();
        if (route->empty())
            route->clear();

        auto const idx = std::distance(solution_.routes.data(), route);
        lastUpdate_[idx] = numUpdates_;
    };

    if (U)
        update(U);

    if (U != V)
        update(V);
}


void LocalSearch::addOperator(BinaryOperator &op)
{
    binaryOps_.emplace_back(&op);
}


LocalSearch::LocalSearch(ProblemData const &data,
                         SearchSpace::Neighbours neighbours,
                         PerturbationManager &perturbationManager)
    : data(data),
      solution_(data),
      searchSpace_(data, neighbours),
      perturbationManager_(perturbationManager),
      lastTest_(data.numClients()),
      lastUpdate_(data.numVehicles())
{
}
