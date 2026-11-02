#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

#include "Measure.h"

#include <cassert>
#include <concepts>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace pyvrp
{

template <typename T>
concept DeltaCostEvaluatable = requires(T arg, size_t dimension) {
    { arg.route() };
    { arg.distance() } -> std::convertible_to<std::pair<Cost, Distance>>;
    { arg.duration() } -> std::convertible_to<std::pair<Cost, Duration>>;
    { arg.excessLoad(dimension) } -> std::same_as<Load>;
};

class CostEvaluator
{
    std::vector<double> loadPenalties_;
    double twPenalty_;
    double distPenalty_;

    [[nodiscard]] inline Cost
    excessLoadPenalties(std::vector<Load> const &excessLoads) const;

public:
    CostEvaluator(std::vector<double> loadPenalties,
                  double twPenalty,
                  double distPenalty);

    [[nodiscard]] inline Cost
    loadPenalty(Load load, Load capacity, size_t dimension) const;

    [[nodiscard]] inline Cost twPenalty(Duration timeWarp) const;

    [[nodiscard]] inline Cost distPenalty(Distance distance,
                                          Distance maxDistance) const;

    [[nodiscard]] inline Cost excessDistPenalty(Distance excessDistance) const;

    template <typename T> [[nodiscard]] Cost penalisedCost(T const &arg) const;

    template <typename T> [[nodiscard]] Cost cost(T const &arg) const;

    template <bool exact = false,
              typename... Args,
              template <typename...>
              class T>
        requires(DeltaCostEvaluatable<T<Args...>>)
    bool deltaCost(Cost &out, T<Args...> const &proposal) const;

    template <bool exact = false,
              typename... uArgs,
              typename... vArgs,
              template <typename...>
              class T>
        requires(DeltaCostEvaluatable<T<uArgs...>>
                 && DeltaCostEvaluatable<T<vArgs...>>)
    bool deltaCost(Cost &out,
                   T<uArgs...> const &uProposal,
                   T<vArgs...> const &vProposal) const;
};

Cost CostEvaluator::excessLoadPenalties(
    std::vector<Load> const &excessLoads) const
{
    assert(excessLoads.size() == loadPenalties_.size());

    Cost cost = 0;
    for (size_t dim = 0; dim != loadPenalties_.size(); ++dim)
        cost += loadPenalties_[dim] * excessLoads[dim].get();

    return cost;
}

Cost CostEvaluator::loadPenalty(Load load,
                                Load capacity,
                                size_t dimension) const
{
    assert(dimension < loadPenalties_.size());
    auto const excessLoad = std::max<Load>(load - capacity, 0);
    return static_cast<Cost>(excessLoad.get() * loadPenalties_[dimension]);
}

Cost CostEvaluator::twPenalty([[maybe_unused]] Duration timeWarp) const
{
    return static_cast<Cost>(timeWarp.get() * twPenalty_);
}

Cost CostEvaluator::distPenalty(Distance distance, Distance maxDistance) const
{
    auto const excessDistance = std::max<Distance>(distance - maxDistance, 0);
    return excessDistPenalty(excessDistance);
}

Cost CostEvaluator::excessDistPenalty(Distance excessDistance) const
{
    return static_cast<Cost>(excessDistance.get() * distPenalty_);
}

template <typename T> Cost CostEvaluator::cost(T const &arg) const
{

    return arg.isFeasible() ? penalisedCost(arg)
                            : std::numeric_limits<Cost>::max();
}

template <bool exact, typename... Args, template <typename...> class T>
    requires(DeltaCostEvaluatable<T<Args...>>)
bool CostEvaluator::deltaCost(Cost &out, T<Args...> const &proposal) const
{
    auto const *route = proposal.route();
    if (!route->empty())
    {
        out -= route->distanceCost();
        out -= excessDistPenalty(route->excessDistance());

        out -= excessLoadPenalties(route->excessLoad());

        out -= route->durationCost();
        out -= twPenalty(route->timeWarp());
    }

    if (route->hasDistanceCost())
    {
        auto const [cost, excess] = proposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    auto const &capacity = route->capacity();
    for (size_t dim = 0; dim != capacity.size(); ++dim)
    {
        if constexpr (!exact)
            if (out >= 0)
                return false;

        out += loadPenalty(proposal.excessLoad(dim), 0, dim);
    }

    if (route->hasDurationCost())
    {
        auto const [cost, timeWarp] = proposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
    }

    return true;
}

template <bool exact,
          typename... uArgs,
          typename... vArgs,
          template <typename...>
          class T>
    requires(DeltaCostEvaluatable<T<uArgs...>>
             && DeltaCostEvaluatable<T<vArgs...>>)
bool CostEvaluator::deltaCost(Cost &out,
                              T<uArgs...> const &uProposal,
                              T<vArgs...> const &vProposal) const
{
    auto const *uRoute = uProposal.route();
    if (!uRoute->empty())
    {
        out -= uRoute->distanceCost();
        out -= excessDistPenalty(uRoute->excessDistance());

        out -= excessLoadPenalties(uRoute->excessLoad());

        out -= uRoute->durationCost();
        out -= twPenalty(uRoute->timeWarp());
    }

    auto const *vRoute = vProposal.route();
    if (!vRoute->empty())
    {
        out -= vRoute->distanceCost();
        out -= excessDistPenalty(vRoute->excessDistance());

        out -= excessLoadPenalties(vRoute->excessLoad());

        out -= vRoute->durationCost();
        out -= twPenalty(vRoute->timeWarp());
    }

    if (uRoute->hasDistanceCost())
    {
        auto const [cost, excess] = uProposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    if (vRoute->hasDistanceCost())
    {
        auto const [cost, excess] = vProposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    auto const &uCapacity = uRoute->capacity();
    for (size_t dim = 0; dim != uCapacity.size(); ++dim)
    {
        if constexpr (!exact)
            if (out >= 0)
                return false;

        out += loadPenalty(uProposal.excessLoad(dim), 0, dim);
    }

    auto const &vCapacity = vRoute->capacity();
    for (size_t dim = 0; dim != vCapacity.size(); ++dim)
    {
        if constexpr (!exact)
            if (out >= 0)
                return false;

        out += loadPenalty(vProposal.excessLoad(dim), 0, dim);
    }

    if constexpr (!exact)
        if (out >= 0)
            return false;

    if (uRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = uProposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
    }

    if (vRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = vProposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
    }

    return true;
}
}

#endif
