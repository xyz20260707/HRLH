#ifndef PYVRP_SEARCH_EXCHANGE_H
#define PYVRP_SEARCH_EXCHANGE_H

#include "LocalSearchOperator.h"
#include "Route.h"

#include <cassert>

namespace pyvrp::search
{

template <size_t N, size_t M> class Exchange : public BinaryOperator
{
    using BinaryOperator::BinaryOperator;

    static_assert(N >= M && N > 0, "N < M or N == 0 does not make sense");

    bool containsDepot(Route::Node *node, size_t segLength) const;

    bool overlap(Route::Node *U, Route::Node *V) const;

    bool adjacent(Route::Node *U, Route::Node *V) const;

    std::pair<Cost, bool>
    evalRelocateMove(Route::Node *U,
                     Route::Node *V,
                     CostEvaluator const &costEvaluator) const;

    std::pair<Cost, bool>
    evalSwapMove(Route::Node *U,
                 Route::Node *V,
                 CostEvaluator const &costEvaluator) const;

public:
    std::pair<Cost, bool> evaluate(Route::Node *U,
                                   Route::Node *V,
                                   CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U, Route::Node *V) const override;
};

template <size_t N, size_t M>
bool Exchange<N, M>::containsDepot(Route::Node *node, size_t segLength) const
{
    auto const first = node->pos();
    auto const last = first + segLength - 1;
    auto const &route = *node->route();

    return first == 0
           || last >= route.size() - 1
           || node->trip() != route[last]->trip();
}

template <size_t N, size_t M>
bool Exchange<N, M>::overlap(Route::Node *U, Route::Node *V) const
{
    return U->route() == V->route()

           && U->pos() <= V->pos() + std::max<size_t>(M, 1) - 1
           && V->pos() <= U->pos() + N - 1;
}

template <size_t N, size_t M>
bool Exchange<N, M>::adjacent(Route::Node *U, Route::Node *V) const
{
    return U->route() == V->route()
           && (U->pos() + N == V->pos() || V->pos() + M == U->pos());
}

template <size_t N, size_t M>
std::pair<Cost, bool> Exchange<N, M>::evalRelocateMove(
    Route::Node *U, Route::Node *V, CostEvaluator const &costEvaluator) const
{
    assert(U->pos() > 0);

    Cost deltaCost = 0;

    if (U->route() != V->route())
    {
        auto const *uRoute = U->route();
        auto const *vRoute = V->route();

        if (V->isStartDepot() && vRoute->empty())
            deltaCost += vRoute->fixedVehicleCost();

        if (uRoute->numClients() == N)
            deltaCost -= uRoute->fixedVehicleCost();

        auto const uProposal = Route::Proposal(uRoute->before(U->pos() - 1),
                                               uRoute->after(U->pos() + N));

        auto const vProposal
            = Route::Proposal(vRoute->before(V->pos()),
                              uRoute->between(U->pos(), U->pos() + N - 1),
                              vRoute->after(V->pos() + 1));

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }
    else
    {
        auto *route = U->route();

        if (U->pos() < V->pos())
            costEvaluator.deltaCost(
                deltaCost,
                Route::Proposal(route->before(U->pos() - 1),
                                route->between(U->pos() + N, V->pos()),
                                route->between(U->pos(), U->pos() + N - 1),
                                route->after(V->pos() + 1)));
        else
            costEvaluator.deltaCost(
                deltaCost,
                Route::Proposal(route->before(V->pos()),
                                route->between(U->pos(), U->pos() + N - 1),
                                route->between(V->pos() + 1, U->pos() - 1),
                                route->after(U->pos() + N)));
    }

    return std::make_pair(deltaCost, deltaCost < 0);
}

template <size_t N, size_t M>
std::pair<Cost, bool> Exchange<N, M>::evalSwapMove(
    Route::Node *U, Route::Node *V, CostEvaluator const &costEvaluator) const
{
    assert(U->pos() > 0 && V->pos() > 0);
    assert(U->route() && V->route());

    Cost deltaCost = 0;

    if (U->route() != V->route())
    {
        auto const *uRoute = U->route();
        auto const *vRoute = V->route();

        auto const uProposal
            = Route::Proposal(uRoute->before(U->pos() - 1),
                              vRoute->between(V->pos(), V->pos() + M - 1),
                              uRoute->after(U->pos() + N));

        auto const vProposal
            = Route::Proposal(vRoute->before(V->pos() - 1),
                              uRoute->between(U->pos(), U->pos() + N - 1),
                              vRoute->after(V->pos() + M));

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }
    else
    {
        auto const *route = U->route();

        if (U->pos() < V->pos())
            costEvaluator.deltaCost(
                deltaCost,
                Route::Proposal(route->before(U->pos() - 1),
                                route->between(V->pos(), V->pos() + M - 1),
                                route->between(U->pos() + N, V->pos() - 1),
                                route->between(U->pos(), U->pos() + N - 1),
                                route->after(V->pos() + M)));
        else
            costEvaluator.deltaCost(
                deltaCost,
                Route::Proposal(route->before(V->pos() - 1),
                                route->between(U->pos(), U->pos() + N - 1),
                                route->between(V->pos() + M, U->pos() - 1),
                                route->between(V->pos(), V->pos() + M - 1),
                                route->after(U->pos() + N)));
    }

    return std::make_pair(deltaCost, deltaCost < 0);
}

template <size_t N, size_t M>
std::pair<Cost, bool> Exchange<N, M>::evaluate(
    Route::Node *U, Route::Node *V, CostEvaluator const &costEvaluator)
{

    if (!U->route() || containsDepot(U, N) || overlap(U, V))
        return std::make_pair(0, false);

    if constexpr (M > 0)
        if (containsDepot(V, M))
            return std::make_pair(0, false);

    if (U->route() == V->route() && U->trip() != V->trip())
        return std::make_pair(0, false);

    if constexpr (M == 0)
    {
        if (U == n(V))
            return std::make_pair(0, false);

        return evalRelocateMove(U, V, costEvaluator);
    }
    else
    {
        if constexpr (N == M)
        {
            if (U->idx() >= V->idx())
                return std::make_pair(0, false);
        }

        if (adjacent(U, V))
            return std::make_pair(0, false);

        return evalSwapMove(U, V, costEvaluator);
    }
}

template <size_t N, size_t M>
void Exchange<N, M>::apply(Route::Node *U, Route::Node *V) const
{

    auto &uRoute = *U->route();
    auto &vRoute = *V->route();
    auto *uToInsert = N == 1 ? U : uRoute[U->pos() + N - 1];
    auto *insertUAfter = M == 0 ? V : vRoute[V->pos() + M - 1];

    for (size_t count = 0; count != N - M; ++count)
    {
        auto *prev = p(uToInsert);
        uRoute.remove(uToInsert->pos());
        vRoute.insert(insertUAfter->pos() + 1, uToInsert);
        uToInsert = prev;
    }

    for (size_t count = 0; count != M; ++count)
    {
        Route::swap(U, V);
        U = n(U);
        V = n(V);
    }
}
}

#endif
