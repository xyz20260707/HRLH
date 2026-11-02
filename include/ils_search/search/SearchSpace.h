#ifndef PYVRP_SEARCH_SEARCHSPACE_H
#define PYVRP_SEARCH_SEARCHSPACE_H

#include "DynamicBitset.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"

#include <vector>

namespace pyvrp::search
{

class SearchSpace
{
public:
    using Neighbours = std::vector<std::vector<size_t>>;

private:

    Neighbours neighbours_;

    DynamicBitset promising_;

    std::vector<size_t> clientOrder_;

    std::vector<std::pair<size_t, size_t>> vehTypeOrder_;

public:
    SearchSpace(ProblemData const &data, Neighbours neighbours);

    void setNeighbours(Neighbours neighbours);

    std::vector<size_t> const &neighboursOf(size_t client) const;

    bool isPromising(size_t client) const;

    void markPromising(size_t client);

    void markPromising(Route::Node const *node);

    void markAllPromising();

    void unmarkAllPromising();

    std::vector<size_t> const &clientOrder() const;

    std::vector<std::pair<size_t, size_t>> const &vehTypeOrder() const;

    void shuffle(RandomNumberGenerator &rng);
};
}

#endif
