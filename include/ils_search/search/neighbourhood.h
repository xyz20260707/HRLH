#ifndef PYVRP_SEARCH_NEIGHBOURHOOD_H
#define PYVRP_SEARCH_NEIGHBOURHOOD_H

#include "ProblemData.h"

#include <vector>

namespace pyvrp::search
{

struct NeighbourhoodParams
{
    double const weightWaitTime;
    size_t const numNeighbours;
    bool const symmetricProximity;

    NeighbourhoodParams(double weightWaitTime = 0.2,
                        size_t numNeighbours = 50,
                        bool symmetricProximity = true);

    bool operator==(NeighbourhoodParams const &other) const = default;
};

std::vector<std::vector<size_t>>
computeNeighbours(ProblemData const &data, NeighbourhoodParams const &params);
}

#endif
