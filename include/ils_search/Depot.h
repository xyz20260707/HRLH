#ifndef PYVRP_DEPOT_H
#define PYVRP_DEPOT_H

#include "Measure.h"

#include <limits>
#include <string>

namespace pyvrp
{

struct Depot
{
    size_t const location;
    Duration const serviceDuration;
    Duration const twEarly;
    Duration const twLate;
    char const *name;

    Depot(size_t location,
          Duration twEarly = 0,
          Duration twLate = std::numeric_limits<Duration>::max(),
          Duration serviceDuration = 0,
          std::string name = "");

    bool operator==(Depot const &other) const;

    Depot(Depot const &depot);
    Depot(Depot &&depot);

    Depot &operator=(Depot const &depot) = delete;
    Depot &operator=(Depot &&depot) = delete;

    ~Depot();
};
}

#endif
