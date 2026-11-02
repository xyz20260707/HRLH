#ifndef PYVRP_LOCATION_H
#define PYVRP_LOCATION_H

#include "Measure.h"

#include <string>

namespace pyvrp
{

struct Location
{
    Coordinate const x;
    Coordinate const y;
    char const *name;

    Location(Coordinate x, Coordinate y, std::string name = "");

    bool operator==(Location const &other) const;

    Location(Location const &location);
    Location(Location &&location);

    Location &operator=(Location const &location) = delete;
    Location &operator=(Location &&location) = delete;

    ~Location();
};
}

#endif
