#include "Location.h"

#include <cstring>

using pyvrp::Location;

namespace
{

static char *duplicate(char const *src)
{
    char *dst = new char[std::strlen(src) + 1];
    std::strcpy(dst, src);
    return dst;
}
}

Location::Location(Coordinate x, Coordinate y, std::string name)
    : x(x), y(y), name(duplicate(name.data()))
{
}

Location::Location(Location const &location)
    : x(location.x), y(location.y), name(duplicate(location.name))
{
}

Location::Location(Location &&location)
    : x(location.x), y(location.y), name(location.name)
{
    location.name = nullptr;
}

bool Location::operator==(Location const &other) const
{
    return x == other.x && y == other.y && std::strcmp(name, other.name) == 0;
}

Location::~Location() { delete[] name; }
