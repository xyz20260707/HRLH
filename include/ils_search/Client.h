#ifndef PYVRP_CLIENT_H
#define PYVRP_CLIENT_H

#include "Measure.h"

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace pyvrp
{

struct Client
{
    size_t const location;
    Duration const serviceDuration;
    Duration const twEarly;
    Duration const twLate;
    std::vector<Load> const delivery;
    std::vector<Load> const pickup;
    Duration const releaseTime;
    Duration const latestReturn;
    Cost const prize;
    bool const required;
    std::optional<size_t> const group;
    char const *name;

    Client(size_t location,
           std::vector<Load> delivery = {},
           std::vector<Load> pickup = {},
           Duration serviceDuration = 0,
           Duration twEarly = 0,
           Duration twLate = std::numeric_limits<Duration>::max(),
           Duration releaseTime = 0,
           Duration latestReturn = std::numeric_limits<Duration>::max(),
           Cost prize = 0,
           bool required = true,
           std::optional<size_t> group = std::nullopt,
           std::string name = "");

    bool operator==(Client const &other) const;

    Client(Client const &client);
    Client(Client &&client);

    Client &operator=(Client const &client) = delete;
    Client &operator=(Client &&client) = delete;

    ~Client();
};
}

#endif
