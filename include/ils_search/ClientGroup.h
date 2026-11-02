#ifndef PYVRP_CLIENTGROUP_H
#define PYVRP_CLIENTGROUP_H

#include "Measure.h"

#include <string>
#include <vector>

namespace pyvrp
{

class ClientGroup
{
    std::vector<size_t> clients_;

public:
    bool const required;
    bool const mutuallyExclusive = true;
    char const *name;

    explicit ClientGroup(std::vector<size_t> clients = {},
                         bool required = true,
                         std::string name = "");

    bool operator==(ClientGroup const &other) const;

    ClientGroup(ClientGroup const &group);
    ClientGroup(ClientGroup &&group);

    ClientGroup &operator=(ClientGroup const &group) = delete;
    ClientGroup &operator=(ClientGroup &&group) = delete;

    ~ClientGroup();

    bool empty() const;
    size_t size() const;

    std::vector<size_t>::const_iterator begin() const;
    std::vector<size_t>::const_iterator end() const;

    std::vector<size_t> const &clients() const;

    void addClient(size_t client);
};
}

#endif
