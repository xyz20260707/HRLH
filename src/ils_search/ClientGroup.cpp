#include "ClientGroup.h"

#include <cstring>

using pyvrp::ClientGroup;

namespace
{

static char *duplicate(char const *src)
{
    char *dst = new char[std::strlen(src) + 1];
    std::strcpy(dst, src);
    return dst;
}
}

ClientGroup::ClientGroup(std::vector<size_t> clients,
                         bool required,
                         std::string name)
    : required(required), name(duplicate(name.data()))
{
    for (auto const client : clients)
        addClient(client);
}

ClientGroup::ClientGroup(ClientGroup const &group)
    : clients_(group.clients_),
      required(group.required),
      name(duplicate(group.name))
{
}

ClientGroup::ClientGroup(ClientGroup &&group)
    : clients_(std::move(group.clients_)),
      required(group.required),
      name(group.name)
{
    group.name = nullptr;
}

bool ClientGroup::operator==(ClientGroup const &other) const
{

    return clients_ == other.clients_
        && required == other.required
        && mutuallyExclusive == other.mutuallyExclusive
        && std::strcmp(name, other.name) == 0;

}

ClientGroup::~ClientGroup() { delete[] name; }

bool ClientGroup::empty() const { return clients_.empty(); }

size_t ClientGroup::size() const { return clients_.size(); }

std::vector<size_t>::const_iterator ClientGroup::begin() const
{
    return clients_.begin();
}

std::vector<size_t>::const_iterator ClientGroup::end() const
{
    return clients_.end();
}

std::vector<size_t> const &ClientGroup::clients() const { return clients_; }

void ClientGroup::addClient(size_t client)
{
    if (std::find(clients_.begin(), clients_.end(), client) != clients_.end())
        throw std::invalid_argument("Client already in group.");

    clients_.push_back(client);
}
