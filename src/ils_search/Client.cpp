#include <algorithm>
#include <optional>
#include "Client.h"

#include <cassert>
#include <cstring>

using pyvrp::Client;

namespace
{

static char *duplicate(char const *src)
{
    char *dst = new char[std::strlen(src) + 1];
    std::strcpy(dst, src);
    return dst;
}

auto &pad(auto &vec1, auto const &vec2)
{
    vec1.resize(std::max(vec1.size(), vec2.size()));
    return vec1;
}

bool isNegative(auto value) { return value < 0; }
}

Client::Client(size_t location,
               std::vector<Load> delivery,
               std::vector<Load> pickup,
               Duration serviceDuration,
               Duration twEarly,
               Duration twLate,
               Duration releaseTime,
               Duration latestReturn,
               Cost prize,
               bool required,
               std::optional<size_t> group,
               std::string name)
    : location(location),
      serviceDuration(serviceDuration),
      twEarly(twEarly),
      twLate(twLate),
      delivery(pad(delivery, pickup)),
      pickup(pad(pickup, delivery)),
      releaseTime(releaseTime),
      latestReturn(latestReturn),
      prize(prize),
      required(required),
      group(group),
      name(duplicate(name.data()))
{
    assert(delivery.size() == pickup.size());

    if (std::any_of(delivery.begin(), delivery.end(), isNegative<Load>))
        throw std::invalid_argument("delivery amounts must be >= 0.");

    if (std::any_of(pickup.begin(), pickup.end(), isNegative<Load>))
        throw std::invalid_argument("pickup amounts must be >= 0.");

    if (serviceDuration < 0)
        throw std::invalid_argument("service_duration must be >= 0.");

    if (twEarly > twLate)
        throw std::invalid_argument("tw_early must be <= tw_late.");

    if (twEarly < 0)
        throw std::invalid_argument("tw_early must be >= 0.");

    if (releaseTime > twLate)
        throw std::invalid_argument("release_time must be <= tw_late");

    if (releaseTime < 0)
        throw std::invalid_argument("release_time must be >= 0.");

    if (latestReturn < 0)
        throw std::invalid_argument("latest_return must be >= 0.");

    if (prize < 0)
        throw std::invalid_argument("prize must be >= 0.");
}

Client::Client(Client const &client)
    : location(client.location),
      serviceDuration(client.serviceDuration),
      twEarly(client.twEarly),
      twLate(client.twLate),
      delivery(client.delivery),
      pickup(client.pickup),
      releaseTime(client.releaseTime),
      latestReturn(client.latestReturn),
      prize(client.prize),
      required(client.required),
      group(client.group),
      name(duplicate(client.name))
{
}

Client::Client(Client &&client)
    : location(client.location),
      serviceDuration(client.serviceDuration),
      twEarly(client.twEarly),
      twLate(client.twLate),
      delivery(std::move(client.delivery)),
      pickup(std::move(client.pickup)),
      releaseTime(client.releaseTime),
      latestReturn(client.latestReturn),
      prize(client.prize),
      required(client.required),
      group(client.group),
      name(client.name)
{
    client.name = nullptr;
}

Client::~Client() { delete[] name; }

bool Client::operator==(Client const &other) const
{

    return location == other.location
        && delivery == other.delivery
        && pickup == other.pickup
        && serviceDuration == other.serviceDuration
        && twEarly == other.twEarly
        && twLate == other.twLate
        && releaseTime == other.releaseTime
        && latestReturn == other.latestReturn
        && prize == other.prize
        && required == other.required
        && group == other.group
        && std::strcmp(name, other.name) == 0;

}
