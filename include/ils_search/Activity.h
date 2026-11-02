#ifndef PYVRP_ACTIVITY_H
#define PYVRP_ACTIVITY_H

#include <cstddef>
#include <ostream>
#include <string>

namespace pyvrp
{

class Activity
{
public:

    enum class ActivityType
    {
        DEPOT = 0,
        CLIENT = 1,
    };

private:
    ActivityType type_;
    size_t idx_;

public:
    Activity(ActivityType type, size_t idx);

    bool operator==(Activity const &other) const = default;

    inline ActivityType type() const;

    inline size_t idx() const;

    inline bool isClient() const;

    inline bool isDepot() const;
};

Activity::ActivityType Activity::type() const { return type_; }

size_t Activity::idx() const { return idx_; }

bool Activity::isClient() const { return type_ == ActivityType::CLIENT; }

bool Activity::isDepot() const { return type_ == ActivityType::DEPOT; }
}

std::ostream &operator<<(std::ostream &out, pyvrp::Activity const &activity);

#endif
