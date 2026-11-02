#include "Activity.h"

using pyvrp::Activity;

namespace
{
char type2char(Activity::ActivityType type)
{
    switch (type)
    {
    case Activity::ActivityType::DEPOT:
        return 'D';
    case Activity::ActivityType::CLIENT:
        return 'C';
    default:
        throw std::invalid_argument("Activity type not understood.");
    }
}
}

Activity::Activity(ActivityType type, size_t idx) : type_(type), idx_(idx) {}

std::ostream &operator<<(std::ostream &out, pyvrp::Activity const &activity)
{
    return out << type2char(activity.type()) << activity.idx();
}
