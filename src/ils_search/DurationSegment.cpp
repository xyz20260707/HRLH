#include "DurationSegment.h"

#include <fstream>

using pyvrp::Duration;
using pyvrp::DurationSegment;

Duration DurationSegment::prevEndLate() const { return prevEndLate_; }

Duration DurationSegment::releaseTime() const { return releaseTime_; }

Duration DurationSegment::slack() const
{

    auto const prevSlack = std::max<Duration>(prevEndLate_ - releaseTime_, 0);
    return std::min(startLate() - startEarly(), prevSlack);
}

DurationSegment::DurationSegment(Client const &client)
    : duration_(client.serviceDuration),
      startEarly_(client.twEarly),
      startLate_(client.twLate),
      releaseTime_(client.releaseTime)
{
}

DurationSegment::DurationSegment(Depot const &depot,
                                 Duration const serviceDuration)
    : duration_(serviceDuration),
      startEarly_(depot.twEarly),
      startLate_(depot.twLate)
{
}

DurationSegment::DurationSegment(VehicleType const &vehicleType,
                                 Duration const twLate)
    : startEarly_(vehicleType.twEarly), startLate_(twLate)
{
}

