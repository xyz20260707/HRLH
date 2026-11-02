#ifndef PYVRP_DURATIONSEGMENT_H
#define PYVRP_DURATIONSEGMENT_H

#include "Matrix.h"
#include "Measure.h"
#include "ProblemData.h"

#include <cassert>
#include <iosfwd>

namespace pyvrp
{

class DurationSegment
{
    Duration duration_ = 0;
    Duration timeWarp_ = 0;
    Duration startEarly_ = 0;
    Duration startLate_
        = std::numeric_limits<Duration>::max();
    Duration releaseTime_ = 0;
    Duration cumDuration_ = 0;
    Duration cumTimeWarp_ = 0;
    Duration prevEndLate_
        = std::numeric_limits<Duration>::max();

public:
    [[nodiscard]] static inline DurationSegment
    merge(Duration const edgeDuration,
          DurationSegment const &first,
          DurationSegment const &second);

    [[nodiscard]] static inline DurationSegment
    merge(DurationSegment const &first, DurationSegment const &second);

    [[nodiscard]] inline DurationSegment finaliseBack() const;

    [[nodiscard]] inline DurationSegment finaliseFront() const;

    [[nodiscard]] inline Duration duration() const;

    [[nodiscard]] inline Duration
    timeWarp(Duration maxDuration = std::numeric_limits<Duration>::max()) const;

    [[nodiscard]] inline Duration startEarly() const;

    [[nodiscard]] inline Duration startLate() const;

    [[nodiscard]] inline Duration endEarly() const;

    [[nodiscard]] inline Duration endLate() const;

    [[nodiscard]] Duration prevEndLate() const;

    [[nodiscard]] Duration releaseTime() const;

    [[nodiscard]] Duration slack() const;

    DurationSegment(Client const &client);

    DurationSegment(Depot const &depot, Duration const serviceDuration);

    DurationSegment(VehicleType const &vehicleType, Duration const twLate);

    inline DurationSegment(Duration duration = 0,
                           Duration timeWarp = 0,
                           Duration startEarly = 0,
                           Duration startLate
                           = std::numeric_limits<Duration>::max(),
                           Duration releaseTime = 0,
                           Duration cumDuration = 0,
                           Duration cumTimeWarp = 0,
                           Duration prevEndLate
                           = std::numeric_limits<Duration>::max());

    inline DurationSegment(DurationSegment const &) = default;
    inline DurationSegment(DurationSegment &&) = default;

    inline DurationSegment &operator=(DurationSegment const &) = default;
    inline DurationSegment &operator=(DurationSegment &&) = default;
};

DurationSegment DurationSegment::merge(Duration const edgeDuration,
                                       DurationSegment const &first,
                                       DurationSegment const &second)
{

    auto const atSecond = first.duration_ - first.timeWarp_ + edgeDuration;

    auto const diffTw = first.startEarly_ + atSecond > second.startLate_
                            ? first.startEarly_ + atSecond - second.startLate_
                            : 0;

    auto const diffWait = second.startEarly_ - atSecond > first.startLate_
                              ? second.startEarly_ - atSecond - first.startLate_
                              : 0;

    auto const secondLate
        = atSecond > second.startLate_ - std::numeric_limits<Duration>::max()
              ? second.startLate_ - atSecond
              : second.startLate_;

    return {first.duration_ + second.duration_ + edgeDuration + diffWait,
            first.timeWarp_ + second.timeWarp_ + diffTw,
            std::max(first.startEarly_, second.startEarly_ - atSecond)
                - diffWait,
            std::min(first.startLate_, secondLate) + diffTw,
            std::max(first.releaseTime_, second.releaseTime_),
            first.cumDuration_ + second.cumDuration_,
            first.cumTimeWarp_ + second.cumTimeWarp_,
            first.prevEndLate_};
}

DurationSegment DurationSegment::merge(DurationSegment const &first,
                                       DurationSegment const &second)
{
    return merge(0, first, second);
}

DurationSegment DurationSegment::finaliseBack() const
{

    DurationSegment const prev = {0, 0, 0, prevEndLate_};
    DurationSegment const finalised = merge(prev, finaliseFront());

    return {0,
            0,
            finalised.endEarly(),

            std::numeric_limits<Duration>::max(),

            finalised.endEarly(),
            cumDuration_ + finalised.duration(),
            cumTimeWarp_ + finalised.timeWarp(),
            finalised.endLate()};
}

DurationSegment DurationSegment::finaliseFront() const
{

    DurationSegment const release = {0, 0, startEarly(), startLate()};
    return merge(release, {duration_, timeWarp_, startEarly_, startLate_});
}

Duration DurationSegment::duration() const
{
    auto const duration = cumDuration_ + duration_;
    return duration + std::max<Duration>(startEarly() - prevEndLate_, 0);
}

Duration DurationSegment::timeWarp(Duration maxDuration) const
{
    auto const timeWarp = cumTimeWarp_ + timeWarp_;
    auto const netDuration = duration() - timeWarp;

    return timeWarp

           + std::max<Duration>(releaseTime_ - startLate_, 0)

           + (netDuration > maxDuration ? netDuration - maxDuration : 0);
}

Duration DurationSegment::startEarly() const
{

    return std::max(startEarly_, releaseTime_);
}

Duration DurationSegment::startLate() const
{

    return std::max(startLate_, releaseTime_);
}

Duration DurationSegment::endEarly() const
{
    auto const tripDuration = duration() - cumDuration_;
    auto const tripTimeWarp = timeWarp() - cumTimeWarp_;
    return startEarly() + tripDuration - tripTimeWarp;
}

Duration DurationSegment::endLate() const
{
    auto const tripDuration = duration() - cumDuration_;
    auto const tripTimeWarp = timeWarp() - cumTimeWarp_;
    auto const netDuration = tripDuration - tripTimeWarp;
    return netDuration > std::numeric_limits<Duration>::max() - startLate()
               ? std::numeric_limits<Duration>::max()
               : startLate() + netDuration;
}

DurationSegment::DurationSegment(Duration duration,
                                 Duration timeWarp,
                                 Duration startEarly,
                                 Duration startLate,
                                 Duration releaseTime,
                                 Duration cumDuration,
                                 Duration cumTimeWarp,
                                 Duration prevEndLate)
    : duration_(duration),
      timeWarp_(timeWarp),
      startEarly_(startEarly),
      startLate_(startLate),
      releaseTime_(releaseTime),
      cumDuration_(cumDuration),
      cumTimeWarp_(cumTimeWarp),
      prevEndLate_(prevEndLate)
{
}
}

#endif
