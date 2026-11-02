#ifndef PYVRP_LOADSEGMENT_H
#define PYVRP_LOADSEGMENT_H

#include "Measure.h"
#include "ProblemData.h"

#include <iosfwd>

namespace pyvrp
{

class LoadSegment
{
    Load delivery_ = 0;
    Load pickup_ = 0;
    Load load_ = 0;
    Load excessLoad_ = 0;

public:
    [[nodiscard]] static inline LoadSegment merge(LoadSegment const &first,
                                                  LoadSegment const &second);

    [[nodiscard]] inline LoadSegment finalise(Load capacity) const;

    [[nodiscard]] Load delivery() const;

    [[nodiscard]] Load pickup() const;

    [[nodiscard]] Load load() const;

    [[nodiscard]] inline Load excessLoad(Load capacity) const;

    LoadSegment() = default;

    LoadSegment(Client const &client, size_t dimension);

    LoadSegment(VehicleType const &vehicleType, size_t dimension);

    inline LoadSegment(Load delivery,
                       Load pickup,
                       Load load,
                       Load excessLoad = 0);

    inline LoadSegment(LoadSegment const &) = default;
    inline LoadSegment(LoadSegment &&) = default;

    inline LoadSegment &operator=(LoadSegment const &) = default;
    inline LoadSegment &operator=(LoadSegment &&) = default;
};

LoadSegment LoadSegment::merge(LoadSegment const &first,
                               LoadSegment const &second)
{

    return {
        first.delivery_ + second.delivery_,
        first.pickup_ + second.pickup_,
        std::max(first.load_ + second.delivery_, second.load_ + first.pickup_),
        first.excessLoad_ + second.excessLoad_};
}

Load LoadSegment::excessLoad(Load capacity) const
{
    return excessLoad_ + std::max<Load>(load_ - capacity, 0);
}

LoadSegment LoadSegment::finalise(Load capacity) const
{
    return {0, 0, 0, excessLoad(capacity)};
}

LoadSegment::LoadSegment(Load delivery, Load pickup, Load load, Load excessLoad)
    : delivery_(delivery), pickup_(pickup), load_(load), excessLoad_(excessLoad)
{
}
}

#endif
