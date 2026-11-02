#include "LoadSegment.h"

#include <fstream>

using pyvrp::Load;
using pyvrp::LoadSegment;

Load LoadSegment::delivery() const { return delivery_; }

Load LoadSegment::pickup() const { return pickup_; }

Load LoadSegment::load() const { return load_; }

LoadSegment::LoadSegment(Client const &client, size_t dimension)
    : delivery_(client.delivery[dimension]),
      pickup_(client.pickup[dimension]),
      load_(std::max<Load>(delivery_, pickup_))
{
}

LoadSegment::LoadSegment(VehicleType const &vehicleType,
                         size_t dimension)
    :

      pickup_(vehicleType.initialLoad[dimension]),
      load_(vehicleType.initialLoad[dimension])
{
}

