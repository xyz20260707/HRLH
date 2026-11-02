#ifndef PYVRP_VEHICLETYPE_H
#define PYVRP_VEHICLETYPE_H

#include "Measure.h"

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace pyvrp
{

struct VehicleType
{
    size_t const numAvailable;
    size_t const startDepot;
    size_t const endDepot;
    std::vector<Load> const capacity;
    Duration const twEarly;
    Duration const twLate;
    Duration const shiftDuration;
    Distance const maxDistance;
    Cost const fixedCost;
    Cost const unitDistanceCost;
    Cost const unitDurationCost;
    size_t const profile;
    Duration const startLate;
    std::vector<Load> const initialLoad;
    std::vector<size_t> const reloadDepots;
    size_t const maxReloads;
    Duration const maxOvertime;
    Cost const unitOvertimeCost;
    Duration const maxDuration;
    char const *name;

    VehicleType(size_t numAvailable = 1,
                std::vector<Load> capacity = {},
                size_t startDepot = 0,
                size_t endDepot = 0,
                Cost fixedCost = 0,
                Duration twEarly = 0,
                Duration twLate = std::numeric_limits<Duration>::max(),
                Duration shiftDuration = std::numeric_limits<Duration>::max(),
                Distance maxDistance = std::numeric_limits<Distance>::max(),
                Cost unitDistanceCost = 1,
                Cost unitDurationCost = 0,
                size_t profile = 0,
                std::optional<Duration> startLate = std::nullopt,
                std::vector<Load> initialLoad = {},
                std::vector<size_t> reloadDepots = {},
                size_t maxReloads = std::numeric_limits<size_t>::max(),
                Duration maxOvertime = 0,
                Cost unitOvertimeCost = 0,
                std::string name = "");

    bool operator==(VehicleType const &other) const;

    VehicleType(VehicleType const &vehicleType);
    VehicleType(VehicleType &&vehicleType);

    VehicleType &operator=(VehicleType const &vehicleType) = delete;
    VehicleType &operator=(VehicleType &&vehicleType) = delete;

    ~VehicleType();

    VehicleType replace(std::optional<size_t> numAvailable,
                        std::optional<std::vector<Load>> capacity,
                        std::optional<size_t> startDepot,
                        std::optional<size_t> endDepot,
                        std::optional<Cost> fixedCost,
                        std::optional<Duration> twEarly,
                        std::optional<Duration> twLate,
                        std::optional<Duration> shiftDuration,
                        std::optional<Distance> maxDistance,
                        std::optional<Cost> unitDistanceCost,
                        std::optional<Cost> unitDurationCost,
                        std::optional<size_t> profile,
                        std::optional<Duration> startLate,
                        std::optional<std::vector<Load>> initialLoad,
                        std::optional<std::vector<size_t>> reloadDepots,
                        std::optional<size_t> maxReloads,
                        std::optional<Duration> maxOvertime,
                        std::optional<Cost> unitOvertimeCost,
                        std::optional<std::string> name) const;

    size_t maxTrips() const;
};
}

#endif
