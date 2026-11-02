#ifndef PYVRP_ROUTE_H
#define PYVRP_ROUTE_H

#include "Activity.h"
#include "CostEvaluator.h"
#include "Measure.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"

#include <iosfwd>
#include <vector>

namespace pyvrp
{

class Route
{
    using Client = size_t;
    using Depot = size_t;
    using VehicleType = size_t;
    using Activities = std::vector<Activity>;

    void validate(ProblemData const &data, Activities const &activities) const;
    void setSchedule(ProblemData const &data, Activities const &activities);
    void setDistance(ProblemData const &data);
    void setLoad(ProblemData const &data);
    void setOtherStatistics(ProblemData const &data);

public:

    class ScheduledActivity : public Activity
    {
        size_t trip_ = 0;
        Duration startTime_ = 0;
        Duration endTime_ = 0;
        Duration waitDuration_ = 0;
        Duration timeWarp_ = 0;

    public:
        ScheduledActivity(Activity activity,
                          size_t trip,
                          Duration startTime,
                          Duration endTime,
                          Duration waitDuration,
                          Duration timeWarp);

        bool operator==(ScheduledActivity const &other) const = default;

        [[nodiscard]] size_t trip() const;
        [[nodiscard]] Duration startTime() const;
        [[nodiscard]] Duration endTime() const;
        [[nodiscard]] Duration duration() const;
        [[nodiscard]] Duration waitDuration() const;
        [[nodiscard]] Duration timeWarp() const;
    };

private:
    using Schedule = std::vector<ScheduledActivity>;

    Schedule schedule_ = {};
    Distance distance_ = 0;
    Cost distanceCost_ = 0;
    Distance excessDistance_ = 0;
    std::vector<Load> delivery_;
    std::vector<Load> pickup_;
    std::vector<Load> excessLoad_;
    Duration duration_ = 0;
    Duration overtime_ = 0;
    Cost durationCost_ = 0;
    Duration timeWarp_ = 0;
    Duration travel_ = 0;
    Duration service_ = 0;
    Duration startTime_ = 0;
    Duration releaseTime_ = 0;
    Duration slack_ = 0;
    Duration latestDepartureTime_ = std::numeric_limits<Duration>::max();
    Cost fixedVehicleCost_ = 0;
    Cost prizes_ = 0;

    VehicleType vehicleType_;

public:
    [[nodiscard]] bool empty() const;

    [[nodiscard]] size_t size() const;

    [[nodiscard]] size_t numClients() const;

    [[nodiscard]] size_t numDepots() const;

    [[nodiscard]] size_t numTrips() const;

    [[nodiscard]] ScheduledActivity const &operator[](size_t idx) const;

    [[nodiscard]] Schedule::const_iterator begin() const;
    [[nodiscard]] Schedule::const_iterator end() const;

    [[nodiscard]] Schedule const &schedule() const;

    [[nodiscard]] Cost fixedVehicleCost() const;

    [[nodiscard]] Distance distance() const;

    [[nodiscard]] Cost distanceCost() const;

    [[nodiscard]] Distance excessDistance() const;

    [[nodiscard]] std::vector<Load> const &delivery() const;

    [[nodiscard]] std::vector<Load> const &pickup() const;

    [[nodiscard]] std::vector<Load> const &excessLoad() const;

    [[nodiscard]] Duration duration() const;

    [[nodiscard]] Duration overtime() const;

    [[nodiscard]] Cost durationCost() const;

    [[nodiscard]] Duration serviceDuration() const;

    [[nodiscard]] Duration timeWarp() const;

    [[nodiscard]] Duration waitDuration() const;

    [[nodiscard]] Duration startTime() const;

    [[nodiscard]] Duration endTime() const;

    [[nodiscard]] Duration slack() const;

    [[nodiscard]] Duration latestDepartureTime() const;

    [[nodiscard]] Duration releaseTime() const;

    [[nodiscard]] Cost prizes() const;

    [[nodiscard]] VehicleType vehicleType() const;

    [[nodiscard]] Depot startDepot() const;

    [[nodiscard]] Depot endDepot() const;

    [[nodiscard]] bool isFeasible() const;

    [[nodiscard]] bool hasExcessLoad() const;

    [[nodiscard]] bool hasExcessDistance() const;

    [[nodiscard]] bool hasTimeWarp() const;

    bool operator==(Route const &other) const;

    Route &operator=(Route const &other) = default;
    Route &operator=(Route &&other) = default;

    Route() = delete;

    Route(Route const &other) = default;
    Route(Route &&other) = default;

    Route(ProblemData const &data,
          std::vector<Client> const &visits,
          VehicleType vehicleType);

    Route(ProblemData const &data,
          Activities const &activities,
          VehicleType vehicleType);


};

template <>
Cost CostEvaluator::penalisedCost(Route const &route) const;
}

#endif
