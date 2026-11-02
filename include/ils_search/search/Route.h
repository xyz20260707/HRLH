#ifndef PYVRP_SEARCH_ROUTE_H
#define PYVRP_SEARCH_ROUTE_H

#include "Activity.h"
#include "CostEvaluator.h"
#include "DurationSegment.h"
#include "LoadSegment.h"
#include "ProblemData.h"

#include <algorithm>
#include <cassert>
#include <concepts>
#include <iosfwd>
#include <utility>

namespace pyvrp::search
{
class SegmentProxy;

template <typename T>
concept Segment
    = requires(T arg, size_t profile, size_t dimension, size_t idx) {
          { arg.route() };
          { arg.front() } -> std::convertible_to<SegmentProxy>;
          { arg.back() } -> std::convertible_to<SegmentProxy>;
          { arg.size() } -> std::same_as<size_t>;
          { arg.numClients() } -> std::same_as<size_t>;
          { arg.startsAtReloadDepot() } -> std::same_as<bool>;
          { arg.endsAtReloadDepot() } -> std::same_as<bool>;
          { arg.distance(profile) } -> std::convertible_to<Distance>;
          { arg.duration(profile) } -> std::convertible_to<DurationSegment>;
          { arg.load(dimension) } -> std::convertible_to<LoadSegment>;
      };

namespace detail
{
template <class Tuple, std::size_t... Indices>
auto constexpr reverse_impl(Tuple &&tuple, std::index_sequence<Indices...>)
{
    return std::make_tuple(std::get<sizeof...(Indices) - 1 - Indices>(
        std::forward<Tuple>(tuple))...);
}

template <class Tuple> auto constexpr reverse(Tuple &&tuple)
{
    auto constexpr size = std::tuple_size_v<std::remove_reference_t<Tuple>>;
    auto constexpr indices = std::make_index_sequence<size>{};
    return reverse_impl(tuple, indices);
}
}

class SegmentProxy
{
    Activity activity_;
    size_t location_;

public:
    inline SegmentProxy(Activity activity, size_t location);
    inline Activity activity() const;
    inline size_t location() const;
};

class Route
{
public:

    template <Segment... Segments> class Proposal
    {
        std::tuple<Segments...> segments_;

    public:
        Proposal(Segments &&...segments);

        Route const *route() const;

        bool empty() const;

        std::pair<Cost, Distance> distance() const;

        std::pair<Cost, Duration> duration() const;

        Load excessLoad(size_t dimension) const;
    };

    class Node
    {
        friend class Route;

        Activity activity_;
        size_t pos_;
        size_t trip_;
        Route *route_;

    public:
        Node(Activity::ActivityType type, size_t idx);
        Node(Activity activity);

        [[nodiscard]] inline Activity activity() const;

        [[nodiscard]] inline size_t idx() const;

        [[nodiscard]] inline Activity::ActivityType type() const;

        [[nodiscard]] inline size_t pos() const;

        [[nodiscard]] inline size_t trip() const;

        [[nodiscard]] inline Route *route() const;

        [[nodiscard]] inline bool isClient() const;

        [[nodiscard]] inline bool isDepot() const;

        [[nodiscard]] inline bool isStartDepot() const;

        [[nodiscard]] inline bool isEndDepot() const;

        [[nodiscard]] inline bool isReloadDepot() const;

        void assign(Route *route, size_t pos, size_t trip);

        void unassign();
    };

private:
    using LoadSegments = std::vector<LoadSegment>;

    class SegmentAfter
    {
        Route const &route_;
        size_t const start;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;
        inline SegmentProxy back() const;

        inline size_t size() const;
        inline size_t numClients() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentAfter(Route const &route, size_t start);
        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline Duration latestReturn() const;
        inline LoadSegment const &load(size_t dimension) const;
    };

    class SegmentBefore
    {
        Route const &route_;
        size_t const end;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;
        inline SegmentProxy back() const;

        inline size_t size() const;
        inline size_t numClients() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBefore(Route const &route, size_t end);
        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline Duration latestReturn() const;
        inline LoadSegment const &load(size_t dimension) const;
    };

    class SegmentBetween
    {
        Route const &route_;
        size_t const start;
        size_t const end;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;
        inline SegmentProxy back() const;

        inline size_t size() const;
        inline size_t numClients() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBetween(Route const &route, size_t start, size_t end);
        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline Duration latestReturn() const;
        inline LoadSegment load(size_t dimension) const;
    };

    ProblemData const &data;

    VehicleType const &vehicleType_;

    Distance distance_;
    Cost distanceCost_;
    Distance excessDistance_;
    Duration duration_;
    Cost durationCost_;
    Duration timeWarp_;

    std::vector<Node> depots_;

    std::vector<Node *> nodes;
    std::vector<size_t> locations;

    std::vector<size_t> numClients_;

    std::vector<Distance> cumDist;

    std::vector<LoadSegments> loadAt;
    std::vector<LoadSegments> loadAfter;
    std::vector<LoadSegments> loadBefore;

    std::vector<Load> load_;
    std::vector<Load> excessLoad_;

    std::vector<DurationSegment> durAt;
    std::vector<DurationSegment> durAfter;
    std::vector<DurationSegment> durBefore;

    std::vector<Duration> latestReturnAt;
    std::vector<Duration> latestReturnAfter;
    std::vector<Duration> latestReturnBefore;

#ifndef NDEBUG

    bool dirty = false;
#endif

public:

    [[nodiscard]] inline Node *operator[](size_t idx);
    [[nodiscard]] inline Node const *operator[](size_t idx) const;

    [[nodiscard]] std::vector<Node *>::const_iterator begin() const;
    [[nodiscard]] std::vector<Node *>::const_iterator end() const;

    [[nodiscard]] inline bool isFeasible() const;

    [[nodiscard]] inline bool hasExcessLoad() const;

    [[nodiscard]] inline bool hasExcessDistance() const;

    [[nodiscard]] inline bool hasTimeWarp() const;

    [[nodiscard]] inline std::vector<Load> const &load() const;

    [[nodiscard]] inline std::vector<Load> const &excessLoad() const;

    [[nodiscard]] inline Distance excessDistance() const;

    [[nodiscard]] inline std::vector<Load> const &capacity() const;

    [[nodiscard]] inline size_t startDepot() const;

    [[nodiscard]] inline size_t endDepot() const;

    [[nodiscard]] inline Cost fixedVehicleCost() const;

    [[nodiscard]] inline Distance distance() const;

    [[nodiscard]] inline Cost distanceCost() const;

    [[nodiscard]] inline Cost unitDistanceCost() const;

    [[nodiscard]] inline bool hasDistanceCost() const;

    [[nodiscard]] inline Duration duration() const;

    [[nodiscard]] inline Duration overtime() const;

    [[nodiscard]] inline Cost durationCost() const;

    [[nodiscard]] inline Cost unitDurationCost() const;

    [[nodiscard]] inline Cost unitOvertimeCost() const;

    [[nodiscard]] inline bool hasDurationCost() const;

    [[nodiscard]] inline Duration shiftDuration() const;

    [[nodiscard]] inline Duration maxDuration() const;

    [[nodiscard]] inline Duration maxOvertime() const;

    [[nodiscard]] inline Distance maxDistance() const;

    [[nodiscard]] inline Duration timeWarp() const;

    [[nodiscard]] inline size_t profile() const;

    [[nodiscard]] inline bool empty() const;

    [[nodiscard]] inline size_t size() const;

    [[nodiscard]] inline size_t numClients() const;

    [[nodiscard]] inline size_t numDepots() const;

    [[nodiscard]] inline size_t numTrips() const;

    [[nodiscard]] inline size_t maxTrips() const;

    [[nodiscard]] inline SegmentBetween at(size_t idx) const;

    [[nodiscard]] inline SegmentAfter after(size_t start) const;

    [[nodiscard]] inline SegmentBefore before(size_t end) const;

    [[nodiscard]] inline SegmentBetween between(size_t start, size_t end) const;

    [[nodiscard]] size_t vehicleType() const;

    void clear();

    void reserve(size_t size);

    void insert(size_t idx, Node *node);

    void push_back(Node *node);

    void remove(size_t idx);

    static void swap(Node *first, Node *second);

    void update();

    bool operator==(Route const &other) const;

    Route(ProblemData const &data, size_t vehicleType);
    ~Route();
};

inline Route::Node *p(Route::Node *node)
{
    auto &route = *node->route();
    return route[node->pos() - 1];
}

inline Route::Node const *p(Route::Node const *node)
{
    auto const &route = *node->route();
    return route[node->pos() - 1];
}

inline Route::Node *n(Route::Node *node)
{
    auto &route = *node->route();
    return route[node->pos() + 1];
}

inline Route::Node const *n(Route::Node const *node)
{
    auto const &route = *node->route();
    return route[node->pos() + 1];
}

SegmentProxy::SegmentProxy(Activity activity, size_t location)
    : activity_(activity), location_(location)
{
}

Activity SegmentProxy::activity() const { return activity_; }

size_t SegmentProxy::location() const { return location_; }

Activity Route::Node::activity() const { return activity_; }

size_t Route::Node::idx() const { return activity_.idx(); }

Activity::ActivityType Route::Node::type() const { return activity_.type(); }

size_t Route::Node::pos() const { return pos_; }

size_t Route::Node::trip() const { return trip_; }

Route *Route::Node::route() const { return route_; }

bool Route::Node::isClient() const { return activity_.isClient(); }

bool Route::Node::isDepot() const { return activity_.isDepot(); }

bool Route::Node::isStartDepot() const
{
    return route_ && this == &route_->depots_[0];
}

bool Route::Node::isEndDepot() const
{
    return route_ && this == &route_->depots_[1];
}

bool Route::Node::isReloadDepot() const
{
    return isDepot() && !isStartDepot() && !isEndDepot();
}

Route::SegmentAfter::SegmentAfter(Route const &route, size_t start)
    : route_(route), start(start)
{
    assert(start < route.size());
}

Route::SegmentBefore::SegmentBefore(Route const &route, size_t end)
    : route_(route), end(end)
{
    assert(end < route.size());
}

Route::SegmentBetween::SegmentBetween(Route const &route,
                                      size_t start,
                                      size_t end)
    : route_(route), start(start), end(end)
{
    assert(start <= end && end < route.size());

    assert(route[end]->trip() - route[start]->trip() <= route[end]->isDepot());
}

Distance Route::SegmentAfter::distance([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return {route_.cumDist.back() - route_.cumDist[start]};
}

DurationSegment
Route::SegmentAfter::duration([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.durAfter[start];
}

Duration Route::SegmentAfter::latestReturn() const
{
    return route_.latestReturnAfter[start];
}

LoadSegment const &Route::SegmentAfter::load(size_t dimension) const
{
    return route_.loadAfter[dimension][start];
}

Distance Route::SegmentBefore::distance([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.cumDist[end];
}

DurationSegment
Route::SegmentBefore::duration([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.durBefore[end];
}

Duration Route::SegmentBefore::latestReturn() const
{
    return route_.latestReturnBefore[end];
}

LoadSegment const &Route::SegmentBefore::load(size_t dimension) const
{
    return route_.loadBefore[dimension][end];
}

Route const *Route::SegmentBefore::route() const { return &route_; }

SegmentProxy Route::SegmentBefore::front() const
{
    return {route_.nodes.front()->activity(), route_.locations.front()};
}

SegmentProxy Route::SegmentBefore::back() const
{
    return {route_.nodes[end]->activity(), route_.locations[end]};
}

size_t Route::SegmentBefore::size() const { return end + 1; }

size_t Route::SegmentBefore::numClients() const
{
    return route_.numClients_[end];
}

bool Route::SegmentBefore::startsAtReloadDepot() const { return false; }

bool Route::SegmentBefore::endsAtReloadDepot() const
{
    return route_.nodes[end]->isReloadDepot();
}

Route const *Route::SegmentAfter::route() const { return &route_; }

SegmentProxy Route::SegmentAfter::front() const
{
    return {route_.nodes[start]->activity(), route_.locations[start]};
}

SegmentProxy Route::SegmentAfter::back() const
{
    return {route_.nodes.back()->activity(), route_.locations.back()};
}

size_t Route::SegmentAfter::size() const { return route_.size() - start; }

size_t Route::SegmentAfter::numClients() const
{

    auto const fromStart = route_.numClients() - route_.numClients_[start];
    return fromStart + route_[start]->isClient();
}

bool Route::SegmentAfter::startsAtReloadDepot() const
{
    return route_.nodes[start]->isReloadDepot();
}
bool Route::SegmentAfter::endsAtReloadDepot() const { return false; }

Route const *Route::SegmentBetween::route() const { return &route_; }

SegmentProxy Route::SegmentBetween::front() const
{
    return {route_.nodes[start]->activity(), route_.locations[start]};
}

SegmentProxy Route::SegmentBetween::back() const
{
    return {route_.nodes[end]->activity(), route_.locations[end]};
}

size_t Route::SegmentBetween::size() const { return end - start + 1; }

size_t Route::SegmentBetween::numClients() const
{

    auto const fromStart = route_.numClients_[end] - route_.numClients_[start];
    return fromStart + route_[start]->isClient();
}

bool Route::SegmentBetween::startsAtReloadDepot() const
{
    return route_.nodes[start]->isReloadDepot();
}

bool Route::SegmentBetween::endsAtReloadDepot() const
{
    return route_.nodes[end]->isReloadDepot();
}

Distance Route::SegmentBetween::distance(size_t profile) const
{
    if (profile != route_.profile())
    {
        auto const &mat = route_.data.distanceMatrix(profile);
        Distance distance = 0;

        for (size_t step = start; step != end; ++step)
        {
            auto const from = route_.locations[step];
            auto const to = route_.locations[step + 1];
            distance += mat(from, to);
        }

        return distance;
    }

    auto const startDist = route_.cumDist[start];
    auto const endDist = route_.cumDist[end];

    assert(startDist <= endDist);
    return endDist - startDist;
}

DurationSegment
Route::SegmentBetween::duration([[maybe_unused]] size_t profile) const
{
    auto const &mat = route_.data.durationMatrix(profile);
    auto segment = route_.durAt[start];

    if (size() != 1 && route_[start]->isReloadDepot())
    {
        auto const &depot = route_.data.depot(route_[start]->idx());
        segment = DurationSegment::merge(segment, {depot.serviceDuration});
    }

    for (size_t step = start; step != end; ++step)
    {
        auto const from = route_.locations[step];
        auto const to = route_.locations[step + 1];
        auto const &durAt = route_.durAt[step + 1];
        segment = DurationSegment::merge(mat(from, to), segment, durAt);
    }

    return segment;
}

Duration Route::SegmentBetween::latestReturn() const
{
    Duration latestReturn = std::numeric_limits<Duration>::max();
    for (size_t idx = start; idx <= end; ++idx)
        latestReturn = std::min(latestReturn, route_.latestReturnAt[idx]);

    return latestReturn;
}

LoadSegment Route::SegmentBetween::load(size_t dimension) const
{
    auto const &loads = route_.loadAt[dimension];

    auto loadSegment = loads[start];
    for (size_t step = start; step != end; ++step)
        loadSegment = LoadSegment::merge(loadSegment, loads[step + 1]);

    return loadSegment;
}

bool Route::isFeasible() const
{
    assert(!dirty);
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance();
}

bool Route::hasExcessLoad() const
{
    assert(!dirty);
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

bool Route::hasExcessDistance() const
{
    assert(!dirty);
    return excessDistance() > 0;
}

bool Route::hasTimeWarp() const
{
    assert(!dirty);
    return timeWarp() > 0;
}

Route::Node *Route::operator[](size_t idx)
{
    assert(idx < nodes.size());
    return nodes[idx];
}

Route::Node const *Route::operator[](size_t idx) const
{
    assert(idx < nodes.size());
    return nodes[idx];
}

std::vector<Load> const &Route::load() const
{
    assert(!dirty);
    return load_;
}

std::vector<Load> const &Route::excessLoad() const
{
    assert(!dirty);
    return excessLoad_;
}

Distance Route::excessDistance() const
{
    assert(!dirty);
    return excessDistance_;
}

std::vector<Load> const &Route::capacity() const
{
    return vehicleType_.capacity;
}

size_t Route::startDepot() const { return vehicleType_.startDepot; }

size_t Route::endDepot() const { return vehicleType_.endDepot; }

Cost Route::fixedVehicleCost() const { return vehicleType_.fixedCost; }

Distance Route::distance() const
{
    assert(!dirty);
    return distance_;
}

Cost Route::distanceCost() const
{
    assert(!dirty);
    return distanceCost_;
}

Cost Route::unitDistanceCost() const { return vehicleType_.unitDistanceCost; }

bool Route::hasDistanceCost() const
{
    return unitDistanceCost() != 0
           || maxDistance() != std::numeric_limits<Distance>::max();
}

Duration Route::duration() const
{
    assert(!dirty);
    return duration_;
}

Duration Route::overtime() const
{
    assert(!dirty);
    return std::max<Duration>(duration() - shiftDuration(), 0);
}

Cost Route::durationCost() const
{
    assert(!dirty);
    return durationCost_;
}

Cost Route::unitDurationCost() const { return vehicleType_.unitDurationCost; }

Cost Route::unitOvertimeCost() const { return vehicleType_.unitOvertimeCost; }

bool Route::hasDurationCost() const
{

    return data.hasTimeWindows()
        || unitDurationCost() != 0
        || (unitOvertimeCost() != 0 && maxOvertime() != 0)
        || maxDuration() != std::numeric_limits<Duration>::max();

}

Duration Route::shiftDuration() const { return vehicleType_.shiftDuration; }

Duration Route::maxDuration() const { return vehicleType_.maxDuration; }

Duration Route::maxOvertime() const { return vehicleType_.maxOvertime; }

Distance Route::maxDistance() const { return vehicleType_.maxDistance; }

Duration Route::timeWarp() const
{
    assert(!dirty);
    return timeWarp_;
}

size_t Route::profile() const { return vehicleType_.profile; }

bool Route::empty() const { return numClients() == 0; }

size_t Route::size() const { return nodes.size(); }

size_t Route::numClients() const { return size() - numDepots(); }

size_t Route::numDepots() const { return depots_.size(); }

size_t Route::numTrips() const { return depots_.size() - 1; }

size_t Route::maxTrips() const { return vehicleType_.maxTrips(); }

Route::SegmentBetween Route::at(size_t idx) const
{
    assert(!dirty);
    return {*this, idx, idx};
}

Route::SegmentAfter Route::after(size_t start) const
{
    assert(!dirty);
    return {*this, start};
}

Route::SegmentBefore Route::before(size_t end) const
{
    assert(!dirty);
    return {*this, end};
}

Route::SegmentBetween Route::between(size_t start, size_t end) const
{
    assert(!dirty);
    return {*this, start, end};
}

template <Segment... Segments>
Route::Proposal<Segments...>::Proposal(Segments &&...segments)
    : segments_(std::forward<Segments>(segments)...)
{
    static_assert(sizeof...(Segments) > 0, "Proposal cannot be empty.");

    [[maybe_unused]] auto &&first = std::get<0>(segments_);
    [[maybe_unused]] auto &&last = std::get<sizeof...(Segments) - 1>(segments_);
    assert(first.route() == last.route());

    [[maybe_unused]] auto const &route = *this->route();
    assert(first.front().activity() == route[0]->activity());
    assert(last.back().activity() == route[route.size() - 1]->activity());
}

template <Segment... Segments> bool Route::Proposal<Segments...>::empty() const
{
    auto const numClients = std::apply(
        [](auto &&...args) { return (args.numClients() + ...); }, segments_);

    return numClients == 0;
}

template <Segment... Segments>
Route const *Route::Proposal<Segments...>::route() const
{
    return std::get<0>(segments_).route();
}

template <Segment... Segments>
std::pair<Cost, Distance> Route::Proposal<Segments...>::distance() const
{
    if (empty())
        return std::make_pair(0, 0);

    auto const &data = route()->data;
    auto const unitDistanceCost = route()->unitDistanceCost();
    auto const maxDistance = route()->maxDistance();
    auto const profile = route()->profile();
    auto const &matrix = data.distanceMatrix(profile);

    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto distance = segment.distance(profile);
        auto lastLoc = segment.back().location();

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            distance += matrix(lastLoc, other.front().location());
            distance += other.distance(profile);
            lastLoc = other.back().location();

            if constexpr (sizeof...(args) != 0)
                self(self, std::forward<decltype(args)>(args)...);
        };

        merge(merge, std::forward<decltype(args)>(args)...);

        auto const excess = std::max<Distance>(distance - maxDistance, 0);
        auto const cost = unitDistanceCost * static_cast<Cost>(distance);
        return std::make_pair(cost, excess);
    };

    return std::apply(fn, segments_);
}

template <Segment... Segments>
std::pair<Cost, Duration> Route::Proposal<Segments...>::duration() const
{
    if (empty())
        return std::make_pair(0, 0);

    auto const &data = route()->data;
    auto const unitDurationCost = route()->unitDurationCost();
    auto const unitOvertimeCost = route()->unitOvertimeCost();
    auto const shiftDuration = route()->shiftDuration();
    auto const maxDuration = route()->maxDuration();
    auto const profile = route()->profile();
    auto const &matrix = data.durationMatrix(profile);

    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto ds = segment.duration(profile);
        auto latestReturn = segment.latestReturn();
        auto firstLoc = segment.front().location();

        if (segment.startsAtReloadDepot())
            ds = ds.finaliseFront();

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            auto edgeDur = matrix(other.back().location(), firstLoc);

            if (other.endsAtReloadDepot())
            {

                auto const &activity = other.back().activity();
                assert(activity.isDepot());

                auto const &depot = data.depot(activity.idx());
                DurationSegment depotDS = {depot, depot.serviceDuration};
                ds = DurationSegment::merge(edgeDur, depotDS, ds);
                ds = ds.finaliseFront();

                edgeDur = 0;
            }

            ds = DurationSegment::merge(edgeDur, other.duration(profile), ds);
            latestReturn = std::min(latestReturn, other.latestReturn());
            firstLoc = other.front().location();

            if constexpr (sizeof...(args) != 0)
            {
                if (other.startsAtReloadDepot() && other.size() > 1)

                    ds = ds.finaliseFront();

                self(self, std::forward<decltype(args)>(args)...);
            }
        };

        merge(merge, std::forward<decltype(args)>(args)...);

        auto const duration = ds.duration();
        auto const overtime = std::max<Duration>(duration - shiftDuration, 0);
        auto const cost = unitDurationCost * static_cast<Cost>(duration)
                          + unitOvertimeCost * static_cast<Cost>(overtime);
        auto timeWarp = ds.timeWarp(maxDuration);
        if (latestReturn != std::numeric_limits<Duration>::max()
            && duration > latestReturn)
            timeWarp += duration - latestReturn;

        return std::make_pair(cost, timeWarp);
    };

    return std::apply(fn, detail::reverse(segments_));
}

template <Segment... Segments>
Load Route::Proposal<Segments...>::excessLoad(size_t dimension) const
{
    if (empty())
        return 0;

    auto const &capacities = route()->capacity();
    auto const capacity = capacities[dimension];

    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto ls = segment.load(dimension);
        if (segment.endsAtReloadDepot())
            ls = ls.finalise(capacity);

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            if (other.startsAtReloadDepot())
                ls = ls.finalise(capacity);

            ls = LoadSegment::merge(ls, other.load(dimension));

            if constexpr (sizeof...(args) != 0)
            {
                if (other.endsAtReloadDepot() && other.size() > 1)

                    ls = ls.finalise(capacity);

                self(self, std::forward<decltype(args)>(args)...);
            }
        };

        merge(merge, std::forward<decltype(args)>(args)...);
        return ls.excessLoad(capacity);
    };

    return std::apply(fn, segments_);
}
}


template <>
pyvrp::Cost
pyvrp::CostEvaluator::penalisedCost(pyvrp::search::Route const &route) const;

#endif
