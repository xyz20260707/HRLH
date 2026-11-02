#pragma once
#include "hrlh.hpp"
#include "loading_bounds.hpp"
#include <functional>
#include <random>

namespace hrlh::routing {

struct RouteMetrics {
    double cost = 0;
    double distance = 0;
    double duration = 0;
    double latest_return = 0;
    double latest_departure = 0;
    std::vector<double> loads;
    double capacity_excess = 0;
    double time_excess = 0;
    bool feasible() const { return capacity_excess == 0 && time_excess == 0; }
};
struct PlanMetrics {
    double cost = 0;
    double capacity_excess = 0;
    double time_excess = 0;
    int used_vehicles = 0;
    int missing_suppliers = 0;
    bool duplicate_or_unknown = false;
    bool routing_feasible = false;
};
enum class PackingAssessment { Feasible, RoutingInfeasible, CertifiedInfeasible, HeuristicFailure, NotEvaluated };

enum class NumericMapping { OriginalUnits, ILSCompatible };

class Problem {
public:
    Problem(const hrlh_model::ProblemInstance& instance,
            const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
            const HrlhConfig& config,
            NumericMapping mapping = NumericMapping::OriginalUnits);
    const hrlh_model::ProblemInstance& instance;
    std::vector<std::vector<double>> demands;
    std::vector<std::vector<std::int64_t>> integer_demands;
    std::vector<std::int64_t> integer_capacities;
    std::map<int, std::size_t> index;
    std::vector<std::string> dimension_names;
    bool use_time;
    const bool ils_compatible;
    const bool align_transport_cost;
    double max_arc_cost = 0;
    double max_normalized_demand = 0;
    RouteMetrics evaluate_route(const Route& route) const;
    PlanMetrics evaluate(const RoutePlan& plan, bool allow_missing = false) const;
    double directed_cost(int from, int to) const;
    double directed_time(int from, int to) const;
    double matrix_proximity(int lhs, int rhs) const;
};

struct SearchCallbacks {
    std::function<void()> stop;
    std::function<PackingAssessment(const RoutePlan&)> evaluate_integrated;

    std::function<PackingAssessment(const Route&)> evaluate_partial_route;
};

inline double uniform01(std::mt19937_64& rng) {
    return static_cast<double>(rng() >> 11) / 9007199254740992.0;
}
inline std::size_t uniform_index(std::mt19937_64& rng, std::size_t n) {
    if (!n) throw std::invalid_argument("empty random choice");
    std::uniform_int_distribution<std::size_t> choose(0, n-1);
    return choose(rng);
}
}
