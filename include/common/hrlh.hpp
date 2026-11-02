#pragma once

#include "problem_model.hpp"
#include "stack_preprocessing.hpp"
#include "skyline_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace hrlh {

using Route = std::vector<int>;
using RoutePlan = std::vector<Route>;

struct RoutePackingCacheConfig {
    bool enabled = true;
    bool enable_feasible_pool = true;
    std::size_t max_feasible_entries = 50000;
    bool skip_skyline_if_cached_reaches_upper_bound = true;
    bool skip_static_pruning_on_feasible_hit = true;
    bool rerun_skyline_on_feasible_hit = true;
    bool use_cached_best_if_current_worse = true;
    bool store_layout = true;
};

struct HrlhConfig {
#ifdef HRLH_STACKS_WITH_GUROBI
    bool use_gurobi_preprocess = true;
#else
    bool use_gurobi_preprocess = false;
#endif
    bool strict_same_canonical_size = true;

    bool require_preprocess_optimal = false;
    int max_iterations = 1000000000;
    int packing_mutation_iters = 3;
    double packing_mutation_swap_ratio = 0.50;
    std::uint64_t seed = 205062346ULL;
    bool enforce_time_feasible = true;
    bool enforce_volume_necessary = true;
    bool verbose = false;

    int initial_diversification_starts = 20;

    double feasible_descent_budget_fraction = 0.20;

    int feasible_descent_no_improvement_limit = 60000;

    double max_wall_seconds = 60.0;
    int max_packing_calls = 0;
    bool enable_ils_routing_search = true;
    int ils_iterations = 1000000000;
    bool ils_use_mandatory_floor_area_capacity = true;

    bool ils_use_dff_capacity_dimensions = true;
    bool ils_align_transport_cost = true;
    bool ils_enforce_latest_return = true;
    bool ils_use_singleton_warm_start = true;
    int ils_penalty_solutions_between_updates = 500;
    double ils_penalty_increase = 1.50;
    double ils_penalty_decrease = 0.90;
    double ils_penalty_target_feasible = 0.65;
    double ils_penalty_feas_tolerance = 0.05;
    double ils_penalty_min = 0.1;
    double ils_penalty_max = 100000.0;
    std::vector<std::string> ils_operator_names{};
    int ils_neighbours = 50;
    double ils_neighbour_weight_wait_time = 0.2;
    bool ils_neighbour_symmetric_proximity = true;
    int ils_no_improvement_limit = 150000;
    int ils_history_length = 300;
    bool ils_exhaustive_on_best = true;
    int ils_min_perturbations = 1;
    int ils_max_perturbations = 25;

    bool enable_packing_guided_route_repair = true;

    bool isolate_route_repair_rng = true;
    bool enforce_route_repair_cost_gate = true;

    int deep_packing_retry_iters = 45;

    bool enable_proven_infeasible_subset_nogoods = true;

    hrlh_components::stack_preprocessing::SolverConfig preprocess_config = [] {
        hrlh_components::stack_preprocessing::SolverConfig value;
        value.enable_spread_joint = true;
        value.mandatory_stack_time_limit = 1.0;
        value.baseline_joint_time_limit = 1.0;
        value.time_limit_R_all = 1.0;
        value.spread_joint_time_limit = 1.0;
        value.gurobi_single_mip_time_limit = 1.0;
        value.gurobi_retry_remove_local_time_limit = false;
        return value;
    }();
    RoutePackingCacheConfig route_packing_cache{};

    bool enable_optional_demand_profit_intensification = true;

    double optional_profit_intensification_budget_fraction = 0.10;
};

struct RoutePackingPlacedRect {
    int spec_index = -1;
    bool is_optional = false;
    int rank_in_type = -1;
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double length = 0.0;
};

struct RoutePackingDetail {
    Route route;
    bool feasible = true;
    int bin_number = 0;
    std::int64_t optional_score_from_2d = 0;
    std::int64_t fixed_inserted_score = 0;
    std::int64_t total_optional_priority_score = 0;
    std::int64_t profit_upper_bound = 0;
    bool static_lb_infeasible = false;
    std::string error_message;
    std::vector<PackingTypeSpecCpp> specs;
    std::vector<int> loaded_mandatory_count_by_spec;
    std::vector<int> loaded_optional_count_by_spec;
    std::vector<RoutePackingPlacedRect> placed_rects;

    std::vector<std::vector<std::vector<int>>> material_stacks_by_spec;
};

struct PackingPlanEval {
    bool feasible = true;
    std::int64_t total_optional_priority_score = 0;
    std::vector<RoutePackingDetail> route_details;
};

struct CandidateSolution {
    RoutePlan routes;
    double transport_cost = 0.0;
    std::int64_t optional_profit = 0;
    double latest_departure_sum = 0.0;
    std::vector<double> route_latest_departures;
    PackingPlanEval packing;
    std::string signature;
};

struct SearchControlState {
    int packing_calls = 0;
    bool time_budget_exceeded = false;
    bool packing_call_budget_exceeded = false;
    double feasible_descent_budget_seconds = 0.0;
    long long feasible_descent_attempts = 0;
    long long feasible_descent_acceptances = 0;
    bool feasible_seed_found = false;
};

class CertifiedNoGoodPool {
public:
    bool insert(Route certified_route);
    const Route* matching_after_addition(
        const Route& route_before_addition,
        int incoming_supplier) const;
    const Route* matching_route(
        const Route& route) const;

private:
    void rebuild_index();
    std::vector<Route> master_antichain_;
    std::unordered_map<int, std::vector<std::size_t>> by_supplier_;
};

struct HrlhResult {
    bool has_solution = false;
    CandidateSolution best_solution;
    SearchControlState search_state;
    hrlh_components::stack_preprocessing::PreprocessAllGroupsResult preprocess;
};

hrlh_components::stack_preprocessing::StackPreprocessingInput to_rack_preprocess_instance(
    const hrlh_model::ProblemInstance& inst);

double compute_transport_cost(const hrlh_model::ProblemInstance& inst,
                              const RoutePlan& plan);

bool route_repair_cost_gate_allows(double candidate_cost,
                                       double incumbent_cost) noexcept;

bool lexicographic_cost_profit_better(double candidate_cost,
                                      std::int64_t candidate_profit,
                                      double incumbent_cost,
                                      std::int64_t incumbent_profit) noexcept;

double route_duration(const hrlh_model::ProblemInstance& inst,
                      const Route& route);

double route_latest_departure(const hrlh_model::ProblemInstance& inst,
                              const Route& route);

std::string route_plan_signature(RoutePlan plan);

std::vector<PackingTypeSpecCpp> build_route_packing_specs(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const Route& route);

std::vector<PackingTypeSpecCpp> build_route_mandatory_packing_specs(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const Route& route);

HrlhResult solve_hrlh(
    const hrlh_model::ProblemInstance& inst,
    const HrlhConfig& cfg = {});

void validate_hrlh_config(const HrlhConfig& cfg);

HrlhConfig default_hrlh_config();

bool apply_hrlh_config_value(HrlhConfig& cfg,
                                    const std::string& key,
                                    const std::string& value);

}
