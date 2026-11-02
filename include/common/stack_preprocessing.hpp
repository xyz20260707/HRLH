#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hrlh_components {
namespace stack_preprocessing {

using GroupKey = std::tuple<int, int, int>;

struct RackItem {
    int item_id = 0;
    int supplier = 0;
    int length = 0;
    int width = 0;
    int height = 0;
    std::int64_t score = 0;
    bool is_optional = false;
    bool is_pallet = false;

    std::set<int> compatible_item_ids;

    GroupKey group_key() const {
        const auto key = canonical_size_key();
        return {supplier, key.first, key.second};
    }
    std::pair<int, int> canonical_size_key() const
    {
        return length <= width ? std::make_pair(length, width) : std::make_pair(width, length);
    }
    std::int64_t area() const { return 1LL * length * width; }
};

struct OptionalRack {
    int item_id = 0;
    int height = 0;
    std::int64_t score = 0;
    std::set<int> compatible_item_ids;
};

bool stack_accepts(const RackItem& item, const RackItem& partner);
bool stack_compatible(const RackItem& lhs, const RackItem& rhs);
bool stack_accepts(const OptionalRack& item, const OptionalRack& partner);
bool stack_compatible(const OptionalRack& lhs, const OptionalRack& rhs);

struct RackGroup {
    GroupKey group_key;
    std::vector<RackItem> mandatory_items;
    std::vector<RackItem> optional_items;
    int base_length = 0;
    int base_width = 0;
    std::int64_t unit_area = 0;
    std::int64_t supplier_pallet_area = 0;
};

struct MinStackCountResult {
    std::vector<std::vector<int>> stacks;
    int stack_count = 0;
    bool proven_optimal = false;
};

struct BaselineStackOptimizationResult {
    GroupKey group_key;
    std::vector<std::vector<int>> mandatory_assignments;
    std::vector<std::vector<int>> final_stacks;
    std::vector<int> selected_optional;
    std::vector<int> unselected_optional;
    int baseline_stack_count = 0;
    std::int64_t baseline_optional_profit = 0;
    std::int64_t baseline_stack_area = 0;
    int base_length = 0;
    int base_width = 0;
    int stack_capacity = 0;
    bool phase1_proven_optimal = false;
    bool phase2_proven_optimal = false;
};

struct ValueFunctionResult {
    int n_stacks = 0;
    int stack_capacity = 0;
    std::optional<std::int64_t> objective;
    std::vector<int> selected_items;
    std::vector<std::vector<int>> stacks;
    std::vector<int> stack_heights;
    bool proven_optimal = false;
};

struct SpreadJointCurvePoint {
    int stack_count = 0;
    int extra_stack_count = 0;
    std::int64_t optional_profit = 0;
    std::vector<std::vector<int>> stacks;
    bool proven_optimal = false;
};

struct GroupPreprocessResult {
    GroupKey group_key;
    int base_length = 0;
    int base_width = 0;
    std::int64_t unit_area = 0;
    BaselineStackOptimizationResult baseline_result;
    std::int64_t area_A_g = 0;
    std::int64_t area_rem_g = 0;
    std::int64_t R_area_g = 0;
    int R_all_g = 0;
    int R_max_g = 0;
    std::map<int, ValueFunctionResult> value_table;
    std::map<int, SpreadJointCurvePoint> spread_joint_curve;
    bool spread_joint_enabled = false;

    std::map<int, std::optional<std::int64_t>> f_values() const;
};

struct PreprocessAllGroupsResult {
    int vehicle_length = 0;
    int vehicle_width = 0;
    int vehicle_height = 0;
    std::map<GroupKey, GroupPreprocessResult> groups;

};

struct SolverConfig {
    double mandatory_stack_time_limit = 300.0;
    double baseline_joint_time_limit = 300.0;
    double time_limit_R_all = 300.0;
    double spread_joint_time_limit = 300.0;
    double mip_gap = 0.0;
    bool verbose = false;

    bool require_optimal = false;

    bool enable_spread_joint = false;
    bool symmetry_breaking = true;
    bool load_order_symmetry = false;
    bool use_warm_start = true;

    double max_wall_seconds = 0.0;
    double gurobi_single_mip_time_limit = 0.0;

    int gurobi_symmetry = 2;

    bool gurobi_retry_enabled = true;
    bool gurobi_retry_if_no_incumbent = true;
    int gurobi_retry_solution_limit = 1;
    int gurobi_retry_mip_focus = 1;
    bool gurobi_retry_remove_local_time_limit = true;
    bool gurobi_retry_respect_global_wall_seconds = true;
    bool gurobi_retry_verbose = false;
    std::function<void()> interrupt_check{};
    std::function<double()> remaining_seconds{};
};

struct StackPreprocessingInput {
    int vehicle_length = 0;
    int vehicle_width = 0;
    int vehicle_height = 0;
    std::vector<RackItem> items;
};

std::map<GroupKey, RackGroup> group_racks_by_supplier_and_footprint(
    const std::vector<RackItem>& items,
    bool strict_same_canonical_size = true);

std::map<GroupKey, RackGroup> group_racks_from_instance(
    const StackPreprocessingInput& instance,
    bool strict_same_canonical_size = true);

MinStackCountResult solve_min_stack_count_gurobi(
    const std::vector<RackItem>& items,
    int capacity,
    std::optional<int> max_stacks = std::nullopt,
    double time_limit = 300.0,
    double mip_gap = 0.0,
    bool verbose = false,
    bool symmetry_breaking = true,
    bool load_order_symmetry = false,
    bool use_warm_start = true,
    bool require_optimal = false,
    int gurobi_symmetry = 2);

MinStackCountResult solve_min_stack_count_gurobi(
    const std::vector<OptionalRack>& items,
    int capacity,
    std::optional<int> max_stacks = std::nullopt,
    double time_limit = 300.0,
    double mip_gap = 0.0,
    bool verbose = false,
    bool symmetry_breaking = true,
    bool load_order_symmetry = false,
    bool use_warm_start = true,
    bool require_optimal = false,
    int gurobi_symmetry = 2);

BaselineStackOptimizationResult optimize_baseline_stacks_gurobi(
    const GroupKey& group_key,
    const std::vector<RackItem>& mandatory_items,
    const std::vector<RackItem>& optional_items,
    int capacity,
    int base_length,
    int base_width,
    const SolverConfig& cfg = {});

ValueFunctionResult solve_value_function_gurobi(
    const std::vector<OptionalRack>& optional_items,
    int n_stacks,
    int stack_capacity,
    const SolverConfig& cfg = {});

std::map<int, ValueFunctionResult> solve_value_table_gurobi(
    const std::vector<OptionalRack>& optional_items,
    int max_stacks,
    int stack_capacity,
    const SolverConfig& cfg = {});

GroupPreprocessResult preprocess_rack_group(
    const RackGroup& group,
    int vehicle_length,
    int vehicle_width,
    int vehicle_height,
    const SolverConfig& cfg = {});

PreprocessAllGroupsResult preprocess_stacks(
    const StackPreprocessingInput& instance,
    bool strict_same_canonical_size = true,
    const SolverConfig& cfg = {});

MinStackCountResult solve_min_stack_count_fallback(const std::vector<RackItem>& items, int capacity);
MinStackCountResult solve_min_stack_count_fallback(const std::vector<OptionalRack>& items, int capacity);
BaselineStackOptimizationResult optimize_baseline_stacks_fallback(
    const GroupKey& group_key,
    const std::vector<RackItem>& mandatory_items,
    const std::vector<RackItem>& optional_items,
    int capacity,
    int base_length,
    int base_width);
ValueFunctionResult solve_value_function_fallback(
    const std::vector<OptionalRack>& optional_items,
    int n_stacks,
    int stack_capacity);
std::map<int, ValueFunctionResult> solve_value_table_fallback(
    const std::vector<OptionalRack>& optional_items,
    int max_stacks,
    int stack_capacity);
GroupPreprocessResult preprocess_rack_group_fallback(
    const RackGroup& group,
    int vehicle_length,
    int vehicle_width,
    int vehicle_height,
    const SolverConfig& cfg = {});
PreprocessAllGroupsResult preprocess_stacks_fallback(
    const StackPreprocessingInput& instance,
    bool strict_same_canonical_size = true,
    const SolverConfig& cfg = {});

void validate_baseline_stack_result(const BaselineStackOptimizationResult& result,
                           const std::vector<RackItem>& mandatory_items,
                           const std::vector<RackItem>& optional_items,
                           int capacity);
void validate_value_result(const ValueFunctionResult& result,
                           const std::vector<OptionalRack>& items);
void validate_monotone_value_tables(const PreprocessAllGroupsResult& result);

}
}
