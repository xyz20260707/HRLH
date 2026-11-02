#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <iostream>
#include <vector>

#ifdef HRLH_MODEL_WITH_GUROBI
#include "gurobi_c++.h"
#endif

namespace hrlh_model {

enum class StackMode : int {
    NO_STACKING = 0,
    SAME_MATERIAL = 1,
    SAME_CONTAINER = 2,
    SAME_FOOTPRINT = 3,
};

struct Item {
    const int item_id;
    const int supplier;
    const int length;
    const int width;
    const int height;
    const bool is_pallet;
    const bool is_optional;
    const std::int64_t score;
    const int dock;
    const int size_class;
    const std::string part_code;
    const std::string container_code;
    const StackMode stack_mode;

    Item() = delete;
    Item(int item_id_, int supplier_, int length_, int width_, int height_,
         bool is_pallet_=false, bool is_optional_=false, std::int64_t score_=0,
         int dock_=1, int size_class_=1,
         std::string part_code_={}, std::string container_code_={},
         StackMode stack_mode_=StackMode::SAME_FOOTPRINT);
};

bool same_stack_candidate_group(const Item& lhs, const Item& rhs);

struct ProblemInstance {
    int vehicle_length = 120;
    int vehicle_width = 100;
    int vehicle_height = 120;
    double alpha_distance_cost = 1.0;
    double fixed_vehicle_cost = 80.0;
    std::vector<int> suppliers;
    std::vector<int> vehicles;
    std::vector<Item> items;
    std::map<std::pair<int,int>, double> distance;
    std::map<std::pair<int,int>, double> travel_time;
    std::map<int, double> service_time;
    std::map<int, double> latest_return;

    std::map<std::pair<int,int>, int> stack_compatibility;

    int q_mb(int m, int b) const;
    bool stack_compatible(int m, int b) const { return q_mb(m, b) == 1; }
    int start_depot() const { return 0; }
    int end_depot() const;
    std::vector<int> nodes() const;
    std::vector<int> mandatory_items() const;
    std::vector<int> optional_items() const;
    std::vector<int> pallet_items() const;
    std::vector<int> rack_items() const;
};

std::pair<int,int> canonical_size_key(int length, int width);
ProblemInstance normalized_rotatable_rack_dimensions(const ProblemInstance& source);
void validate_instance(const ProblemInstance& inst);

struct SolutionItemInfo {
    bool loaded = false;
    std::vector<int> loaded_vehicles;
    std::optional<int> loaded_vehicle;
    double x = 0.0, y = 0.0, z = 0.0;
    int delta1 = 1, delta2 = 0;
    double actual_length = 0.0, actual_width = 0.0, actual_height = 0.0;
    bool is_pallet = false;
    bool is_optional = false;
    int supplier = 0;
    int dock = 1;
    int size_class = 1;
    std::int64_t score = 0;
};

using RouteNode = std::variant<int, std::string>;

struct StackRelation {
    int bottom = 0;
    int top = 0;
    std::optional<int> bottom_vehicle;
    std::optional<int> top_vehicle;
};

struct PlanarRelation {
    std::string type;
    int first = 0;
    int second = 0;
};

struct ModelSolution {
    int status = 0;
    std::string status_name;
    double runtime = 0.0;
    std::optional<double> mip_gap;
    std::optional<double> transport_cost;
    std::optional<double> latest_departure_sum;
    std::optional<double> optional_profit;
    std::map<std::string, double> objective_values;
    std::map<int, std::vector<RouteNode>> routes;
    std::map<int, std::vector<std::pair<int,int>>> route_arcs;
    std::map<int, std::vector<int>> vehicle_items;
    std::vector<int> loaded_items_all;
    std::map<int, SolutionItemInfo> item_solution;
    std::vector<StackRelation> stack_relations;
    std::vector<PlanarRelation> planar_relations;
    std::vector<std::pair<int,int>> same_vehicle_pairs;
    std::map<std::pair<int,int>, double> time_solution;
};

struct ValidationReport {
    bool ok = false;
    int num_violations = 0;
    int num_warnings = 0;
    double manual_transport_cost = 0.0;
    double manual_latest_departure_sum = 0.0;
    double manual_optional_profit = 0.0;
    std::map<std::string, double> manual_objectives;
    std::vector<std::string> violations;
    std::vector<std::string> warnings;
};

ValidationReport validate_solution_against_model(const ProblemInstance& inst, const ModelSolution& sol, double tol=1e-5, bool verbose=true);
std::vector<RouteNode> extract_route_from_arcs(const std::vector<std::pair<int,int>>& chosen_arcs, int start, int end);
std::string gurobi_status_name(int status_code);
std::string solution_detail_json(const ProblemInstance& inst, const ModelSolution& sol);
ModelSolution solve_exact_model(
    const ProblemInstance& inst,
    std::optional<double> time_limit = 300.0,
    std::optional<double> mip_gap = 0.0,
    bool verbose = true,
    double max_wall_seconds = 0.0,
    bool gurobi_retry_if_no_incumbent = true,
    int gurobi_retry_solution_limit = 1,
    int gurobi_retry_mip_focus = 1,
    bool gurobi_retry_remove_local_time_limit = true,
    bool gurobi_retry_verbose = false
);

}
