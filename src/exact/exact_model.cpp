#include <cstdlib>
#include <sstream>
#include "problem_model.hpp"

#ifdef HRLH_MODEL_WITH_GUROBI
#include "gurobi_c++.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <tuple>
#include <set>

namespace hrlh_model {

namespace {
using Arc = std::pair<int,int>;
using ArcVeh = std::tuple<int,int,int>;
using ItemVeh = std::pair<int,int>;
using Pair = std::pair<int,int>;
using PairVeh = std::tuple<int,int,int>;
using Clock = std::chrono::steady_clock;

static bool has_arc(const std::set<Arc>& arcs, int i, int j) { return arcs.count({i,j}) != 0; }

static GRBLinExpr lhat_expr(int m, const std::map<int, Item>& item_by_id, const std::map<int, GRBVar>& d1, const std::map<int, GRBVar>& d2) {
    const auto& it = item_by_id.at(m);
    return it.length * d1.at(m) + it.width * d2.at(m);
}
static GRBLinExpr what_expr(int m, const std::map<int, Item>& item_by_id, const std::map<int, GRBVar>& d1, const std::map<int, GRBVar>& d2) {
    const auto& it = item_by_id.at(m);
    return it.width * d1.at(m) + it.length * d2.at(m);
}

double remaining_seconds(const Clock::time_point& start, double max_wall_seconds) {
    if (max_wall_seconds <= 0.0) return std::numeric_limits<double>::infinity();
    return max_wall_seconds - std::chrono::duration<double>(Clock::now() - start).count();
}

bool is_definitive_no_retry_status(int status) {
    return status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED;
}

int optimize_with_feasibility_retry(GRBModel& model,
                                    const Clock::time_point& solve_start,
                                    double max_wall_seconds,
                                    bool retry_if_no_incumbent,
                                    int retry_solution_limit,
                                    int retry_mip_focus,
                                    bool retry_remove_local_time_limit,
                                    bool retry_verbose) {
    model.optimize();
    int first_status = model.get(GRB_IntAttr_Status);
    int sol_count = model.get(GRB_IntAttr_SolCount);
    if (sol_count > 0) return first_status;
    if (!retry_if_no_incumbent || is_definitive_no_retry_status(first_status)) return first_status;

    double remaining = remaining_seconds(solve_start, max_wall_seconds);
    if (std::isfinite(remaining) && remaining <= 1e-9) return first_status;

    if (retry_verbose) {
        std::cerr << "[HRLH] first solve had no incumbent, status="
                  << gurobi_status_name(first_status)
                  << "; retrying with SolutionLimit=" << std::max(1, retry_solution_limit)
                  << ", MIPFocus=" << retry_mip_focus << "\n";
    }

    model.reset();
    model.set(GRB_IntParam_SolutionLimit, std::max(1, retry_solution_limit));
    model.set(GRB_IntParam_MIPFocus, retry_mip_focus);
    if (retry_remove_local_time_limit) {
        if (std::isfinite(remaining)) {
            model.set(GRB_DoubleParam_TimeLimit, std::max(0.001, remaining));
        } else {
            model.set(GRB_DoubleParam_TimeLimit, GRB_INFINITY);
        }
    }
    model.optimize();
    return model.get(GRB_IntAttr_Status);
}
}

ModelSolution solve_exact_model(const ProblemInstance& source_inst,
                                   std::optional<double> time_limit,
                                   std::optional<double> mip_gap,
                                   bool verbose,
                                   double max_wall_seconds,
                                   bool gurobi_retry_if_no_incumbent,
                                   int gurobi_retry_solution_limit,
                                   int gurobi_retry_mip_focus,
                                   bool gurobi_retry_remove_local_time_limit,
                                   bool gurobi_retry_verbose) {
    const auto inst = normalized_rotatable_rack_dimensions(source_inst);
    validate_instance(inst);
    auto solve_start = Clock::now();
    std::map<int, Item> item_by_id; for (const auto& it: inst.items) item_by_id.emplace(it.item_id,it);
    auto items = std::vector<int>{}; for (const auto& it: inst.items) items.push_back(it.item_id);
    auto mandatory = inst.mandatory_items(); auto optional = inst.optional_items(); auto racks = inst.rack_items(); auto pallets = inst.pallet_items();
    std::set<int> rack_set(racks.begin(), racks.end());
    auto suppliers=inst.suppliers; auto vehicles=inst.vehicles; int start=inst.start_depot(), end=inst.end_depot(); auto nodes=inst.nodes();
    std::vector<Arc> arcs;
    std::set<Arc> arc_set;
    for (int i: nodes) if (i != end) for (int j: nodes) if (j != start && i != j && !(i==start && j==end)) { arcs.push_back({i,j}); arc_set.insert({i,j}); }
    int L=inst.vehicle_length, W=inst.vehicle_width, H=inst.vehicle_height;
    double latest_return_max = 0.0;
    for (int supplier : suppliers) {
        latest_return_max = std::max(latest_return_max, inst.latest_return.at(supplier));
    }
    const auto time_upper_bound = [&](int node) -> double {
        return node == start || node == end ? latest_return_max : inst.latest_return.at(node);
    };
    std::map<Pair,int> compatible;
    for (int m: racks) for (int b: racks) if (m != b) {
        compatible[{m,b}] = inst.q_mb(m, b);
    }

    auto env_ptr = std::make_shared<GRBEnv>(true);
    env_ptr->start();
    auto model_ptr = std::make_shared<GRBModel>(*env_ptr);
    GRBModel& model = *model_ptr;
    model.set(GRB_StringAttr_ModelName, "HRLH_routing_loading_scheduling");
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.set(GRB_IntParam_OutputFlag, verbose ? 1 : 0);
    if (time_limit.has_value()) {
        double limit = *time_limit;
        double remaining = remaining_seconds(solve_start, max_wall_seconds);
        if (std::isfinite(remaining)) limit = std::min(limit, remaining);
        model.set(GRB_DoubleParam_TimeLimit, std::max(0.001, limit));
    } else {
        double remaining = remaining_seconds(solve_start, max_wall_seconds);
        if (std::isfinite(remaining)) model.set(GRB_DoubleParam_TimeLimit, std::max(0.001, remaining));
    }
    if (mip_gap.has_value()) model.set(GRB_DoubleParam_MIPGap, *mip_gap);

    std::map<ArcVeh, GRBVar> X;
    for (auto [i,j]: arcs) for (int k: vehicles) X[{i,j,k}] = model.addVar(0,1,0,GRB_BINARY,"X["+std::to_string(i)+","+std::to_string(j)+","+std::to_string(k)+"]");
    std::map<ItemVeh, GRBVar> u, T;
    for (int m: items) for (int k: vehicles) u[{m,k}] = model.addVar(0,1,0,GRB_BINARY,"u["+std::to_string(m)+","+std::to_string(k)+"]");
    for (int i: nodes) for (int k: vehicles) T[{i,k}] = model.addVar(0,time_upper_bound(i),0,GRB_CONTINUOUS,"T["+std::to_string(i)+","+std::to_string(k)+"]");
    std::map<int, GRBVar> x,y,z,delta1,delta2;
    for (int m: items) {
        x[m]=model.addVar(0,GRB_INFINITY,0,GRB_CONTINUOUS,"x["+std::to_string(m)+"]");
        y[m]=model.addVar(0,GRB_INFINITY,0,GRB_CONTINUOUS,"y["+std::to_string(m)+"]");
        z[m]=model.addVar(0,GRB_INFINITY,0,GRB_CONTINUOUS,"z["+std::to_string(m)+"]");
        delta1[m]=model.addVar(0,1,0,GRB_BINARY,"delta1["+std::to_string(m)+"]");
        delta2[m]=model.addVar(0,1,0,GRB_BINARY,"delta2["+std::to_string(m)+"]");
    }
    std::vector<Pair> ordered_pairs; for (int m: items) for (int b: items) if (m!=b) ordered_pairs.push_back({m,b});
    std::map<Pair, GRBVar> sx, by;
    for (auto [m,b]: ordered_pairs) {
        sx[{m,b}] = model.addVar(0,1,0,GRB_BINARY,"sx["+std::to_string(m)+","+std::to_string(b)+"]");
        by[{m,b}] = model.addVar(0,1,0,GRB_BINARY,"by["+std::to_string(m)+","+std::to_string(b)+"]");
    }
    std::vector<Pair> rack_ordered_pairs; for (int m: racks) for (int b: racks) if (m!=b) rack_ordered_pairs.push_back({m,b});
    std::map<Pair, GRBVar> a;
    for (auto [m,b]: rack_ordered_pairs) a[{m,b}] = model.addVar(0,1,0,GRB_BINARY,"a["+std::to_string(m)+","+std::to_string(b)+"]");
    std::vector<Pair> unordered_pairs; for (std::size_t ii=0; ii<items.size(); ++ii) for (std::size_t jj=ii+1; jj<items.size(); ++jj) unordered_pairs.push_back({items[ii],items[jj]});
    std::map<PairVeh, GRBVar> y_same_vehicle; std::map<Pair, GRBVar> y_pair;
    for (auto [m,b]: unordered_pairs) {
        for (int k: vehicles) y_same_vehicle[{m,b,k}] = model.addVar(0,1,0,GRB_BINARY,"ysv["+std::to_string(m)+","+std::to_string(b)+","+std::to_string(k)+"]");
        y_pair[{m,b}] = model.addVar(0,1,0,GRB_BINARY,"ypair["+std::to_string(m)+","+std::to_string(b)+"]");
    }
    model.update();

    for (int i: suppliers) {
        GRBLinExpr e=0; for (int j: nodes) if (has_arc(arc_set,i,j)) for (int k: vehicles) e += X[{i,j,k}];
        model.addConstr(e == 1, "visit_supplier_once["+std::to_string(i)+"]");
    }
    for (int k: vehicles) {
        GRBLinExpr start_out=0,end_in=0;
        for (int j: nodes) if (has_arc(arc_set,start,j)) start_out += X[{start,j,k}];
        for (int i: nodes) if (has_arc(arc_set,i,end)) end_in += X[{i,end,k}];
        model.addConstr(start_out == end_in, "start_end_balance["+std::to_string(k)+"]");
        model.addConstr(start_out <= 1, "vehicle_used_at_most_once["+std::to_string(k)+"]");
    }
    for (int h: suppliers) for (int k: vehicles) {
        GRBLinExpr inc=0,out=0;
        for (int i: nodes) if (has_arc(arc_set,i,h)) inc += X[{i,h,k}];
        for (int j: nodes) if (has_arc(arc_set,h,j)) out += X[{h,j,k}];
        model.addConstr(inc == out, "flow_balance");
    }
    for (int j: suppliers) for (int k: vehicles) {
        GRBLinExpr visited=0; for (int i: nodes) if (has_arc(arc_set,i,j)) visited += X[{i,j,k}];
        model.addConstr(T[{j,k}] <= inst.latest_return.at(j) * visited, "time_visit_activation");
    }
    for (int k: vehicles) {
        GRBLinExpr start_out=0; for (int j: nodes) if (has_arc(arc_set,start,j)) start_out += X[{start,j,k}];
        model.addConstr(T[{start,k}] <= latest_return_max * start_out, "departure_time_activation");
    }
    for (auto [i,j]: arcs) for (int k: vehicles) {
        double service_i = inst.service_time.count(i) ? inst.service_time.at(i) : 0.0;
        const double U_i = time_upper_bound(i);
        const double travel_ij = inst.travel_time.at({i,j});
        const double M_prop_ij = U_i + service_i + travel_ij;
        model.addConstr(T[{j,k}] >= T[{i,k}] + service_i + travel_ij - M_prop_ij * (1 - X[{i,j,k}]), "time_propagation");
    }
    for (int i: suppliers) for (int k: vehicles) {
        GRBLinExpr visited=0; for (int j: nodes) if (has_arc(arc_set,i,j)) visited += X[{i,j,k}];
        const double latest_i = inst.latest_return.at(i);
        const double M_return_i = std::max(0.0, latest_return_max - latest_i);
        model.addConstr(T[{end,k}] <= latest_i + M_return_i * (1 - visited), "latest_return");
    }
    for (int m: mandatory) { GRBLinExpr e=0; for (int k: vehicles) e+=u[{m,k}]; model.addConstr(e==1,"mandatory_loaded_once"); }
    for (int m: optional) { GRBLinExpr e=0; for (int k: vehicles) e+=u[{m,k}]; model.addConstr(e<=1,"optional_loaded_at_most_once"); }
    for (int k: vehicles) {
        GRBLinExpr e=0; for (int m: items) { const auto& it=item_by_id.at(m); e += (1.0*it.length*it.width*it.height) * u[{m,k}]; }
        model.addConstr(e <= 1.0*L*W*H, "vehicle_volume_necessary");
    }
    int h_min=H; for (int m: racks) h_min = std::min(h_min, item_by_id.at(m).height);
    int max_lower = std::max(0, static_cast<int>(std::floor(1.0*H/h_min))-1);
    for (int m: racks) { GRBLinExpr e=0; for (int b: racks) if (b!=m) e+=a[{b,m}]; model.addConstr(e <= max_lower, "stack_layer_upper_bound"); }

    for (auto [m,b]: unordered_pairs) {
        for (int k: vehicles) {
            model.addConstr(y_same_vehicle[{m,b,k}] <= u[{m,k}], "sameveh_le_um");
            model.addConstr(y_same_vehicle[{m,b,k}] <= u[{b,k}], "sameveh_le_ub");
            model.addConstr(y_same_vehicle[{m,b,k}] >= u[{m,k}] + u[{b,k}] - 1, "sameveh_ge_sum");
        }
        GRBLinExpr e=0; for (int k: vehicles) e+=y_same_vehicle[{m,b,k}]; model.addConstr(y_pair[{m,b}] == e, "sameveh_any_def");
    }
    for (auto [m,b]: unordered_pairs) {
        GRBLinExpr plane_sum = sx[{m,b}] + by[{m,b}] + sx[{b,m}] + by[{b,m}];
        GRBLinExpr stack_sum = 0; if (rack_set.count(m) && rack_set.count(b)) stack_sum = a[{m,b}] + a[{b,m}];
        model.addConstr(plane_sum + stack_sum >= y_pair[{m,b}], "relation_lower");
        model.addConstr(plane_sum <= 2 * (y_pair[{m,b}] - stack_sum), "plane_relation_upper");
        model.addConstr(sx[{m,b}] <= y_pair[{m,b}], "sx_act1"); model.addConstr(sx[{b,m}] <= y_pair[{m,b}], "sx_act2");
        model.addConstr(by[{m,b}] <= y_pair[{m,b}], "by_act1"); model.addConstr(by[{b,m}] <= y_pair[{m,b}], "by_act2");
        if (rack_set.count(m) && rack_set.count(b)) { model.addConstr(stack_sum <= y_pair[{m,b}], "stack_relation_upper"); model.addConstr(a[{m,b}] <= y_pair[{m,b}], "stack_act1"); model.addConstr(a[{b,m}] <= y_pair[{m,b}], "stack_act2"); }
    }
    for (int m: items) for (int b: items) if (m < b) { model.addConstr(sx[{m,b}]+sx[{b,m}] <= 1, "sx_antisym"); model.addConstr(by[{m,b}]+by[{b,m}] <= 1, "by_antisym"); }
    for (int m: racks) for (int b: racks) if (m < b) model.addConstr(a[{m,b}]+a[{b,m}] <= 1, "stack_antisym");
    for (int m: items) model.addConstr(delta1[m]+delta2[m] == 1, "orientation_one");
    for (auto [m,b]: ordered_pairs) {
        model.addConstr(x[m] + lhat_expr(m,item_by_id,delta1,delta2) <= x[b] + L * (1 - sx[{m,b}]), "x_nonoverlap");
        model.addConstr(y[m] + what_expr(m,item_by_id,delta1,delta2) <= y[b] + W * (1 - by[{m,b}]), "y_nonoverlap");
    }
    for (int m: items) {
        model.addConstr(x[m] + lhat_expr(m,item_by_id,delta1,delta2) <= L, "x_boundary");
        model.addConstr(y[m] + what_expr(m,item_by_id,delta1,delta2) <= W, "y_boundary");
        model.addConstr(z[m] + item_by_id.at(m).height <= H, "z_boundary");
    }
    for (auto [m,b]: rack_ordered_pairs) {
        model.addConstr(x[b] >= x[m] - L * (1 - a[{m,b}]), "stack_x_ge");
        model.addConstr(x[b] <= x[m] + L * (1 - a[{m,b}]), "stack_x_le");
        model.addConstr(y[b] >= y[m] - W * (1 - a[{m,b}]), "stack_y_ge");
        model.addConstr(y[b] <= y[m] + W * (1 - a[{m,b}]), "stack_y_le");
        model.addConstr(z[b] >= z[m] + item_by_id.at(m).height - H * (1 - a[{m,b}]), "stack_z_above");
        model.addConstr(delta1[m] >= delta1[b] - (1 - a[{m,b}]), "stack_ori_ge");
        model.addConstr(delta1[m] <= delta1[b] + (1 - a[{m,b}]), "stack_ori_le");
    }
    for (int m: pallets) model.addConstr(z[m] == 0, "pallet_on_floor");
    for (int m: items) { int supplier=item_by_id.at(m).supplier; for (int k: vehicles) { GRBLinExpr visit=0; for (int j: nodes) if (has_arc(arc_set,supplier,j)) visit += X[{supplier,j,k}]; model.addConstr(visit >= u[{m,k}], "load_implies_visit"); } }
    for (int m: mandatory) { int supplier_m=item_by_id.at(m).supplier; for (int i: suppliers) if (supplier_m==i) for (int k: vehicles) { GRBLinExpr visit=0; for (int j: nodes) if (has_arc(arc_set,i,j)) visit += X[{i,j,k}]; model.addConstr(visit <= u[{m,k}], "visit_implies_mandatory_load"); } }
    for (auto [m,b]: rack_ordered_pairs) model.addConstr(a[{m,b}] <= compatible[{m,b}], "stack_compatibility");

    GRBLinExpr transport=0, latest=0, optional_profit_expr=0;
    for (auto [i,j]: arcs) for (int k: vehicles) transport += inst.alpha_distance_cost * inst.distance.at({i,j}) * X[{i,j,k}];
    for (int i: nodes) if (has_arc(arc_set,i,end)) for (int k: vehicles) transport += inst.fixed_vehicle_cost * X[{i,end,k}];
    for (int k: vehicles) latest += T[{start,k}];
    for (int m: optional) for (int k: vehicles) optional_profit_expr += static_cast<double>(item_by_id.at(m).score) * u[{m,k}];
    model.setObjectiveN(transport, 0, 3, 1.0, 0.0, 0.0, "min_transport_cost");
    model.setObjectiveN(-latest, 1, 1, 1.0, 0.0, 0.0, "max_latest_departure");
    model.setObjectiveN(-optional_profit_expr, 2, 2, 1.0, 0.0, 0.0, "max_optional_profit");
    model.update();
    model.set(GRB_DoubleParam_MIPGap, 0.0);
    model.set(GRB_DoubleParam_MIPGapAbs, 0.0);
    model.set(GRB_DoubleParam_IntFeasTol, 1e-7);
    model.set(GRB_DoubleParam_FeasibilityTol, 1e-8);
    model.set(GRB_IntParam_Threads, 1);
    if (const char* limits = std::getenv("HRLH_EXACT_PASS_LIMITS")) {
        std::istringstream values(limits);
        for (int pass = 0; pass < 3; ++pass) {
            double seconds;
            if (!(values >> seconds) || seconds <= 0) throw std::runtime_error("Invalid objective pass limits");
            GRBEnv env = model.getMultiobjEnv(pass);
            env.set(GRB_DoubleParam_TimeLimit, seconds);
            env.set(GRB_IntParam_Threads, 1);
        }
    }
    int final_status = optimize_with_feasibility_retry(model,
                                                       solve_start,
                                                       max_wall_seconds,
                                                       gurobi_retry_if_no_incumbent,
                                                       gurobi_retry_solution_limit,
                                                       gurobi_retry_mip_focus,
                                                       gurobi_retry_remove_local_time_limit,
                                                       verbose || gurobi_retry_verbose);

    ModelSolution sol;
    sol.status = final_status;
    sol.status_name = gurobi_status_name(sol.status);
    sol.runtime = model.get(GRB_DoubleAttr_Runtime);
    int solcnt = model.get(GRB_IntAttr_SolCount);
    if (solcnt > 0) {
        try { sol.mip_gap = model.get(GRB_DoubleAttr_MIPGap); } catch (...) { if (sol.status == GRB_OPTIMAL) sol.mip_gap = 0.0; }
        sol.transport_cost = transport.getValue(); sol.latest_departure_sum = latest.getValue(); sol.optional_profit = optional_profit_expr.getValue();
        sol.objective_values["transport_cost"] = *sol.transport_cost;
        sol.objective_values["latest_departure_sum"] = *sol.latest_departure_sum;
        sol.objective_values["optional_profit"] = *sol.optional_profit;
        for (int k: vehicles) {
            std::vector<Arc> chosen; for (auto [i,j]: arcs) if (X[{i,j,k}].get(GRB_DoubleAttr_X) > 0.5) chosen.push_back({i,j});
            sol.route_arcs[k] = chosen; sol.routes[k] = extract_route_from_arcs(chosen,start,end);
            std::vector<int> loaded; for (int m: items) if (u[{m,k}].get(GRB_DoubleAttr_X) > 0.5) loaded.push_back(m);
            sol.vehicle_items[k] = loaded;
        }
        std::set<int> loaded_all; for (const auto& kv: sol.vehicle_items) for (int m: kv.second) loaded_all.insert(m); sol.loaded_items_all.assign(loaded_all.begin(), loaded_all.end());
        for (int i: nodes) for (int k: vehicles) sol.time_solution[{i,k}] = T[{i,k}].get(GRB_DoubleAttr_X);
        for (int m: items) {
            SolutionItemInfo info; const auto& it=item_by_id.at(m);
            for (int k: vehicles) if (u[{m,k}].get(GRB_DoubleAttr_X) > 0.5) info.loaded_vehicles.push_back(k);
            info.loaded = !info.loaded_vehicles.empty(); if (info.loaded) info.loaded_vehicle = info.loaded_vehicles.front();
            info.x=x[m].get(GRB_DoubleAttr_X); info.y=y[m].get(GRB_DoubleAttr_X); info.z=z[m].get(GRB_DoubleAttr_X);
            info.delta1=static_cast<int>(std::round(delta1[m].get(GRB_DoubleAttr_X))); info.delta2=static_cast<int>(std::round(delta2[m].get(GRB_DoubleAttr_X)));
            info.actual_length = it.length * info.delta1 + it.width * info.delta2;
            info.actual_width = it.width * info.delta1 + it.length * info.delta2;
            info.actual_height = it.height; info.is_pallet=it.is_pallet; info.is_optional=it.is_optional; info.supplier=it.supplier; info.dock=it.dock; info.size_class=it.size_class; info.score=it.score;
            sol.item_solution[m]=info;
        }
        for (auto [m,b]: rack_ordered_pairs) {
            if (a[{m,b}].get(GRB_DoubleAttr_X) > 0.5) {
                StackRelation rec; rec.bottom=m; rec.top=b;
                auto bm = sol.item_solution[m].loaded_vehicle;
                auto tm = sol.item_solution[b].loaded_vehicle;
                rec.bottom_vehicle = bm; rec.top_vehicle = tm;
                sol.stack_relations.push_back(rec);
            }
        }
        for (auto [m,b]: unordered_pairs) {
            if (y_pair[{m,b}].get(GRB_DoubleAttr_X) > 0.5) sol.same_vehicle_pairs.push_back({m,b});
            if (sx[{m,b}].get(GRB_DoubleAttr_X) > 0.5) sol.planar_relations.push_back({"x_left", m, b});
            if (sx[{b,m}].get(GRB_DoubleAttr_X) > 0.5) sol.planar_relations.push_back({"x_left", b, m});
            if (by[{m,b}].get(GRB_DoubleAttr_X) > 0.5) sol.planar_relations.push_back({"y_front", m, b});
            if (by[{b,m}].get(GRB_DoubleAttr_X) > 0.5) sol.planar_relations.push_back({"y_front", b, m});
        }
    }
    return sol;
}

}
#endif
