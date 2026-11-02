#include "problem_model.hpp"
#include <cfenv>
#include <cstdlib>
#include <iostream>
#include <unordered_map>

namespace hrlh_model {

namespace {
std::string bool_json(bool v) { return v ? "true" : "false"; }

std::string quote(const std::string& s) {
    std::ostringstream os;
    os << '"';
    for (char c : s) {
        switch (c) {
            case '"': os << "\\\""; break;
            case '\\': os << "\\\\"; break;
            case '\n': os << "\\n"; break;
            case '\r': os << "\\r"; break;
            case '\t': os << "\\t"; break;
            default: os << c; break;
        }
    }
    os << '"';
    return os.str();
}

template <class T>
std::string vec_json(const std::vector<T>& v) {
    std::ostringstream os;
    os << "[";
    for (std::size_t i=0;i<v.size();++i) {
        if (i) os << ",";
        os << v[i];
    }
    os << "]";
    return os.str();
}

std::string route_json(const std::vector<RouteNode>& route) {
    std::ostringstream os;
    os << "[";
    for (std::size_t i=0; i<route.size(); ++i) {
        if (i) os << ",";
        if (std::holds_alternative<int>(route[i])) os << std::get<int>(route[i]);
        else os << quote(std::get<std::string>(route[i]));
    }
    os << "]";
    return os.str();
}

std::string pair_vec_json(const std::vector<std::pair<int,int>>& v) {
    std::ostringstream os; os << "[";
    for (std::size_t i=0;i<v.size();++i) { if(i) os << ","; os << "[" << v[i].first << "," << v[i].second << "]"; }
    os << "]"; return os.str();
}

std::string double_json(double x) {
    std::ostringstream os;
    os << std::setprecision(17) << x;
    return os.str();
}

}

Item::Item(int item_id_, int supplier_, int length_, int width_, int height_, bool is_pallet_, bool is_optional_, std::int64_t score_, int dock_, int size_class_, std::string part_code_, std::string container_code_, StackMode stack_mode_)
    : item_id(item_id_), supplier(supplier_),
      length(length_), width(width_), height(height_),
      is_pallet(is_pallet_), is_optional(is_optional_), score(score_), dock(dock_), size_class(size_class_),
      part_code(std::move(part_code_)), container_code(std::move(container_code_)), stack_mode(stack_mode_) {
    if (is_optional && is_pallet) {
        throw std::invalid_argument("optional demand items must be racks, not pallets, in this model.");
    }
    const int mode = static_cast<int>(stack_mode);
    if (mode < static_cast<int>(StackMode::NO_STACKING) || mode > static_cast<int>(StackMode::SAME_FOOTPRINT)) {
        throw std::invalid_argument("Invalid stack mode.");
    }
}

bool same_stack_candidate_group(const Item& lhs, const Item& rhs) {
    if (lhs.is_pallet || rhs.is_pallet) return false;
    return lhs.supplier == rhs.supplier &&
           canonical_size_key(lhs.length, lhs.width) == canonical_size_key(rhs.length, rhs.width);
}

int ProblemInstance::end_depot() const {
    if (suppliers.empty()) return 1;
    return *std::max_element(suppliers.begin(), suppliers.end()) + 1;
}
std::vector<int> ProblemInstance::nodes() const {
    std::vector<int> out;
    out.push_back(start_depot());
    out.insert(out.end(), suppliers.begin(), suppliers.end());
    out.push_back(end_depot());
    return out;
}
std::vector<int> ProblemInstance::mandatory_items() const {
    std::vector<int> out;
    for (const auto& it: items) if (!it.is_optional) out.push_back(it.item_id);
    return out;
}
std::vector<int> ProblemInstance::optional_items() const {
    std::vector<int> out;
    for (const auto& it: items) if (it.is_optional) out.push_back(it.item_id);
    return out;
}
std::vector<int> ProblemInstance::pallet_items() const {
    std::vector<int> out;
    for (const auto& it: items) if (it.is_pallet) out.push_back(it.item_id);
    return out;
}
std::vector<int> ProblemInstance::rack_items() const {
    std::vector<int> out;
    for (const auto& it: items) if (!it.is_pallet) out.push_back(it.item_id);
    return out;
}

int ProblemInstance::q_mb(int m, int b) const {
    if (m == b) return 0;
    const auto it = stack_compatibility.find({m, b});
    return it == stack_compatibility.end() ? 0 : it->second;
}

std::pair<int,int> canonical_size_key(int length, int width) { return (length <= width) ? std::make_pair(length,width) : std::make_pair(width,length); }

ProblemInstance normalized_rotatable_rack_dimensions(const ProblemInstance& source) {
    ProblemInstance normalized = source;
    normalized.items.clear();
    normalized.items.reserve(source.items.size());
    for (const auto& item : source.items) {
        const auto footprint = canonical_size_key(item.length, item.width);
        const int length = item.is_pallet ? item.length : footprint.second;
        const int width = item.is_pallet ? item.width : footprint.first;
        normalized.items.emplace_back(
            item.item_id, item.supplier, length, width, item.height,
            item.is_pallet, item.is_optional, item.score, item.dock, item.size_class,
            item.part_code, item.container_code, item.stack_mode);
    }
    return normalized;
}

void validate_instance(const ProblemInstance& inst) {
    int L = inst.vehicle_length, W = inst.vehicle_width, H = inst.vehicle_height;
    if (L <= 0 || W <= 0 || H <= 0) throw std::invalid_argument("Vehicle dimensions must be positive.");
    if (inst.vehicles.empty()) throw std::invalid_argument("At least one vehicle is required.");
    std::set<int> item_ids;
    std::map<int, const Item*> item_by_id;
    for (const auto& it: inst.items) {
        if (!item_ids.insert(it.item_id).second) throw std::invalid_argument("Item ids must be unique.");
        item_by_id[it.item_id] = &it;
    }
    std::set<int> supplier_set(inst.suppliers.begin(), inst.suppliers.end());
    for (const auto& it: inst.items) {
        if (it.is_optional && it.is_pallet) throw std::invalid_argument("optional demand items must be racks, not pallets, in this model.");
        const int mode = static_cast<int>(it.stack_mode);
        if (mode < static_cast<int>(StackMode::NO_STACKING) || mode > static_cast<int>(StackMode::SAME_FOOTPRINT)) throw std::invalid_argument("Invalid stack mode.");
        if (!supplier_set.count(it.supplier)) throw std::invalid_argument("Item has unknown supplier.");
        if (it.height > H) throw std::invalid_argument("Item height exceeds vehicle height.");
        if (!((it.length <= L && it.width <= W) || (it.width <= L && it.length <= W))) {
            throw std::invalid_argument("Item cannot fit vehicle floor in any orientation.");
        }
    }

    for (const auto& kv : inst.stack_compatibility) {
        const int m = kv.first.first, b = kv.first.second, q = kv.second;
        if (q != 0 && q != 1) throw std::invalid_argument("stack_compatibility q_mb must be binary.");
        if (m == b) throw std::invalid_argument("stack_compatibility must not contain self-pairs.");
        if (!item_by_id.count(m) || !item_by_id.count(b)) throw std::invalid_argument("stack_compatibility references an unknown item.");
        if (item_by_id.at(m)->is_pallet || item_by_id.at(b)->is_pallet) throw std::invalid_argument("stack_compatibility is defined only for rack pairs.");
        if (q == 1 && !same_stack_candidate_group(*item_by_id.at(m), *item_by_id.at(b))) {
            throw std::invalid_argument("q_mb=1 outside the same-supplier/same-footprint candidate group.");
        }
        if (q != inst.q_mb(b, m)) throw std::invalid_argument("stack_compatibility q_mb must be symmetric for mutual stacking.");
    }

    auto ns = inst.nodes();
    for (int i: ns) for (int j: ns) if (i != j) {
        if (!inst.distance.count({i,j})) throw std::invalid_argument("Missing distance.");
        if (!inst.travel_time.count({i,j})) throw std::invalid_argument("Missing travel_time.");
    }
    for (int i: inst.suppliers) {
        if (!inst.latest_return.count(i)) throw std::invalid_argument("Missing latest_return.");
        if (!inst.service_time.count(i)) throw std::invalid_argument("Missing service_time.");
    }
}

std::vector<RouteNode> extract_route_from_arcs(const std::vector<std::pair<int,int>>& chosen_arcs, int start, int end) {
    if (chosen_arcs.empty()) return {};
    std::map<int,int> nxt;
    for (auto [i,j]: chosen_arcs) nxt[i]=j;
    std::vector<RouteNode> route;
    route.emplace_back(start);
    int cur=start; std::set<int> seen{start};
    while (nxt.count(cur)) {
        cur=nxt[cur]; route.emplace_back(cur);
        if (cur==end) break;
        if (seen.count(cur)) { route.emplace_back(std::string("CYCLE_DETECTED")); break; }
        seen.insert(cur);
    }
    return route;
}

std::string gurobi_status_name(int status_code) {
    switch(status_code) {
        case 1: return "LOADED"; case 2: return "OPTIMAL"; case 3: return "INFEASIBLE"; case 4: return "INF_OR_UNBD";
        case 5: return "UNBOUNDED"; case 6: return "CUTOFF"; case 7: return "ITERATION_LIMIT"; case 8: return "NODE_LIMIT";
        case 9: return "TIME_LIMIT"; case 10: return "SOLUTION_LIMIT"; case 11: return "INTERRUPTED"; case 12: return "NUMERIC";
        case 13: return "SUBOPTIMAL"; case 14: return "INPROGRESS"; case 15: return "USER_OBJ_LIMIT";
        default: return "STATUS_" + std::to_string(status_code);
    }
}

static bool almost_leq(double lhs, double rhs, double tol) { return lhs <= rhs + tol; }
static bool almost_eq(double lhs, double rhs, double tol) { return std::fabs(lhs-rhs) <= tol; }
static bool overlap_1d(double a0,double a1,double b0,double b1,double tol){ return !(a1 <= b0 + tol || b1 <= a0 + tol); }

ValidationReport validate_solution_against_model(const ProblemInstance& source_inst, const ModelSolution& sol, double tol, bool verbose) {
    const auto inst = normalized_rotatable_rack_dimensions(source_inst);
    validate_instance(inst);
    ValidationReport rep;
    auto items = std::vector<int>{}; for (const auto& it : inst.items) items.push_back(it.item_id);
    auto mandatory=inst.mandatory_items(); auto optional=inst.optional_items(); auto racks=inst.rack_items(); auto pallets=inst.pallet_items();
    std::map<int, Item> item_by_id; for (const auto& it: inst.items) item_by_id.emplace(it.item_id,it);
    std::set<int> optional_set(optional.begin(), optional.end()), rack_set(racks.begin(), racks.end());
    int start=inst.start_depot(), end=inst.end_depot();
    int L=inst.vehicle_length, W=inst.vehicle_width, H=inst.vehicle_height;

    auto stringify_fields = [](const std::map<std::string,std::string>& fields){
        std::ostringstream os; bool first=true; for (const auto& kv: fields) { if(!first) os << ";"; first=false; os << kv.first << "=" << kv.second; } return os.str();
    };
    auto addv=[&](const std::string& type, std::map<std::string,std::string> fields={}){
        std::string msg = type; std::string tail = stringify_fields(fields); if(!tail.empty()) msg += ":" + tail; rep.violations.push_back(msg);
    };
    auto addw=[&](const std::string& type, std::map<std::string,std::string> fields={}){
        std::string msg = type; std::string tail = stringify_fields(fields); if(!tail.empty()) msg += ":" + tail; rep.warnings.push_back(msg);
    };

    if (!(sol.status_name=="OPTIMAL" || sol.status_name=="SUBOPTIMAL" || sol.status_name=="TIME_LIMIT" || sol.status_name=="SOLUTION_LIMIT" || sol.status_name=="USER_OBJ_LIMIT")) {
        addv("no_usable_solution_status", {{"status_name", sol.status_name}});
    }
    if (sol.item_solution.empty()) {
        addv("missing_item_solution", {{"message", "solve_exact_model() did not return item_solution."}});
        rep.num_violations=static_cast<int>(rep.violations.size()); rep.num_warnings=static_cast<int>(rep.warnings.size()); rep.ok=false;
        if (verbose) std::cout << "Solution post-validation report\nok: false\nnum_violations: " << rep.num_violations << "\n";
        return rep;
    }
    for (int m: items) {
        if (!sol.item_solution.count(m)) {
            throw std::out_of_range("item_solution missing item " + std::to_string(m));
        }
    }
    std::set<int> known_items(items.begin(), items.end());
    for (const auto& kv : sol.vehicle_items) {
        for (int m : kv.second) {
            if (!known_items.count(m)) {
                throw std::out_of_range("vehicle_items contains unknown item " + std::to_string(m));
            }
        }
    }

    for (int m: mandatory) {
        int cnt=0; for (int k: inst.vehicles) { auto it=sol.vehicle_items.find(k); if (it!=sol.vehicle_items.end() && std::find(it->second.begin(),it->second.end(),m)!=it->second.end()) ++cnt; }
        if (cnt!=1) addv("mandatory_assignment", {{"item",std::to_string(m)}, {"loaded_count",std::to_string(cnt)}});
    }
    for (int m: optional) {
        int cnt=0; for (int k: inst.vehicles) { auto it=sol.vehicle_items.find(k); if (it!=sol.vehicle_items.end() && std::find(it->second.begin(),it->second.end(),m)!=it->second.end()) ++cnt; }
        if (cnt>1) addv("optional_assignment", {{"item",std::to_string(m)}, {"loaded_count",std::to_string(cnt)}});
    }

    std::map<int,int> supplier_visit_count; for (int i: inst.suppliers) supplier_visit_count[i]=0;
    std::map<int,std::vector<int>> visited_by_vehicle;
    for (int k_init : inst.vehicles) visited_by_vehicle[k_init] = {};
    for (int k: inst.vehicles) {
        auto arcs_it=sol.route_arcs.find(k); std::vector<std::pair<int,int>> arcs = arcs_it==sol.route_arcs.end()?std::vector<std::pair<int,int>>{}:arcs_it->second;
        int start_out=0,end_in=0; for(auto [a0,a1]: arcs){ if(a0==start) start_out++; if(a1==end) end_in++; }
        if(start_out != end_in) addv("start_end_balance", {{"vehicle",std::to_string(k)}, {"start_out",std::to_string(start_out)}, {"end_in",std::to_string(end_in)}});
        if(start_out>1 || end_in>1) addv("vehicle_used_more_than_once", {{"vehicle",std::to_string(k)}, {"start_out",std::to_string(start_out)}, {"end_in",std::to_string(end_in)}});
        if (start_out==1 && sol.routes.count(k)) {
            const auto& r=sol.routes.at(k);
            bool unusual = r.empty() || !std::holds_alternative<int>(r.front()) || std::get<int>(r.front()) != start || !std::holds_alternative<int>(r.back()) || std::get<int>(r.back()) != end;
            if (unusual) addw("route_extraction_unusual", {{"vehicle",std::to_string(k)}});
        }
        for (int i: inst.suppliers) {
            int out_i=0,in_i=0; for(auto [a0,a1]: arcs){ if(a0==i) out_i++; if(a1==i) in_i++; }
            if(out_i != in_i) addv("route_flow_balance", {{"vehicle",std::to_string(k)}, {"supplier",std::to_string(i)}, {"out",std::to_string(out_i)}, {"in",std::to_string(in_i)}});
            if(out_i>0){ supplier_visit_count[i]+=out_i; visited_by_vehicle[k].push_back(i); }
        }
    }
    for (auto [i,cnt]: supplier_visit_count) if (cnt != 1) addv("supplier_visit_once", {{"supplier",std::to_string(i)}, {"visit_count",std::to_string(cnt)}});

    for (int k: inst.vehicles) {
        std::set<int> visited(visited_by_vehicle[k].begin(), visited_by_vehicle[k].end());
        auto loaded_it=sol.vehicle_items.find(k); std::vector<int> loaded = loaded_it==sol.vehicle_items.end()?std::vector<int>{}:loaded_it->second;
        for (int m: loaded) {
            if (!visited.count(item_by_id.at(m).supplier)) addv("load_implies_visit", {{"vehicle",std::to_string(k)}, {"item",std::to_string(m)}, {"supplier",std::to_string(item_by_id.at(m).supplier)}});
        }
        for (int i: visited) for (int m: mandatory) if (item_by_id.at(m).supplier==i && std::find(loaded.begin(),loaded.end(),m)==loaded.end()) addv("visit_implies_mandatory_load", {{"vehicle",std::to_string(k)}, {"supplier",std::to_string(i)}, {"item",std::to_string(m)}});
        long double vol=0; for (int m: loaded) if(item_by_id.count(m)) { const auto& it=item_by_id.at(m); vol += 1.0L*it.length*it.width*it.height; }
        if (!almost_leq(static_cast<double>(vol), static_cast<double>(1.0L*L*W*H), tol)) addv("vehicle_volume", {{"vehicle",std::to_string(k)}, {"volume",std::to_string(static_cast<double>(vol))}, {"capacity",std::to_string(1.0L*L*W*H)}});
    }

    for (int m: items) {
        auto sit=sol.item_solution.find(m); if(sit==sol.item_solution.end()) continue;
        const auto& info=sit->second; const auto& it=item_by_id.at(m);
        if (info.delta1 + info.delta2 != 1) addv("orientation_one", {{"item",std::to_string(m)}, {"delta1",std::to_string(info.delta1)}, {"delta2",std::to_string(info.delta2)}});
        if (info.x < -tol || info.y < -tol || info.z < -tol) addv("negative_coordinate", {{"item",std::to_string(m)}});
        if (!almost_leq(info.x+info.actual_length, L, tol)) addv("x_boundary", {{"item",std::to_string(m)}});
        if (!almost_leq(info.y+info.actual_width, W, tol)) addv("y_boundary", {{"item",std::to_string(m)}});
        if (!almost_leq(info.z+info.actual_height, H, tol)) addv("z_boundary", {{"item",std::to_string(m)}});
        if (it.is_pallet && !almost_eq(info.z, 0.0, tol)) addv("pallet_on_floor", {{"item",std::to_string(m)}, {"z",std::to_string(info.z)}});
    }

    for (const auto& rec: sol.stack_relations) {
        int bottom=rec.bottom, top=rec.top;
        if (!rack_set.count(bottom) || !rack_set.count(top)) addv("stack_non_rack", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
        if(!item_by_id.count(bottom) || !item_by_id.count(top) || !sol.item_solution.count(bottom) || !sol.item_solution.count(top)) continue;
        if (!inst.stack_compatible(bottom, top)) addv("stack_compatibility", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
        const auto& binfo=sol.item_solution.at(bottom); const auto& tinfo=sol.item_solution.at(top);
        if (binfo.loaded_vehicle != tinfo.loaded_vehicle) addv("stack_different_vehicles", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
        if (!almost_eq(binfo.x,tinfo.x,tol)) addv("stack_x_alignment", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
        if (!almost_eq(binfo.y,tinfo.y,tol)) addv("stack_y_alignment", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
        if (binfo.delta1 != tinfo.delta1) addv("stack_orientation", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
        if (!almost_leq(binfo.z + binfo.actual_height, tinfo.z, tol)) addv("stack_z_order", {{"bottom",std::to_string(bottom)}, {"top",std::to_string(top)}});
    }

    for (int k: inst.vehicles) {
        auto vit=sol.vehicle_items.find(k); if (vit==sol.vehicle_items.end()) continue;
        const auto& loaded=vit->second;
        for (std::size_t ii=0; ii<loaded.size(); ++ii) { int m=loaded[ii]; if(!sol.item_solution.count(m)) continue; const auto& im=sol.item_solution.at(m); for (std::size_t jj=ii+1; jj<loaded.size(); ++jj) { int b=loaded[jj]; if(!sol.item_solution.count(b)) continue; const auto& ib=sol.item_solution.at(b); bool ox=overlap_1d(im.x, im.x+im.actual_length, ib.x, ib.x+ib.actual_length, tol); bool oy=overlap_1d(im.y, im.y+im.actual_width, ib.y, ib.y+ib.actual_width, tol); bool oz=overlap_1d(im.z, im.z+im.actual_height, ib.z, ib.z+ib.actual_height, tol); if(ox&&oy&&oz) addv("physical_overlap_3d", {{"vehicle",std::to_string(k)}, {"item_m",std::to_string(m)}, {"item_b",std::to_string(b)}}); } }
    }

    for (int k: inst.vehicles) {
        auto arcs_it=sol.route_arcs.find(k); if (arcs_it!=sol.route_arcs.end()) {
            for(auto [i,j]: arcs_it->second) { double lhs = sol.time_solution.count({i,k})?sol.time_solution.at({i,k}):0.0; lhs += (inst.service_time.count(i)?inst.service_time.at(i):0.0) + inst.travel_time.at({i,j}); double rhs = sol.time_solution.count({j,k})?sol.time_solution.at({j,k}):0.0; if(!almost_leq(lhs,rhs,tol)) addv("time_propagation", {{"vehicle",std::to_string(k)}, {"arc",std::to_string(i)+","+std::to_string(j)}}); }
        }
        for (int i: visited_by_vehicle[k]) { double ret=sol.time_solution.count({end,k})?sol.time_solution.at({end,k}):0.0; if(!almost_leq(ret, inst.latest_return.at(i), tol)) addv("latest_return", {{"vehicle",std::to_string(k)}, {"supplier",std::to_string(i)}, {"return_time",std::to_string(ret)}}); }
    }

    double latest=0, optional_profit_expr=0, dist_sum=0; int used_cnt=0;
    for (int k: inst.vehicles) {
        auto tit=sol.time_solution.find({start,k}); if (tit!=sol.time_solution.end()) latest += tit->second;
        auto vit=sol.vehicle_items.find(k); if (vit!=sol.vehicle_items.end()) for(int m: vit->second) if(optional_set.count(m)) optional_profit_expr += static_cast<double>(item_by_id.at(m).score);
        auto arcs_it=sol.route_arcs.find(k); if(arcs_it!=sol.route_arcs.end()) { bool used=false; for(auto [i,j]: arcs_it->second){ dist_sum += inst.distance.at({i,j}); if(j==end) used=true; } if(used) used_cnt++; }
    }
    rep.manual_transport_cost = inst.alpha_distance_cost * dist_sum + inst.fixed_vehicle_cost * used_cnt;
    rep.manual_latest_departure_sum = latest;
    rep.manual_optional_profit = optional_profit_expr;
    rep.manual_objectives["transport_cost"] = rep.manual_transport_cost;
    rep.manual_objectives["latest_departure_sum"] = rep.manual_latest_departure_sum;
    rep.manual_objectives["optional_profit"] = rep.manual_optional_profit;
    auto check_reported_obj = [&](const std::string& key, const std::string& violation_type, double manual_value) {
        auto it = sol.objective_values.find(key);
        if (it != sol.objective_values.end()) {
            if (!almost_eq(it->second, manual_value, std::max(tol, 1e-4))) {
                addv(violation_type, {{"reported", std::to_string(it->second)}, {"manual", std::to_string(manual_value)}});
            }
        }
    };
    check_reported_obj("transport_cost", "objective_transport", rep.manual_transport_cost);
    check_reported_obj("latest_departure_sum", "objective_latest_departure", rep.manual_latest_departure_sum);
    check_reported_obj("optional_profit", "objective_optional_profit", rep.manual_optional_profit);
    rep.num_violations = static_cast<int>(rep.violations.size()); rep.num_warnings = static_cast<int>(rep.warnings.size()); rep.ok = rep.num_violations == 0;
    if (verbose) {
        std::cout << "=" << std::string(99,'=') << "\nSolution post-validation report\n" << "=" << std::string(99,'=') << "\n";
        std::cout << "ok: " << (rep.ok ? "true" : "false") << "\nnum_violations: " << rep.num_violations << "\nnum_warnings: " << rep.num_warnings << "\n";
        std::cout << "manual_objectives: {transport_cost: " << rep.manual_transport_cost << ", latest_departure_sum: " << rep.manual_latest_departure_sum << ", optional_profit: " << rep.manual_optional_profit << "}\n";
        if(!rep.warnings.empty()) { std::cout << "warnings:\n"; for(std::size_t i=0;i<std::min<std::size_t>(20, rep.warnings.size());++i) std::cout << "  " << rep.warnings[i] << "\n"; }
        if(!rep.violations.empty()) { std::cout << "violations:\n"; for(std::size_t i=0;i<std::min<std::size_t>(50, rep.violations.size());++i) std::cout << "  " << rep.violations[i] << "\n"; }
    }
    return rep;
}

std::string solution_detail_json(const ProblemInstance& inst, const ModelSolution& sol) {
    std::map<int,Item> item_by_id; for(const auto& it: inst.items) item_by_id.emplace(it.item_id,it);
    auto optional = inst.optional_items(); std::set<int> optional_set(optional.begin(), optional.end());
    std::ostringstream os; os << "{";
    os << "\"status_name\":" << quote(sol.status_name) << ",";
    os << "\"objective_values\":{"; bool first=true; for(const auto& kv: sol.objective_values){ if(!first) os << ","; first=false; os << quote(kv.first) << ":" << double_json(kv.second); } os << "},";
    os << "\"vehicles\":["; for(std::size_t vi=0; vi<inst.vehicles.size(); ++vi){ int k=inst.vehicles[vi]; if(vi) os << ","; auto loaded_it=sol.vehicle_items.find(k); std::vector<int> loaded=loaded_it==sol.vehicle_items.end()?std::vector<int>{}:loaded_it->second; long double vol=0; long long score=0; for(int m: loaded){ const auto& it=item_by_id.at(m); vol += 1.0L*it.length*it.width*it.height; if(optional_set.count(m)) score += it.score; } os << "{\"vehicle\":" << k << ",\"route\":" << (sol.routes.count(k)?route_json(sol.routes.at(k)):"[]") << ",\"arcs\":" << (sol.route_arcs.count(k)?pair_vec_json(sol.route_arcs.at(k)):"[]") << ",\"loaded_items\":" << vec_json(loaded) << ",\"loaded_count\":" << loaded.size() << ",\"optional_score\":" << score << ",\"volume\":" << double_json(static_cast<double>(vol)) << "}"; } os << "],";
    os << "\"items\":["; for(std::size_t ii=0; ii<inst.items.size(); ++ii){ const auto& base=inst.items[ii]; if(ii) os << ","; os << "{\"item_id\":" << base.item_id << ",\"supplier\":" << base.supplier << ",\"length\":" << base.length << ",\"width\":" << base.width << ",\"height\":" << base.height << ",\"is_pallet\":" << bool_json(base.is_pallet) << ",\"is_optional\":" << bool_json(base.is_optional) << ",\"score\":" << base.score << ",\"dock\":" << base.dock << ",\"size_class\":" << base.size_class; auto it=sol.item_solution.find(base.item_id); if(it!=sol.item_solution.end()){ const auto& info=it->second; os << ",\"loaded\":" << bool_json(info.loaded) << ",\"loaded_vehicles\":" << vec_json(info.loaded_vehicles) << ",\"loaded_vehicle\":"; if(info.loaded_vehicle) os << *info.loaded_vehicle; else os << "null"; os << ",\"x\":" << double_json(info.x) << ",\"y\":" << double_json(info.y) << ",\"z\":" << double_json(info.z) << ",\"delta1\":" << info.delta1 << ",\"delta2\":" << info.delta2 << ",\"actual_length\":" << double_json(info.actual_length) << ",\"actual_width\":" << double_json(info.actual_width) << ",\"actual_height\":" << double_json(info.actual_height); } os << "}"; } os << "],";
    os << "\"stack_relations\":["; for(std::size_t i=0;i<sol.stack_relations.size();++i){ if(i) os << ","; const auto& r=sol.stack_relations[i]; os << "{\"bottom\":" << r.bottom << ",\"top\":" << r.top << ",\"bottom_vehicle\":"; if(r.bottom_vehicle) os << *r.bottom_vehicle; else os << "null"; os << ",\"top_vehicle\":"; if(r.top_vehicle) os << *r.top_vehicle; else os << "null"; os << "}"; } os << "],";
    os << "\"planar_relations\":["; for(std::size_t i=0;i<sol.planar_relations.size();++i){ if(i) os << ","; const auto& r=sol.planar_relations[i]; os << "{\"type\":" << quote(r.type) << ",\"first\":" << r.first << ",\"second\":" << r.second << "}"; } os << "],";
    os << "\"same_vehicle_pairs\":" << pair_vec_json(sol.same_vehicle_pairs);
    os << "}"; return os.str();
}

#ifndef HRLH_MODEL_WITH_GUROBI
ModelSolution solve_exact_model(const ProblemInstance&,
                                   std::optional<double>,
                                   std::optional<double>,
                                   bool,
                                   double,
                                   bool,
                                   int,
                                   int,
                                   bool,
                                   bool) {
    throw std::runtime_error("solve_exact_model requires HRLH_WITH_GUROBI=ON and a separately installed Gurobi C++ SDK.");
}
#endif

}
