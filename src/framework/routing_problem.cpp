#include "routing_problem.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hrlh::routing {
Problem::Problem(const hrlh_model::ProblemInstance& inst,
                 const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
                 const HrlhConfig& cfg, NumericMapping mapping)
    : instance(inst),
      use_time(mapping==NumericMapping::ILSCompatible?cfg.ils_enforce_latest_return:cfg.enforce_time_feasible),
      ils_compatible(mapping==NumericMapping::ILSCompatible),
      align_transport_cost(cfg.ils_align_transport_cost) {
    search_dff::SupplierRectangles rectangles;
    rectangles.suppliers = inst.suppliers;
    std::vector<std::int64_t> areas;
    const std::int64_t cap = 1LL*inst.vehicle_width*inst.vehicle_length;
    if (!(cap > 0)) throw std::invalid_argument("non-positive floor capacity");
    for (int supplier : inst.suppliers) {
        index.emplace(supplier, demands.size());
        std::int64_t area = 0;
        for (const auto& spec : build_route_mandatory_packing_specs(inst, prep, {supplier})) {
            for (int k = 0; k < spec.mandatory_count; ++k) {
                rectangles.rectangles_by_supplier[supplier].emplace_back(spec.width, spec.length);
                const std::int64_t rectangle_area = 1LL*spec.width*spec.length;
                if (rectangle_area > std::numeric_limits<std::int64_t>::max()-area)
                    throw std::overflow_error("mandatory area accumulation overflow");
                area += rectangle_area;
            }
        }
        areas.push_back(area);
        demands.push_back({});
        integer_demands.push_back({});
    }
    const bool area_enabled=ils_compatible?cfg.ils_use_mandatory_floor_area_capacity:cfg.enforce_volume_necessary;
    if (area_enabled||ils_compatible) {
        dimension_names.push_back(area_enabled?"mandatory_floor_area":"disabled_area_placeholder");
        const auto first_capacity=area_enabled?cap:1000000000000LL;
        integer_capacities.push_back(first_capacity);
        for (std::size_t i=0; i<demands.size(); ++i) {
            const auto quantity=area_enabled?areas[i]:0;
            demands[i].push_back(static_cast<double>(quantity)/first_capacity);
            integer_demands[i].push_back(quantity);
        }
    }
    if (cfg.ils_use_dff_capacity_dimensions) {
        const auto profiles = search_dff::build_fixed_profiles(
            inst.vehicle_width, inst.vehicle_length, rectangles, area_enabled);
        for (const auto& profile : profiles.surviving_profiles) {
            dimension_names.push_back(profile.label);
            integer_capacities.push_back(profile.capacity);
            for (int supplier : inst.suppliers) {
                demands.at(index.at(supplier)).push_back(
                    static_cast<double>(profile.demand_by_supplier.at(supplier))/profile.capacity);
                integer_demands.at(index.at(supplier)).push_back(profile.demand_by_supplier.at(supplier));
            }
        }
    }
    for (const auto& load : demands) for (double value : load)
        max_normalized_demand = std::max(max_normalized_demand, value);
    for (const auto& [arc, distance] : inst.distance)
        max_arc_cost = std::max(max_arc_cost, directed_cost(arc.first,arc.second));
}

double Problem::directed_cost(int from, int to) const {
    if(ils_compatible){
        if(from==instance.end_depot())from=instance.start_depot();
        if(to==instance.end_depot())to=instance.start_depot();
    }
    if (from == to) return 0;
    const double alpha=ils_compatible&&!align_transport_cost?1.:instance.alpha_distance_cost;
    const double value=alpha * instance.distance.at({from,to});
    return ils_compatible?std::llround(value*100.)/100.:value;
}
double Problem::directed_time(int from, int to) const {
    if(ils_compatible){
        if(from==instance.end_depot())from=instance.start_depot();
        if(to==instance.end_depot())to=instance.start_depot();
    }
    if (from == to) return 0;
    const double value=instance.travel_time.at({from,to});
    return ils_compatible?std::llround(value*100.)/100.:value;
}
double Problem::matrix_proximity(int lhs, int rhs) const {
    return (directed_cost(lhs,rhs)+directed_cost(rhs,lhs))/2;
}
RouteMetrics Problem::evaluate_route(const Route& route) const {
    RouteMetrics result;
    result.loads.resize(dimension_names.size(), 0);
    std::vector<std::int64_t> exact_loads(dimension_names.size(), 0);
    if (route.empty()) return result;
    int previous = instance.start_depot();
    double deadline = std::numeric_limits<double>::infinity();
    std::int64_t scaled_cost=0,scaled_duration=0,scaled_deadline=1000000000000LL;
    for (int supplier : route) {
        const auto supplier_index = index.at(supplier);
        result.distance += instance.distance.at({previous,supplier});
        result.cost += directed_cost(previous,supplier);
        result.duration += directed_time(previous,supplier);
        result.duration += instance.service_time.at(supplier);
        deadline = std::min(deadline, instance.latest_return.at(supplier));
        if(ils_compatible){
            scaled_cost+=std::llround(directed_cost(previous,supplier)*100.);
            scaled_duration+=std::llround(directed_time(previous,supplier)*100.)
                +std::llround(instance.service_time.at(supplier)*100.);
            if(use_time)scaled_deadline=std::min<std::int64_t>(scaled_deadline,
                std::llround(instance.latest_return.at(supplier)*100.));
        }
        for (std::size_t d=0; d<result.loads.size(); ++d) {
            const auto quantity=integer_demands[supplier_index][d];
            if (quantity > std::numeric_limits<std::int64_t>::max()-exact_loads[d])
                throw std::overflow_error("route surrogate load overflow");
            exact_loads[d] += quantity;
        }
        previous = supplier;
    }
    result.distance += instance.distance.at({previous,instance.end_depot()});
    result.cost += directed_cost(previous,instance.end_depot());
    result.cost += instance.fixed_vehicle_cost;
    result.duration += directed_time(previous,instance.end_depot());
    if(ils_compatible){
        scaled_cost+=std::llround(directed_cost(previous,instance.end_depot())*100.)
            +std::llround(instance.fixed_vehicle_cost*100.);
        scaled_duration+=std::llround(directed_time(previous,instance.end_depot())*100.);
        result.cost=scaled_cost/100.;
        result.duration=scaled_duration/100.;
        deadline=scaled_deadline/100.;
    }
    result.latest_return = deadline;
    result.latest_departure = deadline-result.duration;
    for (std::size_t d=0; d<exact_loads.size(); ++d) {
        result.loads[d] = static_cast<double>(exact_loads[d])/integer_capacities[d];
        if (exact_loads[d]>integer_capacities[d])
            result.capacity_excess += static_cast<double>(exact_loads[d]-integer_capacities[d])/integer_capacities[d];
    }

    if (ils_compatible) {
        if(use_time&&scaled_duration>scaled_deadline)
            result.time_excess=(scaled_duration-scaled_deadline)/100.;
    } else if (use_time && result.latest_departure < -1e-9)
        result.time_excess = -result.latest_departure;
    return result;
}
PlanMetrics Problem::evaluate(const RoutePlan& plan, bool allow_missing) const {
    PlanMetrics result;
    std::vector<int> visits(index.size(), 0);
    for (const auto& route : plan) {
        if (route.empty()) continue;
        bool known = true;
        for (int supplier : route) {
            auto found = index.find(supplier);
            if (found == index.end()) { result.duplicate_or_unknown = true; known = false; }
            else if (++visits[found->second] > 1) {
                result.duplicate_or_unknown = true;
                known = false;
            }
        }
        if (!known) continue;
        const auto metrics = evaluate_route(route);
        result.cost += metrics.cost;
        result.capacity_excess += metrics.capacity_excess;
        result.time_excess += metrics.time_excess;
        ++result.used_vehicles;
    }
    result.missing_suppliers = static_cast<int>(std::count(visits.begin(), visits.end(), 0));
    result.routing_feasible = !result.duplicate_or_unknown
        && (allow_missing || result.missing_suppliers == 0)
        && result.used_vehicles <= static_cast<int>(instance.vehicles.size())
        && result.capacity_excess == 0 && result.time_excess == 0;

    if (!result.duplicate_or_unknown) {
        if(!ils_compatible)result.cost = compute_transport_cost(instance,plan);
        else {
            std::int64_t cost=0;
            for(const auto& route:plan)cost+=std::llround(evaluate_route(route).cost*100.);
            result.cost=cost/100.;
        }
    }
    return result;
}

}
