#include "hrlh.hpp"
#include "initial_feasible_descent.hpp"
#include "optional_demand_profit_intensification.hpp"
#include "lru_cache.hpp"
#include "loading_bounds.hpp"
#ifdef HRLH_COMPARISON_ENGINE
#include "routing_search.hpp"
#endif
#ifdef HRLH_WITH_ILS_SEARCH
#include "case_protocol.h"
#include "solver_port.h"
#include "Activity.h"
#include "ProblemData.h"
#include "Solution.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <cctype>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <limits>
#include <optional>

namespace hrlh {
namespace {

enum class BudgetKind { Time, PackingCalls, FeasibleDescentTime };

struct SearchBudgetExceeded : public std::runtime_error {
    BudgetKind kind;
    SearchBudgetExceeded(BudgetKind k, const std::string& msg)
        : std::runtime_error(msg), kind(k) {}
};

struct FeasibleRouteCacheEntry {
    long long best_score = 0;
    long long upper_bound = 0;
    bool has_layout = false;
    RoutePackingDetail best_detail;
};

struct MandatoryFeasibleRouteCacheEntry {
};

struct RoutePackingCachePools {
    explicit RoutePackingCachePools(const RoutePackingCacheConfig& cfg)
        : feasible_route_pool(cfg.max_feasible_entries),
          mandatory_feasible_route_pool(cfg.max_feasible_entries) {}
    LruCache<std::string, FeasibleRouteCacheEntry> feasible_route_pool;
    LruCache<std::string, MandatoryFeasibleRouteCacheEntry> mandatory_feasible_route_pool;
};

struct PackingEvaluationRuntimeState {
    std::function<void(const Route&)> on_static_certified_infeasible;
};
#ifdef HRLH_COMPARISON_ENGINE

using BoundedSpecsCache = std::unordered_map<std::string,std::vector<PackingTypeSpecCpp>>;
BoundedSpecsCache& bounded_specs_cache_state() {
    static thread_local BoundedSpecsCache cache;
    return cache;
}
#endif

Route canonical_supplier_set(Route route) {
    std::sort(route.begin(), route.end());
    route.erase(std::unique(route.begin(), route.end()), route.end());
    return route;
}

bool is_supplier_subset_sorted(const Route& subset, const Route& superset) {
    return std::includes(superset.begin(), superset.end(), subset.begin(), subset.end());
}


template <class StaticProofFn>
void learn_certified_nogood_with_static_shrinking(
    CertifiedNoGoodPool& pool,
    const Route& failed_route,
    StaticProofFn&& has_static_proof) {
    Route minimal = canonical_supplier_set(failed_route);
    if (minimal.empty()) return;

    pool.insert(minimal);
    for (std::size_t index = 0; index < minimal.size() && minimal.size() > 1;) {
        Route trial = minimal;
        trial.erase(trial.begin() + static_cast<std::ptrdiff_t>(index));
        if (has_static_proof(trial)) {
            minimal = std::move(trial);
            pool.insert(minimal);
            index = 0;
        } else {
            ++index;
        }
    }
}

void mark_budget_exceeded(SearchControlState& search_state, BudgetKind kind) {
    if (kind == BudgetKind::Time) search_state.time_budget_exceeded = true;
    if (kind == BudgetKind::PackingCalls) search_state.packing_call_budget_exceeded = true;
}

std::uint64_t splitmix64(std::uint64_t& x) {
    std::uint64_t z = (x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

int rand_int(std::uint64_t& state, int lo, int hi) {
    if (hi < lo) return lo;
    return lo + static_cast<int>(splitmix64(state) % static_cast<std::uint64_t>(hi - lo + 1));
}

std::uint64_t isolated_route_repair_seed(std::uint64_t base_seed,
                                     const std::string& signature) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (unsigned char ch : signature) {
        hash ^= static_cast<std::uint64_t>(ch);
        hash *= 1099511628211ULL;
    }
    hash ^= base_seed + 0x56fca173f3b428d1ULL;
    return splitmix64(hash);
}

std::string group_key_string(const hrlh_components::stack_preprocessing::GroupKey& key) {
    return std::to_string(std::get<0>(key)) + ":" + std::to_string(std::get<1>(key)) + ":" + std::to_string(std::get<2>(key));
}

bool covers_each_supplier_once(const hrlh_model::ProblemInstance& inst, const RoutePlan& plan) {
    std::map<int,int> count;
    for (int s : inst.suppliers) count[s] = 0;
    for (const auto& r : plan) {
        for (int s : r) {
            if (!count.count(s)) return false;
            count[s]++;
        }
    }
    for (auto [s,c] : count) if (c != 1) return false;
    return true;
}

std::map<int, const hrlh_model::Item*> item_by_id(const hrlh_model::ProblemInstance& inst) {
    std::map<int, const hrlh_model::Item*> m;
    for (const auto& it : inst.items) m[it.item_id] = &it;
    return m;
}

std::map<int, std::int64_t> mandatory_floor_area_by_supplier_from_preprocess(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep) {
    std::map<int, std::int64_t> out;
    for (int s : inst.suppliers) out[s] = 0;

    for (const auto& [key, group] : prep.groups) {
        int supplier = std::get<0>(key);
        out[supplier] += group.area_A_g;
    }

    for (const auto& it : inst.items) {
        if (it.is_pallet) out[it.supplier] += 1LL * it.length * it.width;
    }
    return out;
}

std::int64_t mandatory_floor_area_for_route(
    const hrlh_model::ProblemInstance& inst,
    const Route& route,
    const std::map<int, std::int64_t>* preprocessed_mandatory_floor_area = nullptr) {
    std::set<int> suppliers(route.begin(), route.end());
    std::int64_t area = 0;
    if (preprocessed_mandatory_floor_area) {
        for (int supplier : suppliers) {
            auto it = preprocessed_mandatory_floor_area->find(supplier);
            if (it != preprocessed_mandatory_floor_area->end()) area += it->second;
        }
        return area;
    }

    for (const auto& it : inst.items) {
        if (!suppliers.count(it.supplier)) continue;
        if (!it.is_optional) area += 1LL * it.length * it.width;
    }
    return area;
}

bool path_basic_feasible(const hrlh_model::ProblemInstance& inst, const RoutePlan& plan,
                          const HrlhConfig& cfg,
                          const std::map<int, std::int64_t>* preprocessed_mandatory_floor_area = nullptr) {
    if (!covers_each_supplier_once(inst, plan)) {

        return false;
    }
    if ((int)plan.size() > (int)inst.vehicles.size()) {

        return false;
    }
    if (cfg.enforce_volume_necessary) {
        const std::int64_t cap = 1LL * inst.vehicle_length * inst.vehicle_width;
        for (const auto& r : plan) {
            if (mandatory_floor_area_for_route(inst, r, preprocessed_mandatory_floor_area) > cap) {

                return false;
            }
        }
    }
    if (cfg.enforce_time_feasible) {
        for (const auto& r : plan) {
            if (!r.empty() && route_latest_departure(inst, r) < -1e-9) {

                return false;
            }
        }
    }
    return true;
}

RoutePlan remove_empty_routes(RoutePlan plan);
RoutePlan initial_one_supplier_routes(const hrlh_model::ProblemInstance& inst);

int compute_area_vehicle_lower_bound(
    const hrlh_model::ProblemInstance& inst,
    const std::map<int, std::int64_t>& mandatory_floor_area) {
    const std::int64_t capacity = 1LL * inst.vehicle_length * inst.vehicle_width;
    std::int64_t total_area = 0;
    for (int supplier : inst.suppliers) {
        auto it = mandatory_floor_area.find(supplier);
        if (it != mandatory_floor_area.end()) total_area += it->second;
    }
    if (capacity <= 0) return 1;
    return std::max(1, static_cast<int>((total_area + capacity - 1) / capacity));
}

double single_route_transport_cost(const hrlh_model::ProblemInstance& inst,
                                   const Route& route) {
    if (route.empty()) return 0.0;
    return compute_transport_cost(inst, RoutePlan{route});
}

RoutePlan lower_bound_greedy_plan(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const std::map<int, std::int64_t>& mandatory_floor_area,
    int route_limit,
    const std::function<void()>& require_time_budget,
    const std::function<bool(const Route&, int)>& addition_nogood,
    const std::function<void(const Route&)>& learn_static_nogood,
    std::uint64_t* diversification_rng = nullptr) {
    if (inst.suppliers.empty()) return {};
    route_limit = std::max(1, std::min(route_limit, static_cast<int>(inst.vehicles.size())));
    RoutePlan plan(static_cast<std::size_t>(route_limit));
    auto suppliers = inst.suppliers;
    std::sort(suppliers.begin(), suppliers.end(), [&](int a, int b) {
        const auto aa = mandatory_floor_area.count(a) ? mandatory_floor_area.at(a) : 0;
        const auto ab = mandatory_floor_area.count(b) ? mandatory_floor_area.at(b) : 0;
        if (aa != ab) return aa > ab;
        const double da = inst.latest_return.at(a);
        const double db = inst.latest_return.at(b);
        if (std::abs(da - db) > 1e-9) return da < db;
        return a < b;
    });
    if (diversification_rng != nullptr && suppliers.size() > 1) {
        for (std::size_t i = suppliers.size() - 1; i > 0; --i) {
            const std::size_t j = static_cast<std::size_t>(rand_int(
                *diversification_rng, 0, static_cast<int>(i)));
            std::swap(suppliers[i], suppliers[j]);
        }
    }
    const std::int64_t capacity = 1LL * inst.vehicle_length * inst.vehicle_width;
    struct Choice {
        double score = std::numeric_limits<double>::infinity();
        int route = -1;
        int position = -1;
    };
    for (int supplier : suppliers) {
        require_time_budget();
        Choice best;
        for (int r = 0; r < route_limit; ++r) {
            const Route& before = plan[static_cast<std::size_t>(r)];
            const double before_cost = single_route_transport_cost(inst, before);
            if (addition_nogood && addition_nogood(before, supplier)) continue;
            const auto supplier_area = mandatory_floor_area.count(supplier)
                ? mandatory_floor_area.at(supplier) : 0;
            const bool exceeds_area = mandatory_floor_area_for_route(
                inst, before, &mandatory_floor_area) + supplier_area > capacity;
            if (exceeds_area) continue;
            bool static_lb_checked = false;
            bool static_lb_infeasible = false;
            for (int pos = 0; pos <= static_cast<int>(before.size()); ++pos) {
                require_time_budget();
                Route after = before;
                after.insert(after.begin() + pos, supplier);
                if (route_latest_departure(inst, after) < -1e-9) continue;
                if (!static_lb_checked) {
                    const auto specs = build_route_mandatory_packing_specs(inst, prep, after);
                    static_lb_infeasible = mandatory_static_lb_check_cpp(
                        specs, inst.vehicle_width, inst.vehicle_length).proven_infeasible;
                    static_lb_checked = true;
                    if (static_lb_infeasible && learn_static_nogood) {
                        learn_static_nogood(after);
                    }
                }
                if (static_lb_infeasible) break;
                const double delta = single_route_transport_cost(inst, after) - before_cost;
                const double score = delta;
                if (score < best.score) best = {score, r, pos};
            }
        }
        if (best.route < 0) return {};
        auto& target = plan[static_cast<std::size_t>(best.route)];
        target.insert(target.begin() + best.position, supplier);
    }
    return remove_empty_routes(std::move(plan));
}

std::string unordered_route_key(Route route);
RoutePlan remove_empty_routes(RoutePlan plan);

struct RouteRepairCostGateSnapshot {
    bool passes = false;
    double candidate_cost = 0.0;
    double incumbent_cost = 0.0;
};

struct CumulativeRouteRepairResult {
    RoutePlan plan;
    bool fully_repaired = false;
};

using RouteRepairNoGoodFn = std::function<bool(const Route&, int)>;
using MandatoryRouteEvaluationFn = std::function<RoutePackingDetail(const Route&)>;
using RouteRepairCostGateFn = std::function<RouteRepairCostGateSnapshot(const RoutePlan&)>;

CumulativeRouteRepairResult run_cumulative_packing_guided_route_repair(
    const hrlh_model::ProblemInstance& inst,
    const RoutePlan& source,
    const PackingPlanEval& initial_packing,
    const std::map<int, std::int64_t>& mandatory_floor_area,
    const RouteRepairNoGoodFn& target_has_nogood,
    const MandatoryRouteEvaluationFn& evaluate_mandatory_route,
    const RouteRepairCostGateFn& assess_cost_gate,
    std::uint64_t& rng) {
    struct RepairCandidate {
        RoutePlan plan;
        std::vector<int> failed;
        RouteRepairCostGateSnapshot gate;
    };

    CumulativeRouteRepairResult result;
    result.plan = source;
    std::map<std::string, RoutePackingDetail> mandatory_by_route_key;
    for (const auto& detail : initial_packing.route_details) {
        if (!detail.route.empty()) {
            mandatory_by_route_key[unordered_route_key(detail.route)] = detail;
        }
    }
    auto get_mandatory = [&](const Route& route) -> RoutePackingDetail {
        const std::string key = unordered_route_key(route);
        const auto found = mandatory_by_route_key.find(key);
        if (found != mandatory_by_route_key.end()) {

            RoutePackingDetail detail = found->second;
            detail.route = route;
            return detail;
        }

        RoutePackingDetail detail = evaluate_mandatory_route(route);
        detail.route = route;
        mandatory_by_route_key.emplace(key, detail);
        return detail;
    };
    auto failed_indices = [&](const RoutePlan& plan) {
        std::vector<int> failed;
        for (int index = 0; index < static_cast<int>(plan.size()); ++index) {
            if (!get_mandatory(plan[static_cast<std::size_t>(index)]).feasible) {
                failed.push_back(index);
            }
        }
        return failed;
    };

    auto supplier_area = [&](int supplier) {
        const auto found = mandatory_floor_area.find(supplier);
        return found == mandatory_floor_area.end() ? std::int64_t{0} : found->second;
    };
    auto insertion_with_minimum_cost = [&](const Route& before, int supplier)
        -> std::optional<Route> {
        const double before_cost = single_route_transport_cost(inst, before);
        double best_delta = std::numeric_limits<double>::infinity();
        std::vector<int> tied_positions;
        for (int position = 0; position <= static_cast<int>(before.size()); ++position) {
            Route positioned = before;
            positioned.insert(positioned.begin() + position, supplier);
            if (route_latest_departure(inst, positioned) < -1e-9) continue;
            const double delta = single_route_transport_cost(inst, positioned) - before_cost;
            if (delta < best_delta) {
                best_delta = delta;
                tied_positions.assign(1, position);
            } else if (delta == best_delta) {
                tied_positions.push_back(position);
            }
        }
        if (tied_positions.empty()) return std::nullopt;
        int chosen_position = tied_positions.front();
        if (tied_positions.size() > 1) {

            chosen_position = tied_positions[static_cast<std::size_t>(rand_int(
                rng, 0, static_cast<int>(tied_positions.size() - 1)))];
        }
        Route after = before;
        after.insert(after.begin() + chosen_position, supplier);
        return after;
    };

    auto current_failed = failed_indices(result.plan);

    const std::int64_t vehicle_area =
        1LL * inst.vehicle_length * inst.vehicle_width;

    while (!current_failed.empty()) {


        std::vector<int> failed_suppliers;
        for (int route_index : current_failed) {
            const auto& route = result.plan[static_cast<std::size_t>(route_index)];
            failed_suppliers.insert(failed_suppliers.end(), route.begin(), route.end());
        }
        std::sort(failed_suppliers.begin(), failed_suppliers.end(), [&](int lhs, int rhs) {
            if (supplier_area(lhs) != supplier_area(rhs)) {
                return supplier_area(lhs) > supplier_area(rhs);
            }
            return lhs < rhs;
        });
        failed_suppliers.erase(
            std::unique(failed_suppliers.begin(), failed_suppliers.end()),
            failed_suppliers.end());
        auto source_index_for = [&](int supplier) {
            for (int route_index : current_failed) {
                const auto& route = result.plan[static_cast<std::size_t>(route_index)];
                if (std::find(route.begin(), route.end(), supplier) != route.end()) {
                    return route_index;
                }
            }
            return -1;
        };
        auto try_candidate = [&](int source_index, int target_index,
                                 const Route& source_after, const Route& target_after,
                                 bool opens_singleton) -> std::optional<RepairCandidate> {
            RoutePlan candidate = result.plan;
            candidate[static_cast<std::size_t>(source_index)] = source_after;
            if (opens_singleton) {
                candidate.push_back(target_after);
            } else {
                candidate[static_cast<std::size_t>(target_index)] = target_after;
            }
            candidate = remove_empty_routes(std::move(candidate));
            const auto gate = assess_cost_gate(candidate);
            if (!gate.passes) return std::nullopt;

            if (!source_after.empty()) {

                (void)get_mandatory(source_after);
            }

            (void)get_mandatory(target_after);
            auto failed = failed_indices(candidate);
            if (failed.size() >= current_failed.size()) return std::nullopt;
            return RepairCandidate{std::move(candidate), std::move(failed), gate};
        };

        auto probe_relocation = [&]() -> std::optional<RepairCandidate> {
            for (int supplier : failed_suppliers) {
                const int source_index = source_index_for(supplier);
                if (source_index < 0) continue;
                for (int target_index = 0;
                     target_index < static_cast<int>(result.plan.size()); ++target_index) {
                    if (target_index == source_index) continue;

                    const Route& target_before = result.plan[static_cast<std::size_t>(target_index)];
                    if (target_has_nogood && target_has_nogood(target_before, supplier)) {

                        continue;
                    }
                    Route target_members = target_before;
                    target_members.push_back(supplier);
                    if (mandatory_floor_area_for_route(
                            inst, target_members, &mandatory_floor_area) > vehicle_area) continue;
                    auto target_after = insertion_with_minimum_cost(target_before, supplier);
                    if (!target_after) continue;
                    Route source_after = result.plan[static_cast<std::size_t>(source_index)];
                    source_after.erase(std::find(source_after.begin(), source_after.end(), supplier));
                    auto candidate = try_candidate(source_index, target_index,
                                                   source_after, *target_after, false);
                    if (candidate) return candidate;
                }
            }
            return std::nullopt;
        };

        auto probe_exchange = [&]() -> std::optional<RepairCandidate> {
            for (int outgoing : failed_suppliers) {
                const int source_index = source_index_for(outgoing);
                if (source_index < 0) continue;
                const Route& source_before = result.plan[static_cast<std::size_t>(source_index)];
                for (int target_index = 0;
                     target_index < static_cast<int>(result.plan.size()); ++target_index) {
                    if (target_index == source_index) continue;
                    const Route& target_before = result.plan[static_cast<std::size_t>(target_index)];
                    for (int incoming : target_before) {

                        if (supplier_area(incoming) >= supplier_area(outgoing)) continue;
                        Route source_base = source_before;
                        source_base.erase(std::find(source_base.begin(), source_base.end(), outgoing));
                        Route target_base = target_before;
                        target_base.erase(std::find(target_base.begin(), target_base.end(), incoming));
                        if (target_has_nogood
                            && (target_has_nogood(source_base, incoming)
                                || target_has_nogood(target_base, outgoing))) {

                            continue;
                        }
                        Route source_members = source_base;
                        source_members.push_back(incoming);
                        Route target_members = target_base;
                        target_members.push_back(outgoing);
                        if (mandatory_floor_area_for_route(
                                inst, source_members, &mandatory_floor_area) > vehicle_area
                            || mandatory_floor_area_for_route(
                                inst, target_members, &mandatory_floor_area) > vehicle_area) continue;
                        auto source_after = insertion_with_minimum_cost(source_base, incoming);
                        if (!source_after) continue;
                        auto target_after = insertion_with_minimum_cost(target_base, outgoing);
                        if (!target_after) continue;
                        auto candidate = try_candidate(source_index, target_index,
                                                       *source_after, *target_after, false);
                        if (candidate) return candidate;
                    }
                }
            }
            return std::nullopt;
        };

        auto probe_singleton = [&]() -> std::optional<RepairCandidate> {
            if (result.plan.size() >= inst.vehicles.size()) return std::nullopt;
            for (int supplier : failed_suppliers) {
                const int source_index = source_index_for(supplier);
                if (source_index < 0
                    || result.plan[static_cast<std::size_t>(source_index)].size() <= 1) continue;

                const Route empty;
                if (target_has_nogood && target_has_nogood(empty, supplier)) {

                    continue;
                }
                const Route target_after{supplier};
                if (mandatory_floor_area_for_route(
                        inst, target_after, &mandatory_floor_area) > vehicle_area
                    || route_latest_departure(inst, target_after) < -1e-9) continue;
                Route source_after = result.plan[static_cast<std::size_t>(source_index)];
                source_after.erase(std::find(source_after.begin(), source_after.end(), supplier));
                auto candidate = try_candidate(source_index,
                                               static_cast<int>(result.plan.size()),
                                               source_after, target_after, true);
                if (candidate) return candidate;
            }
            return std::nullopt;
        };

        std::optional<RepairCandidate> winner;
        auto consider = [&](std::optional<RepairCandidate> candidate) {
            if (!candidate) return;
            if (!winner
                || candidate->failed.size() < winner->failed.size()
                || (candidate->failed.size() == winner->failed.size()
                    && candidate->gate.candidate_cost < winner->gate.candidate_cost)) {
                winner = std::move(candidate);
            }
        };
        consider(probe_relocation());
        consider(probe_exchange());
        if (!winner) consider(probe_singleton());
        if (!winner) {

            break;
        }
        result.plan = std::move(winner->plan);

        current_failed = std::move(winner->failed);
    }

    result.fully_repaired = current_failed.empty();

    return result;
}

std::string unordered_route_key(Route route) {
    std::sort(route.begin(), route.end());
    std::ostringstream os;
    for (std::size_t i = 0; i < route.size(); ++i) {
        if (i) os << ',';
        os << route[i];
    }
    return os.str();
}

RoutePlan remove_empty_routes(RoutePlan plan) {
    RoutePlan out;
    for (auto& r : plan) if (!r.empty()) out.push_back(r);
    if (out.empty()) out.push_back({});
    return out;
}

RoutePlan initial_one_supplier_routes(const hrlh_model::ProblemInstance& inst) {
    RoutePlan plan;
    plan.reserve(inst.suppliers.size());
    for (int supplier : inst.suppliers) plan.push_back(Route{supplier});
    return plan;
}

RoutePlan random_plan(const hrlh_model::ProblemInstance& inst, std::uint64_t& rng) {
    auto suppliers = inst.suppliers;
    for (int i = (int)suppliers.size() - 1; i > 0; --i) std::swap(suppliers[i], suppliers[rand_int(rng, 0, i)]);
    int n_routes = std::max(1, std::min((int)inst.vehicles.size(), (int)suppliers.size()));
    RoutePlan plan(n_routes);
    for (int s : suppliers) plan[rand_int(rng, 0, n_routes - 1)].push_back(s);
    return remove_empty_routes(plan);
}

RoutePlan mutate_plan(RoutePlan plan, const hrlh_model::ProblemInstance& inst,
                      std::uint64_t& rng,
                      const std::function<bool(const Route&, int)>& addition_nogood = {},
                      bool* nogood_rejected = nullptr) {
    if (nogood_rejected != nullptr) *nogood_rejected = false;
    if (plan.empty()) return random_plan(inst, rng);
    const RoutePlan original = plan;
    auto reject_nogood = [&]() {
        if (nogood_rejected != nullptr) *nogood_rejected = true;
        return original;
    };

    static constexpr int descent_ops[] = {0, 1, 2, 4, 5};
    const int op = descent_ops[rand_int(rng, 0, 4)];
    if (op == 0) {
        std::vector<std::pair<int,int>> pos;
        for (int r=0;r<(int)plan.size();++r) for (int i=0;i<(int)plan[r].size();++i) pos.push_back({r,i});
        if (pos.size() >= 2) {
            auto a = pos[rand_int(rng,0,(int)pos.size()-1)];
            auto b = pos[rand_int(rng,0,(int)pos.size()-1)];
            std::swap(plan[a.first][a.second], plan[b.first][b.second]);
            if (a.first != b.first && addition_nogood) {
                Route base_a = original[static_cast<std::size_t>(a.first)];
                Route base_b = original[static_cast<std::size_t>(b.first)];
                const int incoming_a = original[static_cast<std::size_t>(b.first)]
                    [static_cast<std::size_t>(b.second)];
                const int incoming_b = original[static_cast<std::size_t>(a.first)]
                    [static_cast<std::size_t>(a.second)];
                base_a.erase(base_a.begin() + a.second);
                base_b.erase(base_b.begin() + b.second);
                if (addition_nogood(base_a, incoming_a)
                    || addition_nogood(base_b, incoming_b)) {
                    return reject_nogood();
                }
            }
        }
    } else if (op == 1) {
        std::vector<int> nonempty;
        for (int r=0;r<(int)plan.size();++r) if (!plan[r].empty()) nonempty.push_back(r);
        if (!nonempty.empty()) {
            int from = nonempty[rand_int(rng,0,(int)nonempty.size()-1)];
            int idx = rand_int(rng,0,(int)plan[from].size()-1);
            int s = plan[from][idx];
            plan[from].erase(plan[from].begin()+idx);
            int to = rand_int(rng,0,(int)plan.size()-1);
            int pos = rand_int(rng,0,(int)plan[to].size());
            plan[to].insert(plan[to].begin()+pos, s);
            if (to != from && addition_nogood) {
                Route target_base = plan[static_cast<std::size_t>(to)];
                target_base.erase(target_base.begin() + pos);
                if (addition_nogood(target_base, s)) return reject_nogood();
            }
        }
    } else if (op == 2) {
        int r = rand_int(rng,0,(int)plan.size()-1);
        if (plan[r].size() >= 2) {
            int a = rand_int(rng,0,(int)plan[r].size()-1);
            int b = rand_int(rng,0,(int)plan[r].size()-1);
            if (a > b) std::swap(a,b);
            std::reverse(plan[r].begin()+a, plan[r].begin()+b+1);
        }
    } else if (op == 4) {
        int best = -1;
        for (int r=0;r<(int)plan.size();++r) if (plan[r].size() >= 2 && (best<0 || plan[r].size() > plan[best].size())) best = r;
        if (best >= 0 && (int)plan.size() < (int)inst.vehicles.size()) {
            int cut = rand_int(rng,1,(int)plan[best].size()-1);
            Route nr(plan[best].begin()+cut, plan[best].end());
            plan[best].erase(plan[best].begin()+cut, plan[best].end());
            plan.push_back(nr);
        }
    } else {
        if (plan.size() >= 2) {
            int a = rand_int(rng,0,(int)plan.size()-1), b = rand_int(rng,0,(int)plan.size()-1);
            if (a != b) {
                if (addition_nogood) {
                    Route base = plan[static_cast<std::size_t>(a)];
                    for (int supplier : plan[static_cast<std::size_t>(b)]) {
                        if (addition_nogood(base, supplier)) return reject_nogood();
                        base.push_back(supplier);
                    }
                }
                plan[a].insert(plan[a].end(), plan[b].begin(), plan[b].end());
                plan[b].clear();
            }
        }
    }
    return remove_empty_routes(plan);
}

void maybe_update_pool(std::vector<CandidateSolution>& pool, CandidateSolution sol) {
    sol.signature = route_plan_signature(sol.routes);
    if (pool.empty()) {
        pool.push_back(std::move(sol));  return;
    }
    double best_cost = pool.front().transport_cost;
    std::int64_t best_profit = pool.front().optional_profit;
    if (lexicographic_cost_profit_better(
            sol.transport_cost, sol.optional_profit, best_cost, best_profit)) {
        pool.clear(); pool.push_back(std::move(sol));  return;
    }
    if (sol.transport_cost == best_cost && sol.optional_profit == best_profit) {
        for (const auto& s : pool) if (s.signature == sol.signature) return;
        pool.push_back(std::move(sol));  return;
    }
}


bool packing_has_static_proof(const PackingPlanEval& pack) {
    return std::any_of(pack.route_details.begin(), pack.route_details.end(),
        [](const RoutePackingDetail& detail) {
            return !detail.feasible && detail.static_lb_infeasible
                && detail.error_message.find("lower bound preprocessing exception")
                    == std::string::npos;
        });
}

bool packing_has_certified_proof(const PackingPlanEval& pack) {
    return packing_has_static_proof(pack);
}



long long round_scaled(double x, double scale) {
    return static_cast<long long>(std::llround(x * scale));
}

std::map<int, std::int64_t> mandatory_floor_area_by_supplier(const hrlh_model::ProblemInstance& inst) {
    std::map<int, std::int64_t> out;
    for (int s : inst.suppliers) out[s] = 0;
    for (const auto& it : inst.items) {
        if (!it.is_optional) out[it.supplier] += 1LL * it.length * it.width;
    }
    return out;
}

#ifdef HRLH_WITH_ILS_SEARCH
std::vector<search_dff::CapacityProfile> build_fixed_dff_capacity_profiles(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    bool protected_area_capacity) {
    search_dff::SupplierRectangles input;
    input.suppliers = inst.suppliers;
    for (int supplier : inst.suppliers) {
        auto specs = build_route_mandatory_packing_specs(inst, prep, Route{supplier});
        auto& rectangles = input.rectangles_by_supplier[supplier];
        for (const auto& spec : specs) {
            for (int copy = 0; copy < spec.mandatory_count; ++copy) {
                rectangles.push_back({spec.width, spec.length});
            }
        }
    }
    return search_dff::build_fixed_profiles(
        inst.vehicle_width, inst.vehicle_length, input, protected_area_capacity)
        .surviving_profiles;
}

hrlh_ils_search::CaseSpec build_ils_case_spec(const hrlh_model::ProblemInstance& inst,
                                               const HrlhConfig& cfg,
                                               const std::map<int, std::int64_t>* preprocessed_mandatory_floor_area = nullptr,
                                               const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult* prep = nullptr) {
    hrlh_ils_search::CaseSpec spec;
    const double scale = 100.0;
    int n = static_cast<int>(inst.suppliers.size());
    spec.locations.resize(static_cast<size_t>(n + 1));
    spec.depots.push_back({0, 0, 1000000000000LL, 0});

    const auto fallback_area = mandatory_floor_area_by_supplier(inst);
    const auto dff_profiles = cfg.ils_use_dff_capacity_dimensions && prep
        ? build_fixed_dff_capacity_profiles(
            inst, *prep, cfg.ils_use_mandatory_floor_area_capacity)
        : std::vector<search_dff::CapacityProfile>{};
    for (int idx = 0; idx < n; ++idx) {
        int supplier = inst.suppliers[idx];
        hrlh_ils_search::ClientSpec c;
        c.location = static_cast<size_t>(idx + 1);
        c.delivery.assign(1 + dff_profiles.size(), 0);
        std::int64_t mandatory_area = fallback_area.at(supplier);
        if (preprocessed_mandatory_floor_area) {
            auto it = preprocessed_mandatory_floor_area->find(supplier);
            if (it != preprocessed_mandatory_floor_area->end()) mandatory_area = it->second;
        }
        c.pickup = {cfg.ils_use_mandatory_floor_area_capacity ? mandatory_area : 0};
        for (const auto& profile : dff_profiles) {
            auto demand = profile.demand_by_supplier.find(supplier);
            c.pickup.push_back(demand == profile.demand_by_supplier.end() ? 0 : demand->second);
        }
        c.serviceDuration = round_scaled(inst.service_time.at(supplier), scale);
        c.twEarly = 0;
        c.twLate = 1000000000000LL;
        c.releaseTime = 0;
        c.latestReturn = cfg.ils_enforce_latest_return
            ? round_scaled(inst.latest_return.at(supplier), scale)
            : 1000000000000LL;
        c.required = true;
        spec.clients.push_back(c);
    }

    hrlh_ils_search::VehicleTypeSpec vt;
    vt.numAvailable = inst.vehicles.size();
    vt.capacity = {cfg.ils_use_mandatory_floor_area_capacity
        ? 1LL * inst.vehicle_length * inst.vehicle_width
        : 1000000000000LL};
    for (const auto& profile : dff_profiles) vt.capacity.push_back(profile.capacity);
    vt.startDepot = 0;
    vt.endDepot = 0;
    vt.fixedCost = round_scaled(inst.fixed_vehicle_cost, scale);
    vt.twEarly = 0;
    vt.twLate = 1000000000000LL;
    vt.shiftDuration = 1000000000000LL;
    vt.maxDistance = 1000000000000LL;
    vt.unitDistanceCost = 1;
    vt.unitDurationCost = 0;
    vt.profile = 0;
    spec.vehicleTypes.push_back(vt);

    size_t locN = spec.locations.size();
    hrlh_ils_search::MatrixSpec dist, dur;
    dist.values.assign(locN * locN, 0);
    dur.values.assign(locN * locN, 0);
    auto node_for_loc = [&](size_t loc)->int {
        if (loc == 0) return inst.start_depot();
        return inst.suppliers[loc - 1];
    };
    for (size_t i=0; i<locN; ++i) {
        for (size_t j=0; j<locN; ++j) {
            if (i == j) continue;
            int ni = node_for_loc(i);
            int nj = node_for_loc(j);
            const double distance_weight = cfg.ils_align_transport_cost
                ? inst.alpha_distance_cost
                : 1.0;
            dist.values[i * locN + j]
                = round_scaled(distance_weight * inst.distance.at({ni, nj}), scale);
            dur.values[i * locN + j] = round_scaled(inst.travel_time.at({ni, nj}), scale);
        }
    }
    spec.distanceMatrices.push_back(std::move(dist));
    spec.durationMatrices.push_back(std::move(dur));
    spec.solver.seed = static_cast<std::uint32_t>(cfg.seed & 0xffffffffULL);
    spec.solver.maxIterations = static_cast<size_t>(std::max(0, cfg.ils_iterations));
    spec.solver.useSingletonWarmStart = cfg.ils_use_singleton_warm_start;
    spec.solver.penalty.solutionsBetweenUpdates = static_cast<size_t>(std::max(1, cfg.ils_penalty_solutions_between_updates));
    spec.solver.penalty.penaltyIncrease = cfg.ils_penalty_increase;
    spec.solver.penalty.penaltyDecrease = cfg.ils_penalty_decrease;
    spec.solver.penalty.targetFeasible = cfg.ils_penalty_target_feasible;
    spec.solver.penalty.feasTolerance = cfg.ils_penalty_feas_tolerance;
    spec.solver.penalty.minPenalty = cfg.ils_penalty_min;
    spec.solver.penalty.maxPenalty = cfg.ils_penalty_max;
    spec.solver.neighbourhood.numNeighbours = static_cast<size_t>(std::max(1, cfg.ils_neighbours));
    spec.solver.neighbourhood.weightWaitTime = cfg.ils_neighbour_weight_wait_time;
    spec.solver.neighbourhood.symmetricProximity = cfg.ils_neighbour_symmetric_proximity;
    spec.solver.ils.numItersNoImprovement = static_cast<size_t>(std::max(1, cfg.ils_no_improvement_limit));
    spec.solver.ils.historyLength = static_cast<size_t>(std::max(1, cfg.ils_history_length));
    spec.solver.ils.exhaustiveOnBest = cfg.ils_exhaustive_on_best;
    int minPert = std::max(0, cfg.ils_min_perturbations);
    int maxPert = std::max(minPert, cfg.ils_max_perturbations);
    spec.solver.perturbation.minPerturbations = static_cast<size_t>(minPert);
    spec.solver.perturbation.maxPerturbations = static_cast<size_t>(maxPert);
    spec.solver.operators = cfg.ils_operator_names.empty()
        ? hrlh_ils_search::defaultOperators()
        : cfg.ils_operator_names;
    return spec;
}

RoutePlan route_plan_from_ils_solution(const pyvrp::Solution& sol,
                                         const hrlh_model::ProblemInstance& inst) {
    RoutePlan plan;
    for (const auto& pr : sol.routes()) {
        Route r;
        const auto& sched = pr.schedule();
        for (size_t idx = 1; idx + 1 < sched.size(); ++idx) {
            const auto& act = sched[idx];
            if (act.isClient()) {
                size_t cidx = act.idx();
                if (cidx < inst.suppliers.size()) r.push_back(inst.suppliers[cidx]);
            }
        }
        if (!r.empty()) plan.push_back(std::move(r));
    }
    return remove_empty_routes(plan);
}

std::vector<hrlh_ils_search::RouteSpec> ils_initial_routes_from_route_plan(
    const RoutePlan& plan,
    const hrlh_model::ProblemInstance& inst) {
    std::vector<hrlh_ils_search::RouteSpec> routes;
    routes.reserve(plan.size());
    for (const auto& route : plan) {
        if (route.empty()) continue;
        hrlh_ils_search::RouteSpec out;
        out.vehicleType = 0;
        out.activities.reserve(route.size());
        for (int supplier : route) {
            const auto it = std::find(inst.suppliers.begin(), inst.suppliers.end(), supplier);
            if (it == inst.suppliers.end()) {
                throw std::invalid_argument("first-layer route contains an unknown supplier");
            }
            out.activities.push_back({
                pyvrp::Activity::ActivityType::CLIENT,
                static_cast<std::size_t>(it - inst.suppliers.begin())});
        }
        routes.push_back(std::move(out));
    }
    return routes;
}
#endif

}

void CertifiedNoGoodPool::rebuild_index() {
    by_supplier_.clear();
    for (std::size_t id = 0; id < master_antichain_.size(); ++id) {
        for (int supplier : master_antichain_[id]) {
            by_supplier_[supplier].push_back(id);
        }
    }
}

bool CertifiedNoGoodPool::insert(Route certified_route) {
    certified_route = canonical_supplier_set(std::move(certified_route));
    if (certified_route.empty()) return false;
    for (const auto& existing : master_antichain_) {
        if (is_supplier_subset_sorted(existing, certified_route)) return false;
    }
    master_antichain_.erase(
        std::remove_if(master_antichain_.begin(), master_antichain_.end(),
                       [&](const Route& existing) {
                           return is_supplier_subset_sorted(certified_route, existing);
                       }),
        master_antichain_.end());
    master_antichain_.push_back(std::move(certified_route));
    std::sort(master_antichain_.begin(), master_antichain_.end());
    rebuild_index();
    return true;
}

const Route* CertifiedNoGoodPool::matching_after_addition(
    const Route& route_before_addition,
    int incoming_supplier) const {

    const auto posting = by_supplier_.find(incoming_supplier);
    if (posting == by_supplier_.end()) return nullptr;
    Route after = route_before_addition;
    after.push_back(incoming_supplier);
    after = canonical_supplier_set(std::move(after));
    for (std::size_t id : posting->second) {

        const Route& nogood = master_antichain_[id];
        if (is_supplier_subset_sorted(nogood, after)) return &nogood;
    }
    return nullptr;
}

const Route* CertifiedNoGoodPool::matching_route(
    const Route& route) const {

    const Route canonical = canonical_supplier_set(route);
    std::vector<std::size_t> candidates;
    for (int supplier : canonical) {
        const auto posting = by_supplier_.find(supplier);
        if (posting == by_supplier_.end()) continue;
        candidates.insert(candidates.end(), posting->second.begin(), posting->second.end());
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    for (std::size_t id : candidates) {

        const Route& nogood = master_antichain_[id];
        if (is_supplier_subset_sorted(nogood, canonical)) return &nogood;
    }
    return nullptr;
}

HrlhConfig default_hrlh_config() {
    return HrlhConfig{};
}

void validate_hrlh_config(const HrlhConfig& cfg) {
    auto fail = [](const std::string& key, const std::string& detail) {
        throw std::invalid_argument("invalid solver config " + key + ": " + detail);
    };
    auto require = [&](bool condition, const std::string& key, const std::string& detail) {
        if (!condition) fail(key, detail);
    };
    auto finite = [&](double value, const std::string& key) {
        require(std::isfinite(value), key, "must be finite");
    };
    auto nonnegative = [&](double value, const std::string& key) {
        finite(value, key);
        require(value >= 0.0, key, "must be >= 0");
    };
    auto probability = [&](double value, const std::string& key) {
        finite(value, key);
        require(value >= 0.0 && value <= 1.0, key, "must be in [0, 1]");
    };
    auto stage_time_limit = [&](double value, const std::string& key) {
        finite(value, key);
        require(value == -1.0 || value >= 0.0, key, "must be -1 or >= 0");
    };

    require(cfg.max_iterations >= 0, "routing.max_iterations", "must be >= 0");
    require(cfg.ils_iterations >= 0, "routing.ils_iterations", "must be >= 0");

    require(cfg.ils_penalty_solutions_between_updates >= 1,
            "routing.ils_penalty_solutions_between_updates", "must be >= 1");
    finite(cfg.ils_penalty_increase, "routing.ils_penalty_increase");
    require(cfg.ils_penalty_increase > 0.0, "routing.ils_penalty_increase", "must be > 0");
    finite(cfg.ils_penalty_decrease, "routing.ils_penalty_decrease");
    require(cfg.ils_penalty_decrease > 0.0, "routing.ils_penalty_decrease", "must be > 0");
    probability(cfg.ils_penalty_target_feasible, "routing.ils_penalty_target_feasible");
    probability(cfg.ils_penalty_feas_tolerance, "routing.ils_penalty_feas_tolerance");
    finite(cfg.ils_penalty_min, "routing.ils_penalty_min");
    require(cfg.ils_penalty_min > 0.0, "routing.ils_penalty_min", "must be > 0");
    finite(cfg.ils_penalty_max, "routing.ils_penalty_max");
    require(cfg.ils_penalty_max >= cfg.ils_penalty_min,
            "routing.ils_penalty_max", "must be >= ils_penalty_min");
    require(cfg.ils_neighbours >= 1, "routing.ils_neighbours", "must be >= 1");
    nonnegative(cfg.ils_neighbour_weight_wait_time, "routing.ils_neighbour_weight_wait_time");
    require(cfg.ils_no_improvement_limit >= 1, "routing.ils_no_improvement_limit", "must be >= 1");
    require(cfg.ils_history_length >= 1, "routing.ils_history_length", "must be >= 1");
    require(cfg.ils_min_perturbations >= 0, "routing.ils_min_perturbations", "must be >= 0");
    require(cfg.ils_max_perturbations >= cfg.ils_min_perturbations,
            "routing.ils_max_perturbations", "must be >= ils_min_perturbations");

    require(cfg.deep_packing_retry_iters >= 0,
            "routing.deep_packing_retry_iters", "must be >= 0");
    require(cfg.initial_diversification_starts >= 0,
            "routing.initial_diversification_starts", "must be >= 0");

    probability(cfg.feasible_descent_budget_fraction,
                "routing.feasible_descent_budget_fraction");
    require(cfg.feasible_descent_no_improvement_limit >= 0,
            "routing.feasible_descent_no_improvement_limit", "must be >= 0");

    require(cfg.packing_mutation_iters >= 0, "packing.packing_iters", "must be >= 0");
    probability(cfg.packing_mutation_swap_ratio, "packing.mutation_swap_ratio");

    const auto& prep = cfg.preprocess_config;
    stage_time_limit(prep.mandatory_stack_time_limit, "preprocess.mandatory_stack_time_limit");
    stage_time_limit(prep.baseline_joint_time_limit, "preprocess.baseline_joint_time_limit");
    stage_time_limit(prep.time_limit_R_all, "preprocess.time_limit_r_all");
    stage_time_limit(prep.spread_joint_time_limit, "preprocess.spread_joint_time_limit");
    nonnegative(prep.mip_gap, "preprocess.mip_gap");
    nonnegative(prep.max_wall_seconds, "preprocess.max_wall_seconds");
    nonnegative(prep.gurobi_single_mip_time_limit, "preprocess.gurobi_single_mip_time_limit");
    require(prep.gurobi_symmetry >= -1 && prep.gurobi_symmetry <= 2,
            "preprocess.gurobi_symmetry", "must be in [-1, 2]");
    require(prep.gurobi_retry_solution_limit >= 1, "gurobi_retry.solution_limit", "must be >= 1");
    require(prep.gurobi_retry_mip_focus >= 0 && prep.gurobi_retry_mip_focus <= 3,
            "gurobi_retry.mip_focus", "must be in [0, 3]");

    require(std::isfinite(cfg.optional_profit_intensification_budget_fraction)
                && cfg.optional_profit_intensification_budget_fraction >= 0.0
                && cfg.optional_profit_intensification_budget_fraction <= 1.0,
            "packing.optional_profit_intensification_budget_fraction", "must be finite and in [0, 1]");

    nonnegative(cfg.max_wall_seconds, "budget.max_wall_seconds");
    require(cfg.max_packing_calls >= 0, "budget.max_packing_calls", "must be >= 0");
}

#ifdef HRLH_WITH_ILS_SEARCH

#endif

namespace {

std::string cfg_trim(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::string cfg_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

int cfg_parse_int_strict(const std::string& raw, const std::string& key) {
    const std::string v = cfg_trim(raw);
    std::size_t used = 0;
    try {
        int out = std::stoi(v, &used);
        if (used != v.size()) throw std::invalid_argument("trailing characters");
        return out;
    } catch (const std::exception&) {
        throw std::invalid_argument("invalid integer value for " + key + ": " + raw);
    }
}

std::int64_t cfg_parse_int64_strict(const std::string& raw, const std::string& key) {
    const std::string v = cfg_trim(raw);
    std::size_t used = 0;
    try {
        long long out = std::stoll(v, &used);
        if (used != v.size()) throw std::invalid_argument("trailing characters");
        return static_cast<std::int64_t>(out);
    } catch (const std::exception&) {
        throw std::invalid_argument("invalid integer value for " + key + ": " + raw);
    }
}

double cfg_parse_double_strict(const std::string& raw, const std::string& key) {
    const std::string v = cfg_trim(raw);
    std::size_t used = 0;
    try {
        double out = std::stod(v, &used);
        if (used != v.size() || !std::isfinite(out)) throw std::invalid_argument("invalid double");
        return out;
    } catch (const std::exception&) {
        throw std::invalid_argument("invalid floating-point value for " + key + ": " + raw);
    }
}

bool cfg_parse_bool(const std::string& raw) {
    std::string v = cfg_lower(cfg_trim(raw));
    if (v == "1" || v == "true" || v == "yes" || v == "on") return true;
    if (v == "0" || v == "false" || v == "no" || v == "off") return false;
    throw std::invalid_argument("invalid boolean value: " + raw);
}

bool cfg_parse_use_gurobi(const std::string& raw) {
    std::string v = cfg_lower(cfg_trim(raw));
    if (v == "auto" || v == "default" || v == "compile" || v == "compiled") {
#ifdef HRLH_STACKS_WITH_GUROBI
        return true;
#else
        return false;
#endif
    }
    return cfg_parse_bool(raw);
}

std::vector<std::string> cfg_parse_csv(const std::string& raw) {
    std::vector<std::string> out;
    std::stringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token = cfg_trim(token);
        if (!token.empty()) out.push_back(token);
    }
    return out;
}

}

bool apply_hrlh_config_value(HrlhConfig& cfg,
                                    const std::string& raw_key,
                                    const std::string& value) {
    const std::string key = cfg_lower(raw_key);
    const auto i = [&](const std::string& v) { return cfg_parse_int_strict(v, key); };
    const auto d = [&](const std::string& v) { return cfg_parse_double_strict(v, key); };

    if (key.rfind("instance.", 0) == 0 || key.rfind("random_instance.", 0) == 0) return false;
    if (key == "global.seed") {
        const auto parsed = cfg_parse_int64_strict(value, key);
        if (parsed < 0) throw std::invalid_argument("invalid solver config global.seed: must be >= 0");
        cfg.seed = static_cast<std::uint64_t>(parsed);
        return true;
    }

    if (key == "preprocess.use_gurobi") cfg.use_gurobi_preprocess = cfg_parse_use_gurobi(value);
    else if (key == "preprocess.strict_same_canonical_size") cfg.strict_same_canonical_size = cfg_parse_bool(value);
    else if (key == "preprocess.enable_spread_joint") cfg.preprocess_config.enable_spread_joint = cfg_parse_bool(value);
    else if (key == "preprocess.require_preprocess_optimal") { const bool v = cfg_parse_bool(value); cfg.require_preprocess_optimal = v; cfg.preprocess_config.require_optimal = v; }
    else if (key == "preprocess.mandatory_stack_time_limit") cfg.preprocess_config.mandatory_stack_time_limit = d(value);
    else if (key == "preprocess.baseline_joint_time_limit") cfg.preprocess_config.baseline_joint_time_limit = d(value);
    else if (key == "preprocess.time_limit_r_all") cfg.preprocess_config.time_limit_R_all = d(value);
    else if (key == "preprocess.spread_joint_time_limit") cfg.preprocess_config.spread_joint_time_limit = d(value);
    else if (key == "preprocess.mip_gap") cfg.preprocess_config.mip_gap = d(value);
    else if (key == "preprocess.verbose") cfg.preprocess_config.verbose = cfg_parse_bool(value);
    else if (key == "preprocess.require_optimal") { const bool v = cfg_parse_bool(value); cfg.preprocess_config.require_optimal = v; cfg.require_preprocess_optimal = v; }
    else if (key == "preprocess.symmetry_breaking") cfg.preprocess_config.symmetry_breaking = cfg_parse_bool(value);
    else if (key == "preprocess.load_order_symmetry") cfg.preprocess_config.load_order_symmetry = cfg_parse_bool(value);
    else if (key == "preprocess.use_warm_start") cfg.preprocess_config.use_warm_start = cfg_parse_bool(value);
    else if (key == "preprocess.max_wall_seconds") cfg.preprocess_config.max_wall_seconds = d(value);
    else if (key == "preprocess.gurobi_single_mip_time_limit") cfg.preprocess_config.gurobi_single_mip_time_limit = d(value);
    else if (key == "preprocess.gurobi_symmetry") cfg.preprocess_config.gurobi_symmetry = i(value);

    else if (key == "gurobi_retry.enabled") cfg.preprocess_config.gurobi_retry_enabled = cfg_parse_bool(value);
    else if (key == "gurobi_retry.retry_if_no_incumbent") cfg.preprocess_config.gurobi_retry_if_no_incumbent = cfg_parse_bool(value);
    else if (key == "gurobi_retry.solution_limit") cfg.preprocess_config.gurobi_retry_solution_limit = i(value);
    else if (key == "gurobi_retry.mip_focus") cfg.preprocess_config.gurobi_retry_mip_focus = i(value);
    else if (key == "gurobi_retry.remove_local_time_limit") cfg.preprocess_config.gurobi_retry_remove_local_time_limit = cfg_parse_bool(value);
    else if (key == "gurobi_retry.respect_global_wall_seconds") cfg.preprocess_config.gurobi_retry_respect_global_wall_seconds = cfg_parse_bool(value);
    else if (key == "gurobi_retry.retry_verbose") cfg.preprocess_config.gurobi_retry_verbose = cfg_parse_bool(value);

    else if (key == "routing.max_iterations") cfg.max_iterations = i(value);
    else if (key == "routing.enable_ils_routing_search") cfg.enable_ils_routing_search = cfg_parse_bool(value);
    else if (key == "routing.ils_iterations") cfg.ils_iterations = i(value);
    else if (key == "routing.ils_use_mandatory_floor_area_capacity") cfg.ils_use_mandatory_floor_area_capacity = cfg_parse_bool(value);
    else if (key == "routing.ils_use_dff_capacity_dimensions") cfg.ils_use_dff_capacity_dimensions = cfg_parse_bool(value);
    else if (key == "routing.ils_align_transport_cost") cfg.ils_align_transport_cost = cfg_parse_bool(value);
    else if (key == "routing.ils_enforce_latest_return") cfg.ils_enforce_latest_return = cfg_parse_bool(value);
    else if (key == "routing.ils_use_singleton_warm_start") cfg.ils_use_singleton_warm_start = cfg_parse_bool(value);
    else if (key == "routing.ils_penalty_solutions_between_updates") cfg.ils_penalty_solutions_between_updates = i(value);
    else if (key == "routing.ils_penalty_increase") cfg.ils_penalty_increase = d(value);
    else if (key == "routing.ils_penalty_decrease") cfg.ils_penalty_decrease = d(value);
    else if (key == "routing.ils_penalty_target_feasible") cfg.ils_penalty_target_feasible = d(value);
    else if (key == "routing.ils_penalty_feas_tolerance") cfg.ils_penalty_feas_tolerance = d(value);
    else if (key == "routing.ils_penalty_min") cfg.ils_penalty_min = d(value);
    else if (key == "routing.ils_penalty_max") cfg.ils_penalty_max = d(value);
    else if (key == "routing.ils_operators") cfg.ils_operator_names = cfg_parse_csv(value);
    else if (key == "routing.ils_neighbours") cfg.ils_neighbours = i(value);
    else if (key == "routing.ils_neighbour_weight_wait_time") cfg.ils_neighbour_weight_wait_time = d(value);
    else if (key == "routing.ils_neighbour_symmetric_proximity") cfg.ils_neighbour_symmetric_proximity = cfg_parse_bool(value);
    else if (key == "routing.ils_no_improvement_limit") cfg.ils_no_improvement_limit = i(value);
    else if (key == "routing.ils_history_length") cfg.ils_history_length = i(value);
    else if (key == "routing.ils_exhaustive_on_best") cfg.ils_exhaustive_on_best = cfg_parse_bool(value);
    else if (key == "routing.ils_min_perturbations") cfg.ils_min_perturbations = i(value);
    else if (key == "routing.ils_max_perturbations") cfg.ils_max_perturbations = i(value);
    else if (key == "routing.enable_packing_guided_route_repair") cfg.enable_packing_guided_route_repair = cfg_parse_bool(value);
    else if (key == "routing.isolate_route_repair_rng") cfg.isolate_route_repair_rng = cfg_parse_bool(value);
    else if (key == "routing.enforce_route_repair_cost_gate") cfg.enforce_route_repair_cost_gate = cfg_parse_bool(value);
    else if (key == "routing.deep_packing_retry_iters"
             || key == "routing.promising_packing_retry_iters") cfg.deep_packing_retry_iters = i(value);
    else if (key == "routing.enable_proven_infeasible_subset_nogoods") cfg.enable_proven_infeasible_subset_nogoods = cfg_parse_bool(value);
    else if (key == "routing.initial_diversification_starts") cfg.initial_diversification_starts = i(value);
    else if (key == "routing.feasible_descent_budget_fraction") cfg.feasible_descent_budget_fraction = d(value);
    else if (key == "routing.feasible_descent_no_improvement_limit") cfg.feasible_descent_no_improvement_limit = i(value);
    else if (key == "routing.enforce_time_feasible") cfg.enforce_time_feasible = cfg_parse_bool(value);
    else if (key == "routing.enforce_volume_necessary") cfg.enforce_volume_necessary = cfg_parse_bool(value);

    else if (key == "packing.packing_iters" || key == "packing.mutation_iters") cfg.packing_mutation_iters = i(value);
    else if (key == "packing.mutation_swap_ratio") cfg.packing_mutation_swap_ratio = d(value);

    else if (key == "route_packing_cache.enabled") cfg.route_packing_cache.enabled = cfg_parse_bool(value);
    else if (key == "route_packing_cache.enable_feasible_pool") cfg.route_packing_cache.enable_feasible_pool = cfg_parse_bool(value);
    else if (key == "route_packing_cache.max_feasible_entries") do { const int v = i(value); if (v < 0) throw std::invalid_argument("invalid solver config route_packing_cache.max_feasible_entries: must be >= 0"); cfg.route_packing_cache.max_feasible_entries = static_cast<std::size_t>(v); } while (false);
    else if (key == "route_packing_cache.skip_skyline_if_cached_reaches_upper_bound") cfg.route_packing_cache.skip_skyline_if_cached_reaches_upper_bound = cfg_parse_bool(value);
    else if (key == "route_packing_cache.skip_static_pruning_on_feasible_hit") cfg.route_packing_cache.skip_static_pruning_on_feasible_hit = cfg_parse_bool(value);
    else if (key == "route_packing_cache.rerun_skyline_on_feasible_hit") cfg.route_packing_cache.rerun_skyline_on_feasible_hit = cfg_parse_bool(value);
    else if (key == "route_packing_cache.use_cached_best_if_current_worse") cfg.route_packing_cache.use_cached_best_if_current_worse = cfg_parse_bool(value);
    else if (key == "route_packing_cache.store_layout") cfg.route_packing_cache.store_layout = cfg_parse_bool(value);

    else if (key == "packing.enable_optional_demand_profit_intensification") cfg.enable_optional_demand_profit_intensification = cfg_parse_bool(value);
    else if (key == "packing.optional_profit_intensification_budget_fraction") cfg.optional_profit_intensification_budget_fraction = d(value);
    else if (key == "budget.max_wall_seconds") cfg.max_wall_seconds = d(value);
    else if (key == "budget.max_packing_calls") cfg.max_packing_calls = i(value);

    else if (key == "output.verbose") cfg.verbose = cfg_parse_bool(value);
    else return false;
    return true;
}

hrlh_components::stack_preprocessing::StackPreprocessingInput to_rack_preprocess_instance(
    const hrlh_model::ProblemInstance& inst) {
    hrlh_components::stack_preprocessing::StackPreprocessingInput out;
    out.vehicle_length = inst.vehicle_length;
    out.vehicle_width = inst.vehicle_width;
    out.vehicle_height = inst.vehicle_height;
    for (const auto& it : inst.items) {
        hrlh_components::stack_preprocessing::RackItem r;
        r.item_id = it.item_id;
        r.supplier = it.supplier;
        r.length = it.length;
        r.width = it.width;
        r.height = it.height;
        r.is_pallet = it.is_pallet;
        r.is_optional = it.is_optional;
        r.score = it.score;

        for (const auto& q : inst.stack_compatibility) {
            if (q.first.first == it.item_id && q.second == 1) r.compatible_item_ids.insert(q.first.second);
        }
        out.items.push_back(r);
    }
    return out;
}

double compute_transport_cost(const hrlh_model::ProblemInstance& inst, const RoutePlan& plan) {
    double cost = 0.0;
    int start = inst.start_depot();
    int end = inst.end_depot();
    for (const auto& r : plan) {
        if (r.empty()) continue;
        int prev = start;
        for (int s : r) { cost += inst.alpha_distance_cost * inst.distance.at({prev, s}); prev = s; }
        cost += inst.alpha_distance_cost * inst.distance.at({prev, end});
        cost += inst.fixed_vehicle_cost;
    }
    return cost;
}

bool route_repair_cost_gate_allows(double candidate_cost,
                                       double incumbent_cost) noexcept {
    return !(candidate_cost > incumbent_cost);
}

bool lexicographic_cost_profit_better(double candidate_cost,
                                      std::int64_t candidate_profit,
                                      double incumbent_cost,
                                      std::int64_t incumbent_profit) noexcept {
    return candidate_cost < incumbent_cost
        || (candidate_cost == incumbent_cost
            && candidate_profit > incumbent_profit);
}

double route_duration(const hrlh_model::ProblemInstance& inst, const Route& route) {
    if (route.empty()) return 0.0;
    int start = inst.start_depot();
    int end = inst.end_depot();
    double d = 0.0;
    int prev = start;
    for (int s : route) {
        d += inst.travel_time.at({prev, s});
        d += inst.service_time.at(s);
        prev = s;
    }
    d += inst.travel_time.at({prev, end});
    return d;
}

double route_latest_departure(const hrlh_model::ProblemInstance& inst, const Route& route) {
    if (route.empty()) return 0.0;
    double min_latest = std::numeric_limits<double>::infinity();
    for (int s : route) min_latest = std::min(min_latest, inst.latest_return.at(s));
    return min_latest - route_duration(inst, route);
}

std::string route_plan_signature(RoutePlan plan) {
    for (auto& r : plan) r.shrink_to_fit();
    std::sort(plan.begin(), plan.end());
    std::ostringstream os;
    bool fr=true;
    for (const auto& r : plan) {
        if (!fr) os << "|"; fr=false;
        for (size_t i=0;i<r.size();++i) { if(i) os << ","; os << r[i]; }
    }
    return os.str();
}

std::string make_route_cache_key(const Route& route) {
    Route clients = route;
    std::sort(clients.begin(), clients.end());
    std::ostringstream os;
    for (size_t i = 0; i < clients.size(); ++i) {
        if (i) os << "|";
        os << clients[i];
    }
    return os.str();
}

std::uint64_t deterministic_packing_seed(std::uint64_t base_seed,
                                       const std::string& key) {
    std::uint64_t hash = 1469598103934665603ULL ^ base_seed;
    for (unsigned char ch : key) {
        hash ^= static_cast<std::uint64_t>(ch);
        hash *= 1099511628211ULL;
    }
    return splitmix64(hash);
}

long long compute_route_optional_profit_upper_bound(const std::vector<PackingTypeSpecCpp>& specs) {
    long long ub = 0;
    for (const auto& s : specs) {
        ub += s.fixed_inserted_score;
        for (long long v : s.optional_marginal_profits) ub += v;
    }
    return ub;
}

static std::vector<PackingTypeSpecCpp> build_route_packing_specs_raw(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const Route& route) {
    std::set<int> suppliers(route.begin(), route.end());
    std::vector<PackingTypeSpecCpp> specs;

    for (const auto& kv : prep.groups) {
        const auto& g = kv.second;
        int supplier = std::get<0>(g.group_key);
        if (!suppliers.count(supplier)) continue;
        std::map<int,long long> fv;
        long long fixed_score = static_cast<long long>(g.baseline_result.baseline_optional_profit);
        if (g.spread_joint_enabled) {
            const auto base = g.spread_joint_curve.at(0).optional_profit;
            fixed_score = static_cast<long long>(base);
            for (const auto& [r, point] : g.spread_joint_curve)
                fv[r] = static_cast<long long>(point.optional_profit - base);
        } else {
            for (const auto& [r, res] : g.value_table)
                fv[r] = static_cast<long long>(res.objective.value_or(0));
            if (fv.empty()) fv[0] = 0;
        }
        GroupPreprocessResultCpp gg;
        gg.group_key = group_key_string(g.group_key);
        gg.base_width = g.base_width;
        gg.base_length = g.base_length;
        gg.baseline_stack_count = g.baseline_result.baseline_stack_count;
        gg.R_max_g = g.spread_joint_enabled
            ? g.spread_joint_curve.rbegin()->first : g.R_max_g;
        gg.f_values = fv;
        gg.baseline_optional_profit = fixed_score;
        if (gg.baseline_stack_count > 0 || gg.R_max_g > 0 || gg.baseline_optional_profit > 0)
            specs.push_back(make_packing_type_from_preprocess_group_cpp(gg));
    }

    std::map<std::tuple<int,int,int>, int> pallet_counts;
    for (const auto& it : inst.items) {
        if (!it.is_pallet || !suppliers.count(it.supplier)) continue;
        pallet_counts[{it.supplier, it.width, it.length}]++;
    }
    for (const auto& [key, cnt] : pallet_counts) {
        int supplier, width, length;
        std::tie(supplier,width,length) = key;
        specs.push_back(make_pallet_type_spec_cpp("P:" + std::to_string(supplier) + ":" + std::to_string(width) + "x" + std::to_string(length), width, length, cnt));
    }
    return specs;
}

std::vector<PackingTypeSpecCpp> build_route_packing_specs(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const Route& route) {
    auto specs=build_route_packing_specs_raw(inst,prep,route);
    const bool has_spread=!prep.groups.empty()&&prep.groups.begin()->second.spread_joint_enabled;
    if(!has_spread)return specs;

    std::ostringstream signature;
    signature<<inst.vehicle_width<<':'<<inst.vehicle_length;
    for(const auto& s:specs) {
        signature<<'|'<<s.type_id<<':'<<s.width<<':'<<s.length<<':'
                 <<s.mandatory_count<<':'<<s.optional_count<<':'<<s.fixed_inserted_score;
        for(long long value:s.optional_marginal_profits)signature<<','<<value;
    }
#ifdef HRLH_COMPARISON_ENGINE
    auto& bounded_specs_cache = bounded_specs_cache_state();
#else
    static thread_local std::unordered_map<std::string,std::vector<PackingTypeSpecCpp>> bounded_specs_cache;
#endif
    const std::string key=signature.str();
    if(auto found=bounded_specs_cache.find(key);found!=bounded_specs_cache.end())return found->second;

    search_dff::SupplierRectangles rectangles;
    rectangles.suppliers.push_back(0);
    std::map<int,int> fake_supplier_by_spec;
    std::int64_t mandatory_area=0;
    for(int si=0;si<int(specs.size());++si) {
        const auto& s=specs[si];
        const std::int64_t unit_area=1LL*s.width*s.length;
        mandatory_area+=unit_area*s.mandatory_count;
        for(int copy=0;copy<s.mandatory_count;++copy)
            rectangles.rectangles_by_supplier[0].push_back({s.width,s.length});
        if(s.optional_count>0) {
            const int fake=si+1;
            fake_supplier_by_spec[si]=fake;
            rectangles.suppliers.push_back(fake);
            rectangles.rectangles_by_supplier[fake].push_back({s.width,s.length});
        }
    }
    std::vector<search_dff::CapacityProfile> profiles;
    try {
        profiles=search_dff::build_fixed_profiles(
            inst.vehicle_width,inst.vehicle_length,rectangles,false).raw_profiles;
    } catch(const std::exception&) {

    }
    const std::int64_t area_left=1LL*inst.vehicle_width*inst.vehicle_length-mandatory_area;
    for(const auto& [si,fake]:fake_supplier_by_spec) {
        auto& s=specs[si];
        const std::int64_t unit_area=1LL*s.width*s.length;
        std::int64_t cap=area_left<0 ? 0 : area_left/unit_area;
        cap=std::min<std::int64_t>(cap,s.optional_count);
        for(const auto& profile:profiles) {
            const auto coefficient=profile.demand_by_supplier.at(fake);
            const auto used=profile.demand_by_supplier.at(0);
            if(coefficient>0) {
                const auto residual=profile.capacity-used;
                cap=std::min<std::int64_t>(cap,residual<0 ? 0 : residual/coefficient);
            }
        }
        s.optional_count=static_cast<int>(cap);
        s.optional_marginal_profits.resize(static_cast<std::size_t>(s.optional_count));
    }
    if(bounded_specs_cache.size()>=2048)bounded_specs_cache.clear();
    bounded_specs_cache.emplace(key,specs);
    return specs;
}

std::vector<PackingTypeSpecCpp> build_route_mandatory_packing_specs(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const Route& route) {
    auto specs = build_route_packing_specs_raw(inst, prep, route);
    std::vector<PackingTypeSpecCpp> mandatory_specs;
    mandatory_specs.reserve(specs.size());
    for (auto& spec : specs) {
        spec.optional_count = 0;
        spec.optional_marginal_profits.clear();
        spec.fixed_inserted_score = 0;
        if (spec.mandatory_count > 0) {
            mandatory_specs.push_back(std::move(spec));
        }
    }
    return mandatory_specs;
}

void materialize_spread_solution(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    CandidateSolution& solution) {
    if (prep.groups.empty() || !prep.groups.begin()->second.spread_joint_enabled) return;
    if (solution.packing.route_details.size() != solution.routes.size())
        throw std::runtime_error("Spread-Joint final route-detail count mismatch");
    std::map<std::string, const hrlh_components::stack_preprocessing::GroupPreprocessResult*> group_by_type;
    for (const auto& [key, group] : prep.groups)
        group_by_type.emplace(group_key_string(key), &group);
    std::map<int, const hrlh_model::Item*> item_by_id;
    for (const auto& item : inst.items) item_by_id.emplace(item.item_id, &item);
    std::set<int> globally_loaded;
    std::int64_t physical_profit = 0;
    for (std::size_t ri = 0; ri < solution.routes.size(); ++ri) {
        auto& detail = solution.packing.route_details[ri];
        if (!detail.feasible || detail.route != solution.routes[ri])
            throw std::runtime_error("Spread-Joint final route mismatch");
        if (detail.loaded_optional_count_by_spec.size() != detail.specs.size())
            throw std::runtime_error("Spread-Joint missing final optional counts");
        detail.material_stacks_by_spec.assign(detail.specs.size(), {});
        std::int64_t route_profit = 0;
        std::int64_t expected_route_area = 0;
        for (std::size_t si = 0; si < detail.specs.size(); ++si) {
            const auto& spec = detail.specs[si];
            auto& stacks = detail.material_stacks_by_spec[si];
            if (spec.type_id.rfind("P:", 0) == 0) {
                for (const auto& item : inst.items)
                    if (item.is_pallet &&
                        std::find(detail.route.begin(), detail.route.end(), item.supplier) != detail.route.end() &&
                        spec.type_id == "P:" + std::to_string(item.supplier) + ":" +
                            std::to_string(item.width) + "x" + std::to_string(item.length))
                        stacks.push_back({item.item_id});
                if (static_cast<int>(stacks.size()) != spec.mandatory_count ||
                    detail.loaded_optional_count_by_spec[si] != 0)
                    throw std::runtime_error("Spread-Joint final pallet count mismatch");
            } else {
                const auto found = group_by_type.find(spec.type_id);
                if (found == group_by_type.end())
                    throw std::runtime_error("Spread-Joint final group lookup failed");
                const int r = detail.loaded_optional_count_by_spec[si];
                if (r < 0 || r > spec.optional_count)
                    throw std::runtime_error("Spread-Joint final optional count out of range");
                const auto point = found->second->spread_joint_curve.find(r);
                if (point == found->second->spread_joint_curve.end() ||
                    point->second.stack_count != spec.mandatory_count + r)
                    throw std::runtime_error("Spread-Joint final whole assignment missing");
                stacks = point->second.stacks;
            }
            for (const auto& stack : stacks) {
                if (stack.empty()) throw std::runtime_error("Spread-Joint final empty stack");
                int height = 0;
                for (int id : stack) {
                    const auto item = item_by_id.at(id);
                    if (!globally_loaded.insert(id).second ||
                        std::find(detail.route.begin(), detail.route.end(), item->supplier) == detail.route.end())
                        throw std::runtime_error("Spread-Joint final duplicate or wrong-supplier item");
                    height += item->height;
                    if (item->is_optional) route_profit += item->score;
                }
                if (height > inst.vehicle_height)
                    throw std::runtime_error("Spread-Joint final stack height exceeded");
            }
            expected_route_area += static_cast<std::int64_t>(stacks.size()) *
                                   spec.width * spec.length;
        }
        std::int64_t placed_route_area = 0;
        for (std::size_t i = 0; i < detail.placed_rects.size(); ++i) {
            const auto& a = detail.placed_rects[i];
            placed_route_area += static_cast<std::int64_t>(std::llround(a.width * a.length));
            if (a.x < -1e-7 || a.y < -1e-7 ||
                a.x + a.width > inst.vehicle_width + 1e-7 ||
                a.y + a.length > inst.vehicle_length + 1e-7)
                throw std::runtime_error("Spread-Joint final 2D rectangle outside vehicle");
            for (std::size_t j = i + 1; j < detail.placed_rects.size(); ++j) {
                const auto& b = detail.placed_rects[j];
                if (a.x < b.x + b.width - 1e-7 && b.x < a.x + a.width - 1e-7 &&
                    a.y < b.y + b.length - 1e-7 && b.y < a.y + a.length - 1e-7)
                    throw std::runtime_error("Spread-Joint final 2D rectangle overlap");
            }
        }
        if (placed_route_area != expected_route_area)
            throw std::runtime_error("Spread-Joint final 2D/stack total area mismatch");
        if (route_profit != detail.total_optional_priority_score)
            throw std::runtime_error("Spread-Joint final route physical profit mismatch");
        physical_profit += route_profit;
    }
    for (const auto& item : inst.items)
        if (!item.is_optional && !globally_loaded.count(item.item_id))
            throw std::runtime_error("Spread-Joint final mandatory item missing");
    if (physical_profit != solution.optional_profit ||
        physical_profit != solution.packing.total_optional_priority_score)
        throw std::runtime_error("Spread-Joint final physical profit mismatch");
}

void apply_route_eval_to_detail(RoutePackingDetail& detail,
                                const RouteEvalFullOutCpp& er,
                                long long upper_bound) {
    detail.feasible = er.scalar.feasible;
    detail.bin_number = er.scalar.bin_number;
    detail.optional_score_from_2d = er.scalar.optional_score_from_2d;
    detail.fixed_inserted_score = er.scalar.fixed_inserted_score;
    detail.total_optional_priority_score = er.scalar.total_optional_priority_score;
    detail.profit_upper_bound = upper_bound;
    detail.static_lb_infeasible = er.scalar.static_lb_infeasible;
    detail.error_message = er.scalar.static_lb_reason;
    detail.loaded_mandatory_count_by_spec = er.scalar.loaded_mandatory_count_by_type;
    detail.loaded_optional_count_by_spec = er.scalar.loaded_optional_count_by_type;
    detail.placed_rects.clear();
    detail.placed_rects.reserve(er.scalar.placed_rects.size());
    for (const auto& pr : er.scalar.placed_rects) {
        RoutePackingPlacedRect out_pr;
        out_pr.spec_index = pr.type_index;
        out_pr.is_optional = pr.is_optional;
        out_pr.rank_in_type = pr.rank_in_type;
        out_pr.x = pr.x;
        out_pr.y = pr.y;
        out_pr.width = pr.width;
        out_pr.length = pr.length;
        detail.placed_rects.push_back(out_pr);
    }
}

RoutePackingDetail cached_detail_for_route(
    const Route& route,
    const std::vector<PackingTypeSpecCpp>& specs,
    const FeasibleRouteCacheEntry& entry) {
    RoutePackingDetail detail;
    if (entry.has_layout) {
        detail = entry.best_detail;
    } else {
        detail.feasible = true;
        detail.total_optional_priority_score = entry.best_score;
        detail.profit_upper_bound = entry.upper_bound;
        detail.specs = specs;
    }
    detail.route = route;
    detail.feasible = true;
    detail.profit_upper_bound = entry.upper_bound;
    return detail;
}

PackingPlanEval evaluate_route_plan_packing_with_cache(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const RoutePlan& plan,
    int mutation_iters,
    double mutation_swap_ratio,
    std::uint64_t& rng_state,
    int deep_retry_iters,
    const RoutePackingCacheConfig& cache_cfg,
    RoutePackingCachePools* cache_pools,
    PackingEvaluationRuntimeState* packing_runtime,
    const std::function<void()>& stop_callback) {
    PackingPlanEval out;
    for (const auto& route : plan) {
        if (stop_callback) stop_callback();
        if (route.empty()) continue;
        RoutePackingDetail detail;
        detail.route = route;
        const std::uint64_t route_seed = splitmix64(rng_state);
        const bool cache_enabled = cache_cfg.enabled && cache_pools != nullptr;
        const std::string key = make_route_cache_key(route);

        detail.specs = build_route_packing_specs(inst, prep, route);
        detail.profit_upper_bound = compute_route_optional_profit_upper_bound(detail.specs);

        FeasibleRouteCacheEntry* feasible = nullptr;
        bool has_feasible_cache = false;
        if (cache_enabled && cache_cfg.enable_feasible_pool) {
            feasible = cache_pools->feasible_route_pool.get_mutable(key);
            has_feasible_cache = feasible != nullptr;

        }

        if (has_feasible_cache &&
            cache_cfg.skip_skyline_if_cached_reaches_upper_bound &&
            feasible->best_score >= detail.profit_upper_bound) {

            detail = cached_detail_for_route(route, detail.specs, *feasible);
            out.total_optional_priority_score += detail.total_optional_priority_score;
            out.route_details.push_back(std::move(detail));
            continue;
        }

        if (has_feasible_cache && !cache_cfg.rerun_skyline_on_feasible_hit) {
            detail = cached_detail_for_route(route, detail.specs, *feasible);
            out.total_optional_priority_score += detail.total_optional_priority_score;
            out.route_details.push_back(std::move(detail));
            continue;
        }

        SkylineEvalOptions options;
        if (has_feasible_cache && cache_cfg.skip_static_pruning_on_feasible_hit) {
            options.run_static_precheck = false;

        }

        std::string seed = std::to_string(route_seed);
        PyRandomRng rng(seed);
        RouteEvalFullOutCpp er;
        try {

            er = evaluate_single_route_priority_packing_cpp_api_full(
                detail.specs,
                inst.vehicle_width,
                inst.vehicle_length,
                mutation_iters,
                mutation_swap_ratio,
                true,
                rng,
                stop_callback,
                options);
            apply_route_eval_to_detail(detail, er, detail.profit_upper_bound);
        } catch (const SearchBudgetExceeded&) {
            throw;
        } catch (const std::exception& e) {

            detail.feasible = false;
            detail.bin_number = 0;
            detail.optional_score_from_2d = 0;
            detail.fixed_inserted_score = 0;
            detail.total_optional_priority_score = 0;
            detail.error_message = e.what();
        }

        if (!detail.feasible && detail.static_lb_infeasible && !has_feasible_cache
            && packing_runtime != nullptr && packing_runtime->on_static_certified_infeasible) {
            packing_runtime->on_static_certified_infeasible(route);
        }
        if (!detail.feasible && has_feasible_cache) {

            detail = cached_detail_for_route(route, detail.specs, *feasible);
            out.total_optional_priority_score += detail.total_optional_priority_score;
            out.route_details.push_back(std::move(detail));
            continue;
        }
        if (!detail.feasible && !detail.static_lb_infeasible && deep_retry_iters >= 0) {

            SkylineEvalOptions deep_options;
            deep_options.run_static_precheck = false;
            std::uint64_t deep_seed_state = deterministic_packing_seed(
                route_seed, key + "|deep_skyline");
            PyRandomRng deep_rng(std::to_string(deep_seed_state));
            RoutePackingDetail deep_detail;
            deep_detail.route = route;
            deep_detail.specs = detail.specs;
            deep_detail.profit_upper_bound = detail.profit_upper_bound;
            try {
                auto deep_er = evaluate_single_route_priority_packing_cpp_api_full(
                    deep_detail.specs,
                    inst.vehicle_width,
                    inst.vehicle_length,
                    deep_retry_iters,
                    mutation_swap_ratio,
                    true,
                    deep_rng,
                    stop_callback,
                    deep_options);
                apply_route_eval_to_detail(
                    deep_detail, deep_er, deep_detail.profit_upper_bound);
            } catch (const SearchBudgetExceeded&) {
                throw;
            } catch (const std::exception& error) {
                deep_detail.feasible = false;
                deep_detail.error_message = error.what();
            }
            detail = std::move(deep_detail);
        }

        if (detail.feasible && cache_enabled && cache_cfg.enable_feasible_pool) {
            bool should_update = !has_feasible_cache || detail.total_optional_priority_score > feasible->best_score;
            if (should_update) {
                FeasibleRouteCacheEntry entry;
                if (has_feasible_cache) entry = *feasible;
                entry.best_score = detail.total_optional_priority_score;
                entry.upper_bound = detail.profit_upper_bound;

                if (cache_cfg.store_layout) {
                    entry.has_layout = true;
                    entry.best_detail = detail;
                    entry.best_detail.route = route;
                }
                cache_pools->feasible_route_pool.put(key, entry);

            } else if (has_feasible_cache && cache_cfg.use_cached_best_if_current_worse) {
                detail = cached_detail_for_route(route, detail.specs, *feasible);
            }
        }

        if (!detail.feasible) out.feasible = false;
        out.total_optional_priority_score += detail.total_optional_priority_score;
        out.route_details.push_back(std::move(detail));
        if (!out.feasible) break;
    }
    return out;
}

PackingPlanEval evaluate_route_plan_mandatory_packing_with_cache(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    const RoutePlan& plan,
    int mutation_iters,
    double mutation_swap_ratio,
    std::uint64_t& rng_state,
    int deep_retry_iters,
    const RoutePackingCacheConfig& cache_cfg,
    RoutePackingCachePools* cache_pools,
    PackingEvaluationRuntimeState* packing_runtime,
    const std::function<void()>& stop_callback) {
    PackingPlanEval out;
    for (const auto& route : plan) {
        if (stop_callback) stop_callback();
        if (route.empty()) continue;
        RoutePackingDetail detail;
        detail.route = route;
        const std::uint64_t route_seed = splitmix64(rng_state);
        const bool cache_enabled = cache_cfg.enabled && cache_pools != nullptr;
        const std::string key = make_route_cache_key(route);

        if (cache_enabled) {
            MandatoryFeasibleRouteCacheEntry* mandatory_feasible =
                cache_pools->mandatory_feasible_route_pool.get_mutable(key);
            if (mandatory_feasible != nullptr) {

                detail.feasible = true;
                detail.total_optional_priority_score = 0;
                detail.profit_upper_bound = 0;
                out.route_details.push_back(std::move(detail));
                continue;
            }
        }

        detail.specs = build_route_mandatory_packing_specs(inst, prep, route);
        detail.profit_upper_bound = 0;

        std::string seed = std::to_string(route_seed);
        PyRandomRng rng(seed);
        RouteEvalFullOutCpp er;
        try {

            er = evaluate_single_route_mandatory_packing_cpp_api_full(
                detail.specs,
                inst.vehicle_width,
                inst.vehicle_length,
                mutation_iters,
                mutation_swap_ratio,
                rng,
                stop_callback);
            apply_route_eval_to_detail(detail, er, 0);
            detail.optional_score_from_2d = 0;
            detail.fixed_inserted_score = 0;
            detail.total_optional_priority_score = 0;
            detail.profit_upper_bound = 0;
        } catch (const SearchBudgetExceeded&) {
            throw;
        } catch (const std::exception& e) {
            detail.feasible = false;
            detail.bin_number = 0;
            detail.optional_score_from_2d = 0;
            detail.fixed_inserted_score = 0;
            detail.total_optional_priority_score = 0;
            detail.profit_upper_bound = 0;
            detail.error_message = e.what();
        }

        if (!detail.feasible && detail.static_lb_infeasible
            && packing_runtime != nullptr && packing_runtime->on_static_certified_infeasible) {
            packing_runtime->on_static_certified_infeasible(route);
        }
        if (!detail.feasible && !detail.static_lb_infeasible) {

            SkylineEvalOptions deep_options;
            deep_options.run_static_precheck = false;
            std::uint64_t deep_seed_state = deterministic_packing_seed(
                route_seed, key + "|deep_primary_skyline");
            PyRandomRng deep_rng(std::to_string(deep_seed_state));
            RoutePackingDetail deep_detail;
            deep_detail.route = route;
            deep_detail.specs = detail.specs;
            deep_detail.profit_upper_bound = 0;
            try {
                const auto deep_er = evaluate_single_route_mandatory_packing_cpp_api_full(
                    deep_detail.specs,
                    inst.vehicle_width,
                    inst.vehicle_length,
                    deep_retry_iters,
                    mutation_swap_ratio,
                    deep_rng,
                    stop_callback,
                    deep_options);
                apply_route_eval_to_detail(deep_detail, deep_er, 0);
            } catch (const SearchBudgetExceeded&) {
                throw;
            } catch (const std::exception& error) {
                deep_detail.feasible = false;
                deep_detail.error_message = error.what();
            }
            detail = std::move(deep_detail);
        }

        if (detail.feasible && cache_enabled) {
            MandatoryFeasibleRouteCacheEntry entry;

            cache_pools->mandatory_feasible_route_pool.put(key, entry);
        }

        if (!detail.feasible) out.feasible = false;
        out.route_details.push_back(std::move(detail));
        if (!out.feasible) break;
    }
    out.total_optional_priority_score = 0;
    return out;
}

#ifdef HRLH_COMPARISON_ENGINE
struct FrozenCommonStart {
    hrlh_model::ProblemInstance instance;
    HrlhConfig config;
    hrlh_components::stack_preprocessing::PreprocessAllGroupsResult prep;
    RoutePackingCachePools route_cache;
    BoundedSpecsCache bounded_specs;
    SearchControlState search_state;
    std::uint64_t rng;
    RoutePlan current;
    std::vector<CandidateSolution> pool;
    std::vector<RoutePlan> initial_candidates;
    std::set<std::string> processed_route_repairs;
    CertifiedNoGoodPool nogoods;
    std::optional<RoutePlan> warm_start;
    bool warm_mandatory_feasible = false;
    double elapsed;
    FrozenCommonStart(const hrlh_model::ProblemInstance& inst, const HrlhConfig& cfg)
        : instance(inst), config(cfg), route_cache(cfg.route_packing_cache), rng(cfg.seed), elapsed(0) {}
};

static HrlhResult solve_controlled(
    const hrlh_model::ProblemInstance& source_inst, const HrlhConfig& cfg,
    const FrozenCommonStart* frozen, const BackendRunControl* control,
    std::shared_ptr<const FrozenCommonStart>* capture) {
#else
HrlhResult solve_hrlh(const hrlh_model::ProblemInstance& source_inst,
                                           const HrlhConfig& cfg) {
#endif
    validate_hrlh_config(cfg);
    const auto inst = hrlh_model::normalized_rotatable_rack_dimensions(source_inst);
    const auto solve_start = std::chrono::steady_clock::now();
    HrlhResult result;
    SearchControlState& search_state = result.search_state;
    const bool profit_intensification_enabled = cfg.enable_optional_demand_profit_intensification;

    const bool has_global_budget = cfg.max_wall_seconds > 0.0;
    const double final_optional_profit_reserve = has_global_budget && profit_intensification_enabled
        ? cfg.max_wall_seconds * cfg.optional_profit_intensification_budget_fraction : 0.0;
    const double main_search_budget = has_global_budget
        ? cfg.max_wall_seconds - final_optional_profit_reserve : 0.0;

    auto elapsed_seconds = [&]() {
#ifdef HRLH_COMPARISON_ENGINE
        return (frozen ? frozen->elapsed : 0.0) + std::chrono::duration<double>(
#else
        return std::chrono::duration<double>(
#endif
            std::chrono::steady_clock::now() - solve_start).count();
    };
    std::optional<double> active_phase_deadline;
    auto time_budget_exceeded = [&]() {
        return has_global_budget && elapsed_seconds() >= main_search_budget;
    };
    auto require_time_budget = [&]() {
        if (time_budget_exceeded()) {
            search_state.time_budget_exceeded = true;
            throw SearchBudgetExceeded(BudgetKind::Time, "HRLH solve time budget exceeded");
        }
        if (active_phase_deadline && elapsed_seconds() >= *active_phase_deadline) {
            throw SearchBudgetExceeded(
                BudgetKind::FeasibleDescentTime,
                "integrated-feasible transport-cost descent phase budget exceeded");
        }
    };

    hrlh_model::validate_instance(inst);
#ifdef HRLH_COMPARISON_ENGINE
    if (!frozen && has_global_budget && main_search_budget <= 0.0) {
#else
    if (has_global_budget && main_search_budget <= 0.0) {
#endif
        search_state.time_budget_exceeded = true;

        return result;
    }
#ifdef HRLH_COMPARISON_ENGINE

    if (!frozen) require_time_budget();
#else
    require_time_budget();
#endif
    auto prep_inst = to_rack_preprocess_instance(inst);
    auto preprocess_cfg = cfg.preprocess_config;
    preprocess_cfg.require_optimal = preprocess_cfg.require_optimal && cfg.require_preprocess_optimal;
    if (cfg.verbose && !preprocess_cfg.verbose) preprocess_cfg.verbose = true;

    const auto preprocess_start = std::chrono::steady_clock::now();
    const double local_preprocess_budget = preprocess_cfg.max_wall_seconds;
    const bool has_local_preprocess_budget = local_preprocess_budget > 0.0;
    if (has_global_budget || has_local_preprocess_budget) {

        auto remaining_preprocess_budget = [&]() {
            double remaining = std::numeric_limits<double>::infinity();
            if (has_global_budget) {
                remaining = std::min(remaining, main_search_budget - elapsed_seconds());
            }
            if (has_local_preprocess_budget) {
                double local_elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - preprocess_start).count();
                remaining = std::min(remaining, local_preprocess_budget - local_elapsed);
            }
            return remaining;
        };
        preprocess_cfg.interrupt_check = [&, remaining_preprocess_budget]() {
            if (remaining_preprocess_budget() <= 0.0) {
                search_state.time_budget_exceeded = true;
                throw SearchBudgetExceeded(BudgetKind::Time,
                    "HRLH preprocessing time budget exceeded");
            }
        };
        preprocess_cfg.remaining_seconds = remaining_preprocess_budget;
        if (preprocess_cfg.max_wall_seconds <= 0.0 && has_global_budget) {
            preprocess_cfg.max_wall_seconds = main_search_budget;
        }
    }

    hrlh_components::stack_preprocessing::PreprocessAllGroupsResult prep;
#ifdef HRLH_COMPARISON_ENGINE
    if (frozen) {
        prep = frozen->prep;
        bounded_specs_cache_state() = frozen->bounded_specs;
        search_state = frozen->search_state;
    } else {
#endif
    try {
#ifdef HRLH_STACKS_WITH_GUROBI
    if (cfg.use_gurobi_preprocess) {
        prep = hrlh_components::stack_preprocessing::preprocess_stacks(prep_inst, cfg.strict_same_canonical_size, preprocess_cfg);
    } else {
        if (cfg.verbose) std::cerr << "Using fallback rack preprocessing because Gurobi preprocessing was not requested.\n";
        prep = hrlh_components::stack_preprocessing::preprocess_stacks_fallback(prep_inst, cfg.strict_same_canonical_size, preprocess_cfg);
    }
#else
    if (cfg.use_gurobi_preprocess) {
        throw std::runtime_error("Gurobi preprocessing was requested but is unavailable in this build. Reconfigure with -DHRLH_WITH_GUROBI=ON and GUROBI_HOME set, or set preprocess.use_gurobi=false in the INI file.");
    }
    if (cfg.verbose) std::cerr << "Using fallback rack preprocessing; this build has HRLH_WITH_GUROBI=OFF.\n";
    prep = hrlh_components::stack_preprocessing::preprocess_stacks_fallback(prep_inst, cfg.strict_same_canonical_size, preprocess_cfg);
#endif
    } catch (const SearchBudgetExceeded& e) {
        mark_budget_exceeded(search_state, e.kind);

        return result;
    }
#ifdef HRLH_COMPARISON_ENGINE
    }
#endif
    result.preprocess = prep;
#ifdef HRLH_COMPARISON_ENGINE
    if (!frozen && time_budget_exceeded()) {
#else
    if (time_budget_exceeded()) {
#endif
        search_state.time_budget_exceeded = true;

        return result;
    }
    hrlh_components::stack_preprocessing::validate_monotone_value_tables(prep);
#ifdef HRLH_COMPARISON_ENGINE
    if (!frozen) require_time_budget();
    else if (time_budget_exceeded()) search_state.time_budget_exceeded = true;
#else
    require_time_budget();
#endif

    std::uint64_t rng = cfg.seed;
    auto route_mandatory_floor_area = mandatory_floor_area_by_supplier_from_preprocess(inst, prep);
    const int vehicle_lower_bound =
        compute_area_vehicle_lower_bound(inst, route_mandatory_floor_area);

    RoutePackingCachePools route_cache_pools(cfg.route_packing_cache);
    PackingEvaluationRuntimeState packing_runtime;

    auto packing_cache_cfg = cfg.route_packing_cache;
    if (profit_intensification_enabled) packing_cache_cfg.store_layout = true;

    RoutePlan current;
    std::vector<CandidateSolution> pool;
#ifdef HRLH_COMPARISON_ENGINE
    std::vector<RoutePlan> initial_candidates;
#endif

    auto route_plan_latest_departure_sum = [&](const RoutePlan& plan) {
        double sum = 0.0;
        for (const auto& r : plan) sum += route_latest_departure(inst, r);
        return sum;
    };

    auto require_packing_call_budget = [&]() {
        if (cfg.max_packing_calls > 0 && search_state.packing_calls >= cfg.max_packing_calls) {
            search_state.packing_call_budget_exceeded = true;
            throw SearchBudgetExceeded(BudgetKind::PackingCalls, "HRLH solve packing-call budget exceeded");
        }
    };

    enum class CandidateOrigin {
        FeasibleSeed, Initial, RoutingSearch, FeasibleDescent, PackingFeedback
    };
    enum class IntegratedAssessment {
        Feasible, ProvenPackingInfeasible, HeuristicPackingFailure, NotEvaluated
    };

    std::set<std::string> route_repair_processed;
    CertifiedNoGoodPool proven_nogoods;
#ifdef HRLH_COMPARISON_ENGINE

    if (frozen) {
        rng = frozen->rng;
        route_cache_pools = frozen->route_cache;
        current = frozen->current;
        pool = frozen->pool;
        route_repair_processed = frozen->processed_route_repairs;
        proven_nogoods = frozen->nogoods;
        initial_candidates = frozen->initial_candidates;
    }
#endif

    auto matching_nogood = [&](const RoutePlan& plan) -> const Route* {
        if (!cfg.enable_proven_infeasible_subset_nogoods) return nullptr;
        for (const auto& route : plan) {
            if (const Route* nogood = proven_nogoods.matching_route(route)) {
                return nogood;
            }
        }
        return nullptr;
    };
    auto matching_nogood_after_addition = [&](const Route& route,
                                               int incoming_supplier) -> const Route* {
        if (!cfg.enable_proven_infeasible_subset_nogoods) return nullptr;
        return proven_nogoods.matching_after_addition(
            route, incoming_supplier);
    };
    auto route_has_static_proof = [&](const Route& route) {
        require_time_budget();

        const auto specs = build_route_mandatory_packing_specs(inst, prep, route);
        return mandatory_static_lb_check_cpp(
            specs, inst.vehicle_width, inst.vehicle_length).proven_infeasible;
    };
    auto learn_minimal_nogood = [&](const Route& failed_route) {
        if (!cfg.enable_proven_infeasible_subset_nogoods || failed_route.empty()) return;
        learn_certified_nogood_with_static_shrinking(
            proven_nogoods,
            failed_route,
            route_has_static_proof);
    };
    packing_runtime.on_static_certified_infeasible = learn_minimal_nogood;

    std::function<IntegratedAssessment(
        const RoutePlan&, CandidateOrigin, bool)> evaluate_candidate;
    evaluate_candidate = [&](const RoutePlan& plan, CandidateOrigin origin,
                             bool allow_derived) {
        require_time_budget();

        if (!path_basic_feasible(inst, plan, cfg, &route_mandatory_floor_area)) {
            return IntegratedAssessment::NotEvaluated;
        }

        if (const Route* nogood = matching_nogood(plan)) {
            (void)nogood;

            return IntegratedAssessment::ProvenPackingInfeasible;
        }
        double cost = compute_transport_cost(inst, plan);
        const bool has_best_cost = !pool.empty();
        const double best_cost = (pool.empty() ? 0.0 : pool.front().transport_cost);
        if (has_best_cost && cost > best_cost) {

            return IntegratedAssessment::NotEvaluated;
        }

        require_packing_call_budget();
        search_state.packing_calls++;
        const int candidate_packing_iters = cfg.packing_mutation_iters;

        auto pack = evaluate_route_plan_packing_with_cache(inst, prep, plan,
            candidate_packing_iters, cfg.packing_mutation_swap_ratio, rng,
            cfg.deep_packing_retry_iters,
            packing_cache_cfg, &route_cache_pools, &packing_runtime,
            [&]() { require_time_budget(); });
        require_time_budget();
        if (!pack.feasible) {
            const bool has_certified_proof = packing_has_certified_proof(pack);

            if (cfg.enable_packing_guided_route_repair) {

                const std::string failed_signature = route_plan_signature(plan);
                if (allow_derived && route_repair_processed.insert(failed_signature).second) {
                    std::uint64_t local_route_repair_rng = isolated_route_repair_seed(
                        cfg.seed, failed_signature);
                    std::uint64_t& route_repair_rng = cfg.isolate_route_repair_rng
                        ? local_route_repair_rng : rng;
                    auto current_route_repair_incumbent = [&]()
                        -> std::optional<double> {

                        if (pool.empty()) return std::nullopt;
                        return pool.front().transport_cost;
                    };
                    auto assess_route_repair_cost_gate = [&](const RoutePlan& repair) {
                        RouteRepairCostGateSnapshot snapshot;
                        if (repair.size() > inst.vehicles.size()) {

                            return snapshot;
                        }
                        snapshot.candidate_cost = compute_transport_cost(inst, repair);
                        const auto incumbent = current_route_repair_incumbent();
                        if (!incumbent) {
                            snapshot.passes = true;
                            return snapshot;
                        }
                        snapshot.incumbent_cost = *incumbent;

                        if (cfg.enforce_route_repair_cost_gate
                            && !route_repair_cost_gate_allows(
                                snapshot.candidate_cost, snapshot.incumbent_cost)) {

                            return snapshot;
                        }
                        snapshot.passes = true;

                        return snapshot;
                    };

                    auto route_repair_target_has_nogood = [&](const Route& target_before,
                                                           int incoming_supplier) {
                        if (matching_nogood_after_addition(
                                target_before, incoming_supplier) == nullptr) return false;

                        return true;
                    };
                    auto evaluate_repair_mandatory_route = [&](const Route& route) {
                        require_packing_call_budget();
                        search_state.packing_calls++;
                        auto mandatory = evaluate_route_plan_mandatory_packing_with_cache(
                            inst, prep, RoutePlan{route},
                            cfg.packing_mutation_iters,
                            cfg.packing_mutation_swap_ratio,
                            route_repair_rng,
                            cfg.deep_packing_retry_iters,
                            packing_cache_cfg,
                            &route_cache_pools,
                            &packing_runtime,
                            [&]() { require_time_budget(); });
                        if (mandatory.route_details.empty()) {
                            throw std::logic_error(
                                "ordinary feedback mandatory route evaluation returned no detail");
                        }
                        return mandatory.route_details.front();
                    };
                    auto ordinary = run_cumulative_packing_guided_route_repair(
                        inst, plan, pack, route_mandatory_floor_area,
                        route_repair_target_has_nogood,
                        evaluate_repair_mandatory_route,
                        assess_route_repair_cost_gate,
                        route_repair_rng);

                    if (ordinary.fully_repaired) {
                        evaluate_candidate(
                            ordinary.plan,
                            CandidateOrigin::PackingFeedback,
                            false);
                    }

                }
            }
            return has_certified_proof
                ? IntegratedAssessment::ProvenPackingInfeasible
                : IntegratedAssessment::HeuristicPackingFailure;
        }

        CandidateSolution sol;
        sol.routes = plan;
        sol.transport_cost = cost;
        sol.optional_profit = pack.total_optional_priority_score;
        sol.packing = std::move(pack);
        if (origin == CandidateOrigin::FeasibleDescent
            && (pool.empty() || cost < pool.front().transport_cost)) {
            pool.clear();
            pool.push_back(std::move(sol));

        } else {
            maybe_update_pool(pool, std::move(sol));
        }
        return IntegratedAssessment::Feasible;
    };

    IntegratedAssessment lower_bound_initial_assessment = IntegratedAssessment::NotEvaluated;
#ifdef HRLH_COMPARISON_ENGINE
    if (!frozen) {
#endif
    try {
        auto initial_addition_nogood = [&](const Route& target_before,
                                            int incoming_supplier) {
            if (matching_nogood_after_addition(
                    target_before, incoming_supplier) == nullptr) {
                return false;
            }

            return true;
        };
        auto initial_static_nogood = [&](const Route& route) {
            learn_minimal_nogood(route);
        };
        const RoutePlan singleton_initial = initial_one_supplier_routes(inst);
#ifdef HRLH_COMPARISON_ENGINE
        initial_candidates.push_back(singleton_initial);
#endif
        const auto singleton_assessment = evaluate_candidate(
            singleton_initial, CandidateOrigin::FeasibleSeed, true);
        search_state.feasible_seed_found = singleton_assessment == IntegratedAssessment::Feasible;

        std::set<std::string> initial_signatures;
        const auto singleton_signature = route_plan_signature(singleton_initial);
        const int max_routes = std::max(1, static_cast<int>(inst.vehicles.size()));
        for (int k = std::max(1, vehicle_lower_bound); k <= max_routes; ++k) {
            require_time_budget();
            RoutePlan candidate = lower_bound_greedy_plan(
                inst, prep, route_mandatory_floor_area, k, require_time_budget,
                initial_addition_nogood, initial_static_nogood);
            if (!covers_each_supplier_once(inst, candidate)) continue;
            const auto signature = route_plan_signature(candidate);
            if (!initial_signatures.insert(signature).second) continue;
#ifdef HRLH_COMPARISON_ENGINE
            initial_candidates.push_back(candidate);
#endif
            const auto assessment = signature == singleton_signature
                ? singleton_assessment
                : evaluate_candidate(candidate, CandidateOrigin::Initial, false);

            if (current.empty()
                || lower_bound_initial_assessment != IntegratedAssessment::HeuristicPackingFailure
                || (assessment == IntegratedAssessment::HeuristicPackingFailure
                    && candidate.size() < current.size())) {
                current = std::move(candidate);
                lower_bound_initial_assessment = assessment;
            }
            if (assessment == IntegratedAssessment::Feasible) break;
        }

        const int diversification_starts = cfg.initial_diversification_starts;

        for (int start = 0; start < diversification_starts; ++start) {
            require_time_budget();

            const int first_k = std::min(max_routes,
                std::max(1, vehicle_lower_bound));
            RoutePlan candidate;
            for (int k = first_k; k <= max_routes && candidate.empty(); ++k) {
                candidate = lower_bound_greedy_plan(
                    inst, prep, route_mandatory_floor_area, k,
                    require_time_budget, initial_addition_nogood,
                    initial_static_nogood, &rng);
            }
            if (!covers_each_supplier_once(inst, candidate)) continue;
            const auto signature = route_plan_signature(candidate);
            if (!initial_signatures.insert(signature).second) continue;
#ifdef HRLH_COMPARISON_ENGINE
            initial_candidates.push_back(candidate);
#endif
            (void)evaluate_candidate(
                candidate, CandidateOrigin::Initial, false);
        }
    } catch (const SearchBudgetExceeded& e) {
        mark_budget_exceeded(search_state, e.kind);
        if (cfg.verbose) std::cerr << e.what() << " during initial candidate evaluation.\n";
    }
#ifdef HRLH_COMPARISON_ENGINE
    }
#endif

    std::optional<RoutePlan> first_layer_initial_plan;
    bool first_layer_initial_mandatory_feasible = false;
#ifdef HRLH_COMPARISON_ENGINE
    if (frozen) {
        first_layer_initial_plan = frozen->warm_start;
        first_layer_initial_mandatory_feasible = frozen->warm_mandatory_feasible;
    } else {
#endif
    if (!pool.empty()) {
        auto best = pool.begin();
        double best_latest = route_plan_latest_departure_sum(best->routes);
        for (auto it = std::next(pool.begin()); it != pool.end(); ++it) {
            const double latest = route_plan_latest_departure_sum(it->routes);
            if (latest > best_latest + 1e-9 ||
                (std::abs(latest - best_latest) <= 1e-9 && it->signature < best->signature)) {
                best = it;
                best_latest = latest;
            }
        }
        first_layer_initial_plan = best->routes;
        first_layer_initial_mandatory_feasible = true;
    }
    if (lower_bound_initial_assessment == IntegratedAssessment::HeuristicPackingFailure
        && path_basic_feasible(inst, current, cfg, &route_mandatory_floor_area)
        && !first_layer_initial_plan) {
        bool current_has_static_proof = false;
        for (const auto& route : current) {
            const auto specs = build_route_mandatory_packing_specs(inst, prep, route);
            if (mandatory_static_lb_check_cpp(
                    specs, inst.vehicle_width, inst.vehicle_length).proven_infeasible) {
                current_has_static_proof = true;
                break;
            }
        }
        if (!current_has_static_proof) {
            first_layer_initial_plan = current;

        }
    }
#ifdef HRLH_COMPARISON_ENGINE
    }
#endif

#ifdef HRLH_COMPARISON_ENGINE
    if (!frozen && first_layer_initial_plan && first_layer_initial_mandatory_feasible
#else
    if (first_layer_initial_plan && first_layer_initial_mandatory_feasible
#endif
        && search_state.feasible_seed_found
        && cfg.feasible_descent_budget_fraction > 0.0) {
        RoutePlan descent_anchor = *first_layer_initial_plan;
        double descent_anchor_cost = compute_transport_cost(inst, descent_anchor);


        const double descent_start = elapsed_seconds();

        if (has_global_budget) {
            const double remaining_main = std::max(0.0, main_search_budget - descent_start);
            search_state.feasible_descent_budget_seconds
                = remaining_main * cfg.feasible_descent_budget_fraction;
            active_phase_deadline = descent_start
                + search_state.feasible_descent_budget_seconds;
        }
        const long long no_wall_attempt_limit = has_global_budget
            ? std::numeric_limits<long long>::max()
            : static_cast<long long>(std::max(0, cfg.max_iterations));
        long long no_improvement_streak = 0;
        auto note_descent_no_improvement = [&]() {
            ++no_improvement_streak;

        };
        RoutePackingCacheConfig descent_no_cache;
        descent_no_cache.enabled = false;
        auto evaluate_descent_mandatory = [&](const RoutePlan& plan) -> bool {
            require_time_budget();

            if (!path_basic_feasible(
                    inst, plan, cfg, &route_mandatory_floor_area)) {
                return false;
            }

            if (matching_nogood(plan) != nullptr) {

                return false;
            }

            const auto checks = descent_packing_checks(plan, descent_anchor);

            if (checks.routes_to_check.empty()) {

                return true;
            }

            require_packing_call_budget();
            search_state.packing_calls++;

            auto mandatory_pack = evaluate_route_plan_mandatory_packing_with_cache(
                    inst, prep, checks.routes_to_check, cfg.packing_mutation_iters,
                    cfg.packing_mutation_swap_ratio, rng,
                    cfg.deep_packing_retry_iters, descent_no_cache, nullptr, &packing_runtime, [&]() { require_time_budget(); });

            require_time_budget();
            if (!mandatory_pack.feasible) {
                return false;
            }

            return true;
        };
        while (search_state.feasible_descent_attempts < no_wall_attempt_limit
               && !search_state.time_budget_exceeded
               && !search_state.packing_call_budget_exceeded) {
            if (cfg.feasible_descent_no_improvement_limit > 0
                && no_improvement_streak
                    >= cfg.feasible_descent_no_improvement_limit) {

                break;
            }
            try {
                require_time_budget();
                ++search_state.feasible_descent_attempts;
                bool nogood_rejected = false;
                RoutePlan candidate = mutate_plan(
                    descent_anchor, inst, rng,
                    [&](const Route& target_before, int incoming_supplier) {
                        if (matching_nogood_after_addition(
                                target_before, incoming_supplier) == nullptr) {
                            return false;
                        }

                        return true;
                    },
                    &nogood_rejected);
                if (nogood_rejected) {

                    note_descent_no_improvement();
                    continue;
                }
                const double candidate_cost = compute_transport_cost(inst, candidate);
                if (!(candidate_cost < descent_anchor_cost)) {

                    note_descent_no_improvement();
                    continue;
                }
                if (evaluate_descent_mandatory(candidate)) {
                    no_improvement_streak = 0;
                    ++search_state.feasible_descent_acceptances;
                    descent_anchor = std::move(candidate);
                    descent_anchor_cost = candidate_cost;
                } else {
                    note_descent_no_improvement();
                }
            } catch (const SearchBudgetExceeded& e) {
                if (e.kind != BudgetKind::FeasibleDescentTime) {
                    mark_budget_exceeded(search_state, e.kind);
                }
                break;
            }
        }

        active_phase_deadline.reset();
        if (search_state.feasible_descent_acceptances > 0) {
            IntegratedAssessment final_assessment
                = IntegratedAssessment::NotEvaluated;

            try {
                final_assessment = evaluate_candidate(
                    descent_anchor, CandidateOrigin::FeasibleDescent, false);
            } catch (const SearchBudgetExceeded& e) {
                if (e.kind == BudgetKind::FeasibleDescentTime) {
                    throw std::logic_error(
                        "descent-local deadline remained active during final complete evaluation");
                } else {
                    mark_budget_exceeded(search_state, e.kind);
                }
            }
            if (final_assessment
                == IntegratedAssessment::ProvenPackingInfeasible) {
                throw std::logic_error(
                    "final feasible-descent evaluation contradicted known mandatory feasibility");
            }

        }


        first_layer_initial_plan = std::move(descent_anchor);
    }
    if (first_layer_initial_plan) {
        current = *first_layer_initial_plan;

    }

#ifdef HRLH_COMPARISON_ENGINE
    if (capture) {

        for (const auto& candidate : pool)
            if (std::find(initial_candidates.begin(),initial_candidates.end(),candidate.routes)
                    == initial_candidates.end())
                initial_candidates.push_back(candidate.routes);
        auto state = std::make_shared<FrozenCommonStart>(source_inst,cfg);
        state->prep = prep;
        state->route_cache = route_cache_pools;
        state->bounded_specs = bounded_specs_cache_state();
        state->search_state = search_state;
        state->rng = rng;
        state->current = current;
        state->pool = pool;
        state->initial_candidates = initial_candidates;
        state->processed_route_repairs = route_repair_processed;
        state->nogoods = proven_nogoods;
        state->warm_start = first_layer_initial_plan;
        state->warm_mandatory_feasible = first_layer_initial_mandatory_feasible;
        state->elapsed = elapsed_seconds();
        *capture = std::move(state);
        return result;
    }

#endif
#ifdef HRLH_WITH_ILS_SEARCH
#ifdef HRLH_COMPARISON_ENGINE
    if (!(control && control->search) && cfg.enable_ils_routing_search && first_layer_initial_plan &&
#else
    if (cfg.enable_ils_routing_search && first_layer_initial_plan &&
#endif
        !search_state.time_budget_exceeded && !search_state.packing_call_budget_exceeded) {
        RoutePlan ils_start = *first_layer_initial_plan;
        try {
            auto spec = build_ils_case_spec(inst, cfg, &route_mandatory_floor_area, &prep);

            auto data = hrlh_ils_search::buildProblem(spec);
            hrlh_ils_search::SolverParamsSpec params = spec.solver;
            params.seed = static_cast<std::uint32_t>(isolated_route_repair_seed(
                cfg.seed, route_plan_signature(ils_start)
                    + "|pyvrp_feedback_epoch|0"));
            params.initialRoutes = ils_initial_routes_from_route_plan(ils_start, inst);
            auto cb = [&](const pyvrp::Solution& sol) {
                RoutePlan plan = route_plan_from_ils_solution(sol, inst);
                const auto assessment = evaluate_candidate(
                    plan, CandidateOrigin::RoutingSearch, true);

                if (!cfg.enable_packing_guided_route_repair) return true;
                if (assessment == IntegratedAssessment::ProvenPackingInfeasible) return false;
                return true;
            };
            auto stop_cb = [&]() {
                require_time_budget();
            };

            auto solved = hrlh_ils_search::solveWithFeedback(
                data, params, cb, stop_cb);
            (void)evaluate_candidate(
                route_plan_from_ils_solution(solved.best, inst),
                CandidateOrigin::RoutingSearch, true);
        } catch (const SearchBudgetExceeded& e) {
            mark_budget_exceeded(search_state, e.kind);
            if (cfg.verbose) std::cerr << e.what() << " during PyVRP callback/local-search evaluation.\n";
        }
    }
#else
    if (cfg.enable_ils_routing_search && cfg.verbose) {
        std::cerr << "PyVRP path search disabled at compile time; returning the initialization/descent incumbent.\n";
    }
#endif

#ifdef HRLH_COMPARISON_ENGINE
    if (control && control->search && first_layer_initial_plan
        && !search_state.time_budget_exceeded && !search_state.packing_call_budget_exceeded) {
        routing::Problem routing_problem(inst, prep, cfg,routing::NumericMapping::ILSCompatible);
        routing::SearchCallbacks callbacks;
        callbacks.stop = require_time_budget;
        callbacks.evaluate_integrated = [&](const RoutePlan& plan) {

            const auto assessment = evaluate_candidate(plan,CandidateOrigin::RoutingSearch,true);
            auto mapped = assessment == IntegratedAssessment::Feasible
                ? routing::PackingAssessment::Feasible
                : assessment == IntegratedAssessment::ProvenPackingInfeasible
                    ? routing::PackingAssessment::CertifiedInfeasible
                    : assessment == IntegratedAssessment::HeuristicPackingFailure
                        ? routing::PackingAssessment::HeuristicFailure
                        : routing::PackingAssessment::NotEvaluated;
            if (!cfg.enable_packing_guided_route_repair
                && mapped == routing::PackingAssessment::CertifiedInfeasible)
                mapped = routing::PackingAssessment::NotEvaluated;
            return mapped;
        };
        callbacks.evaluate_partial_route = [&](const Route& route) {
            require_time_budget();
            if (matching_nogood(RoutePlan{route})) {

                return routing::PackingAssessment::CertifiedInfeasible;
            }
            require_packing_call_budget();
            ++search_state.packing_calls;
            const auto mandatory = evaluate_route_plan_mandatory_packing_with_cache(
                inst,prep,{route},cfg.packing_mutation_iters,cfg.packing_mutation_swap_ratio,
                rng,cfg.deep_packing_retry_iters,packing_cache_cfg,&route_cache_pools,&packing_runtime,require_time_budget);
            if (mandatory.feasible) return routing::PackingAssessment::Feasible;
            return packing_has_certified_proof(mandatory)
                ? routing::PackingAssessment::CertifiedInfeasible
                : routing::PackingAssessment::HeuristicFailure;
        };
        try {
            control->search(routing_problem,*first_layer_initial_plan,initial_candidates,
                            callbacks,cfg.seed);
        } catch (const SearchBudgetExceeded& e) {
            mark_budget_exceeded(search_state,e.kind);
        }
    }

#endif
    for (auto& sol : pool) {
        sol.latest_departure_sum = 0.0;
        sol.route_latest_departures.clear();
        for (const auto& r : sol.routes) {
            double ld = route_latest_departure(inst, r);
            sol.route_latest_departures.push_back(ld);
            sol.latest_departure_sum += ld;
        }
    }
    std::sort(pool.begin(), pool.end(), [&](const CandidateSolution& a, const CandidateSolution& b){
        if (a.transport_cost != b.transport_cost) return a.transport_cost < b.transport_cost;
        if (a.optional_profit != b.optional_profit) return a.optional_profit > b.optional_profit;
        return a.latest_departure_sum > b.latest_departure_sum;
    });

    if (!pool.empty()) {
        auto& final_solution = pool.front();


        if (profit_intensification_enabled) {

            const double final_optional_profit_allowance = has_global_budget
                ? final_optional_profit_reserve : 5.0;
            const double available = has_global_budget
                ? std::max(0.0, cfg.max_wall_seconds - elapsed_seconds())
                : final_optional_profit_allowance;
            OptionalDemandProfitContext profit_context;

            intensify_optional_demand_profit(inst, prep, final_solution, cfg,
                profit_context, std::min(available, final_optional_profit_allowance));

        }

        materialize_spread_solution(inst, prep, final_solution);
    }
    if (!pool.empty()) {
        result.has_solution = true;
        result.best_solution = pool.front();

    }


    return result;
}

#ifdef HRLH_COMPARISON_ENGINE
HrlhResult solve_hrlh(const hrlh_model::ProblemInstance& instance,
                                           const HrlhConfig& config) {
    return solve_controlled(instance,config,nullptr,nullptr,nullptr);
#endif
}
#ifdef HRLH_COMPARISON_ENGINE

std::shared_ptr<const FrozenCommonStart> freeze_common_start(
    const hrlh_model::ProblemInstance& instance, const HrlhConfig& config) {
    std::shared_ptr<const FrozenCommonStart> frozen;
    solve_controlled(instance,config,nullptr,nullptr,&frozen);
    if (!frozen) throw std::runtime_error("Initialization exhausted budget before a common start could be frozen");
    return frozen;
}
HrlhResult resume_common_start(const std::shared_ptr<const FrozenCommonStart>& frozen,
                                    const BackendRunControl& control) {
    if (!frozen) throw std::invalid_argument("null common-start snapshot");
    if (control.algorithm != "ILS" && !control.search)
        throw std::invalid_argument("requested comparison backend is not implemented/registered");
    return solve_controlled(frozen->instance,frozen->config,frozen.get(),&control,nullptr);
}

}
#endif
