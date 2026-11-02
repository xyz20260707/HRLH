#include "stack_preprocessing.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <functional>
#include <memory>

#ifdef HRLH_STACKS_WITH_GUROBI
#include "gurobi_c++.h"
#endif

namespace hrlh_components {
namespace stack_preprocessing {

namespace {
void normalize_rack_footprint(RackItem& item)
{
    if (!item.is_pallet && item.length < item.width) std::swap(item.length, item.width);
}
}

bool stack_accepts(const RackItem& item, const RackItem& partner)
{
    return item.compatible_item_ids.count(partner.item_id) != 0;
}

bool stack_compatible(const RackItem& lhs, const RackItem& rhs)
{
    return stack_accepts(lhs, rhs) && stack_accepts(rhs, lhs);
}

bool stack_accepts(const OptionalRack& item, const OptionalRack& partner)
{
    return item.compatible_item_ids.count(partner.item_id) != 0;
}

bool stack_compatible(const OptionalRack& lhs, const OptionalRack& rhs)
{
    return stack_accepts(lhs, rhs) && stack_accepts(rhs, lhs);
}

namespace {

using Clock = std::chrono::steady_clock;

thread_local const SolverConfig* g_active_preprocess_cfg = nullptr;

void check_active_preprocess_budget()
{
    if (g_active_preprocess_cfg && g_active_preprocess_cfg->interrupt_check) {
        g_active_preprocess_cfg->interrupt_check();
    }
}

double remaining_preprocess_seconds()
{
    if (g_active_preprocess_cfg && g_active_preprocess_cfg->remaining_seconds) {
        return g_active_preprocess_cfg->remaining_seconds();
    }
    return std::numeric_limits<double>::infinity();
}

double effective_gurobi_time_limit(double configured_limit)
{
    double limit = configured_limit;
    bool has_limit = configured_limit >= 0.0;
    if (g_active_preprocess_cfg && g_active_preprocess_cfg->gurobi_single_mip_time_limit > 0.0) {
        limit = has_limit ? std::min(limit, g_active_preprocess_cfg->gurobi_single_mip_time_limit)
                          : g_active_preprocess_cfg->gurobi_single_mip_time_limit;
        has_limit = true;
    }
    double remaining = remaining_preprocess_seconds();
    if (std::isfinite(remaining)) {
        limit = has_limit ? std::min(limit, remaining) : remaining;
        has_limit = true;
    }
    if (!has_limit) return configured_limit;
    return std::max(0.001, limit);
}

struct ActivePreprocessConfigScope
{
    const SolverConfig* previous = nullptr;
    explicit ActivePreprocessConfigScope(const SolverConfig& cfg)
        : previous(g_active_preprocess_cfg)
    {
        g_active_preprocess_cfg = &cfg;
    }
    ~ActivePreprocessConfigScope()
    {
        g_active_preprocess_cfg = previous;
    }
};

SolverConfig with_default_budget_callbacks(SolverConfig cfg)
{
    if (cfg.max_wall_seconds > 0.0 && (!cfg.interrupt_check || !cfg.remaining_seconds)) {
        auto start = std::make_shared<Clock::time_point>(Clock::now());
        double max_seconds = cfg.max_wall_seconds;
        if (!cfg.remaining_seconds) {
            cfg.remaining_seconds = [start, max_seconds]() {
                return max_seconds - std::chrono::duration<double>(Clock::now() - *start).count();
            };
        }
        if (!cfg.interrupt_check) {
            cfg.interrupt_check = [start, max_seconds]() {
                if (std::chrono::duration<double>(Clock::now() - *start).count() >= max_seconds) {
                    throw std::runtime_error("rack preprocessing time budget exceeded");
                }
            };
        }
    }
    return cfg;
}

template <class T>
void check_unique_ids(const std::vector<T>& items, const std::string& name)
{
    std::set<int> ids;
    for (const auto& it : items) {
        if (!ids.insert(it.item_id).second) {
            throw std::invalid_argument(name + " contain duplicate item_id values.");
        }
    }
}

template <class T>
void validate_heights(const std::vector<T>& items, int capacity, bool require_fit)
{
    if (capacity <= 0) {
        throw std::invalid_argument("capacity must be positive.");
    }
    for (const auto& it : items) {
        if (it.height <= 0) {
            throw std::invalid_argument("Item " + std::to_string(it.item_id) + " has non-positive height.");
        }
        if (require_fit && it.height > capacity) {
            throw std::invalid_argument("Item " + std::to_string(it.item_id) + " height exceeds stack capacity.");
        }
    }
}

template <class T>
void validate_stack_metadata(const std::vector<T>& items)
{
    std::map<int, const T*> by_id_local;
    for (const auto& item : items) {
        if (item.compatible_item_ids.count(item.item_id)) {
            throw std::invalid_argument("q_mb compatibility must not contain self-pairs.");
        }
        by_id_local[item.item_id] = &item;
    }
    for (const auto& item : items) {
        for (int partner_id : item.compatible_item_ids) {
            const auto it = by_id_local.find(partner_id);
            if (it != by_id_local.end() && !it->second->compatible_item_ids.count(item.item_id)) {
                throw std::invalid_argument("q_mb compatibility must be symmetric for mutual stacking.");
            }
        }
    }
}

template <class T>
bool compatible_with_stack(const T& item,
                           const std::vector<int>& stack,
                           const std::map<int, T>& item_by_id)
{
    for (int id : stack) {
        if (!stack_compatible(item, item_by_id.at(id))) return false;
    }
    return true;
}

template <class T>
bool has_incompatible_pair(const std::vector<T>& items)
{
    for (std::size_t i = 0; i < items.size(); ++i) {
        for (std::size_t j = i + 1; j < items.size(); ++j) {
            if (!stack_compatible(items[i], items[j])) return true;
        }
    }
    return false;
}

template <class T>
std::vector<std::vector<int>> first_fit_decreasing_generic(const std::vector<T>& items, int capacity)
{
    validate_heights(items, capacity, true);
    validate_stack_metadata(items);
    std::vector<int> order(items.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (items[a].height != items[b].height) return items[a].height > items[b].height;
        return items[a].item_id < items[b].item_id;
    });
    std::vector<std::vector<int>> stacks;
    std::vector<int> loads;
    std::map<int, T> item_by_id;
    for (const auto& item : items) item_by_id[item.item_id] = item;
    for (int idx : order) {
        bool placed = false;
        for (size_t k = 0; k < stacks.size(); ++k) {
            if (loads[k] + items[idx].height <= capacity &&
                compatible_with_stack(items[idx], stacks[k], item_by_id)) {
                stacks[k].push_back(items[idx].item_id);
                loads[k] += items[idx].height;
                placed = true;
                break;
            }
        }
        if (!placed) {
            stacks.push_back({items[idx].item_id});
            loads.push_back(items[idx].height);
        }
    }
    for (auto& s : stacks) std::sort(s.begin(), s.end());
    return stacks;
}

template <class T>
std::map<int, T> by_id(const std::vector<T>& items)
{
    std::map<int, T> m;
    for (const auto& it : items) m[it.item_id] = it;
    return m;
}

std::string key_to_string(const GroupKey& k)
{
    return "(" + std::to_string(std::get<0>(k)) + ", " + std::to_string(std::get<1>(k)) + ", " + std::to_string(std::get<2>(k)) + ")";
}

#ifdef HRLH_STACKS_WITH_GUROBI
std::string status_name(int status)
{
    switch (status) {
        case GRB_OPTIMAL: return "OPTIMAL";
        case GRB_INFEASIBLE: return "INFEASIBLE";
        case GRB_TIME_LIMIT: return "TIME_LIMIT";
        case GRB_INTERRUPTED: return "INTERRUPTED";
        case GRB_UNBOUNDED: return "UNBOUNDED";
        case GRB_INF_OR_UNBD: return "INF_OR_UNBD";
        case GRB_NUMERIC: return "NUMERIC";
        case GRB_SUBOPTIMAL: return "SUBOPTIMAL";
        default: return "STATUS_" + std::to_string(status);
    }
}

const SolverConfig& active_or_default_gurobi_cfg()
{
    static const SolverConfig default_cfg{};
    return g_active_preprocess_cfg ? *g_active_preprocess_cfg : default_cfg;
}

struct GurobiRetrySolveInfo
{
    int first_status = 0;
    int final_status = 0;
    int sol_count = 0;
};

bool is_definitive_no_retry_status(int status)
{
    return status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED;
}

GurobiRetrySolveInfo optimize_with_feasibility_retry(GRBModel& model,
                                                     const SolverConfig& cfg,
                                                     const std::string& phase_name)
{
    GurobiRetrySolveInfo info;
    model.optimize();
    info.first_status = model.get(GRB_IntAttr_Status);
    info.final_status = info.first_status;
    info.sol_count = model.get(GRB_IntAttr_SolCount);
    if (info.sol_count > 0) return info;
    if (!cfg.gurobi_retry_enabled || !cfg.gurobi_retry_if_no_incumbent) return info;
    if (is_definitive_no_retry_status(info.first_status)) return info;

    double remaining = remaining_preprocess_seconds();
    if (cfg.gurobi_retry_respect_global_wall_seconds && std::isfinite(remaining) && remaining <= 1e-9) {
        return info;
    }

    if (cfg.verbose || cfg.gurobi_retry_verbose) {
        std::cerr << "[rack_preprocess] " << phase_name
                  << " first solve had no incumbent, status=" << status_name(info.first_status)
                  << "; retrying with SolutionLimit=" << std::max(1, cfg.gurobi_retry_solution_limit)
                  << ", MIPFocus=" << cfg.gurobi_retry_mip_focus << "\n";
    }

    model.reset();
    model.set(GRB_IntParam_SolutionLimit, std::max(1, cfg.gurobi_retry_solution_limit));
    model.set(GRB_IntParam_MIPFocus, cfg.gurobi_retry_mip_focus);
    model.set(GRB_IntParam_OutputFlag, (cfg.verbose || cfg.gurobi_retry_verbose) ? 1 : 0);
    if (cfg.gurobi_retry_remove_local_time_limit) {
        if (cfg.gurobi_retry_respect_global_wall_seconds && std::isfinite(remaining)) {
            model.set(GRB_DoubleParam_TimeLimit, std::max(0.001, remaining));
        } else {
            model.set(GRB_DoubleParam_TimeLimit, GRB_INFINITY);
        }
    }

    model.optimize();
    info.final_status = model.get(GRB_IntAttr_Status);
    info.sol_count = model.get(GRB_IntAttr_SolCount);
    return info;
}
#endif

template <class T>
MinStackCountResult min_stack_count_fallback_impl(const std::vector<T>& items, int capacity)
{
    check_unique_ids(items, "min-stack items");
    validate_heights(items, capacity, true);
    validate_stack_metadata(items);
    if (items.empty()) {
        return {{}, 0, true};
    }

    std::vector<int> order(items.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (items[a].height != items[b].height) return items[a].height > items[b].height;
        return items[a].item_id < items[b].item_id;
    });
    auto ffd = first_fit_decreasing_generic(items, capacity);
    int best_bins = static_cast<int>(ffd.size());
    std::vector<std::vector<int>> best = ffd;

    std::vector<int> loads;
    std::vector<std::vector<int>> stacks;
    const auto item_by_id = by_id(items);
    const bool has_conflicts = has_incompatible_pair(items);
    int total_remaining = 0;
    for (auto& it : items) total_remaining += it.height;

    std::function<void(int, int)> dfs = [&](int pos, int rem_height) {
        check_active_preprocess_budget();
        if (static_cast<int>(stacks.size()) >= best_bins) return;
        int free_height = 0;
        for (int load : loads) free_height += capacity - load;
        int height_requiring_new_stacks = std::max(0, rem_height - free_height);
        int area_lb_extra = (height_requiring_new_stacks + capacity - 1) / capacity;
        if (static_cast<int>(stacks.size()) + area_lb_extra >= best_bins) return;
        if (pos == static_cast<int>(order.size())) {
            best_bins = static_cast<int>(stacks.size());
            best = stacks;
            for (auto& s : best) std::sort(s.begin(), s.end());
            return;
        }
        const auto& it = items[order[pos]];
        std::set<int> tried_loads;
        for (size_t k = 0; k < loads.size(); ++k) {
            if (loads[k] + it.height <= capacity &&
                compatible_with_stack(it, stacks[k], item_by_id) &&
                (has_conflicts || !tried_loads.count(loads[k]))) {
                if (!has_conflicts) tried_loads.insert(loads[k]);
                loads[k] += it.height;
                stacks[k].push_back(it.item_id);
                dfs(pos + 1, rem_height - it.height);
                stacks[k].pop_back();
                loads[k] -= it.height;
            }
        }
        loads.push_back(it.height);
        stacks.push_back({it.item_id});
        dfs(pos + 1, rem_height - it.height);
        stacks.pop_back();
        loads.pop_back();
    };
    dfs(0, total_remaining);
    std::sort(best.begin(), best.end());
    return {best, best_bins, true};
}

struct MultiKnapsackOut {
    std::int64_t best_score = 0;
    std::vector<std::vector<int>> stacks;
};

template <class T>
MultiKnapsackOut multiple_knapsack_fallback(const std::vector<T>& raw_items, int n_stacks, int capacity)
{
    std::vector<T> items;
    for (const auto& it : raw_items) {
        if (it.height <= capacity && it.score > 0) items.push_back(it);
    }
    validate_stack_metadata(items);
    MultiKnapsackOut out;
    out.stacks.assign(n_stacks, {});
    if (n_stacks <= 0 || items.empty()) return out;

    std::vector<int> loads(n_stacks, 0);
    std::vector<std::vector<int>> stacks(n_stacks);
    const auto item_by_id = by_id(items);
    const bool has_conflicts = has_incompatible_pair(items);
    std::int64_t current_score = 0;

    std::vector<int> order(items.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        long double lhs = static_cast<long double>(items[a].score) * items[b].height;
        long double rhs = static_cast<long double>(items[b].score) * items[a].height;
        if (lhs != rhs) return lhs > rhs;
        return items[a].item_id < items[b].item_id;
    });
    std::vector<std::int64_t> suffix(order.size() + 1, 0);
    for (int p = static_cast<int>(order.size()) - 1; p >= 0; --p) suffix[p] = suffix[p + 1] + items[order[p]].score;

    std::function<void(int)> dfs = [&](int pos) {
        check_active_preprocess_budget();
        if (pos == static_cast<int>(order.size())) {
            if (current_score > out.best_score) {
                out.best_score = current_score;
                out.stacks = stacks;
                for (auto& s : out.stacks) std::sort(s.begin(), s.end());
            }
            return;
        }
        if (current_score + suffix[pos] <= out.best_score) return;
        const auto& it = items[order[pos]];
        std::set<int> tried_loads;
        for (int k = 0; k < n_stacks; ++k) {
            if (loads[k] + it.height <= capacity &&
                compatible_with_stack(it, stacks[k], item_by_id) &&
                (has_conflicts || !tried_loads.count(loads[k]))) {
                if (!has_conflicts) tried_loads.insert(loads[k]);
                loads[k] += it.height;
                stacks[k].push_back(it.item_id);
                current_score += it.score;
                dfs(pos + 1);
                current_score -= it.score;
                stacks[k].pop_back();
                loads[k] -= it.height;
            }
        }
        dfs(pos + 1);
    };
    dfs(0);
    std::sort(out.stacks.begin(), out.stacks.end());
    return out;
}

}

std::map<int, std::optional<std::int64_t>> GroupPreprocessResult::f_values() const
{
    std::map<int, std::optional<std::int64_t>> out;
    for (const auto& kv : value_table) out[kv.first] = kv.second.objective;
    return out;
}



std::map<GroupKey, RackGroup> group_racks_by_supplier_and_footprint(
    const std::vector<RackItem>& items,
    bool strict_same_canonical_size)
{
    check_unique_ids(items, "rack items");
    validate_stack_metadata(items);
    std::map<GroupKey, RackGroup> groups;
    for (const auto& source_item : items) {
        RackItem item = source_item;
        normalize_rack_footprint(item);
        if (item.is_pallet) continue;
        if (item.length <= 0 || item.width <= 0) throw std::invalid_argument("non-positive length/width");
        if (item.height <= 0) throw std::invalid_argument("non-positive height");
        auto key = item.group_key();
        auto& g = groups[key];
        g.group_key = key;
        if (item.is_optional) g.optional_items.push_back(item);
        else g.mandatory_items.push_back(item);
    }
    for (auto& kv : groups) {
        auto& g = kv.second;
        std::vector<RackItem> all = g.mandatory_items;
        all.insert(all.end(), g.optional_items.begin(), g.optional_items.end());
        if (all.empty()) continue;
        g.base_length = all[0].length;
        g.base_width = all[0].width;
        g.unit_area = all[0].area();
        if (strict_same_canonical_size) {
            std::set<std::pair<int,int>> keys;
            for (const auto& it : all) keys.insert(it.canonical_size_key());
            if (keys.size() != 1) throw std::invalid_argument("Group " + key_to_string(kv.first) + " contains different canonical size keys.");
        }
        std::set<std::int64_t> areas;
        for (const auto& it : all) areas.insert(it.area());
        if (areas.size() != 1) throw std::invalid_argument("Group " + key_to_string(kv.first) + " contains different bottom areas.");
    }
    return groups;
}

std::map<GroupKey, RackGroup> group_racks_from_instance(
    const StackPreprocessingInput& instance,
    bool strict_same_canonical_size)
{
    std::vector<RackItem> racks;
    std::map<int, std::int64_t> pallet_area_by_supplier;
    for (const auto& item : instance.items) {
        if (item.is_pallet) {
            pallet_area_by_supplier[item.supplier] += 1LL * item.length * item.width;
        }
        else racks.push_back(item);
    }
    auto groups = group_racks_by_supplier_and_footprint(racks, strict_same_canonical_size);
    for (auto& kv : groups) {
        const auto it = pallet_area_by_supplier.find(std::get<0>(kv.first));
        kv.second.supplier_pallet_area = it == pallet_area_by_supplier.end() ? 0 : it->second;
    }

    return groups;
}

MinStackCountResult solve_min_stack_count_fallback(const std::vector<RackItem>& items, int capacity)
{
    return min_stack_count_fallback_impl(items, capacity);
}
MinStackCountResult solve_min_stack_count_fallback(const std::vector<OptionalRack>& items, int capacity)
{
    return min_stack_count_fallback_impl(items, capacity);
}

BaselineStackOptimizationResult optimize_baseline_stacks_fallback(
    const GroupKey& group_key,
    const std::vector<RackItem>& mandatory_items,
    const std::vector<RackItem>& optional_items,
    int capacity,
    int base_length,
    int base_width)
{
    check_unique_ids(mandatory_items, "mandatory items");
    check_unique_ids(optional_items, "optional items");
    validate_heights(mandatory_items, capacity, true);
    validate_heights(optional_items, capacity, false);
    validate_stack_metadata(mandatory_items);
    validate_stack_metadata(optional_items);

    if (mandatory_items.empty()) {
        BaselineStackOptimizationResult r;
        r.group_key = group_key;
        r.unselected_optional.reserve(optional_items.size());
        for (auto& it : optional_items) r.unselected_optional.push_back(it.item_id);

        r.baseline_stack_count = 0;
        r.base_length = base_length;
        r.base_width = base_width;
        r.stack_capacity = capacity;
        r.phase1_proven_optimal = r.phase2_proven_optimal = true;
        return r;
    }

    auto phase1 = solve_min_stack_count_fallback(mandatory_items, capacity);
    int R = phase1.stack_count;

    std::vector<RackItem> prim = mandatory_items;
    std::sort(prim.begin(), prim.end(), [](const RackItem& a, const RackItem& b) {
        if (a.height != b.height) return a.height > b.height;
        return a.item_id < b.item_id;
    });
    std::vector<int> loads(R, 0);
    std::vector<std::vector<int>> p_stacks(R);
    std::map<int, RackItem> all_by_id = by_id(mandatory_items);
    for (const auto& item : optional_items) all_by_id[item.item_id] = item;
    std::vector<RackItem> all_items = mandatory_items;
    all_items.insert(all_items.end(), optional_items.begin(), optional_items.end());
    const bool has_conflicts = has_incompatible_pair(all_items);
    std::int64_t best_score = -1;
    std::vector<std::vector<int>> best_mandatory, best_final;
    std::vector<int> best_inserted;

    auto optional_by_id = by_id(optional_items);
    std::function<void(int)> dfs_mandatory = [&](int pos) {
        check_active_preprocess_budget();
        if (pos == static_cast<int>(prim.size())) {
            for (int k = 0; k < R; ++k) if (p_stacks[k].empty()) return;

            std::vector<int> rem(R);
            for (int k = 0; k < R; ++k) rem[k] = capacity - loads[k];
            std::vector<RackItem> sec = optional_items;
            std::sort(sec.begin(), sec.end(), [](const RackItem& a, const RackItem& b) {
                if (a.score != b.score) return a.score > b.score;
                return a.item_id < b.item_id;
            });
            std::vector<std::vector<int>> sec_stacks(R);
            std::int64_t curr = 0;
            std::vector<std::int64_t> suffix(sec.size()+1,0);
            for (int i = static_cast<int>(sec.size()) - 1; i >= 0; --i) suffix[i] = suffix[i+1] + std::max<std::int64_t>(0, sec[i].score);
            std::int64_t local_best = -1;
            std::vector<std::vector<int>> local_sec_best;
            std::function<void(int)> dfs_sec = [&](int p) {
                check_active_preprocess_budget();
                if (p == static_cast<int>(sec.size())) {
                    if (curr > local_best) { local_best = curr; local_sec_best = sec_stacks; }
                    return;
                }
                if (curr + suffix[p] < local_best) return;
                const auto& it = sec[p];
                if (it.score > 0 && it.height <= capacity) {
                    std::set<int> tried;
                    for (int k = 0; k < R; ++k) {
                        bool compatible = compatible_with_stack(it, p_stacks[k], all_by_id) &&
                                          compatible_with_stack(it, sec_stacks[k], all_by_id);
                        if (rem[k] >= it.height && compatible &&
                            (has_conflicts || !tried.count(rem[k]))) {
                            if (!has_conflicts) tried.insert(rem[k]);
                            rem[k] -= it.height;
                            sec_stacks[k].push_back(it.item_id);
                            curr += it.score;
                            dfs_sec(p + 1);
                            curr -= it.score;
                            sec_stacks[k].pop_back();
                            rem[k] += it.height;
                        }
                    }
                }
                dfs_sec(p + 1);
            };
            dfs_sec(0);
            if (local_best > best_score) {
                best_score = local_best;
                best_mandatory = p_stacks;
                for (auto& s : best_mandatory) std::sort(s.begin(), s.end());
                best_final.clear();
                best_final.resize(R);
                best_inserted.clear();
                for (int k = 0; k < R; ++k) {
                    std::vector<int> sec = local_sec_best[k];
                    std::sort(sec.begin(), sec.end());
                    best_final[k] = best_mandatory[k];
                    best_final[k].insert(best_final[k].end(), sec.begin(), sec.end());
                    for (int id : sec) best_inserted.push_back(id);
                }
                std::sort(best_inserted.begin(), best_inserted.end());
            }
            return;
        }
        const auto& it = prim[pos];
        std::set<int> tried_loads;
        for (int k = 0; k < R; ++k) {
            if (loads[k] + it.height <= capacity &&
                compatible_with_stack(it, p_stacks[k], all_by_id) &&
                (has_conflicts || !tried_loads.count(loads[k]))) {
                if (!has_conflicts) tried_loads.insert(loads[k]);
                loads[k] += it.height;
                p_stacks[k].push_back(it.item_id);
                dfs_mandatory(pos + 1);
                p_stacks[k].pop_back();
                loads[k] -= it.height;
            }
        }
    };
    dfs_mandatory(0);

    BaselineStackOptimizationResult r;
    r.group_key = group_key;
    r.mandatory_assignments = best_mandatory;
    r.final_stacks = best_final;
    r.selected_optional = best_inserted;
    std::set<int> ins(best_inserted.begin(), best_inserted.end());
    for (auto& it : optional_items) if (!ins.count(it.item_id)) r.unselected_optional.push_back(it.item_id);
    std::sort(r.unselected_optional.begin(), r.unselected_optional.end());
    r.baseline_stack_count = static_cast<int>(best_mandatory.size());
    r.baseline_optional_profit = std::max<std::int64_t>(0, best_score);
    r.baseline_stack_area = 1LL * r.baseline_stack_count * base_length * base_width;
    r.base_length = base_length;
    r.base_width = base_width;
    r.stack_capacity = capacity;
    r.phase1_proven_optimal = r.phase2_proven_optimal = true;
    validate_baseline_stack_result(r, mandatory_items, optional_items, capacity);
    return r;
}

ValueFunctionResult solve_value_function_fallback(
    const std::vector<OptionalRack>& optional_items,
    int n_stacks,
    int stack_capacity)
{
    if (n_stacks < 0) throw std::invalid_argument("n_stacks must be non-negative.");
    if (stack_capacity <= 0) throw std::invalid_argument("stack_capacity must be positive.");
    check_unique_ids(optional_items, "optional items");
    validate_stack_metadata(optional_items);
    ValueFunctionResult r;
    r.n_stacks = n_stacks;
    r.stack_capacity = stack_capacity;
    r.stacks.assign(n_stacks, {});
    r.stack_heights.assign(n_stacks, 0);
    if (n_stacks == 0 || optional_items.empty()) {
        r.objective = 0;
        r.proven_optimal = true;
        return r;
    }
    auto mk = multiple_knapsack_fallback(optional_items, n_stacks, stack_capacity);
    r.objective = mk.best_score;
    r.stacks = mk.stacks;
    auto item_by_id = by_id(optional_items);
    r.stack_heights.clear();
    r.selected_items.clear();
    for (auto& s : r.stacks) {
        int h = 0;
        for (int id : s) { h += item_by_id[id].height; r.selected_items.push_back(id); }
        r.stack_heights.push_back(h);
    }
    std::sort(r.selected_items.begin(), r.selected_items.end());
    r.proven_optimal = true;
    validate_value_result(r, optional_items);
    return r;
}

std::map<int, ValueFunctionResult> solve_value_table_fallback(
    const std::vector<OptionalRack>& optional_items,
    int max_stacks,
    int stack_capacity)
{
    if (max_stacks < 0) throw std::invalid_argument("max_stacks must be non-negative.");
    std::map<int, ValueFunctionResult> res;
    std::int64_t total_positive = 0;
    for (auto& it : optional_items) if (it.height <= stack_capacity && it.score > 0) total_positive += it.score;
    for (int r = 0; r <= max_stacks; ++r) {
        res[r] = solve_value_function_fallback(optional_items, r, stack_capacity);
        if (res[r].proven_optimal && res[r].objective.value_or(-1) == total_positive) {
            for (int rr = r+1; rr <= max_stacks; ++rr) {
                auto copy = res[r];
                copy.n_stacks = rr;
                while (static_cast<int>(copy.stacks.size()) < rr) copy.stacks.push_back({});
                while (static_cast<int>(copy.stack_heights.size()) < rr) copy.stack_heights.push_back(0);
                res[rr] = copy;
            }
            break;
        }
    }
    return res;
}

GroupPreprocessResult preprocess_rack_group_fallback(
    const RackGroup& group,
    int vehicle_length,
    int vehicle_width,
    int vehicle_height,
    const SolverConfig& cfg)
{
    (void)cfg;
    check_active_preprocess_budget();
    int L = vehicle_length, W = vehicle_width, H = vehicle_height;
    if (L <= 0 || W <= 0 || H <= 0) throw std::invalid_argument("vehicle dimensions must be positive");
    if (group.unit_area <= 0) throw std::invalid_argument("invalid group unit_area");
    auto baseline = optimize_baseline_stacks_fallback(group.group_key, group.mandatory_items, group.optional_items, H, group.base_length, group.base_width);
    std::map<int, RackItem> optional_by_id;
    for (const auto& it : group.optional_items) optional_by_id[it.item_id] = it;
    std::vector<RackItem> remaining;
    for (int id : baseline.unselected_optional) remaining.push_back(optional_by_id[id]);
    std::int64_t area_A = baseline.baseline_stack_area;
    std::int64_t area_rem = 1LL * L * W - area_A - group.supplier_pallet_area;
    std::int64_t R_area = std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(static_cast<long double>(area_rem) / static_cast<long double>(group.unit_area))));
    std::optional<MinStackCountResult> R_all_res;
    int R_all = 0;
    if (!remaining.empty()) {
        R_all_res = solve_min_stack_count_fallback(remaining, H);
        R_all = R_all_res->stack_count;
    }
    int R_max = static_cast<int>(std::max<std::int64_t>(0, std::min<std::int64_t>(R_area, R_all)));
    std::vector<OptionalRack> sec_value;
    for (auto& it : remaining) {
        sec_value.push_back({it.item_id, it.height, it.score, it.compatible_item_ids});
    }
    auto value_table = solve_value_table_fallback(sec_value, R_max, H);
    return {group.group_key, group.base_length, group.base_width, group.unit_area,
            baseline, area_A, area_rem, R_area, R_all, R_max,
            value_table};
}

PreprocessAllGroupsResult preprocess_stacks_fallback(
    const StackPreprocessingInput& instance,
    bool strict_same_canonical_size,
    const SolverConfig& cfg)
{
    SolverConfig local_cfg = with_default_budget_callbacks(cfg);
    ActivePreprocessConfigScope active_scope(local_cfg);
    check_active_preprocess_budget();
    auto groups = group_racks_from_instance(instance, strict_same_canonical_size);
    PreprocessAllGroupsResult all;
    all.vehicle_length = instance.vehicle_length;
    all.vehicle_width = instance.vehicle_width;
    all.vehicle_height = instance.vehicle_height;
    for (const auto& kv : groups) {
        check_active_preprocess_budget();
        all.groups[kv.first] = preprocess_rack_group_fallback(kv.second, instance.vehicle_length, instance.vehicle_width, instance.vehicle_height, local_cfg);
    }
    return all;
}

#ifdef HRLH_STACKS_WITH_GUROBI

MinStackCountResult solve_min_stack_count_gurobi(
    const std::vector<RackItem>& items,
    int capacity,
    std::optional<int> max_stacks,
    double time_limit,
    double mip_gap,
    bool verbose,
    bool symmetry_breaking,
    bool load_order_symmetry,
    bool use_warm_start,
    bool require_optimal,
    int gurobi_symmetry)
{
    check_unique_ids(items, "min-stack items");
    validate_heights(items, capacity, true);
    validate_stack_metadata(items);
    if (items.empty()) return {{}, 0, true};

    int n = static_cast<int>(items.size());
    int Knum = max_stacks ? std::max(1, std::min(n, *max_stacks)) : n;
    std::vector<int> K(Knum); std::iota(K.begin(), K.end(), 0);
    auto ffd = first_fit_decreasing_generic(items, capacity);

    GRBEnv env(true);
    env.set(GRB_IntParam_OutputFlag, verbose ? 1 : 0);

    env.set(GRB_IntParam_Threads, 1);
    env.start();
    GRBModel model(env);
    model.set(GRB_StringAttr_ModelName, "min_stack_count_1d_bin_packing");
    check_active_preprocess_budget();
    double effective_limit = effective_gurobi_time_limit(time_limit);
    if (effective_limit >= 0) model.set(GRB_DoubleParam_TimeLimit, effective_limit);
    model.set(GRB_DoubleParam_MIPGap, mip_gap);
    model.set(GRB_IntParam_Symmetry, gurobi_symmetry);

    std::vector<std::vector<GRBVar>> x(n, std::vector<GRBVar>(Knum));
    std::vector<GRBVar> z(Knum);
    for (int i = 0; i < n; ++i) for (int k = 0; k < Knum; ++k) x[i][k] = model.addVar(0, 1, 0, GRB_BINARY);
    for (int k = 0; k < Knum; ++k) z[k] = model.addVar(0, 1, 0, GRB_BINARY);
    model.update();
    for (int i = 0; i < n; ++i) {
        GRBLinExpr e = 0; for (int k = 0; k < Knum; ++k) e += x[i][k];
        model.addConstr(e == 1);
    }
    for (int k = 0; k < Knum; ++k) {
        GRBLinExpr count = 0, height = 0;
        for (int i = 0; i < n; ++i) { count += x[i][k]; height += items[i].height * x[i][k]; }
        model.addConstr(count <= n * z[k]);
        model.addConstr(count >= z[k]);
        model.addConstr(height <= capacity * z[k]);
    }
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (stack_compatible(items[i], items[j])) continue;
            for (int k = 0; k < Knum; ++k) model.addConstr(x[i][k] + x[j][k] <= 1);
        }
    }
    if (symmetry_breaking) for (int k = 0; k + 1 < Knum; ++k) model.addConstr(z[k] >= z[k+1]);
    if (load_order_symmetry) {
        for (int k = 0; k + 1 < Knum; ++k) {
            GRBLinExpr a = 0, b = 0;
            for (int i = 0; i < n; ++i) { a += items[i].height * x[i][k]; b += items[i].height * x[i][k+1]; }
            model.addConstr(a >= b);
        }
    }
    GRBLinExpr obj = 0; for (int k = 0; k < Knum; ++k) obj += z[k];
    model.setObjective(obj, GRB_MINIMIZE);
    if (use_warm_start) {
        for (int k = 0; k < Knum; ++k) z[k].set(GRB_DoubleAttr_Start, 0.0);
        std::map<int,int> id_to_idx; for (int i = 0; i < n; ++i) id_to_idx[items[i].item_id] = i;
        for (int k = 0; k < std::min<int>(Knum, ffd.size()); ++k) {
            z[k].set(GRB_DoubleAttr_Start, 1.0);
            for (int id : ffd[k]) x[id_to_idx[id]][k].set(GRB_DoubleAttr_Start, 1.0);
        }
    }
    auto solve_info = optimize_with_feasibility_retry(model, active_or_default_gurobi_cfg(), "min_stack_count_1d_bin_packing");
    check_active_preprocess_budget();
    int status = solve_info.final_status;
    bool optimal = status == GRB_OPTIMAL;
    if (solve_info.sol_count == 0) throw std::runtime_error("No feasible min-stack solution. Status=" + status_name(status));
    if (require_optimal && !optimal) throw std::runtime_error("Minimum stack count was not proven optimal. Status=" + status_name(status));
    std::vector<std::vector<int>> stacks;
    for (int k = 0; k < Knum; ++k) if (z[k].get(GRB_DoubleAttr_X) > 0.5) {
        std::vector<int> s;
        for (int i = 0; i < n; ++i) if (x[i][k].get(GRB_DoubleAttr_X) > 0.5) s.push_back(items[i].item_id);
        std::sort(s.begin(), s.end()); stacks.push_back(s);
    }
    return {stacks, static_cast<int>(stacks.size()), optimal};
}

MinStackCountResult solve_min_stack_count_gurobi(
    const std::vector<OptionalRack>& items,
    int capacity,
    std::optional<int> max_stacks,
    double time_limit,
    double mip_gap,
    bool verbose,
    bool symmetry_breaking,
    bool load_order_symmetry,
    bool use_warm_start,
    bool require_optimal,
    int gurobi_symmetry)
{
    std::vector<RackItem> converted;
    for (auto& it : items) {
        converted.push_back({it.item_id, 0, 1, 1, it.height, it.score, true, false, it.compatible_item_ids});
    }
    auto r = solve_min_stack_count_gurobi(converted, capacity, max_stacks, time_limit, mip_gap, verbose, symmetry_breaking, load_order_symmetry, use_warm_start, require_optimal, gurobi_symmetry);
    return r;
}

BaselineStackOptimizationResult optimize_baseline_stacks_gurobi(
    const GroupKey& group_key,
    const std::vector<RackItem>& mandatory_items,
    const std::vector<RackItem>& optional_items,
    int capacity,
    int base_length,
    int base_width,
    const SolverConfig& cfg)
{
    if (mandatory_items.empty()) return optimize_baseline_stacks_fallback(group_key, mandatory_items, optional_items, capacity, base_length, base_width);
    validate_stack_metadata(mandatory_items);
    validate_stack_metadata(optional_items);
    auto phase1 = solve_min_stack_count_gurobi(mandatory_items, capacity, std::nullopt,
        cfg.mandatory_stack_time_limit, cfg.mip_gap, cfg.verbose, cfg.symmetry_breaking,
        cfg.load_order_symmetry, cfg.use_warm_start, cfg.require_optimal, cfg.gurobi_symmetry);
    int Rstar = phase1.stack_count;
    int nP = static_cast<int>(mandatory_items.size());
    int nS = static_cast<int>(optional_items.size());
    int Knum = nP;

    GRBEnv env(true); env.set(GRB_IntParam_OutputFlag, cfg.verbose ? 1 : 0); env.set(GRB_IntParam_Threads, 1); env.start();
    GRBModel model(env);
    model.set(GRB_StringAttr_ModelName, "baseline_joint_optional_profit");
    check_active_preprocess_budget();
    model.set(GRB_DoubleParam_TimeLimit, effective_gurobi_time_limit(cfg.baseline_joint_time_limit));
    model.set(GRB_DoubleParam_MIPGap, cfg.mip_gap);
    model.set(GRB_IntParam_Symmetry, cfg.gurobi_symmetry);
    std::vector<std::vector<GRBVar>> x(nP, std::vector<GRBVar>(Knum));
    std::vector<std::vector<GRBVar>> y(nS, std::vector<GRBVar>(Knum));
    std::vector<GRBVar> z(Knum);
    for (int i = 0; i < nP; ++i) for (int k = 0; k < Knum; ++k) x[i][k] = model.addVar(0,1,0,GRB_BINARY);
    for (int b = 0; b < nS; ++b) for (int k = 0; k < Knum; ++k) y[b][k] = model.addVar(0,1,0,GRB_BINARY);
    for (int k = 0; k < Knum; ++k) z[k] = model.addVar(0,1,0,GRB_BINARY);
    model.update();
    for (int i = 0; i < nP; ++i) { GRBLinExpr e=0; for (int k=0;k<Knum;++k) e += x[i][k]; model.addConstr(e == 1); }
    for (int b = 0; b < nS; ++b) { GRBLinExpr e=0; for (int k=0;k<Knum;++k) e += y[b][k]; model.addConstr(e <= 1); }
    for (int k = 0; k < Knum; ++k) {
        GRBLinExpr pc = 0, h = 0;
        for (int i = 0; i < nP; ++i) { pc += x[i][k]; h += mandatory_items[i].height * x[i][k]; }
        for (int b = 0; b < nS; ++b) { h += optional_items[b].height * y[b][k]; model.addConstr(y[b][k] <= z[k]); }
        model.addConstr(pc <= nP * z[k]);
        model.addConstr(h <= capacity * z[k]);
    }
    for (int i = 0; i < nP; ++i) {
        for (int j = i + 1; j < nP; ++j) {
            if (stack_compatible(mandatory_items[i], mandatory_items[j])) continue;
            for (int k = 0; k < Knum; ++k) model.addConstr(x[i][k] + x[j][k] <= 1);
        }
    }
    for (int i = 0; i < nP; ++i) {
        for (int b = 0; b < nS; ++b) {
            if (stack_compatible(mandatory_items[i], optional_items[b])) continue;
            for (int k = 0; k < Knum; ++k) model.addConstr(x[i][k] + y[b][k] <= 1);
        }
    }
    for (int a = 0; a < nS; ++a) {
        for (int b = a + 1; b < nS; ++b) {
            if (stack_compatible(optional_items[a], optional_items[b])) continue;
            for (int k = 0; k < Knum; ++k) model.addConstr(y[a][k] + y[b][k] <= 1);
        }
    }
    { GRBLinExpr e=0; for (int k=0;k<Knum;++k) e += z[k]; model.addConstr(e == Rstar); }
    if (cfg.symmetry_breaking) for (int k=0;k+1<Knum;++k) model.addConstr(z[k] >= z[k+1]);
    if (cfg.load_order_symmetry) for (int k=0;k+1<Knum;++k) {
        GRBLinExpr a=0,b=0; for (int i=0;i<nP;++i){ a += mandatory_items[i].height*x[i][k]; b += mandatory_items[i].height*x[i][k+1];}
        model.addConstr(a >= b);
    }
    GRBLinExpr obj=0; for (int b=0;b<nS;++b) for(int k=0;k<Knum;++k) obj += static_cast<double>(optional_items[b].score) * y[b][k];
    model.setObjective(obj, GRB_MAXIMIZE);
    if (cfg.use_warm_start) {
        for (int k = 0; k < Knum; ++k) {
            z[k].set(GRB_DoubleAttr_Start, 0.0);
            for (int i = 0; i < nP; ++i) x[i][k].set(GRB_DoubleAttr_Start, 0.0);
            for (int b = 0; b < nS; ++b) y[b][k].set(GRB_DoubleAttr_Start, 0.0);
        }
        std::map<int, int> mandatory_id_to_idx;
        for (int i = 0; i < nP; ++i) mandatory_id_to_idx[mandatory_items[i].item_id] = i;
        for (int k = 0; k < std::min<int>(Knum, phase1.stacks.size()); ++k) {
            z[k].set(GRB_DoubleAttr_Start, 1.0);
            for (int id : phase1.stacks[k]) {
                auto it = mandatory_id_to_idx.find(id);
                if (it != mandatory_id_to_idx.end()) x[it->second][k].set(GRB_DoubleAttr_Start, 1.0);
            }
        }
    }
    auto solve_info = optimize_with_feasibility_retry(model, cfg, "baseline_joint_optional_profit");
    check_active_preprocess_budget();
    int status = solve_info.final_status;
    bool optimal = status == GRB_OPTIMAL;
    if (solve_info.sol_count == 0) throw std::runtime_error("Baseline joint stack optimization no feasible solution");
    if (cfg.require_optimal && !optimal) throw std::runtime_error("Baseline joint stack optimization not proven optimal");
    BaselineStackOptimizationResult r;
    r.group_key = group_key; r.base_length=base_length; r.base_width=base_width; r.stack_capacity=capacity;
    r.phase1_proven_optimal=phase1.proven_optimal; r.phase2_proven_optimal=optimal;
    std::set<int> inserted;
    for (int k=0;k<Knum;++k) if (z[k].get(GRB_DoubleAttr_X)>0.5) {
        std::vector<int> prim, sec, fin;
        for(int i=0;i<nP;++i) if(x[i][k].get(GRB_DoubleAttr_X)>0.5) prim.push_back(mandatory_items[i].item_id);
        for(int b=0;b<nS;++b) if(y[b][k].get(GRB_DoubleAttr_X)>0.5) { sec.push_back(optional_items[b].item_id); inserted.insert(optional_items[b].item_id); }
        std::sort(prim.begin(), prim.end());
        std::sort(sec.begin(), sec.end());
        fin = prim;
        fin.insert(fin.end(), sec.begin(), sec.end());
        r.mandatory_assignments.push_back(prim); r.final_stacks.push_back(fin);
    }
    r.selected_optional.assign(inserted.begin(), inserted.end());
    for(auto& it: optional_items) if(!inserted.count(it.item_id)) r.unselected_optional.push_back(it.item_id);
    std::sort(r.unselected_optional.begin(), r.unselected_optional.end());
    r.baseline_stack_count = static_cast<int>(r.mandatory_assignments.size());
    r.baseline_optional_profit = 0; for(auto& it: optional_items) if(inserted.count(it.item_id)) r.baseline_optional_profit += it.score;
    r.baseline_stack_area = 1LL * r.baseline_stack_count * base_length * base_width;
    validate_baseline_stack_result(r, mandatory_items, optional_items, capacity);
    return r;
}

ValueFunctionResult solve_value_function_gurobi(
    const std::vector<OptionalRack>& optional_items,
    int n_stacks,
    int stack_capacity,
    const SolverConfig& cfg)
{
    if (n_stacks < 0) throw std::invalid_argument("n_stacks must be non-negative");
    if (stack_capacity <= 0) throw std::invalid_argument("stack_capacity must be positive");
    check_unique_ids(optional_items, "optional items");
    validate_stack_metadata(optional_items);
    if (n_stacks == 0 || optional_items.empty()) return solve_value_function_fallback(optional_items, n_stacks, stack_capacity);
    std::vector<OptionalRack> items;
    for(auto& it: optional_items) if(it.height <= stack_capacity && it.score > 0) items.push_back(it);
    if(items.empty()) return solve_value_function_fallback(optional_items, n_stacks, stack_capacity);
    int n = static_cast<int>(items.size());
    GRBEnv env(true); env.set(GRB_IntParam_OutputFlag, cfg.verbose ? 1 : 0); env.set(GRB_IntParam_Threads, 1); env.start();
    GRBModel model(env); model.set(GRB_StringAttr_ModelName, "optional_stack_profit_function");
    check_active_preprocess_budget();
    model.set(GRB_DoubleParam_TimeLimit, effective_gurobi_time_limit(cfg.spread_joint_time_limit)); model.set(GRB_DoubleParam_MIPGap, cfg.mip_gap); model.set(GRB_IntParam_Symmetry, cfg.gurobi_symmetry);
    std::vector<std::vector<GRBVar>> x(n, std::vector<GRBVar>(n_stacks));
    std::vector<GRBVar> u(n);
    for(int i=0;i<n;++i){ u[i]=model.addVar(0,1,0,GRB_BINARY); for(int k=0;k<n_stacks;++k) x[i][k]=model.addVar(0,1,0,GRB_BINARY); }
    model.update();
    GRBLinExpr obj=0; for(int i=0;i<n;++i) obj += static_cast<double>(items[i].score) * u[i]; model.setObjective(obj, GRB_MAXIMIZE);
    for(int i=0;i<n;++i){ GRBLinExpr e=0; for(int k=0;k<n_stacks;++k) e += x[i][k]; model.addConstr(e == u[i]); }
    for(int k=0;k<n_stacks;++k){ GRBLinExpr h=0; for(int i=0;i<n;++i) h += items[i].height * x[i][k]; model.addConstr(h <= stack_capacity); }
    for(int i=0;i<n;++i) for(int j=i+1;j<n;++j) if(!stack_compatible(items[i],items[j])) {
        for(int k=0;k<n_stacks;++k) model.addConstr(x[i][k] + x[j][k] <= 1);
    }
    if(cfg.symmetry_breaking && n_stacks >= 2) for(int k=0;k+1<n_stacks;++k){ GRBLinExpr a=0,b=0; for(int i=0;i<n;++i){ a += items[i].height*x[i][k]; b += items[i].height*x[i][k+1];} model.addConstr(a >= b); }
    auto solve_info = optimize_with_feasibility_retry(model, cfg, "optional_stack_profit_function");
    check_active_preprocess_budget();
    int status=solve_info.final_status; bool opt=status==GRB_OPTIMAL;
    ValueFunctionResult r; r.n_stacks=n_stacks; r.stack_capacity=stack_capacity; r.stacks.assign(n_stacks,{}); r.stack_heights.assign(n_stacks,0); r.proven_optimal=opt;
    if(solve_info.sol_count==0){ r.objective=std::nullopt; return r; }
    if(cfg.require_optimal && !opt) throw std::runtime_error("Optional stack profit optimization not proven optimal");
    for(int i=0;i<n;++i) for(int k=0;k<n_stacks;++k) if(x[i][k].get(GRB_DoubleAttr_X)>0.5) r.stacks[k].push_back(items[i].item_id);
    auto item_by_id=by_id(items); r.selected_items.clear(); r.stack_heights.clear();
    for(auto& s:r.stacks){ std::sort(s.begin(),s.end()); int h=0; for(int id:s){h+=item_by_id[id].height; r.selected_items.push_back(id);} r.stack_heights.push_back(h); }
    std::sort(r.selected_items.begin(), r.selected_items.end()); r.objective=static_cast<std::int64_t>(std::llround(model.get(GRB_DoubleAttr_ObjVal)));
    validate_value_result(r, optional_items); return r;
}

#else

MinStackCountResult solve_min_stack_count_gurobi(const std::vector<RackItem>& items, int capacity, std::optional<int>, double, double, bool, bool, bool, bool, bool, int)
{ return solve_min_stack_count_fallback(items, capacity); }
MinStackCountResult solve_min_stack_count_gurobi(const std::vector<OptionalRack>& items, int capacity, std::optional<int>, double, double, bool, bool, bool, bool, bool, int)
{ return solve_min_stack_count_fallback(items, capacity); }
BaselineStackOptimizationResult optimize_baseline_stacks_gurobi(const GroupKey& group_key, const std::vector<RackItem>& mandatory_items, const std::vector<RackItem>& optional_items, int capacity, int base_length, int base_width, const SolverConfig&)
{ return optimize_baseline_stacks_fallback(group_key, mandatory_items, optional_items, capacity, base_length, base_width); }
ValueFunctionResult solve_value_function_gurobi(const std::vector<OptionalRack>& optional_items, int n_stacks, int stack_capacity, const SolverConfig&)
{ return solve_value_function_fallback(optional_items, n_stacks, stack_capacity); }
#endif

std::map<int, ValueFunctionResult> solve_value_table_gurobi(
    const std::vector<OptionalRack>& optional_items,
    int max_stacks,
    int stack_capacity,
    const SolverConfig& cfg)
{
#ifdef HRLH_STACKS_WITH_GUROBI
    if (max_stacks < 0) throw std::invalid_argument("max_stacks must be non-negative");
    std::map<int, ValueFunctionResult> res;
    std::int64_t total_positive = 0;
    for(auto& it: optional_items) if(it.height <= stack_capacity && it.score > 0) total_positive += it.score;
    for(int r=0; r<=max_stacks; ++r){
        res[r] = solve_value_function_gurobi(optional_items, r, stack_capacity, cfg);
        if(res[r].proven_optimal && res[r].objective.value_or(-1)==total_positive){
            for(int rr=r+1; rr<=max_stacks; ++rr){ auto copy=res[r]; copy.n_stacks=rr; while(static_cast<int>(copy.stacks.size())<rr) copy.stacks.push_back({}); while(static_cast<int>(copy.stack_heights.size())<rr) copy.stack_heights.push_back(0); res[rr]=copy; }
            break;
        }
    }
    return res;
#else
    (void)cfg;
    return solve_value_table_fallback(optional_items, max_stacks, stack_capacity);
#endif
}

#ifdef HRLH_STACKS_WITH_GUROBI
namespace {

std::vector<RackItem> spread_items(const RackGroup& group)
{
    auto items = group.mandatory_items;
    items.insert(items.end(), group.optional_items.begin(), group.optional_items.end());
    return items;
}

std::int64_t spread_profit(const std::vector<std::vector<int>>& stacks,
                           const std::map<int,RackItem>& items)
{
    std::int64_t value = 0;
    for (const auto& stack : stacks) for (int id : stack) {
        const auto& item = items.at(id);
        if (item.is_optional) value += item.score;
    }
    return value;
}

void validate_spread_point(const SpreadJointCurvePoint& point,
                           const RackGroup& group, int height_cap)
{
    if (static_cast<int>(point.stacks.size()) != point.stack_count)
        throw std::runtime_error("Spread-Joint exact stack count mismatch");
    const auto items = spread_items(group);
    const auto item_by_id = by_id(items);
    std::set<int> seen;
    for (const auto& stack : point.stacks) {
        if (stack.empty()) throw std::runtime_error("Spread-Joint empty exact-count stack");
        int height = 0;
        for (int id : stack) {
            if (!seen.insert(id).second) throw std::runtime_error("Spread-Joint duplicate item");
            height += item_by_id.at(id).height;
        }
        if (height > height_cap) throw std::runtime_error("Spread-Joint stack height exceeded");
        for (std::size_t i=0;i<stack.size();++i) for (std::size_t j=i+1;j<stack.size();++j)
            if (!stack_compatible(item_by_id.at(stack[i]), item_by_id.at(stack[j])))
                throw std::runtime_error("Spread-Joint incompatible physical stack");
    }
    for (const auto& item : group.mandatory_items)
        if (!seen.count(item.item_id)) throw std::runtime_error("Spread-Joint missing mandatory item");
    if (spread_profit(point.stacks,item_by_id) != point.optional_profit)
        throw std::runtime_error("Spread-Joint physical profit mismatch");
}

SpreadJointCurvePoint extend_spread_point(const SpreadJointCurvePoint& previous,
                                          const RackGroup& group, int height_cap)
{
    SpreadJointCurvePoint next = previous;
    ++next.stack_count;
    ++next.extra_stack_count;
    next.proven_optimal = false;
    std::set<int> selected;
    for (const auto& stack : next.stacks) selected.insert(stack.begin(),stack.end());
    const RackItem* best_unselected = nullptr;
    for (const auto& item : group.optional_items) {
        if (selected.count(item.item_id) || item.height > height_cap) continue;
        if (!best_unselected || item.score > best_unselected->score)
            best_unselected = &item;
    }
    if (best_unselected) {
        next.stacks.push_back({best_unselected->item_id});
        next.optional_profit += best_unselected->score;
    } else {
        auto it = std::find_if(next.stacks.begin(), next.stacks.end(),
                               [](const auto& stack){return stack.size()>1;});
        if (it == next.stacks.end())
            throw std::runtime_error("Spread-Joint cannot construct next exact count");
        const int id = it->back();
        it->pop_back();
        next.stacks.push_back({id});
    }
    validate_spread_point(next,group,height_cap);
    return next;
}

SpreadJointCurvePoint solve_spread_joint_point(const RackGroup& group,
                                                const SpreadJointCurvePoint& warm,
                                                int height_cap,
                                                const SolverConfig& cfg)
{
    const auto items = spread_items(group);
    const int n = static_cast<int>(items.size());
    const int K = warm.stack_count;
    check_active_preprocess_budget();
    GRBEnv env(true);
    env.set(GRB_IntParam_OutputFlag, cfg.verbose ? 1 : 0);
    env.start();
    GRBModel model(env);
    model.set(GRB_StringAttr_ModelName,"full_spread_joint_exact_count");
    model.set(GRB_IntParam_Threads,1);
    model.set(GRB_IntParam_Seed,0);
    model.set(GRB_IntParam_Symmetry,cfg.gurobi_symmetry);
    model.set(GRB_DoubleParam_MIPGap,0.0);
    model.set(GRB_DoubleParam_TimeLimit,effective_gurobi_time_limit(cfg.spread_joint_time_limit));
    std::vector<std::vector<GRBVar>> x(n,std::vector<GRBVar>(K));
    for (int i=0;i<n;++i) for (int k=0;k<K;++k)
        x[i][k]=model.addVar(0.0,1.0,
            items[i].is_optional ? static_cast<double>(items[i].score) : 0.0,GRB_BINARY);
    model.set(GRB_IntAttr_ModelSense,GRB_MAXIMIZE);
    model.update();
    for (int i=0;i<n;++i) {
        GRBLinExpr assigned=0;
        for (int k=0;k<K;++k) assigned += x[i][k];
        model.addConstr(items[i].is_optional ? assigned <= 1 : assigned == 1);
    }
    for (int k=0;k<K;++k) {
        GRBLinExpr occupancy=0,height=0;
        for (int i=0;i<n;++i) {
            occupancy += x[i][k];
            height += items[i].height*x[i][k];
        }
        model.addConstr(occupancy >= 1);
        model.addConstr(height <= height_cap);
        for (int i=0;i<n;++i) for (int j=i+1;j<n;++j)
            if (!stack_compatible(items[i],items[j]))
                model.addConstr(x[i][k]+x[j][k]<=1);
    }
    if (cfg.use_warm_start) {
        std::map<int,int> item_index;
        for (int i=0;i<n;++i) item_index[items[i].item_id]=i;
        for (int i=0;i<n;++i) for (int k=0;k<K;++k)
            x[i][k].set(GRB_DoubleAttr_Start,0.0);
        for (int k=0;k<K;++k) for (int id:warm.stacks[k])
            x[item_index.at(id)][k].set(GRB_DoubleAttr_Start,1.0);
    }
    auto solve_info=optimize_with_feasibility_retry(model,cfg,"full_spread_joint_exact_count");
    SpreadJointCurvePoint result=warm;
    result.proven_optimal=solve_info.final_status==GRB_OPTIMAL;
    if (solve_info.sol_count>0) {
        std::vector<std::vector<int>> stacks(K);
        for (int i=0;i<n;++i) for (int k=0;k<K;++k)
            if (x[i][k].get(GRB_DoubleAttr_X)>0.5)
                stacks[k].push_back(items[i].item_id);
        const auto item_by_id=by_id(items);
        const auto value=spread_profit(stacks,item_by_id);
        if (value>=warm.optional_profit) {
            result.stacks=std::move(stacks);
            result.optional_profit=value;
        } else if (result.proven_optimal) {
            throw std::runtime_error("Spread-Joint optimal result worse than feasible warm start");
        }
    } else {
        result.proven_optimal=false;
    }
    if (cfg.require_optimal && !result.proven_optimal)
        throw std::runtime_error("Spread-Joint exact-count point not proven optimal");
    validate_spread_point(result,group,height_cap);
    return result;
}

}
#endif

GroupPreprocessResult preprocess_rack_group(
    const RackGroup& group,
    int vehicle_length,
    int vehicle_width,
    int vehicle_height,
    const SolverConfig& cfg)
{
#ifdef HRLH_STACKS_WITH_GUROBI
    int L=vehicle_length, W=vehicle_width, H=vehicle_height;
    if(L<=0||W<=0||H<=0) throw std::invalid_argument("vehicle dimensions must be positive");
    if(group.unit_area<=0) throw std::invalid_argument("invalid group unit_area");
    auto baseline = optimize_baseline_stacks_gurobi(group.group_key, group.mandatory_items, group.optional_items, H, group.base_length, group.base_width, cfg);
    std::map<int,RackItem> optional_by_id; for(auto& it:group.optional_items) optional_by_id[it.item_id]=it;
    std::vector<RackItem> remaining; for(int id: baseline.unselected_optional) remaining.push_back(optional_by_id[id]);
    std::int64_t area_A=baseline.baseline_stack_area; std::int64_t area_rem=1LL*L*W-area_A-group.supplier_pallet_area; std::int64_t R_area=std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(static_cast<long double>(area_rem)/static_cast<long double>(group.unit_area))));
    std::optional<MinStackCountResult> R_all_res; int R_all=0;
    if(!remaining.empty()){ R_all_res=solve_min_stack_count_gurobi(remaining,H,std::nullopt,cfg.time_limit_R_all,cfg.mip_gap,cfg.verbose,cfg.symmetry_breaking,cfg.load_order_symmetry,cfg.use_warm_start,cfg.require_optimal,cfg.gurobi_symmetry); R_all=R_all_res->stack_count; }
    int R_max=static_cast<int>(std::max<std::int64_t>(0, std::min<std::int64_t>(R_area,R_all)));
    std::vector<OptionalRack> sec_value;
    for(auto& it:remaining) sec_value.push_back({it.item_id,it.height,it.score,it.compatible_item_ids});

    auto value_table=cfg.enable_spread_joint
        ? std::map<int,ValueFunctionResult>{}
        : solve_value_table_gurobi(sec_value,R_max,H,cfg);
    return {group.group_key, group.base_length, group.base_width, group.unit_area, baseline, area_A, area_rem, R_area, R_all, R_max, value_table};
#else
    (void)cfg;
    return preprocess_rack_group_fallback(group, vehicle_length, vehicle_width, vehicle_height, cfg);
#endif
}

PreprocessAllGroupsResult preprocess_stacks(
    const StackPreprocessingInput& instance,
    bool strict_same_canonical_size,
    const SolverConfig& cfg)
{
    SolverConfig local_cfg = with_default_budget_callbacks(cfg);
    ActivePreprocessConfigScope active_scope(local_cfg);
    check_active_preprocess_budget();
    auto groups = group_racks_from_instance(instance, strict_same_canonical_size);
    PreprocessAllGroupsResult all; all.vehicle_length=instance.vehicle_length; all.vehicle_width=instance.vehicle_width; all.vehicle_height=instance.vehicle_height;
    for(const auto& kv:groups) {
        check_active_preprocess_budget();
        all.groups[kv.first]=preprocess_rack_group(kv.second, instance.vehicle_length, instance.vehicle_width, instance.vehicle_height, local_cfg);
    }
#ifdef HRLH_STACKS_WITH_GUROBI
    if (local_cfg.enable_spread_joint) {
        const std::int64_t floor_area=1LL*instance.vehicle_length*instance.vehicle_width;
        std::map<int,std::int64_t> mandatory_area_by_supplier;
        std::set<int> pallet_counted;
        for (const auto& [key,group]:all.groups) {
            const int supplier=std::get<0>(key);
            if (pallet_counted.insert(supplier).second)
                mandatory_area_by_supplier[supplier]+=groups.at(key).supplier_pallet_area;
            mandatory_area_by_supplier[supplier]+=group.baseline_result.baseline_stack_area;
        }
        for (auto& [key,group]:all.groups) {
            check_active_preprocess_budget();
            const auto& source=groups.at(key);
            const int supplier=std::get<0>(key);
            group.area_rem_g=floor_area-mandatory_area_by_supplier.at(supplier);
            group.R_area_g=std::max<std::int64_t>(0,group.area_rem_g/group.unit_area);
            group.R_max_g=static_cast<int>(std::min<std::int64_t>(group.R_area_g,group.R_all_g));
            SpreadJointCurvePoint base;
            base.stack_count=group.baseline_result.baseline_stack_count;
            base.extra_stack_count=0;
            base.optional_profit=group.baseline_result.baseline_optional_profit;
            base.stacks=group.baseline_result.final_stacks;
            base.proven_optimal=group.baseline_result.phase1_proven_optimal
                && group.baseline_result.phase2_proven_optimal;
            validate_spread_point(base,source,instance.vehicle_height);
            group.spread_joint_curve[0]=base;
            group.spread_joint_enabled=true;
            std::int64_t all_positive_profit=0;
            for (const auto& item:source.optional_items)
                all_positive_profit+=std::max<std::int64_t>(0,item.score);
            for (int r=1;r<=group.R_max_g;++r) {
                if (group.spread_joint_curve.at(r-1).optional_profit==all_positive_profit) break;
                check_active_preprocess_budget();
                const auto warm=extend_spread_point(group.spread_joint_curve.at(r-1),
                                                     source,instance.vehicle_height);
                auto point=solve_spread_joint_point(source,warm,instance.vehicle_height,local_cfg);
                if (point.optional_profit<group.spread_joint_curve.at(r-1).optional_profit)
                    throw std::runtime_error("Spread-Joint profit curve decreased");
                group.spread_joint_curve[r]=std::move(point);
            }
            group.R_max_g=group.spread_joint_curve.rbegin()->first;
        }
    }
#endif
    return all;
}

void validate_baseline_stack_result(const BaselineStackOptimizationResult& result,
                           const std::vector<RackItem>& mandatory_items,
                           const std::vector<RackItem>& optional_items,
                           int capacity)
{
    if(result.baseline_stack_count<0 ||
       result.mandatory_assignments.size()!=static_cast<std::size_t>(result.baseline_stack_count) ||
       result.final_stacks.size()!=static_cast<std::size_t>(result.baseline_stack_count))
        throw std::runtime_error("baseline stack count mismatch");
    std::map<int,RackItem> item_map;
    std::set<int> mandatory_ids;
    for(const auto& it:mandatory_items) {
        if(!item_map.emplace(it.item_id,it).second)throw std::runtime_error("duplicate baseline input item");
        mandatory_ids.insert(it.item_id);
    }
    for(const auto& it:optional_items)
        if(!item_map.emplace(it.item_id,it).second)throw std::runtime_error("duplicate baseline input item");
    std::vector<int> seen_mandatory; for(auto& st:result.mandatory_assignments) for(int id:st) seen_mandatory.push_back(id); std::sort(seen_mandatory.begin(),seen_mandatory.end());
    std::vector<int> expected; for(auto& it:mandatory_items) expected.push_back(it.item_id); std::sort(expected.begin(),expected.end());
    if(seen_mandatory != expected) throw std::runtime_error("mandatory coverage mismatch");
    std::set<int> inserted(result.selected_optional.begin(), result.selected_optional.end());
    std::set<int> remaining(result.unselected_optional.begin(), result.unselected_optional.end());
    if(inserted.size()!=result.selected_optional.size()||remaining.size()!=result.unselected_optional.size())
        throw std::runtime_error("duplicate optional partition item");
    for(int id: inserted) if(remaining.count(id)) throw std::runtime_error("inserted and remaining overlap");
    std::set<int> sec; for(auto& it:optional_items) sec.insert(it.item_id); std::set<int> unioned=inserted; unioned.insert(remaining.begin(), remaining.end()); if(unioned != sec) throw std::runtime_error("optional partition mismatch");
    std::set<int> physical_mandatory,physical_optional,physical_seen;
    for(std::size_t k=0;k<result.final_stacks.size();++k){
        const auto& st=result.final_stacks[k];
        if(st.empty())throw std::runtime_error("empty baseline stack");
        int h=0;
        std::vector<int> assigned_mandatory;
        for(int id:st) {
            if(!physical_seen.insert(id).second)throw std::runtime_error("duplicate baseline physical item");
            h += item_map.at(id).height;
            if(mandatory_ids.count(id)) {
                physical_mandatory.insert(id);
                assigned_mandatory.push_back(id);
            } else physical_optional.insert(id);
        }
        auto reported=result.mandatory_assignments[k];
        std::sort(assigned_mandatory.begin(),assigned_mandatory.end());
        std::sort(reported.begin(),reported.end());
        if(assigned_mandatory!=reported)throw std::runtime_error("baseline mandatory assignment mismatch");
        if(h>capacity) throw std::runtime_error("stack height violation");
        for(std::size_t i=0;i<st.size();++i) for(std::size_t j=i+1;j<st.size();++j) {
            if(!stack_compatible(item_map.at(st[i]), item_map.at(st[j]))) throw std::runtime_error("q_mb compatibility violation");
        }
    }
    if(physical_mandatory!=mandatory_ids||physical_optional!=inserted)
        throw std::runtime_error("baseline physical coverage mismatch");
    std::int64_t manual=0; for(int id:inserted) manual += item_map[id].score; if(manual != result.baseline_optional_profit) throw std::runtime_error("inserted score mismatch");
}

void validate_value_result(const ValueFunctionResult& result, const std::vector<OptionalRack>& items)
{
    auto m = by_id(items);
    std::vector<int> seen;
    for(size_t k=0;k<result.stacks.size();++k){
        int h=0;
        for(int id:result.stacks[k]){ h += m.at(id).height; seen.push_back(id);}
        for(std::size_t i=0;i<result.stacks[k].size();++i) for(std::size_t j=i+1;j<result.stacks[k].size();++j) {
            if(!stack_compatible(m.at(result.stacks[k][i]),m.at(result.stacks[k][j]))) throw std::runtime_error("value q_mb compatibility violation");
        }
        if(k<result.stack_heights.size() && h != result.stack_heights[k]) throw std::runtime_error("reported stack height inconsistent");
        if(h > result.stack_capacity) throw std::runtime_error("stack capacity violation");
    }
    std::set<int> unique(seen.begin(), seen.end()); if(unique.size() != seen.size()) throw std::runtime_error("optional assigned more than once");
    if(result.objective){ std::int64_t score=0; for(int id:seen) score += std::max<std::int64_t>(0, m[id].score); if(score != *result.objective) throw std::runtime_error("value objective mismatch"); }
}

void validate_monotone_value_tables(const PreprocessAllGroupsResult& result)
{
    for(const auto& kv:result.groups){
        std::optional<std::int64_t> last;
        for(const auto& rv:kv.second.value_table){
            if(!rv.second.objective) continue;
            if(last && *rv.second.objective < *last) throw std::runtime_error("non-monotone value table");
            last = rv.second.objective;
        }
        last.reset();
        int expected_r=0;
        for(const auto& [r,point]:kv.second.spread_joint_curve) {
            if(r!=expected_r++ || point.extra_stack_count!=r ||
               point.stack_count!=kv.second.baseline_result.baseline_stack_count+r)
                throw std::runtime_error("Spread-Joint curve has missing or inconsistent exact-count point");
            if(last && point.optional_profit<*last)
                throw std::runtime_error("non-monotone Spread-Joint physical curve");
            last=point.optional_profit;
        }
    }
}

}
}
