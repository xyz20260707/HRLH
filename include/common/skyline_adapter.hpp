#pragma once
#include "skyline.hpp"
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <numeric>
#include <functional>

struct PackingTypeSpecCpp {
    std::string type_id;
    int width = 0;
    int length = 0;
    int mandatory_count = 0;
    int optional_count = 0;
    std::vector<long long> optional_marginal_profits;
    long long fixed_inserted_score = 0;
};

struct RoutePlacedRectCpp {
    int type_index = -1;
    bool is_optional = false;
    int rank_in_type = -1;
    int x = 0;
    int y = 0;
    int width = 0;
    int length = 0;
};

struct RouteEvalOutCpp {
    bool feasible = false;
    int bin_number = 0;
    long long optional_score_from_2d = 0;
    long long fixed_inserted_score = 0;
    long long total_optional_priority_score = 0;
    bool static_lb_infeasible = false;
    std::string static_lb_reason;
    std::vector<int> loaded_mandatory_count_by_type;
    std::vector<int> loaded_optional_count_by_type;
    std::vector<RoutePlacedRectCpp> placed_rects;
};

struct RoutePackingInput {
    std::vector<PackingTypeSpecCpp> specs;
    long long fixed_inserted_score = 0;
};

struct RouteEvalFullOutCpp {
    RouteEvalOutCpp scalar;
};

struct MandatoryStaticLbCheckCpp {
    bool proven_infeasible = false;
    bool preprocessing_error = false;
    std::string reason;
};

inline void validate_packing_type_spec_cpp(const PackingTypeSpecCpp& s) {
    if (s.width <= 0 || s.length <= 0) {
        throw std::runtime_error("Type " + s.type_id + " has non-positive width/length.");
    }
    if (s.mandatory_count < 0 || s.optional_count < 0) {
        throw std::runtime_error("Type " + s.type_id + " has negative counts.");
    }
    if ((int)s.optional_marginal_profits.size() != s.optional_count) {
        throw std::runtime_error("Type " + s.type_id + ": optional_marginal_profits length mismatch.");
    }
}

inline std::vector<long long> f_values_to_marginal_profits_cpp(const std::map<int,long long>& f_values) {

    if (f_values.empty()) return {};
    if (f_values.begin()->first != 0) throw std::runtime_error("f_values must contain f(0).");
    std::vector<long long> marginal;
    long long last = f_values.begin()->second;
    for (auto it = std::next(f_values.begin()); it != f_values.end(); ++it) {
        long long val = it->second;
        if (val < last) throw std::runtime_error("f_values must be nondecreasing.");
        marginal.push_back(val - last);
        last = val;
    }
    return marginal;
}

struct GroupPreprocessResultCpp {
    std::string group_key;
    int base_width = 0;
    int base_length = 0;
    int baseline_stack_count = 0;
    int R_max_g = 0;
    std::map<int,long long> f_values;
    long long baseline_optional_profit = 0;
};

inline PackingTypeSpecCpp make_packing_type_from_preprocess_group_cpp(const GroupPreprocessResultCpp& group) {
    auto marginal = f_values_to_marginal_profits_cpp(group.f_values);
    if ((int)marginal.size() != group.R_max_g) {
        throw std::runtime_error("R_max_g does not match f_values marginal length.");
    }
    PackingTypeSpecCpp out{
        group.group_key,
        group.base_width,
        group.base_length,
        group.baseline_stack_count,
        group.R_max_g,
        marginal,
        group.baseline_optional_profit
    };
    validate_packing_type_spec_cpp(out);
    return out;
}

inline PackingTypeSpecCpp make_pallet_type_spec_cpp(const std::string& type_id, int width, int length, int count) {
    PackingTypeSpecCpp out{type_id, width, length, count, 0, {}, 0};
    validate_packing_type_spec_cpp(out);
    return out;
}

inline RoutePackingInput build_route_packing_input(const std::vector<PackingTypeSpecCpp>& specs) {
    std::set<std::string> seen;
    std::vector<std::string> ids;
    for (const auto& s : specs) {
        validate_packing_type_spec_cpp(s);
        ids.push_back(s.type_id);
        if (!seen.insert(s.type_id).second) {
            std::stringstream ss;
            ss << "Duplicate type_id values: [";
            for (size_t i=0; i<ids.size(); ++i) { if (i) ss << ", "; ss << ids[i]; }
            ss << "]";
            throw std::runtime_error(ss.str());
        }
    }
    RoutePackingInput d;
    d.specs = specs;
    for (const auto& s : specs) {
        d.fixed_inserted_score += s.fixed_inserted_score;
    }
    return d;
}

inline void specs_to_core_types_cpp(const std::vector<PackingTypeSpecCpp>& specs,
                                    std::vector<TypeRec>& types) {
    types.clear();
    types.reserve(specs.size());
    for (const auto& s : specs) {
        TypeRec t;
        t.w = s.width;
        t.l = s.length;
        t.d1 = s.mandatory_count;
        t.d2 = s.optional_count;
        t.profits = s.optional_marginal_profits;
        types.push_back(t);
    }
}

inline MandatoryStaticLbCheckCpp mandatory_static_lb_check_cpp(
    const std::vector<PackingTypeSpecCpp>& specs,
    int vehicle_width,
    int vehicle_length) {
    MandatoryStaticLbCheckCpp out;
    std::vector<Item> mandatory_items;
    int next_id = 1;
    for (const auto& spec : specs) {
        validate_packing_type_spec_cpp(spec);
        for (int copy = 0; copy < spec.mandatory_count; ++copy) {
            Item item;
            item.id = next_id++;
            item.length = spec.width;
            item.width = spec.length;
            item.min_dim = std::min(item.length, item.width);
            mandatory_items.push_back(item);
        }
    }
    std::string reason;
    const bool rejected = mandatory_lb_infeasible_cpp(
        mandatory_items, vehicle_width, vehicle_length, &reason);
    out.reason = reason;
    out.preprocessing_error = rejected && reason == "lower bound preprocessing exception";
    out.proven_infeasible = rejected && !out.preprocessing_error;
    return out;
}

inline void copy_eval_layout_to_api(RouteEvalOutCpp& out, const EvalRes& er) {
    out.loaded_mandatory_count_by_type = er.loaded_mandatory_count_by_type;
    out.loaded_optional_count_by_type = er.loaded_optional_count_by_type;
    out.placed_rects.clear();
    out.placed_rects.reserve(er.placed_rects.size());
    for (const auto& pr : er.placed_rects) {
        RoutePlacedRectCpp r;
        r.type_index = pr.type_idx;
        r.is_optional = pr.is_optional;
        r.rank_in_type = pr.rank_in_type;
        r.x = pr.x;
        r.y = pr.y;
        r.width = pr.w;
        r.length = pr.h;
        out.placed_rects.push_back(r);
    }
}

inline bool rect_fits_floor_cpp(int w, int l, int W, int H) {
    return (w <= W && l <= H) || (l <= W && w <= H);
}

template <class RNG>
inline RouteEvalOutCpp evaluate_route_packing_input(
    const RoutePackingInput& bd,
    int W,
    int H,
    int mutation_iters,
    double mutation_swap_count,
    bool add_fixed_inserted_score,
    RNG& rng,
    const std::function<void()>& stop_callback = {},
    const SkylineEvalOptions& options = {}
) {
    SkylineStopCallbackScope skyline_stop_scope(stop_callback);
    if (stop_callback) stop_callback();
    std::vector<TypeRec> types;
    specs_to_core_types_cpp(bd.specs, types);

    RouteEvalOutCpp out;
    long long fixed_total = bd.fixed_inserted_score;
    out.fixed_inserted_score = fixed_total;

    bool has_mand = false;
    for (const auto& t : types) if (t.d1 > 0) { has_mand = true; break; }

    if (!has_mand) {
        std::vector<Item> merged_items;
        std::vector<int> id_type(1, -1);
        std::vector<int> active_type_idx;
        std::vector<char> active_type(types.size(), 0);
        int iid = 1;
        long long full_optional_area = 0;
        for (int ti = 0; ti < (int)types.size(); ++ti) {
            const auto& t = types[ti];
            for (int k = 0; k < t.d2; ++k) {
                full_optional_area += 1LL * t.w * t.l;
                if (!rect_fits_floor_cpp(t.w, t.l, W, H)) { ++iid; continue; }
                Item it; it.id = iid; it.length = t.w; it.width = t.l; it.min_dim = std::min(it.length, it.width);
                merged_items.push_back(it);
                if ((int)id_type.size() <= iid) id_type.resize(iid + 1, -1);
                id_type[iid] = ti;
                if (!active_type[ti]) { active_type[ti] = 1; active_type_idx.push_back(ti); }
                ++iid;
            }
        }
        if (merged_items.empty()) {
            out.feasible = true; out.bin_number = 0; out.optional_score_from_2d = 0;
            out.total_optional_priority_score = add_fixed_inserted_score ? fixed_total : 0;
            return out;
        }
        auto er = solve_skyline_packing_cpp(types, active_type_idx, merged_items, id_type,
                                            W, H, full_optional_area, mutation_iters,
                                            mutation_swap_count, rng, stop_callback);
        out.feasible = er.feasible;
        out.bin_number = 1;
        out.optional_score_from_2d = er.score;
        out.total_optional_priority_score = er.score + ((add_fixed_inserted_score && er.feasible) ? fixed_total : 0);
        copy_eval_layout_to_api(out, er);
        return out;
    }

    std::vector<int> type_idx(types.size());
    std::iota(type_idx.begin(), type_idx.end(), 0);
    auto er = evaluate_two_stage_skyline(types, type_idx, W, H, mutation_iters, mutation_swap_count, rng, stop_callback, options);
    out.feasible = er.feasible;
    out.bin_number = er.bin_count;
    out.optional_score_from_2d = er.score;
    out.total_optional_priority_score = er.score + ((add_fixed_inserted_score && er.feasible) ? fixed_total : 0);
    out.static_lb_infeasible = er.static_lb_infeasible;
    out.static_lb_reason = er.static_lb_reason;
    copy_eval_layout_to_api(out, er);
    return out;
}

template <class RNG>
inline RouteEvalFullOutCpp evaluate_single_route_mandatory_packing_cpp_api_full(
    const std::vector<PackingTypeSpecCpp>& specs,
    int W,
    int H,
    int mutation_iters,
    double mutation_swap_count,
    RNG& rng,
    const std::function<void()>& stop_callback = {},
    const SkylineEvalOptions& options = {}
) {
    RouteEvalFullOutCpp out;
    SkylineStopCallbackScope skyline_stop_scope(stop_callback);
    if (stop_callback) stop_callback();

    std::set<std::string> seen;
    std::vector<std::string> ids;
    std::vector<TypeRec> types;
    types.reserve(specs.size());
    for (const auto& s : specs) {
        validate_packing_type_spec_cpp(s);
        ids.push_back(s.type_id);
        if (!seen.insert(s.type_id).second) {
            std::stringstream ss;
            ss << "Duplicate type_id values: [";
            for (size_t i = 0; i < ids.size(); ++i) {
                if (i) ss << ", ";
                ss << ids[i];
            }
            ss << "]";
            throw std::runtime_error(ss.str());
        }
        TypeRec t;
        t.w = s.width;
        t.l = s.length;
        t.d1 = s.mandatory_count;
        t.d2 = 0;
        types.push_back(std::move(t));
    }

    std::vector<int> type_idx;
    for (int ti = 0; ti < static_cast<int>(types.size()); ++ti) {
        if (types[ti].d1 > 0) type_idx.push_back(ti);
    }

    auto er = eval_bin_mandatory_only(
        types, type_idx, W, H, mutation_iters, mutation_swap_count, rng, stop_callback, options);
    out.scalar.feasible = er.feasible;
    out.scalar.bin_number = er.bin_count;
    out.scalar.optional_score_from_2d = 0;
    out.scalar.fixed_inserted_score = 0;
    out.scalar.total_optional_priority_score = 0;
    out.scalar.static_lb_infeasible = er.static_lb_infeasible;
    out.scalar.static_lb_reason = er.static_lb_reason;
    copy_eval_layout_to_api(out.scalar, er);
    return out;
}

template <class RNG>
inline RouteEvalFullOutCpp evaluate_single_route_priority_packing_cpp_api_full(
    const std::vector<PackingTypeSpecCpp>& specs,
    int W,
    int H,
    int mutation_iters,
    double mutation_swap_count,
    bool add_fixed_inserted_score,
    RNG& rng,
    const std::function<void()>& stop_callback = {},
    const SkylineEvalOptions& options = {}
) {
    RouteEvalFullOutCpp out;
    if (stop_callback) stop_callback();
    const auto input = build_route_packing_input(specs);
    out.scalar = evaluate_route_packing_input(
        input, W, H, mutation_iters, mutation_swap_count, add_fixed_inserted_score, rng, stop_callback, options);
    return out;
}
