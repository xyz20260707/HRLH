#include "loading_bounds.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

#ifdef _MSC_VER
#include <intrin.h>
#endif

namespace hrlh::search_dff {
namespace {

using u64 = std::uint64_t;

struct Product128 {
    u64 high = 0;
    u64 low = 0;
};

Product128 exact_product(u64 lhs, u64 rhs) {
#ifdef _MSC_VER
    Product128 out;
    out.low = _umul128(lhs, rhs, &out.high);
    return out;
#else
    const unsigned __int128 product =
        static_cast<unsigned __int128>(lhs) * static_cast<unsigned __int128>(rhs);
    return {static_cast<u64>(product >> 64), static_cast<u64>(product)};
#endif
}

int compare_products(u64 a, u64 b, u64 c, u64 d) {
    const Product128 lhs = exact_product(a, b);
    const Product128 rhs = exact_product(c, d);
    if (lhs.high != rhs.high) return lhs.high < rhs.high ? -1 : 1;
    if (lhs.low != rhs.low) return lhs.low < rhs.low ? -1 : 1;
    return 0;
}

std::int64_t checked_mul(std::int64_t lhs, std::int64_t rhs) {
    if (lhs < 0 || rhs < 0) throw std::invalid_argument("DFF values must be nonnegative");
    const Product128 product = exact_product(static_cast<u64>(lhs), static_cast<u64>(rhs));
    if (product.high != 0
        || product.low > static_cast<u64>(std::numeric_limits<std::int64_t>::max())) {
        throw std::overflow_error("search DFF integer scaling overflow");
    }
    return static_cast<std::int64_t>(product.low);
}

std::int64_t checked_add(std::int64_t lhs, std::int64_t rhs) {
    if (lhs < 0 || rhs < 0
        || lhs > std::numeric_limits<std::int64_t>::max() - rhs) {
        throw std::overflow_error("search DFF demand aggregation overflow");
    }
    return lhs + rhs;
}

const Epsilon& epsilon_for(const FunctionSpec& function) {
    const auto& values = fixed_epsilons();
    if (function.epsilon_index < 0
        || function.epsilon_index >= static_cast<int>(values.size())) {
        throw std::invalid_argument("DFF epsilon index is out of range");
    }
    return values[static_cast<std::size_t>(function.epsilon_index)];
}

std::int64_t function_denominator(FunctionSpec function, int extent) {
    switch (function.kind) {
        case FunctionKind::Identity:
        case FunctionKind::Upsilon:
            return extent;
        case FunctionKind::U1:
            return 2;
        case FunctionKind::Phi: {
            const auto& epsilon = epsilon_for(function);
            const int m = epsilon.denominator / epsilon.numerator;
            return 2LL * m;
        }
    }
    throw std::invalid_argument("unknown DFF function kind");
}

std::string function_name(FunctionSpec function) {
    switch (function.kind) {
        case FunctionKind::Identity: return "I";
        case FunctionKind::U1: return "u1";
        case FunctionKind::Upsilon:
            return "U" + std::to_string(function.epsilon_index);
        case FunctionKind::Phi:
            return "phi" + std::to_string(function.epsilon_index);
    }
    return "unknown";
}

struct CandidateSpec {
    FunctionSpec first;
    FunctionSpec second;
    std::string label;
};

std::vector<CandidateSpec> fixed_candidate_specs() {
    std::vector<CandidateSpec> specs;
    specs.reserve(27);
    const FunctionSpec identity{FunctionKind::Identity, -1};
    const FunctionSpec u1{FunctionKind::U1, -1};
    for (int p = 0; p < 3; ++p) {
        const FunctionSpec up{FunctionKind::Upsilon, p};
        const FunctionSpec phip{FunctionKind::Phi, p};
        for (const auto& pair : std::vector<std::pair<FunctionSpec, FunctionSpec>>{
                 {u1, up}, {up, u1}, {u1, phip}, {phip, u1},
                 {identity, up}, {up, identity}}) {
            specs.push_back({pair.first, pair.second,
                function_name(pair.first) + "x" + function_name(pair.second)});
        }
    }
    for (int p = 0; p < 3; ++p) {
        for (int q = 0; q < 3; ++q) {
            const FunctionSpec first{FunctionKind::Phi, p};
            const FunctionSpec second{FunctionKind::Phi, q};
            specs.push_back({first, second,
                function_name(first) + "x" + function_name(second)});
        }
    }
    if (specs.size() != 27) throw std::logic_error("fixed search DFF set is not 27");
    return specs;
}

void normalize_profile(CapacityProfile& profile, const std::vector<int>& suppliers) {
    std::int64_t divisor = profile.capacity;
    for (int supplier : suppliers) {
        divisor = std::gcd(divisor, profile.demand_by_supplier.at(supplier));
    }
    if (divisor <= 1) return;
    profile.capacity /= divisor;
    for (int supplier : suppliers) profile.demand_by_supplier[supplier] /= divisor;
}

std::int64_t orientation_numerator(const CandidateSpec& candidate,
                                   int first_side,
                                   int first_extent,
                                   int second_side,
                                   int second_extent) {
    const Rational first = evaluate_function(candidate.first, first_side, first_extent);
    const Rational second = evaluate_function(candidate.second, second_side, second_extent);
    return checked_mul(first.numerator, second.numerator);
}

CapacityProfile build_candidate_profile(const CandidateSpec& candidate,
                                        int vehicle_width,
                                        int vehicle_length,
                                        const SupplierRectangles& input) {
    CapacityProfile profile;
    profile.label = candidate.label;
    profile.capacity = checked_mul(
        function_denominator(candidate.first, vehicle_width),
        function_denominator(candidate.second, vehicle_length));
    for (int supplier : input.suppliers) {
        std::int64_t demand = 0;
        const auto found = input.rectangles_by_supplier.find(supplier);
        if (found != input.rectangles_by_supplier.end()) {
            for (const auto& [width, length] : found->second) {
                if (width <= 0 || length <= 0) {
                    throw std::invalid_argument("mandatory rectangle has non-positive side");
                }
                const bool original_valid = width <= vehicle_width && length <= vehicle_length;
                const bool rotated_valid = length <= vehicle_width && width <= vehicle_length;
                if (!original_valid && !rotated_valid) {
                    demand = checked_add(demand, checked_add(profile.capacity, 1));
                    continue;
                }
                std::int64_t coefficient = std::numeric_limits<std::int64_t>::max();
                if (original_valid) {
                    coefficient = orientation_numerator(candidate,
                        width, vehicle_width, length, vehicle_length);
                }
                if (rotated_valid) {
                    coefficient = std::min(coefficient,
                        orientation_numerator(candidate,
                            length, vehicle_width, width, vehicle_length));
                }
                demand = checked_add(demand, coefficient);
            }
        }
        profile.demand_by_supplier[supplier] = demand;
    }
    normalize_profile(profile, input.suppliers);
    return profile;
}

bool is_nonbinding(const CapacityProfile& profile, const std::vector<int>& suppliers) {
    std::int64_t total = 0;
    for (int supplier : suppliers) {
        const std::int64_t demand = profile.demand_by_supplier.at(supplier);
        if (demand > profile.capacity - total) return false;
        total += demand;
    }
    return true;
}

bool is_zero_information(const CapacityProfile& profile, const std::vector<int>& suppliers) {
    return std::all_of(suppliers.begin(), suppliers.end(), [&](int supplier) {
        return profile.demand_by_supplier.at(supplier) == 0;
    });
}

std::string canonical_key(const CapacityProfile& profile, const std::vector<int>& suppliers) {
    std::ostringstream key;
    key << profile.capacity;
    for (int supplier : suppliers) key << ':' << profile.demand_by_supplier.at(supplier);
    return key.str();
}

bool dominates(const CapacityProfile& dominator,
               const CapacityProfile& dominated,
               const std::vector<int>& suppliers) {
    bool strict = false;
    for (int supplier : suppliers) {
        const auto lhs = static_cast<u64>(dominator.demand_by_supplier.at(supplier));
        const auto rhs = static_cast<u64>(dominated.demand_by_supplier.at(supplier));
        const int comparison = compare_products(
            lhs, static_cast<u64>(dominated.capacity),
            rhs, static_cast<u64>(dominator.capacity));
        if (comparison < 0) return false;
        if (comparison > 0) strict = true;
    }
    return strict;
}

CapacityProfile build_area_profile(int vehicle_width,
                                   int vehicle_length,
                                   const SupplierRectangles& input) {
    CapacityProfile area;
    area.label = "protected_area";
    area.capacity = checked_mul(vehicle_width, vehicle_length);
    for (int supplier : input.suppliers) {
        std::int64_t demand = 0;
        const auto found = input.rectangles_by_supplier.find(supplier);
        if (found != input.rectangles_by_supplier.end()) {
            for (const auto& [width, length] : found->second) {
                demand = checked_add(demand, checked_mul(width, length));
            }
        }
        area.demand_by_supplier[supplier] = demand;
    }
    normalize_profile(area, input.suppliers);
    return area;
}

}

const std::vector<Epsilon>& fixed_epsilons() {
    static const std::vector<Epsilon> values{{3, 20}, {3, 10}, {9, 20}};
    return values;
}

Rational evaluate_function(FunctionSpec function, int side, int extent) {
    if (extent <= 0 || side < 0 || side > extent) {
        throw std::invalid_argument("DFF argument must satisfy 0 <= side <= extent");
    }
    if (function.kind == FunctionKind::Identity) return {side, extent};
    if (function.kind == FunctionKind::U1) {
        const std::int64_t twice_side = 2LL * side;
        if (twice_side < extent) return {0, 2};
        if (twice_side == extent) return {1, 2};
        return {2, 2};
    }

    const Epsilon& epsilon = epsilon_for(function);
    if (function.kind == FunctionKind::Upsilon) {
        const std::int64_t scaled_side = 1LL * side * epsilon.denominator;
        const std::int64_t lower = 1LL * extent * epsilon.numerator;
        const std::int64_t upper = 1LL * extent
            * (epsilon.denominator - epsilon.numerator);
        if (scaled_side > upper) return {extent, extent};
        if (scaled_side >= lower) return {side, extent};
        return {0, extent};
    }

    if (function.kind == FunctionKind::Phi) {
        const int m = epsilon.denominator / epsilon.numerator;
        const std::int64_t denominator = 2LL * m;
        const std::int64_t twice_side = 2LL * side;
        if (twice_side == extent) return {m, denominator};
        if (twice_side < extent) {
            const std::int64_t level =
                (1LL * side * epsilon.denominator)
                / (1LL * extent * epsilon.numerator);
            return {2LL * level, denominator};
        }
        const std::int64_t level =
            (1LL * (extent - side) * epsilon.denominator)
            / (1LL * extent * epsilon.numerator);
        return {2LL * (m - level), denominator};
    }
    throw std::invalid_argument("unknown DFF function kind");
}

ProfileBuildResult build_fixed_profiles(
    int vehicle_width,
    int vehicle_length,
    const SupplierRectangles& input,
    bool protected_area_capacity) {
    if (vehicle_width <= 0 || vehicle_length <= 0) {
        throw std::invalid_argument("vehicle floor dimensions must be positive");
    }
    ProfileBuildResult result;
    const auto candidates = fixed_candidate_specs();
    result.raw_profiles.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        result.raw_profiles.push_back(build_candidate_profile(
            candidate, vehicle_width, vehicle_length, input));
    }

    std::vector<CapacityProfile> filtered;
    for (const auto& profile : result.raw_profiles) {
        if (is_nonbinding(profile, input.suppliers)) continue;
        if (is_zero_information(profile, input.suppliers)) continue;
        filtered.push_back(profile);
    }

    std::set<std::string> duplicate_keys;
    std::vector<CapacityProfile> unique;
    for (const auto& profile : filtered) {
        if (duplicate_keys.insert(canonical_key(profile, input.suppliers)).second) {
            unique.push_back(profile);
        }
    }

    const CapacityProfile area = build_area_profile(
        vehicle_width, vehicle_length, input);
    std::vector<bool> dominated(unique.size(), false);
    for (std::size_t p = 0; p < unique.size(); ++p) {
        if (protected_area_capacity && dominates(area, unique[p], input.suppliers)) {
            dominated[p] = true;
            continue;
        }
        for (std::size_t q = 0; q < unique.size(); ++q) {
            if (p != q && dominates(unique[q], unique[p], input.suppliers)) {
                dominated[p] = true;
                break;
            }
        }
    }
    for (std::size_t i = 0; i < unique.size(); ++i) {
        if (!dominated[i]) result.surviving_profiles.push_back(std::move(unique[i]));
    }
    return result;
}

}
