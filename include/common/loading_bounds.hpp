#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace hrlh::search_dff {

struct Rational {
    std::int64_t numerator = 0;
    std::int64_t denominator = 1;
};

struct Epsilon {
    int numerator = 0;
    int denominator = 1;
};

enum class FunctionKind {
    Identity,
    U1,
    Upsilon,
    Phi,
};

struct FunctionSpec {
    FunctionKind kind = FunctionKind::Identity;
    int epsilon_index = -1;
};

struct SupplierRectangles {
    std::vector<int> suppliers;
    std::map<int, std::vector<std::pair<int, int>>> rectangles_by_supplier;
};

struct CapacityProfile {
    std::string label;
    std::int64_t capacity = 0;
    std::map<int, std::int64_t> demand_by_supplier;
};

struct ProfileBuildResult {
    std::vector<CapacityProfile> raw_profiles;
    std::vector<CapacityProfile> surviving_profiles;
};

const std::vector<Epsilon>& fixed_epsilons();

Rational evaluate_function(FunctionSpec function, int side, int extent);

ProfileBuildResult build_fixed_profiles(
    int vehicle_width,
    int vehicle_length,
    const SupplierRectangles& input,
    bool protected_area_capacity);

}
