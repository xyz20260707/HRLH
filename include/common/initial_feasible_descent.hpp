#pragma once
#include "hrlh.hpp"
#include <algorithm>
#include <vector>

namespace hrlh {

struct DescentPackingChecks {
    RoutePlan routes_to_check;
};

inline DescentPackingChecks descent_packing_checks(
    const RoutePlan& candidate, const RoutePlan& accepted_anchor) {
    std::vector<Route> feasible_sets;
    feasible_sets.reserve(accepted_anchor.size());
    for (auto route : accepted_anchor) {
        if (route.empty()) continue;
        std::sort(route.begin(), route.end());
        route.erase(std::unique(route.begin(), route.end()), route.end());
        feasible_sets.push_back(std::move(route));
    }
    DescentPackingChecks checks;
    for (const auto& route : candidate) {
        if (route.empty()) continue;
        Route suppliers = route;
        std::sort(suppliers.begin(), suppliers.end());
        suppliers.erase(std::unique(suppliers.begin(), suppliers.end()), suppliers.end());
        bool inherited = false;
        for (const auto& feasible : feasible_sets) {
            if (std::includes(feasible.begin(), feasible.end(),
                              suppliers.begin(), suppliers.end())) {
                inherited = true;
                break;
            }
        }
        if (!inherited) checks.routes_to_check.push_back(route);
    }
    return checks;
}

}
