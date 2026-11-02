#pragma once
#include "routing_problem.hpp"
#include <memory>

namespace hrlh {

struct FrozenCommonStart;
using BackendSearch = std::function<void(
    const routing::Problem&, const RoutePlan&, const std::vector<RoutePlan>&,
    const routing::SearchCallbacks&, std::uint64_t)>;
struct BackendRunControl {
    std::string algorithm = "ILS";
    BackendSearch search;
};
std::shared_ptr<const FrozenCommonStart> freeze_common_start(
    const hrlh_model::ProblemInstance&, const HrlhConfig&);
HrlhResult resume_common_start(
    const std::shared_ptr<const FrozenCommonStart>&, const BackendRunControl&);

}
