#pragma once
#include "hrlh.hpp"
#include <memory>

namespace hrlh {

struct OptionalDemandProfitContext {
    struct Impl;
    std::shared_ptr<Impl> state;
};

void intensify_optional_demand_profit(const hrlh_model::ProblemInstance& instance,
                                  const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& preprocess,
                                  CandidateSolution& solution,
                                  const HrlhConfig& config,
                                  OptionalDemandProfitContext& context,
                                  double available_seconds);
}
