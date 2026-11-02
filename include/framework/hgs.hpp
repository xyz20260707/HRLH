#pragma once
#include "routing_problem.hpp"
#include <limits>
namespace hrlh::hgs {

struct RunControl {long long iterations=1000000000LL;
    int initial_population=100;int no_improvement_restart=20000;};
void run(const routing::Problem&,const RoutePlan&,const std::vector<RoutePlan>&,
         const routing::SearchCallbacks&,std::uint64_t,
         const RunControl& = {});
}
