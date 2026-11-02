#ifndef HRLH_ILS_SEARCH_SOLVER_PORT_H
#define HRLH_ILS_SEARCH_SOLVER_PORT_H

#include "Solution.h"
#include "case_protocol.h"

#include <cstdint>
#include <functional>
#include <string>

namespace hrlh_ils_search
{
struct SolveResult
{
    pyvrp::Solution best;

    explicit SolveResult(pyvrp::Solution solution) : best(std::move(solution)) {}
};

using CandidateFeedbackCallback = std::function<bool(pyvrp::Solution const &)>;
using StopCallback = std::function<void()>;

SolveResult solveWithFeedback(pyvrp::ProblemData const &data,
                              SolverParamsSpec const &params,
                              CandidateFeedbackCallback feedbackCallback,
                              StopCallback stopCallback);

}

#endif
