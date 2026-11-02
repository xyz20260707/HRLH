#pragma once
#include <hrlh.hpp>
namespace hrlh::independent {
struct Report {std::vector<std::string> errors;double C=0,F=0;std::int64_t P=0;
    bool ok()const{return errors.empty();}};

Report check(const hrlh_model::ProblemInstance&,const CandidateSolution&);
}
