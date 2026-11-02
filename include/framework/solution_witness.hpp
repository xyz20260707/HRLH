#pragma once
#include <hrlh.hpp>
#include <iomanip>
#include <sstream>
namespace hrlh {
inline std::string solution_witness_json(const CandidateSolution& s){
    std::ostringstream o;o<<std::setprecision(17)<<"{\"C\":"<<s.transport_cost<<",\"P\":"<<s.optional_profit<<",\"F\":"<<s.latest_departure_sum<<",\"routes\":[";
    for(std::size_t r=0;r<s.routes.size();++r){if(r)o<<',';o<<'[';for(std::size_t k=0;k<s.routes[r].size();++k){if(k)o<<',';o<<s.routes[r][k];}o<<']';}
    o<<"],\"route_departures\":[";for(std::size_t k=0;k<s.route_latest_departures.size();++k){if(k)o<<',';o<<s.route_latest_departures[k];}
    o<<"],\"packing\":[";
    for(std::size_t ri=0;ri<s.packing.route_details.size();++ri){if(ri)o<<',';const auto& d=s.packing.route_details[ri];
        o<<"{\"P\":"<<d.total_optional_priority_score<<",\"rectangles\":[";
        for(std::size_t k=0;k<d.placed_rects.size();++k){if(k)o<<',';const auto& x=d.placed_rects[k];o<<'['<<x.spec_index<<','<<x.is_optional<<','<<x.rank_in_type<<','<<x.x<<','<<x.y<<','<<x.width<<','<<x.length<<']';}
        o<<"],\"stacks_by_spec\":[";
        for(std::size_t si=0;si<d.material_stacks_by_spec.size();++si){if(si)o<<',';o<<'[';
            for(std::size_t j=0;j<d.material_stacks_by_spec[si].size();++j){if(j)o<<',';o<<'[';const auto& stack=d.material_stacks_by_spec[si][j];
                for(std::size_t k=0;k<stack.size();++k){if(k)o<<',';o<<stack[k];}o<<']';}o<<']';}
        o<<"]}";}
    o<<"]}";return o.str();
}
}
