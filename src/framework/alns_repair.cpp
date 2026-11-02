#include "alns.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace hrlh::alns {
State repair(const routing::Problem& problem,State state,Repair op,
             const routing::SearchCallbacks& callbacks) {
    validate_state(problem,state);
    if(op!=Repair::Greedy&&op!=Repair::Regret2&&op!=Repair::Regret3)
        throw std::invalid_argument("unknown ALNS repair operator");
    const int k=op==Repair::Regret3?3:op==Repair::Regret2?2:1;
    struct Option {double delta;std::size_t route;Route positioned;};
    while(!state.missing.empty()) {
        if(callbacks.stop)callbacks.stop();
        bool found=false;
        int selected_supplier=0;
        Option selected{};
        double best_regret=-std::numeric_limits<double>::infinity();
        for(int supplier:state.missing) {
            std::vector<Option> options;
            for(std::size_t ri=0;ri<=state.routes.size();++ri) {
                if(callbacks.stop)callbacks.stop();
                const Route before=ri==state.routes.size()?Route{}:state.routes[ri];
                const double previous=problem.evaluate_route(before).cost;
                Option best{std::numeric_limits<double>::infinity(),ri,{}};
                for(std::size_t pos=0;pos<=before.size();++pos) {
                    Route after=before;after.insert(after.begin()+pos,supplier);
                    const auto value=problem.evaluate_route(after);
                    if(!value.feasible())continue;
                    const double delta=value.cost-previous;
                    if(delta<best.delta)best={delta,ri,std::move(after)};
                }
                if(best.positioned.empty())continue;

                if(callbacks.evaluate_partial_route
                    &&callbacks.evaluate_partial_route(best.positioned)!=routing::PackingAssessment::Feasible)
                    continue;
                options.push_back(std::move(best));
            }
            if(options.empty())continue;
            std::sort(options.begin(),options.end(),[](const auto& a,const auto& b) {
                if(a.delta!=b.delta)return a.delta<b.delta;
                return a.route<b.route;
            });
            double regret=0;
            if(k>1) {
                if(options.size()<static_cast<std::size_t>(k))regret=std::numeric_limits<double>::infinity();
                else for(int rank=1;rank<k;++rank)regret+=options[rank].delta-options[0].delta;
            }
            if(!found||(k==1?options[0].delta<selected.delta
                    :regret>best_regret||(regret==best_regret&&options[0].delta<selected.delta))) {
                found=true;selected_supplier=supplier;selected=std::move(options[0]);best_regret=regret;
            }
        }
        if(!found)break;
        if(selected.route==state.routes.size())state.routes.push_back(std::move(selected.positioned));
        else state.routes[selected.route]=std::move(selected.positioned);
        state.missing.erase(std::find(state.missing.begin(),state.missing.end(),selected_supplier));
    }
    validate_state(problem,state);
    return state;
}
}
