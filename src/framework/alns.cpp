#include "alns.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>

namespace hrlh::alns {
namespace {
template<std::size_t N>std::size_t roulette(const std::array<double,N>& w,std::mt19937_64& rng) {
    double total=std::accumulate(w.begin(),w.end(),0.0);
    if(total==0)return routing::uniform_index(rng,N);
    double x=routing::uniform01(rng)*total;
    for(std::size_t i=0;i<N;++i)if(x<w[i])return i;else x-=w[i];
    return N-1;
}
std::string signature(const State& state) {
    Route missing=state.missing;std::sort(missing.begin(),missing.end());
    return route_plan_signature(state.routes)+"|missing:"+route_plan_signature({missing});
}
}
void validate_config(const AlnsConfig& cfg) {
    auto finite=[](double value){return std::isfinite(value);};
    if(cfg.iterations<0||cfg.no_improvement<0||cfg.segment_length<=0
        ||!finite(cfg.reaction)||cfg.reaction<0||cfg.reaction>1
        ||!finite(cfg.minimum_removed_fraction)||!finite(cfg.maximum_removed_fraction)
        ||cfg.minimum_removed_fraction<0||cfg.maximum_removed_fraction>1
        ||cfg.minimum_removed_fraction>cfg.maximum_removed_fraction
        ||!finite(cfg.determinism)||cfg.determinism<1
        ||!finite(cfg.cooling)||cfg.cooling<=0||cfg.cooling>1
        ||!finite(cfg.initial_relative_worsening)||cfg.initial_relative_worsening<0
        ||!finite(cfg.initial_acceptance)||cfg.initial_acceptance<=0||cfg.initial_acceptance>=1)
        throw std::invalid_argument("invalid HRLH-ALNS configuration");
    for(double value:cfg.rewards)if(!finite(value)||value<0)throw std::invalid_argument("invalid ALNS reward");
    for(double value:cfg.shaw_weights)if(!finite(value)||value<0)throw std::invalid_argument("invalid Shaw weight");
}
bool sa_accept(double current,double candidate,double temperature,std::mt19937_64& rng) {
    if(!std::isfinite(current)||!std::isfinite(candidate)||!std::isfinite(temperature)||temperature<0)
        throw std::invalid_argument("non-finite or negative SA value");
    if(candidate<=current)return true;
    if(temperature==0)return false;
    return routing::uniform01(rng)<std::exp((current-candidate)/temperature);
}
AdaptiveSelection::AdaptiveSelection(const AlnsConfig& cfg):reaction(cfg.reaction) {
    validate_config(cfg);dw.fill(1);rw.fill(1);
}
std::pair<std::size_t,std::size_t> AdaptiveSelection::select(std::mt19937_64& rng)const {
    const auto d=roulette(dw,rng);const auto r=roulette(rw,rng);return {d,r};
}
void AdaptiveSelection::record(std::size_t d,std::size_t r,double reward) {
    if(d>=dw.size()||r>=rw.size()||!std::isfinite(reward)||reward<0)
        throw std::invalid_argument("invalid operator score update");
    ++dc[d];++rc[r];ds[d]+=reward;rs[r]+=reward;
}
void AdaptiveSelection::update_segment() {
    for(std::size_t d=0;d<dw.size();++d) {
        if(dc[d])dw[d]=(1-reaction)*dw[d]+reaction*ds[d]/dc[d];
        dc[d]=0;ds[d]=0;
    }
    for(std::size_t r=0;r<rw.size();++r) {
        if(rc[r])rw[r]=(1-reaction)*rw[r]+reaction*rs[r]/rc[r];
        rc[r]=0;rs[r]=0;
    }
}
void run(const routing::Problem& problem,const RoutePlan& initial,const std::vector<RoutePlan>&,
         const routing::SearchCallbacks& callbacks,std::uint64_t seed,
         const AlnsConfig& cfg) {
    validate_config(cfg);
    std::mt19937_64 rng(seed);
    State current{initial,{}};
    validate_state(problem,current);
    double current_value=objective(problem,current),best_value=current_value;
    NeighbourGraphHistory history;
    history.observe(problem,current);
    double temperature=-cfg.initial_relative_worsening*current_value/std::log(cfg.initial_acceptance);
    AdaptiveSelection operators(cfg);
    int without_improvement=0;
    std::set<std::string> accepted_signatures{signature(current)};
    const auto n=problem.instance.suppliers.size();
    const auto low=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(cfg.minimum_removed_fraction*n)));
    const auto high=std::max(low,static_cast<std::size_t>(std::floor(cfg.maximum_removed_fraction*n)));
    for(int it=0;it<cfg.iterations&&(!cfg.no_improvement||without_improvement<cfg.no_improvement);++it) {
        if(callbacks.stop)callbacks.stop();
        auto [d,r]=operators.select(rng);
        std::uniform_int_distribution<std::size_t> number(low,high);
        State candidate=repair(problem,destroy(problem,current,static_cast<Destroy>(d),number(rng),rng,cfg,
                                              &history,callbacks.stop),
                               static_cast<Repair>(r),callbacks);

        const auto metrics=problem.evaluate(candidate.routes,true);

        const bool routing_rejected=metrics.duplicate_or_unknown
            ||metrics.capacity_excess!=0||metrics.time_excess!=0;
        bool proved_infeasible=routing_rejected;

        if(candidate.missing.empty()&&metrics.routing_feasible) {

            if(callbacks.evaluate_integrated) {
                const auto status=callbacks.evaluate_integrated(candidate.routes);
                if(status==routing::PackingAssessment::CertifiedInfeasible||
                   status==routing::PackingAssessment::RoutingInfeasible)
                    proved_infeasible=true;
            }
        }
        const double value=objective(problem,candidate);

        if(!proved_infeasible&&!metrics.duplicate_or_unknown
            &&metrics.capacity_excess==0&&metrics.time_excess==0)
            history.observe(problem,candidate);
        const bool novel=accepted_signatures.count(signature(candidate))==0;
        const bool accepted=!proved_infeasible&&sa_accept(current_value,value,temperature,rng);
        double reward=0;
        ++without_improvement;
        if(accepted) {

            if(value<best_value) {best_value=value;without_improvement=0;reward=cfg.rewards[0];}
            else if(value<current_value)reward=cfg.rewards[1];
            else if(value==current_value||novel)reward=cfg.rewards[2];
            accepted_signatures.insert(signature(candidate));
            current=std::move(candidate);current_value=value;
        }
        operators.record(d,r,reward);
        if((it+1)%cfg.segment_length==0)operators.update_segment();
        temperature*=cfg.cooling;

    }
}
}
