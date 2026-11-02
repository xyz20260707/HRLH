#include "optional_demand_profit_intensification.hpp"
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <sstream>

namespace hrlh::optional_profit_detail {
using Clock = std::chrono::steady_clock;
struct Deadline {};
struct Key {
    int type = 0, rank = 0;
    bool optional = false;
    bool operator==(const Key& b) const { return type == b.type && rank == b.rank && optional == b.optional; }
};
struct Placed { Key key; int x=0,y=0,w=0,h=0; };
struct Spec { PackingTypeSpecCpp original; std::vector<long long> values; };
struct ProfitTarget {
    std::vector<int> counts;
    long long gain = 0;
};
struct Route {
    std::vector<Spec> specs;
    std::vector<Placed> best;
    long long best_score = 0, profit_upper_bound = 0;
    bool targets_dirty = true;
    std::vector<ProfitTarget> targets;

    std::set<std::vector<int>> certified_targets;
};

static bool overlap(const Placed& a, const Placed& b) {
    return a.x < b.x+b.w && b.x < a.x+a.w && a.y < b.y+b.h && b.y < a.y+a.h;
}
static std::vector<int> counts(const Route& r, const std::vector<Placed>& p) {
    std::vector<int> c(r.specs.size(),0);
    for (const auto& v:p) if(v.key.optional) ++c.at(v.key.type);
    return c;
}
static long long score(const Route& r, const std::vector<int>& c) {
    long long v=0;
    for(std::size_t i=0;i<c.size();++i) v+=r.specs[i].values.at(c[i]);
    return v;
}
static bool valid_floor(const Route& r, const std::vector<Placed>& p, int W,int H) {
    std::vector<std::vector<int>> m(r.specs.size()),o(r.specs.size());
    for(const auto& v:p) {
        if(v.key.type<0 || v.key.type>=int(r.specs.size()) || v.x<0 || v.y<0 || v.w<=0 || v.h<=0 || v.x+v.w>W || v.y+v.h>H) return false;
        const auto& s=r.specs[v.key.type].original;
        if(!((v.w==s.width && v.h==s.length)||(v.w==s.length && v.h==s.width))) return false;
        (v.key.optional?o:m)[v.key.type].push_back(v.key.rank);
    }
    for(std::size_t i=0;i<m.size();++i) {
        if(int(m[i].size())!=r.specs[i].original.mandatory_count || int(o[i].size())>r.specs[i].original.optional_count) return false;
        std::sort(m[i].begin(),m[i].end());std::sort(o[i].begin(),o[i].end());
        for(int k=0;k<int(m[i].size());++k) if(m[i][k]!=k) return false;
        for(int k=0;k<int(o[i].size());++k) if(o[i][k]!=k) return false;
    }
    for(std::size_t i=0;i<p.size();++i) for(std::size_t j=i+1;j<p.size();++j) if(overlap(p[i],p[j])) return false;
    return true;
}

static long long required_area(const Route& r,const std::vector<int>& c) {
    long long area=0;
    for(std::size_t i=0;i<c.size();++i)
        area+=1LL*r.specs[i].original.width*r.specs[i].original.length*
              (r.specs[i].original.mandatory_count+c[i]);
    return area;
}
static long long route_profit_upper_bound(const Route& r) {
    long long upper=0;
    for(const auto& s:r.specs) {
        if(s.values.size()!=std::size_t(s.original.optional_count+1))
            throw std::runtime_error("Profit intensification value curve size mismatch");
        upper+=*std::max_element(s.values.begin(),s.values.end());
    }
    return upper;
}
static void rebuild_targets(Route& r,
                            const std::function<void()>& stop) {
    stop();
    const auto baseline=counts(r,r.best);
    if(score(r,baseline)!=r.best_score)
        throw std::runtime_error("Profit intensification latest incumbent counts/profit mismatch");
    std::vector<ProfitTarget> targets;
    for(int to=0;to<int(baseline.size());++to) {
        stop();
        if(baseline[to]>=r.specs[to].original.optional_count)continue;
        auto add=baseline;++add[to];
        long long gain=score(r,add)-r.best_score;
        if(gain>0)targets.push_back({add,gain});
        for(int from=0;from<int(baseline.size());++from) {
            stop();
            if(from==to||baseline[from]==0)continue;
            auto replace=add;--replace[from];
            gain=score(r,replace)-r.best_score;
            if(gain>0)targets.push_back({std::move(replace),gain});
        }
    }
    std::sort(targets.begin(),targets.end(),[](const auto& a,const auto& b) {
        return a.gain!=b.gain?a.gain>b.gain:a.counts<b.counts;
    });
    r.targets=std::move(targets);
    r.targets_dirty=false;
}

struct StackAssignment {
    long long value=0;
    std::vector<std::vector<int>> mandatory, optional;
};
using Curves=std::map<std::string,std::map<int,StackAssignment>>;
struct CachedCurves {
    Curves curves;
};
static std::string group_id(const hrlh_components::stack_preprocessing::GroupKey& k) {
    return std::to_string(std::get<0>(k))+":"+std::to_string(std::get<1>(k))+":"+std::to_string(std::get<2>(k));
}
static std::map<int,const hrlh_model::Item*> item_map(const hrlh_model::ProblemInstance& inst) {
    std::map<int,const hrlh_model::Item*> out;
    for(const auto& i:inst.items)out[i.item_id]=&i;
    return out;
}
static bool compatible(const hrlh_model::ProblemInstance& inst,int a,int b) {
    auto ab=inst.stack_compatibility.find({a,b}),ba=inst.stack_compatibility.find({b,a});
    return ab!=inst.stack_compatibility.end()&&ba!=inst.stack_compatibility.end()&&ab->second==1&&ba->second==1;
}
static void check_assignment(const hrlh_model::ProblemInstance& inst,
                             const StackAssignment& a,int R,int r) {
    if(int(a.mandatory.size())!=R||int(a.optional.size())>r)throw std::runtime_error("Stack assignment whole-r rack count mismatch");
    auto items=item_map(inst);std::set<int> used;long long score=0;
    auto check=[&](const std::vector<int>& stack) {
        if(stack.empty())throw std::runtime_error("Stack assignment empty physical rack");
        int height=0;
        for(int id:stack) {
            if(!used.insert(id).second)throw std::runtime_error("Stack assignment duplicate material");
            const auto& i=*items.at(id);height+=i.height;
            if(i.is_optional)score+=i.score;
            for(int other:stack)if(other!=id&&!compatible(inst,id,other))throw std::runtime_error("Stack assignment incompatible stack");
        }
        if(height>inst.vehicle_height)throw std::runtime_error("Stack assignment height violation");
    };
    for(const auto& s:a.mandatory)check(s);
    for(const auto& s:a.optional)check(s);
    if(score!=a.value)throw std::runtime_error("Stack assignment physical profit mismatch");
}

static Curves build_curves(const hrlh_model::ProblemInstance& inst,
                          const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
                          const CandidateSolution& best,
                          CachedCurves& cache) {
    auto& curves=cache.curves;std::set<std::string> needed;
    for(const auto& d:best.packing.route_details)for(const auto& s:d.specs)if(s.type_id.rfind("P:",0)!=0)needed.insert(s.type_id);
    for(const auto& [key,g]:prep.groups)if(needed.count(group_id(key))) {
        if(curves.count(group_id(key)))continue;
        auto& table=curves[group_id(key)];
        if(g.spread_joint_enabled) {
            const int R=g.baseline_result.baseline_stack_count;
            const auto items=item_map(inst);
            for(const auto& [r,point]:g.spread_joint_curve) {
                StackAssignment a;
                a.value=point.optional_profit;
                for(const auto& stack:point.stacks) {
                    const bool has_mandatory=std::any_of(stack.begin(),stack.end(),[&](int id) {
                        return !items.at(id)->is_optional;
                    });
                    if(has_mandatory && int(a.mandatory.size())<R) a.mandatory.push_back(stack);
                    else a.optional.push_back(stack);
                }
                const auto missing=R-int(a.mandatory.size());
                if(missing>0&&missing<=int(a.optional.size())) {
                    a.mandatory.insert(a.mandatory.end(),a.optional.begin(),a.optional.begin()+missing);
                    a.optional.erase(a.optional.begin(),a.optional.begin()+missing);
                }
                check_assignment(inst,a,R,r);
                if(int(a.optional.size())!=r)
                    throw std::runtime_error("Spread-Joint exact optional count mismatch");
                table[r]=std::move(a);
            }
            continue;
        }
        for(const auto& [r,v]:g.value_table) {
            StackAssignment a;a.value=g.baseline_result.baseline_optional_profit+v.objective.value_or(0);
            a.mandatory=g.baseline_result.final_stacks;
            for(const auto& s:v.stacks)if(!s.empty())a.optional.push_back(s);
            table[r]=std::move(a);
        }
        if(table.empty())table[0]={g.baseline_result.baseline_optional_profit,g.baseline_result.final_stacks,{}};
    }

    for(auto& [key,table]:curves) {
        StackAssignment previous;bool first=true;
        for(auto& [r,a]:table) {
            if(!first&&previous.value>a.value) {
                const auto group_it=std::find_if(prep.groups.begin(),prep.groups.end(),[&](const auto& entry) {
                    return group_id(entry.first)==key;
                });
                if(group_it!=prep.groups.end()&&group_it->second.spread_joint_enabled)
                    throw std::runtime_error("Spread-Joint whole-assignment curve decreased");
                a=previous;
            }
            previous=a;first=false;
        }
    }
    return curves;
}

static std::vector<Placed> expand_floor(const RoutePackingDetail& d,
                                       const std::function<void()>& stop) {
    std::vector<int> remaining=d.loaded_mandatory_count_by_spec;
    for(std::size_t i=0;i<remaining.size();++i)remaining[i]+=d.loaded_optional_count_by_spec.at(i);
    std::vector<Placed> placed;std::set<std::vector<int>> failed;
    std::function<bool(int)> block;
    block=[&](int bi) {
        stop();if(bi==int(d.placed_rects.size()))return std::all_of(remaining.begin(),remaining.end(),[](int n){return n==0;});
        auto state=remaining;state.insert(state.begin(),bi);if(failed.count(state))return false;
        const auto& b=d.placed_rects[bi];
        for(int axis=0;axis<2;++axis) {
            int common=int(axis?b.length:b.width),target=int(axis?b.width:b.length);
            struct Option {int type,w,h,extent;};std::vector<Option> options;
            for(int ti=0;ti<int(d.specs.size());++ti) {
                const auto& s=d.specs[ti];
                if(s.width==common)options.push_back({ti,axis?s.length:s.width,axis?s.width:s.length,s.length});
                else if(s.length==common)options.push_back({ti,axis?s.width:s.length,axis?s.length:s.width,s.width});
            }
            const std::size_t base=placed.size();
            std::function<bool(int,int,int)> fill=[&](int oi,int left,int cursor) {
                stop();if(left==0)return block(bi+1);
                if(oi==int(options.size()))return false;
                auto o=options[oi];const int maximum=std::min(remaining[o.type],left/o.extent);
                for(int n=maximum;n>=0;--n) {
                    remaining[o.type]-=n;
                    for(int k=0;k<n;++k)placed.push_back({{o.type,0,false},int(b.x)+(axis?cursor+k*o.extent:0),int(b.y)+(axis?0:cursor+k*o.extent),o.w,o.h});
                    if(fill(oi+1,left-n*o.extent,cursor+n*o.extent))return true;
                    placed.resize(placed.size()-n);remaining[o.type]+=n;
                }
                return false;
            };
            if(fill(0,target,0)) {
                std::vector<int> count(d.specs.size(),0);
                for(auto& p:placed) {
                    int n=count[p.key.type]++;
                    p.key.optional=n>=d.loaded_mandatory_count_by_spec[p.key.type];
                    p.key.rank=p.key.optional?n-d.loaded_mandatory_count_by_spec[p.key.type]:n;
                }
                return placed.size()>base;
            }
        }
        failed.insert(std::move(state));return false;
    };
    if(!block(0))throw std::runtime_error("Optional demand profit intensification cannot reconstruct existing compound strip");
    return placed;
}
static long long total_score(const std::vector<Route>& routes) {
    long long out=0;for(const auto& r:routes)out+=r.best_score;return out;
}
struct TargetPackingResult {
    bool feasible=false, certified=false;
    std::vector<Placed> placed;
};
static TargetPackingResult try_fixed_target(
    const Route& route,const ProfitTarget& target,int W,int H,
    const HrlhConfig& cfg,StdRng& rng,const std::function<void()>& stop) {
    stop();
    if(required_area(route,target.counts)>1LL*W*H) return {false,true,{}};
    std::vector<PackingTypeSpecCpp> fixed;
    fixed.reserve(route.specs.size());
    for(std::size_t i=0;i<route.specs.size();++i) {
        stop();
        auto spec=route.specs[i].original;
        spec.mandatory_count+=target.counts[i];spec.optional_count=0;
        spec.optional_marginal_profits.clear();spec.fixed_inserted_score=0;
        fixed.push_back(std::move(spec));
    }
    MandatoryStaticLbCheckCpp precheck;
    {
        SkylineStopCallbackScope scope(stop);
        precheck=mandatory_static_lb_check_cpp(fixed,W,H);
    }
    if(precheck.proven_infeasible) return {false,true,{}};
    auto run_skyline=[&](bool deep)->RouteEvalFullOutCpp {
        stop();
        const auto seed=rng.eng();
        PyRandomRng packing_rng(std::to_string(seed));
        SkylineEvalOptions options;
        options.run_static_precheck=!deep&&precheck.preprocessing_error;
        RouteEvalFullOutCpp result;
        try {
            result=evaluate_single_route_mandatory_packing_cpp_api_full(
                fixed,W,H,deep?cfg.deep_packing_retry_iters:cfg.packing_mutation_iters,
                cfg.packing_mutation_swap_ratio,packing_rng,stop,options);
        } catch(const Deadline&) {throw;}
        catch(const std::exception&) {result.scalar.feasible=false;}
        return result;
    };
    auto result=run_skyline(false);
    if(result.scalar.static_lb_infeasible) return {false,true,{}};
    if(!result.scalar.feasible&&cfg.deep_packing_retry_iters>=0) result=run_skyline(true);
    if(!result.scalar.feasible) return {};
    if(result.scalar.loaded_mandatory_count_by_type.size()!=fixed.size()||
       result.scalar.loaded_optional_count_by_type.size()!=fixed.size())
        throw std::runtime_error("Profit intensification target evaluator returned wrong count-vector size");
    RoutePackingDetail witness;
    for(std::size_t i=0;i<fixed.size();++i) {
        if(result.scalar.loaded_mandatory_count_by_type[i]!=fixed[i].mandatory_count||
           result.scalar.loaded_optional_count_by_type[i]!=0)
            throw std::runtime_error("Profit intensification target evaluator did not load the complete fixed target");
        witness.specs.push_back(route.specs[i].original);
        witness.loaded_mandatory_count_by_spec.push_back(route.specs[i].original.mandatory_count);
    }
    witness.loaded_optional_count_by_spec=target.counts;
    for(const auto& p:result.scalar.placed_rects)
        witness.placed_rects.push_back({p.type_index,p.is_optional,p.rank_in_type,
            double(p.x),double(p.y),double(p.width),double(p.length)});
    auto placed=expand_floor(witness,stop);
    if(counts(route,placed)!=target.counts||!valid_floor(route,placed,W,H))
        throw std::runtime_error("Profit intensification fixed target expansion/counts/geometry mismatch");
    return {true,false,std::move(placed)};
}

template<class Trial>
static void search_profit_targets(std::vector<Route>& routes,int W,int H,
    const std::function<void()>& stop,Trial&& trial) {
    long long global_profit_upper_bound=0;
    for(auto& route:routes) {
        stop();route.profit_upper_bound=route_profit_upper_bound(route);
        if(route.best_score>route.profit_upper_bound)
            throw std::runtime_error("Profit intensification incumbent exceeds stored-curve upper bound");
        global_profit_upper_bound+=route.profit_upper_bound;
    }
    while(true) {
        stop();
        if(total_score(routes)>=global_profit_upper_bound) return;
        for(std::size_t ri=0;ri<routes.size();++ri) {
            stop();auto& route=routes[ri];
            if(route.best_score>=route.profit_upper_bound)continue;
            if(route.targets_dirty)rebuild_targets(route,stop);
            for(const auto& target:route.targets) {
                stop();
                if(route.certified_targets.count(target.counts)) continue;
                auto result=trial(route,target);
                if(result.certified) {route.certified_targets.insert(target.counts);continue;}
                if(!result.feasible)continue;
                const auto value=score(route,target.counts);
                if(value<=route.best_score||value-route.best_score!=target.gain)
                    throw std::runtime_error("Profit intensification reused a stale/non-improving target");
                if(counts(route,result.placed)!=target.counts||!valid_floor(route,result.placed,W,H))
                    throw std::runtime_error("Profit intensification accepted target witness mismatch");
                route.best=std::move(result.placed);route.best_score=value;
                route.targets_dirty=true;route.targets.clear();
                break;
            }
        }
    }
}
}

namespace hrlh {
struct OptionalDemandProfitContext::Impl {
    optional_profit_detail::CachedCurves cache;
    StdRng rng;
    explicit Impl(std::uint64_t seed):rng(seed){}
};
void intensify_optional_demand_profit(const hrlh_model::ProblemInstance& inst,
                                  const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& preprocess,
                                  CandidateSolution& solution,const HrlhConfig& cfg,
                                  OptionalDemandProfitContext& context,
                                  double available_seconds) {
    using namespace optional_profit_detail;
    if((!cfg.enable_optional_demand_profit_intensification)||available_seconds<=0)return;
    if(!context.state)context.state=std::make_shared<OptionalDemandProfitContext::Impl>(cfg.seed);

    const auto start=Clock::now();auto elapsed=[&](){return std::chrono::duration<double>(Clock::now()-start).count();};
    auto remaining=[&](){return available_seconds-elapsed();};
    auto stop=[&](){if(remaining()<=0)throw Deadline{};};
    auto candidate=solution;auto items=item_map(inst);
    long long joint_gain=0;
    std::vector<optional_profit_detail::Route> routes;Curves curves;
    try {

        for(const auto& d:candidate.packing.route_details) {
            optional_profit_detail::Route r;r.best=expand_floor(d,stop);
            for(int si=0;si<int(d.specs.size());++si) {
                Spec s;s.original=d.specs[si];s.values={s.original.fixed_inserted_score};
                for(auto profit:s.original.optional_marginal_profits)s.values.push_back(s.values.back()+profit);
                r.specs.push_back(std::move(s));
            }

            r.best_score=d.total_optional_priority_score;routes.push_back(std::move(r));
        }
        curves=build_curves(inst,preprocess,candidate,context.state->cache);
        for(auto& r:routes) {
            for(auto& s:r.specs)if(s.original.type_id.rfind("P:",0)!=0) {
                const auto& table=curves.at(s.original.type_id);
                for(int n=0;n<=s.original.optional_count;++n)s.values[n]=table.at(n).value;
                s.original.fixed_inserted_score=s.values[0];
                for(int n=0;n<s.original.optional_count;++n)s.original.optional_marginal_profits[n]=s.values[n+1]-s.values[n];
            }

            bool changed=true;
            while(changed) {
                changed=false;auto c=counts(r,r.best);
                for(int ti=0;ti<int(r.specs.size());++ti)if(r.specs[ti].original.type_id.rfind("P:",0)!=0) {
                    const auto& a=curves.at(r.specs[ti].original.type_id).at(c[ti]);
                    int actual=int(a.optional.size());
                    if(actual<c[ti]) {
                        if(curves.at(r.specs[ti].original.type_id).at(actual).value!=a.value)throw std::runtime_error("Stack assignment empty rack normalisation mismatch");
                        r.best.erase(std::remove_if(r.best.begin(),r.best.end(),[&](const Placed& p){return p.key.type==ti&&p.key.optional&&p.key.rank>=actual;}),r.best.end());
                        changed=true;
                    }
                }
            }

        }
        joint_gain=total_score(routes)-candidate.optional_profit;
        if(joint_gain<0)throw std::runtime_error("Stack assignment profit worsened");
        auto& rng=context.state->rng;
        try {
            search_profit_targets(routes,inst.vehicle_width,inst.vehicle_length,
                stop,[&](const optional_profit_detail::Route& r,const ProfitTarget& t) {
                    return try_fixed_target(r,t,inst.vehicle_width,inst.vehicle_length,
                        cfg,rng,stop);
                });
        } catch(const Deadline&) {}

        std::set<int> loaded;long long physical_profit=0;
        for(std::size_t ri=0;ri<routes.size();++ri) {
            auto& r=routes[ri];auto& d=candidate.packing.route_details[ri];
            if(!valid_floor(r,r.best,inst.vehicle_width,inst.vehicle_length))throw std::runtime_error("Profit intensification final floor invalid");
            d.placed_rects.clear();d.loaded_optional_count_by_spec=counts(r,r.best);
            d.material_stacks_by_spec.clear();
            d.material_stacks_by_spec.resize(r.specs.size());
            d.fixed_inserted_score=0;d.profit_upper_bound=0;
            for(std::size_t si=0;si<r.specs.size();++si) {
                d.specs[si]=r.specs[si].original;auto& content=d.material_stacks_by_spec[si];
                const auto& s=d.specs[si];d.fixed_inserted_score+=s.fixed_inserted_score;
                d.profit_upper_bound+=*std::max_element(r.specs[si].values.begin(),r.specs[si].values.end());
                if(s.type_id.rfind("P:",0)==0) {
                    for(const auto& it:inst.items)if(it.is_pallet&&std::find(d.route.begin(),d.route.end(),it.supplier)!=d.route.end()
                        &&s.type_id=="P:"+std::to_string(it.supplier)+":"+std::to_string(it.width)+"x"+std::to_string(it.length))content.push_back({it.item_id});
                } else {
                    const auto& a=curves.at(s.type_id).at(d.loaded_optional_count_by_spec[si]);
                    check_assignment(inst,a,s.mandatory_count,d.loaded_optional_count_by_spec[si]);
                    if(int(a.optional.size())!=d.loaded_optional_count_by_spec[si])throw std::runtime_error("Profit intensification selected empty rack");
                    content=a.mandatory;content.insert(content.end(),a.optional.begin(),a.optional.end());
                }
                for(const auto& stack:content)for(int id:stack) {
                    if(!loaded.insert(id).second)throw std::runtime_error("Optional demand profit intensification duplicate physical material");
                    if(items.at(id)->is_optional)physical_profit+=items.at(id)->score;
                    if(std::find(d.route.begin(),d.route.end(),items.at(id)->supplier)==d.route.end())throw std::runtime_error("Optional demand profit intensification wrong supplier");
                }
            }
            for(const auto& p:r.best)d.placed_rects.push_back({p.key.type,p.key.optional,p.key.rank,double(p.x),double(p.y),double(p.w),double(p.h)});
            d.bin_number=r.best.empty()?0:1;d.total_optional_priority_score=r.best_score;d.optional_score_from_2d=r.best_score-d.fixed_inserted_score;
        }
        for(const auto& it:inst.items)if(!it.is_optional&&!loaded.count(it.item_id))throw std::runtime_error("Optional demand profit intensification missing mandatory material");
        if(physical_profit!=total_score(routes))throw std::runtime_error("Optional demand profit intensification full physical objective mismatch");
        candidate.optional_profit=physical_profit;candidate.packing.total_optional_priority_score=physical_profit;
        if(candidate.optional_profit<solution.optional_profit)throw std::runtime_error("Optional demand profit intensification incumbent worsened");

        solution=std::move(candidate);

    } catch(const Deadline&) {

    }

}
}
