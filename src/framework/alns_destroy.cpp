#include "alns.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>

namespace hrlh::alns {
namespace {
Route assigned(const State& state) {
    Route clients;
    for (const auto& route : state.routes) clients.insert(clients.end(),route.begin(),route.end());
    return clients;
}
void normalize(State& state) {
    state.routes.erase(std::remove_if(state.routes.begin(),state.routes.end(),
        [](const Route& r) { return r.empty(); }),state.routes.end());
}
void remove(State& state,int supplier) {
    for(auto& route:state.routes) {
        auto found=std::find(route.begin(),route.end(),supplier);
        if(found!=route.end()) {
            route.erase(found); state.missing.push_back(supplier); normalize(state); return;
        }
    }
    throw std::logic_error("ALNS attempted to remove an unassigned supplier");
}
std::size_t rank_choice(std::mt19937_64& rng,std::size_t size,double p) {
    return std::min(size-1,static_cast<std::size_t>(std::pow(routing::uniform01(rng),p)*size));
}

Route mst_cluster(const routing::Problem& problem,const Route& route,std::mt19937_64& rng,
                  const std::function<void()>& stop) {
    if(route.size()<2)return route;
    const auto n=route.size();
    std::vector<double> key(n,std::numeric_limits<double>::infinity());
    std::vector<int> parent(n,-1); std::vector<bool> in_tree(n,false); key[0]=0;
    for(std::size_t step=0;step<n;++step) {
        if(stop)stop();
        std::size_t u=n;
        for(std::size_t v=0;v<n;++v)
            if(!in_tree[v]&&(u==n||key[v]<key[u]))u=v;
        in_tree[u]=true;
        for(std::size_t v=0;v<n;++v) if(!in_tree[v]) {
            double distance=problem.matrix_proximity(route[u],route[v]);
            if(distance<key[v]) {key[v]=distance;parent[v]=static_cast<int>(u);}
        }
    }
    std::size_t cut=1;
    for(std::size_t v=2;v<n;++v)if(key[v]>key[cut])cut=v;
    std::vector<std::vector<std::size_t>> neighbours(n);
    for(std::size_t v=1;v<n;++v)if(v!=cut) {
        neighbours[v].push_back(parent[v]); neighbours[parent[v]].push_back(v);
    }
    const std::size_t start=routing::uniform01(rng)<.5?cut:static_cast<std::size_t>(parent[cut]);
    std::vector<std::size_t> queue{start};std::set<std::size_t> seen{start};
    for(std::size_t i=0;i<queue.size();++i)
        for(auto v:neighbours[queue[i]])if(seen.insert(v).second)queue.push_back(v);
    Route cluster;for(auto v:queue)cluster.push_back(route[v]);return cluster;
}
std::vector<std::pair<int,int>> edges(const routing::Problem& p,const Route& r) {
    std::vector<std::pair<int,int>> result;
    int from=p.instance.start_depot();
    for(int to:r){result.emplace_back(from,to);from=to;}
    result.emplace_back(from,p.instance.end_depot());
    return result;
}
bool is_supplier(const routing::Problem& p,int node) {return p.index.count(node)!=0;}

struct CrossingPair {std::size_t first,second; Route endpoints;};
std::vector<CrossingPair> cross_route_pairs(const routing::Problem& p,const State& state,
                                           const std::function<void()>& stop) {
    std::vector<CrossingPair> pairs;
    for(std::size_t i=0;i<state.routes.size();++i)for(std::size_t j=i+1;j<state.routes.size();++j) {
        if(stop)stop();
        std::set<int> endpoints;
        for(auto [a,b]:edges(p,state.routes[i]))for(auto [c,d]:edges(p,state.routes[j])) {

            if(!is_supplier(p,b)||!is_supplier(p,d))continue;
            const double old=p.directed_cost(a,b)+p.directed_cost(c,d);
            const double crossed=p.directed_cost(a,d)+p.directed_cost(c,b);
            if(crossed<old)for(int node:{a,b,c,d})if(is_supplier(p,node))endpoints.insert(node);
        }
        if(!endpoints.empty())pairs.push_back({i,j,Route(endpoints.begin(),endpoints.end())});
    }
    return pairs;
}
int medoid(const routing::Problem& p,const Route& r) {
    double best=std::numeric_limits<double>::infinity();int selected=r.front();
    for(int s:r) {
        double sum=0;for(int t:r)sum+=p.matrix_proximity(s,t);
        if(sum<best){best=sum;selected=s;}
    }
    return selected;
}
}
void validate_state(const routing::Problem& problem,const State& state) {
    std::map<int,int> count;
    for(int supplier:problem.instance.suppliers)count.emplace(supplier,0);
    auto visit=[&](int supplier) {
        auto found=count.find(supplier);
        if(found==count.end()||++found->second!=1)throw std::invalid_argument("invalid ALNS supplier coverage");
    };
    for(const auto& route:state.routes) {
        if(route.empty())throw std::invalid_argument("ALNS state contains empty route");
        for(int supplier:route)visit(supplier);
    }
    for(int supplier:state.missing)visit(supplier);
    for(auto [id,n]:count)if(n!=1)throw std::invalid_argument("ALNS state silently lost a supplier");
}
double objective(const routing::Problem& problem,const State& state) {
    validate_state(problem,state);
    double value=problem.evaluate(state.routes,true).cost;
    for(int supplier:state.missing)value+=problem.evaluate_route({supplier}).cost;
    const auto fleet=state.routes.size()+state.missing.size();
    if(fleet>problem.instance.vehicles.size())
        value+=10*problem.max_arc_cost*(fleet-problem.instance.vehicles.size());
    return value;
}
void NeighbourGraphHistory::observe(const routing::Problem& problem,const State& state) {

    const double value=objective(problem,state);
    if(!std::isfinite(value))throw std::invalid_argument("non-finite ALNS history objective");
    auto record=[&](const Route& route) {
        for(auto arc:edges(problem,route)) {
            auto [entry,inserted]=weights.emplace(arc,value);
            if(!inserted&&value<entry->second)entry->second=value;
        }
    };
    for(const auto& route:state.routes)record(route);
    for(int supplier:state.missing)record(Route{supplier});
}
double NeighbourGraphHistory::edge_weight(int from,int to) const {
    const auto found=weights.find({from,to});
    return found==weights.end()?std::numeric_limits<double>::infinity():found->second;
}
double NeighbourGraphHistory::removal_score(const routing::Problem& problem,const Route& route,
                                           std::size_t position) const {
    if(position>=route.size())throw std::out_of_range("invalid ALNS history supplier position");
    const int supplier=route[position];
    (void)problem.index.at(supplier);
    const int before=position?route[position-1]:problem.instance.start_depot();
    const int after=position+1<route.size()?route[position+1]:problem.instance.end_depot();
    return edge_weight(before,supplier)+edge_weight(supplier,after);
}
State destroy(const routing::Problem& problem,const State& source,Destroy op,
              std::size_t count,std::mt19937_64& rng,const AlnsConfig& cfg,
              const NeighbourGraphHistory* history,const std::function<void()>& stop) {
    validate_state(problem,source);
    if(stop)stop();
    State result=source;
    auto available=assigned(result);
    count=std::min(count,available.size());
    if(!count)return result;
    if(op==Destroy::Random) {
        std::shuffle(available.begin(),available.end(),rng);
        for(std::size_t i=0;i<count;++i)remove(result,available[i]);
    } else if(op==Destroy::Shaw) {
        std::map<int,std::size_t> route_of;
        for(std::size_t ri=0;ri<source.routes.size();++ri)for(int s:source.routes[ri])route_of[s]=ri;
        double dscale=0,amin=std::numeric_limits<double>::infinity(),amax=0;
        double tmin=std::numeric_limits<double>::infinity(),tmax=-tmin;
        for(int s:available) {
            const double load=problem.demands.at(problem.index.at(s)).empty()?0:problem.demands.at(problem.index.at(s))[0];
            amin=std::min(amin,load);amax=std::max(amax,load);
            tmin=std::min(tmin,problem.instance.latest_return.at(s));tmax=std::max(tmax,problem.instance.latest_return.at(s));
            for(int t:available)dscale=std::max(dscale,problem.matrix_proximity(s,t));
        }
        Route removed{available[routing::uniform_index(rng,available.size())]};
        remove(result,removed.front());
        while(removed.size()<count) {
            int seed=removed[routing::uniform_index(rng,removed.size())];
            available=assigned(result);
            std::vector<std::pair<double,int>> ranks;
            for(int s:available) {
                const auto& a=problem.demands.at(problem.index.at(s));
                const auto& b=problem.demands.at(problem.index.at(seed));
                const double demand=(a.empty()?0:a[0])-(b.empty()?0:b[0]);
                double score=cfg.shaw_weights[0]*(dscale?problem.matrix_proximity(seed,s)/dscale:0)
                    +cfg.shaw_weights[1]*(tmax>tmin?std::abs(problem.instance.latest_return.at(seed)-problem.instance.latest_return.at(s))/(tmax-tmin):0)
                    +cfg.shaw_weights[2]*(amax>amin?std::abs(demand)/(amax-amin):0)
                    +cfg.shaw_weights[3]*(route_of[seed]==route_of[s]?-1:1);
                ranks.emplace_back(score,s);
            }
            std::sort(ranks.begin(),ranks.end());
            int chosen=ranks[rank_choice(rng,ranks.size(),cfg.determinism)].second;
            remove(result,chosen);removed.push_back(chosen);
        }
    } else if(op==Destroy::Worst) {
        for(std::size_t step=0;step<count;++step) {
            std::vector<std::pair<double,int>> ranks;
            for(const auto& route:result.routes)for(int s:route) {
                Route after=route;after.erase(std::find(after.begin(),after.end(),s));
                const double before_cost=problem.evaluate_route(route).cost;
                const double after_cost=problem.evaluate_route(after).cost;
                const double reduction=before_cost-after_cost;
                ranks.emplace_back(-reduction,s);
            }
            std::sort(ranks.begin(),ranks.end());
            remove(result,ranks[rank_choice(rng,ranks.size(),cfg.determinism)].second);
        }
    } else if(op==Destroy::NeighbourGraph) {
        NeighbourGraphHistory initial_history;
        if(!history) {initial_history.observe(problem,source);history=&initial_history;}
        for(std::size_t step=0;step<count;++step) {
            if(stop)stop();
            std::vector<std::pair<double,int>> ranks;

            for(const auto& route:result.routes)for(std::size_t pos=0;pos<route.size();++pos)
                ranks.emplace_back(-history->removal_score(problem,route,pos),route[pos]);
            std::sort(ranks.begin(),ranks.end());
            remove(result,ranks[rank_choice(rng,ranks.size(),cfg.determinism)].second);
        }
    } else if(op==Destroy::Cluster) {

        std::vector<bool> processed(source.routes.size(),false);
        std::size_t selected=routing::uniform_index(rng,source.routes.size());
        Route removed;
        while(removed.size()<count&&selected<source.routes.size()) {
            if(stop)stop();
            processed[selected]=true;
            const Route cluster=mst_cluster(problem,source.routes[selected],rng,stop);
            for(int s:cluster) {
                if(removed.size()>=count)break;
                remove(result,s);removed.push_back(s);
            }
            if(removed.size()>=count)break;
            const int anchor=removed[routing::uniform_index(rng,removed.size())];
            selected=source.routes.size();
            double closest=std::numeric_limits<double>::infinity();
            for(std::size_t ri=0;ri<source.routes.size();++ri)if(!processed[ri]) {
                if(stop)stop();
                for(int supplier:source.routes[ri]) {
                    const double distance=problem.matrix_proximity(anchor,supplier);
                    if(distance<closest) {closest=distance;selected=ri;}
                }
            }
        }
    } else if(op==Destroy::Overlap) {
        while(result.missing.size()-source.missing.size()<count) {
            if(stop)stop();

            const auto pairs=cross_route_pairs(problem,result,stop);
            if(pairs.empty())break;
            const Route removal=pairs[routing::uniform_index(rng,pairs.size())].endpoints;
            for(int s:removal) {
                if(result.missing.size()-source.missing.size()>=count)break;
                remove(result,s);
            }
        }
    } else if(op==Destroy::TourPair) {
        const auto pairs=cross_route_pairs(problem,result,stop);
        if(!pairs.empty()) {
            const auto& chosen=pairs[routing::uniform_index(rng,pairs.size())];
            Route removal=result.routes[chosen.first];
            removal.insert(removal.end(),result.routes[chosen.second].begin(),result.routes[chosen.second].end());
            for(int s:removal) {
                remove(result,s);
            }
        }
    } else if(op==Destroy::InnerRoute) {
        std::vector<std::pair<std::size_t,std::size_t>> nested;
        for(std::size_t outer=0;outer<result.routes.size();++outer) {
            int center=medoid(problem,result.routes[outer]);double radius=0;
            for(int s:result.routes[outer])radius=std::max(radius,problem.matrix_proximity(center,s));
            if(radius==0||result.routes[outer].size()<2)continue;
            for(std::size_t inner=0;inner<result.routes.size();++inner)if(inner!=outer) {
                bool inside=true;for(int s:result.routes[inner])
                    if(problem.matrix_proximity(center,s)>=radius){inside=false;break;}
                if(inside)nested.emplace_back(outer,inner);
            }
        }
        if(!nested.empty()) {
            auto [outer,inner]=nested[routing::uniform_index(rng,nested.size())];
            const Route outside=result.routes[outer],inside=result.routes[inner];
            for(int s:inside)remove(result,s);
            auto found=std::find(result.routes.begin(),result.routes.end(),outside);
            if(found==result.routes.end())throw std::logic_error("inner-route split lost outer route");
            const auto midpoint=(outside.size()+1)/2;
            *found=Route(outside.begin(),outside.begin()+midpoint);
            result.routes.push_back(Route(outside.begin()+midpoint,outside.end()));
        }
    } else if(op==Destroy::Intersection) {
        std::vector<Route> crossings;
        for(const auto& r:result.routes) {
            const auto arcs=edges(problem,r);
            for(std::size_t i=0;i<arcs.size();++i)for(std::size_t j=i+2;j<arcs.size();++j) {
                auto [a,b]=arcs[i];auto [c,d]=arcs[j];
                if(!is_supplier(problem,b)||!is_supplier(problem,c))continue;

                const auto last=std::find(r.begin(),r.end(),c);
                const auto first=std::find(r.begin(),r.end(),b);
                if(first>=last)continue;
                Route reversed=r;std::reverse(reversed.begin()+(first-r.begin()),reversed.begin()+(last-r.begin())+1);
                if(problem.evaluate_route(reversed).cost<problem.evaluate_route(r).cost) {
                    Route nodes;for(int node:{a,b,c,d})if(is_supplier(problem,node)&&std::find(nodes.begin(),nodes.end(),node)==nodes.end())nodes.push_back(node);
                    crossings.push_back(nodes);
                }
            }
        }
        if(!crossings.empty())for(int s:crossings[routing::uniform_index(rng,crossings.size())]) {
            if(result.missing.size()-source.missing.size()>=count)break;remove(result,s);
        }
    } else {
        throw std::invalid_argument("unknown ALNS destroy operator");
    }

    while(result.missing.size()-source.missing.size()<count) {
        if(stop)stop();
        available=assigned(result);remove(result,available[routing::uniform_index(rng,available.size())]);
    }
    validate_state(problem,result);
    return result;
}
}
