#include "independent_checker.hpp"
#include <cmath>
#include <limits>
namespace hrlh::independent {

Report check(const hrlh_model::ProblemInstance& inst,const CandidateSolution& sol){
    Report out;auto fail=[&](const std::string& e){out.errors.push_back(e);};
    std::map<int,const hrlh_model::Item*> items;std::map<int,int> visits,loaded;
    for(const auto& item:inst.items)if(!items.emplace(item.item_id,&item).second)fail("duplicate raw item ID");
    for(int s:inst.suppliers)visits.emplace(s,0);
    if(sol.routes.size()>inst.vehicles.size())fail("fleet limit");
    if(sol.routes.empty())fail("empty final route plan");
    if(!sol.packing.feasible)fail("final packing status is not feasible");
    if(sol.packing.route_details.size()!=sol.routes.size())fail("route/packing output count");
    long double C=0,F=0;
    for(std::size_t ri=0;ri<sol.routes.size();++ri){
        const auto& route=sol.routes[ri];if(route.empty()){fail("empty used route");continue;}
        int previous=0;double duration=0,deadline=std::numeric_limits<double>::infinity();bool known=true;
        C+=inst.fixed_vehicle_cost;
        for(int s:route){auto v=visits.find(s);if(v==visits.end()){fail("unknown supplier");known=false;continue;}
            if(++v->second!=1)fail("supplier not visited exactly once");
            auto d=inst.distance.find({previous,s}),t=inst.travel_time.find({previous,s});
            if(d==inst.distance.end()||t==inst.travel_time.end()){fail("missing route arc");known=false;}
            else{C+=inst.alpha_distance_cost*d->second;duration+=t->second;}
            duration+=inst.service_time.at(s);deadline=std::min(deadline,inst.latest_return.at(s));previous=s;
        }

        int end=inst.suppliers.empty()?1:*std::max_element(inst.suppliers.begin(),inst.suppliers.end())+1;
        auto d=inst.distance.find({previous,end}),t=inst.travel_time.find({previous,end});
        if(d==inst.distance.end()||t==inst.travel_time.end()){fail("missing end-depot arc");known=false;}
        else{C+=inst.alpha_distance_cost*d->second;duration+=t->second;}
        const double depart=deadline-duration;F+=depart;

        if(known&&depart< -1e-9)fail("route return deadline");
        if(ri<sol.route_latest_departures.size()&&(!std::isfinite(sol.route_latest_departures[ri])||std::abs(sol.route_latest_departures[ri]-depart)>1e-8))fail("route F output mismatch");
        if(ri>=sol.packing.route_details.size())continue;
        const auto& detail=sol.packing.route_details[ri];
        if(!detail.feasible)fail("route packing status is not feasible");
        if(detail.route!=route)fail("route/packing output identity");
        if(detail.material_stacks_by_spec.size()!=detail.specs.size()){fail("missing complete single-material stack export");continue;}
        std::vector<std::vector<const RoutePackingPlacedRect*>> rectangles(detail.specs.size());
        for(const auto& r:detail.placed_rects){
            if(r.spec_index<0||static_cast<std::size_t>(r.spec_index)>=detail.specs.size()){fail("rectangle spec index");continue;}
            rectangles[r.spec_index].push_back(&r);
            if(!std::isfinite(r.x)||!std::isfinite(r.y)||!std::isfinite(r.width)||!std::isfinite(r.length)
                ||r.width<=0||r.length<=0||r.x<0||r.y<0||r.x+r.width>inst.vehicle_width||r.y+r.length>inst.vehicle_length)fail("rectangle boundary/finite dimensions");
        }
        for(std::size_t a=0;a<detail.placed_rects.size();++a)for(std::size_t b=a+1;b<detail.placed_rects.size();++b){
            const auto& x=detail.placed_rects[a];const auto& y=detail.placed_rects[b];
            if(x.x<y.x+y.width&&y.x<x.x+x.width&&x.y<y.y+y.length&&y.y<x.y+x.length)fail("rectangle overlap");
        }
        std::int64_t routeProfit=0;
        for(std::size_t si=0;si<detail.specs.size();++si){
            const auto& stacks=detail.material_stacks_by_spec[si];const auto& spec=detail.specs[si];
            if(rectangles[si].size()!=stacks.size()){fail("stack/rectangle bijection");continue;}
            const int optional=static_cast<int>(std::count_if(rectangles[si].begin(),rectangles[si].end(),[](auto r){return r->is_optional;}));
            if(static_cast<int>(rectangles[si].size())-optional!=spec.mandatory_count)fail("mandatory footprint count");
            if(si>=detail.loaded_mandatory_count_by_spec.size()||detail.loaded_mandatory_count_by_spec[si]!=spec.mandatory_count)fail("reported mandatory footprint count");
            if(si>=detail.loaded_optional_count_by_spec.size()||optional!=detail.loaded_optional_count_by_spec[si]||optional>spec.optional_count)fail("optional footprint count");
            for(std::size_t k=0;k<stacks.size();++k){
                const auto& stack=stacks[k];const auto& rect=*rectangles[si][k];std::int64_t height=0;
                if(stack.empty())fail("empty material stack");
                for(std::size_t i=0;i<stack.size();++i){
                    auto found=items.find(stack[i]);if(found==items.end()){fail("unknown material ID");continue;}
                    const auto& item=*found->second;
                    if(++loaded[item.item_id]!=1)fail("duplicate material assignment");
                    if(std::find(route.begin(),route.end(),item.supplier)==route.end())fail("material supplier not on route");
                    const bool original=rect.width==item.width&&rect.length==item.length;
                    const bool rotated=rect.width==item.length&&rect.length==item.width;
                    if(!original&&!rotated)fail("material footprint orientation");
                    if(item.is_pallet&&stack.size()!=1)fail("pallet not on floor singleton");
                    height+=item.height;
                    if(item.is_optional){if(item.score>std::numeric_limits<std::int64_t>::max()-routeProfit)fail("profit integer overflow");else routeProfit+=item.score;}
                    for(std::size_t j=0;j<i;++j){
                        auto other=items.find(stack[j]);if(other==items.end())continue;
                        auto q=inst.stack_compatibility.find({stack[j],stack[i]});
                        if(item.is_pallet||other->second->is_pallet||item.supplier!=other->second->supplier
                            ||q==inst.stack_compatibility.end()||q->second!=1)fail("incompatible rack stack");
                    }
                }
                if(height>inst.vehicle_height)fail("stack height");
            }
        }
        if(routeProfit!=detail.total_optional_priority_score)fail("route physical P mismatch");
        if(routeProfit>std::numeric_limits<std::int64_t>::max()-out.P)fail("total profit overflow");else out.P+=routeProfit;
    }
    for(auto [s,count]:visits)if(count!=1)fail("supplier coverage");
    for(const auto& item:inst.items)if(!item.is_optional&&loaded[item.item_id]!=1)fail("mandatory material coverage");
    out.C=static_cast<double>(C);out.F=static_cast<double>(F);
    if(!std::isfinite(out.C)||!std::isfinite(out.F))fail("raw objective recomputation is non-finite");
    if(!std::isfinite(sol.transport_cost)||std::abs(sol.transport_cost-out.C)>1e-8*std::max(1.,std::abs(out.C)))fail("C output mismatch");
    if(!std::isfinite(sol.latest_departure_sum)||std::abs(sol.latest_departure_sum-out.F)>1e-8*std::max(1.,std::abs(out.F)))fail("F output mismatch");
    if(sol.optional_profit!=out.P||sol.packing.total_optional_priority_score!=out.P)fail("physical P output mismatch");
    if(sol.route_latest_departures.size()!=sol.routes.size())fail("route F count");
    return out;
}
}
