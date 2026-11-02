#pragma once
#include <hrlh.hpp>
#include <stdexcept>
namespace hrlh::physical_validation {
inline hrlh_model::ModelSolution base_solution(const hrlh_model::ProblemInstance& raw,
                                                 double C,double P,double F){
    hrlh_model::ModelSolution s;s.status_name="SUBOPTIMAL";
    s.transport_cost=C;s.optional_profit=P;s.latest_departure_sum=F;
    s.objective_values={{"transport_cost",C},{"optional_profit",P},{"latest_departure_sum",F}};
    for(const auto& item:raw.items){
        auto& info=s.item_solution[item.item_id];info.supplier=item.supplier;
        info.dock=item.dock;info.size_class=item.size_class;info.score=item.score;
        info.is_pallet=item.is_pallet;info.is_optional=item.is_optional;
    }
    return s;
}
inline void install_routes(hrlh_model::ModelSolution& s,const hrlh_model::ProblemInstance& raw,
                           const RoutePlan& routes,const std::vector<double>& departures){
    if(routes.size()!=departures.size()||routes.size()>raw.vehicles.size())
        throw std::runtime_error("original validator route/departure mismatch");
    for(std::size_t ri=0;ri<routes.size();++ri){
        const int k=raw.vehicles.at(ri);int previous=raw.start_depot();double time=departures[ri];
        s.routes[k].push_back(previous);s.time_solution[{previous,k}]=time;
        for(int supplier:routes[ri]){
            s.route_arcs[k].push_back({previous,supplier});
            time+=(raw.service_time.count(previous)?raw.service_time.at(previous):0.)
                +raw.travel_time.at({previous,supplier});
            s.routes[k].push_back(supplier);s.time_solution[{supplier,k}]=time;previous=supplier;
        }
        s.route_arcs[k].push_back({previous,raw.end_depot()});
        time+=raw.service_time.at(previous)+raw.travel_time.at({previous,raw.end_depot()});
        s.routes[k].push_back(raw.end_depot());s.time_solution[{raw.end_depot(),k}]=time;
    }
}
inline void install_item(hrlh_model::ModelSolution& s,int id,int k,double x,double y,double z,
                         double length,double width,double height,int delta1){
    auto& info=s.item_solution.at(id);info.loaded=true;info.loaded_vehicle=k;info.loaded_vehicles={k};
    info.x=x;info.y=y;info.z=z;info.actual_length=length;info.actual_width=width;info.actual_height=height;
    info.delta1=delta1;info.delta2=1-delta1;s.vehicle_items[k].push_back(id);s.loaded_items_all.push_back(id);
}
inline hrlh_model::ValidationReport check_original(const hrlh_model::ProblemInstance& raw,
                                                  const CandidateSolution& candidate){
    const auto normalized=hrlh_model::normalized_rotatable_rack_dimensions(raw);
    std::map<int,const hrlh_model::Item*> items;
    for(const auto& item:normalized.items)items.emplace(item.item_id,&item);
    auto s=base_solution(raw,candidate.transport_cost,candidate.optional_profit,candidate.latest_departure_sum);
    install_routes(s,raw,candidate.routes,candidate.route_latest_departures);
    for(std::size_t ri=0;ri<candidate.packing.route_details.size();++ri){
        const auto& d=candidate.packing.route_details[ri];const int k=raw.vehicles.at(ri);
        std::vector<std::vector<const RoutePackingPlacedRect*>> rects(d.specs.size());
        for(const auto& r:d.placed_rects)rects.at(r.spec_index).push_back(&r);
        for(std::size_t si=0;si<d.material_stacks_by_spec.size();++si){
            const auto& stacks=d.material_stacks_by_spec[si];
            if(rects.at(si).size()!=stacks.size())throw std::runtime_error("validator stack/rectangle mismatch");
            for(std::size_t j=0;j<stacks.size();++j){
                const auto& r=*rects[si][j];double z=0;int below=-1;
                for(int id:stacks[j]){
                    const auto& item=*items.at(id);
                    const int delta1=(r.length==item.length&&r.width==item.width)?1:0;

                    install_item(s,id,k,r.y,r.x,z,r.length,r.width,item.height,delta1);
                    if(below>=0)s.stack_relations.push_back({below,id,k,k});
                    z+=item.height;below=id;
                }
            }
        }
    }
    return hrlh_model::validate_solution_against_model(raw,s,1e-5,false);
}
}
