#pragma once
#include <hrlh.hpp>
namespace hrlh {

inline void recover_preprocessed_material_witness(
    const hrlh_model::ProblemInstance& inst,
    const hrlh_components::stack_preprocessing::PreprocessAllGroupsResult& prep,
    CandidateSolution& solution) {
    std::map<std::string,const hrlh_components::stack_preprocessing::GroupPreprocessResult*> groups;
    for(const auto& [key,group]:prep.groups)
        groups.emplace(std::to_string(std::get<0>(key))+":"+std::to_string(std::get<1>(key))+":"+std::to_string(std::get<2>(key)),&group);
    for(auto& detail:solution.packing.route_details) {
        if(!detail.material_stacks_by_spec.empty())continue;
        if(detail.loaded_optional_count_by_spec.size()!=detail.specs.size())
            throw std::runtime_error("Cannot recover material witness: missing footprint counts");
        detail.material_stacks_by_spec.resize(detail.specs.size());
        for(std::size_t si=0;si<detail.specs.size();++si) {
            const auto& spec=detail.specs[si];auto& stacks=detail.material_stacks_by_spec[si];
            const int extra=detail.loaded_optional_count_by_spec[si];
            if(extra<0||extra>spec.optional_count)
                throw std::runtime_error("Cannot recover material witness: optional footprint count out of range");
            if(spec.type_id.rfind("P:",0)==0) {
                for(const auto& item:inst.items)
                    if(item.is_pallet&&std::find(detail.route.begin(),detail.route.end(),item.supplier)!=detail.route.end()&&
                       spec.type_id=="P:"+std::to_string(item.supplier)+":"+std::to_string(item.width)+"x"+std::to_string(item.length))
                        stacks.push_back({item.item_id});
            } else {
                const auto found=groups.find(spec.type_id);
                if(found==groups.end())throw std::runtime_error("Cannot recover material witness: rack group missing");
                const auto& group=*found->second;
                if(group.spread_joint_enabled)stacks=group.spread_joint_curve.at(extra).stacks;
                else {
                    stacks=group.baseline_result.final_stacks;
                    if(extra>0) {
                        const auto& additional=group.value_table.at(extra).stacks;
                        stacks.insert(stacks.end(),additional.begin(),additional.end());
                    }
                }
            }
            if(static_cast<int>(stacks.size())!=spec.mandatory_count+extra||
               std::any_of(stacks.begin(),stacks.end(),[](const auto& stack){return stack.empty();}))
                throw std::runtime_error("Stored preprocessing assignments do not provide a nonempty material stack for every returned footprint; no alternative packing is substituted");
        }
    }
}

inline void expand_compound_witness(RoutePackingDetail& detail){
    std::vector<int> remaining=detail.loaded_mandatory_count_by_spec;
    if(remaining.size()!=detail.specs.size()||detail.loaded_optional_count_by_spec.size()!=remaining.size())
        throw std::runtime_error("incomplete counts in physical witness");
    for(std::size_t i=0;i<remaining.size();++i){
        const auto optional=detail.loaded_optional_count_by_spec[i];
        if(remaining[i]<0||optional<0||optional>std::numeric_limits<int>::max()-remaining[i]
            ||detail.specs[i].width<=0||detail.specs[i].length<=0)
            throw std::runtime_error("invalid counts/dimensions in physical witness");
        remaining[i]+=optional;
    }
    std::vector<int> observed(detail.specs.size(),0);bool atomic=true;
    for(const auto& r:detail.placed_rects){
        if(r.spec_index<0||static_cast<std::size_t>(r.spec_index)>=detail.specs.size()){atomic=false;break;}
        const auto& s=detail.specs[r.spec_index];
        if(!((r.width==s.width&&r.length==s.length)||(r.width==s.length&&r.length==s.width))){atomic=false;break;}
        ++observed[r.spec_index];
    }
    if(atomic&&observed==remaining)return;
    std::vector<RoutePackingPlacedRect> expanded;std::set<std::vector<int>> failed;
    std::function<bool(int)> block;
    block=[&](int bi){
        if(bi==static_cast<int>(detail.placed_rects.size()))return std::all_of(remaining.begin(),remaining.end(),[](int n){return n==0;});
        auto state=remaining;state.insert(state.begin(),bi);if(failed.count(state))return false;
        const auto& b=detail.placed_rects[bi];
        if(!std::isfinite(b.x)||!std::isfinite(b.y)||!std::isfinite(b.width)||!std::isfinite(b.length)
            ||b.x<0||b.y<0||b.width<=0||b.length<=0
            ||b.x>std::numeric_limits<int>::max()||b.y>std::numeric_limits<int>::max()
            ||b.width>std::numeric_limits<int>::max()||b.length>std::numeric_limits<int>::max()
            ||b.x!=std::floor(b.x)||b.y!=std::floor(b.y)||b.width!=std::floor(b.width)||b.length!=std::floor(b.length))
            throw std::runtime_error("non-integral compound strip in physical witness");
        for(int axis=0;axis<2;++axis){
            const int common=static_cast<int>(axis?b.length:b.width),target=static_cast<int>(axis?b.width:b.length);
            struct Option{int type,w,h,extent;};std::vector<Option> options;
            for(int ti=0;ti<static_cast<int>(detail.specs.size());++ti){
                const auto& s=detail.specs[ti];
                if(s.width==common)options.push_back({ti,axis?s.length:s.width,axis?s.width:s.length,s.length});
                else if(s.length==common)options.push_back({ti,axis?s.width:s.length,axis?s.length:s.width,s.width});
            }
            std::function<bool(int,int,int)> fill;
            fill=[&](int oi,int left,int cursor){
                if(left==0)return block(bi+1);
                if(oi==static_cast<int>(options.size()))return false;
                const auto o=options[oi];const int maximum=std::min(remaining[o.type],left/o.extent);
                for(int n=maximum;n>=0;--n){
                    remaining[o.type]-=n;
                    for(int k=0;k<n;++k)expanded.push_back({o.type,false,0,b.x+(axis?cursor+k*o.extent:0),b.y+(axis?0:cursor+k*o.extent),double(o.w),double(o.h)});
                    if(fill(oi+1,left-n*o.extent,cursor+n*o.extent))return true;
                    expanded.resize(expanded.size()-n);remaining[o.type]+=n;
                }
                return false;
            };
            if(fill(0,target,0))return true;
        }
        failed.insert(std::move(state));return false;
    };
    if(!block(0))throw std::runtime_error("cannot reconstruct returned compound strip; preserve failed witness");
    std::vector<int> seen(detail.specs.size(),0);
    for(auto& r:expanded){const int count=seen[r.spec_index]++;const int mandatory=detail.loaded_mandatory_count_by_spec[r.spec_index];
        r.is_optional=count>=mandatory;r.rank_in_type=r.is_optional?count-mandatory:count;}
    detail.placed_rects=std::move(expanded);
}

inline void complete_physical_witness(CandidateSolution& sol){
    for(auto& detail:sol.packing.route_details)expand_compound_witness(detail);
}
}
