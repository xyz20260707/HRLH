#pragma once
#include <hrlh.hpp>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <cctype>
namespace hrlh::io {
inline std::string trim(std::string s){
    const auto white=[](unsigned char c){return std::isspace(c);};
    s.erase(s.begin(),std::find_if_not(s.begin(),s.end(),white));s.erase(std::find_if_not(s.rbegin(),s.rend(),white).base(),s.end());return s;
}
inline HrlhConfig read_config(const std::filesystem::path& path){
    std::ifstream input(path);if(!input)throw std::runtime_error("config not found");
    std::map<std::string,std::string> kv;std::string section,line;
    while(std::getline(input,line)){
        if(line.starts_with("\xef\xbb\xbf"))line.erase(0,3);
        auto at=line.find_first_of("#;");line=trim(line.substr(0,at));if(line.empty())continue;
        if(line.front()=='['&&line.back()==']'){section=trim(line.substr(1,line.size()-2));
            std::transform(section.begin(),section.end(),section.begin(),[](unsigned char c){return std::tolower(c);});continue;}
        auto eq=line.find('=');if(eq==std::string::npos)throw std::runtime_error("invalid INI line");
        auto key=trim(line.substr(0,eq));std::transform(key.begin(),key.end(),key.begin(),[](unsigned char c){return std::tolower(c);});
        if(key.find('.')==std::string::npos&&!section.empty())key=section+'.'+key;kv[key]=trim(line.substr(eq+1));
    }
    auto cfg=default_hrlh_config();
    if(kv.count("instance.seed"))apply_hrlh_config_value(cfg,"global.seed",kv.at("instance.seed"));
    if(kv.count("preprocess.require_preprocess_optimal"))apply_hrlh_config_value(cfg,"preprocess.require_preprocess_optimal",kv.at("preprocess.require_preprocess_optimal"));
    for(const auto& [key,value]:kv){
        if(key=="instance.seed"||key=="preprocess.require_preprocess_optimal")continue;
        if(key=="preprocess.require_optimal"){apply_hrlh_config_value(cfg,"preprocess.require_preprocess_optimal",value);continue;}
        if(!apply_hrlh_config_value(cfg,key,value)&&!key.starts_with("instance.")&&!key.starts_with("random_instance."))throw std::runtime_error("unknown solver key: "+key);
    }
    if(!std::isfinite(cfg.max_wall_seconds)||!std::isfinite(cfg.optional_profit_intensification_budget_fraction))throw std::runtime_error("non-finite comparison time budget/fraction");
    validate_hrlh_config(cfg);return cfg;
}
inline std::string quote(const std::string& v){std::ostringstream o;o<<'"';for(unsigned char c:v){
    if(c=='"'||c=='\\')o<<'\\'<<c;else if(c=='\n')o<<"\\n";else if(c=='\r')o<<"\\r";else if(c=='\t')o<<"\\t";
    else if(c<32)throw std::invalid_argument("control character in JSON metadata");else o<<c;}o<<'"';return o.str();}

inline hrlh_model::ProblemInstance read_case(std::istream& input){
    hrlh_model::ProblemInstance inst;int suppliers=0,vehicles=0,items=0,arcs=0,q=0;
    input>>inst.vehicle_length>>inst.vehicle_width>>inst.vehicle_height>>inst.alpha_distance_cost>>inst.fixed_vehicle_cost>>suppliers>>vehicles>>items>>arcs>>q;
    if(!input||suppliers<1||vehicles<1||items<0||arcs<0||q<0)throw std::runtime_error("invalid protocol header");
    for(int k=1;k<=vehicles;++k)inst.vehicles.push_back(k);
    for(int k=0;k<suppliers;++k){int s;double service,latest;input>>s>>service>>latest;if(!input)throw std::runtime_error("truncated supplier data");inst.suppliers.push_back(s);inst.service_time[s]=service;inst.latest_return[s]=latest;}
    for(int k=0;k<items;++k){int id,s,l,w,h,pallet,optional,dock,size,mode;std::int64_t profit;
        input>>id>>s>>l>>w>>h>>pallet>>optional>>profit>>dock>>size>>mode;
        if(!input)throw std::runtime_error("truncated protocol item");
        inst.items.emplace_back(id,s,l,w,h,pallet!=0,optional!=0,profit,dock,size,"","",static_cast<hrlh_model::StackMode>(mode));}
    for(int k=0;k<arcs;++k){int a,b;double d,t;input>>a>>b>>d>>t;if(!input)throw std::runtime_error("truncated protocol arc");
        if(!inst.distance.emplace(std::make_pair(a,b),d).second)throw std::runtime_error("duplicate protocol arc");inst.travel_time[{a,b}]=t;}
    for(int k=0;k<q;++k){int a,b,v;input>>a>>b>>v;if(!input)throw std::runtime_error("truncated q_mb");
        if(!inst.stack_compatibility.emplace(std::make_pair(a,b),v).second)throw std::runtime_error("duplicate q_mb");}
    for(int s:inst.suppliers){inst.distance[{s,0}]=inst.distance.at({s,inst.end_depot()});inst.travel_time[{s,0}]=inst.travel_time.at({s,inst.end_depot()});
        inst.distance[{inst.end_depot(),s}]=inst.distance.at({0,s});inst.travel_time[{inst.end_depot(),s}]=inst.travel_time.at({0,s});}
    inst.distance[{0,inst.end_depot()}]=inst.distance[{inst.end_depot(),0}]=0;
    inst.travel_time[{0,inst.end_depot()}]=inst.travel_time[{inst.end_depot(),0}]=0;
    std::string trailing;if(input>>trailing)throw std::runtime_error("trailing protocol tokens");
    hrlh_model::validate_instance(inst);return inst;
}
}
