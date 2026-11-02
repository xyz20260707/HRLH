#include "PackingAwareMoves.h"
namespace hrlh_hgs_moves {
MovePreview preview(int kind,const std::vector<int>& a,const std::vector<int>& b,
                    int u,int v,bool same){
    MovePreview out;
    if(u<0||u>=static_cast<int>(a.size())||v<-1||v>=static_cast<int>(b.size()))return out;
    out.first=a;out.second=b;
    if(kind<=3){
        int len=kind==1?1:2;
        if(u+len>static_cast<int>(a.size())||(same&&v>=u&&v<u+len))return {};
        std::vector<int> block(a.begin()+u,a.begin()+u+len);
        if(kind==3)std::reverse(block.begin(),block.end());
        out.first.erase(out.first.begin()+u,out.first.begin()+u+len);
        if(same){int pos=v+1;if(v>=u+len)pos-=len;out.first.insert(out.first.begin()+pos,block.begin(),block.end());out.second=out.first;}
        else out.second.insert(out.second.begin()+(v+1),block.begin(),block.end());
    }else if(kind<=6){
        if(v<0)return {};
        int lenU=kind==4?1:2,lenV=kind==6?2:1;
        if(u+lenU>static_cast<int>(a.size())||v+lenV>static_cast<int>(b.size())||(same&&u<v+lenV&&v<u+lenU))return {};
        std::vector<int> left(a.begin()+u,a.begin()+u+lenU),right(b.begin()+v,b.begin()+v+lenV);
        if(same){
            out.first.clear();
            for(int i=0;i<static_cast<int>(a.size());){
                if(i==u){out.first.insert(out.first.end(),right.begin(),right.end());i+=lenU;}
                else if(i==v){out.first.insert(out.first.end(),left.begin(),left.end());i+=lenV;}
                else out.first.push_back(a[i++]);
            }out.second=out.first;
        }else{
            out.first.erase(out.first.begin()+u,out.first.begin()+u+lenU);out.first.insert(out.first.begin()+u,right.begin(),right.end());
            out.second.erase(out.second.begin()+v,out.second.begin()+v+lenV);out.second.insert(out.second.begin()+v,left.begin(),left.end());
        }
    }else if(kind==7){
        if(!same||v<=u+1)return {};
        std::reverse(out.first.begin()+u+1,out.first.begin()+v+1);out.second=out.first;
    }else if(kind==8||kind==9){
        if(same)return {};
        out.first.assign(a.begin(),a.begin()+(u+1));
        if(kind==8){
            std::vector<int> prefix(b.begin(),b.begin()+(v+1));std::reverse(prefix.begin(),prefix.end());out.first.insert(out.first.end(),prefix.begin(),prefix.end());
            out.second.assign(a.begin()+(u+1),a.end());std::reverse(out.second.begin(),out.second.end());out.second.insert(out.second.end(),b.begin()+(v+1),b.end());
        }else{
            out.first.insert(out.first.end(),b.begin()+(v+1),b.end());out.second.assign(b.begin(),b.begin()+(v+1));out.second.insert(out.second.end(),a.begin()+(u+1),a.end());
        }
    }else return {};
    out.valid=true;return out;
}

static std::pair<double,std::vector<int>> insertion(Params& p,const std::vector<int>& r,
                                                  int item,double pc,double pt){
    double best=std::numeric_limits<double>::infinity();std::vector<int> bestRoute;
    for(std::size_t pos=0;pos<=r.size();++pos){auto t=r;t.insert(t.begin()+pos,item);
        double cost=p.routeObjective(t,pc,pt);if(cost<best){best=cost;bestRoute=std::move(t);}}
    return {best,std::move(bestRoute)};
}
SwapStarPreview best_swap_star(Params& p,const std::vector<int>& a,
    const std::vector<int>& b,double pc,double pt){
    SwapStarPreview result;const double base=p.routeObjective(a,pc,pt)+p.routeObjective(b,pc,pt);
    auto keep=[&](double cost,const std::vector<int>& x,const std::vector<int>& y){if(cost-base<result.delta){result.delta=cost-base;result.first=x;result.second=y;}};
    for(std::size_t u=0;u<a.size();++u){p.checkBudget();auto ar=a;ar.erase(ar.begin()+u);
        for(std::size_t v=0;v<b.size();++v){auto br=b;br.erase(br.begin()+v);
            auto x=insertion(p,ar,b[v],pc,pt),y=insertion(p,br,a[u],pc,pt);keep(x.first+y.first,x.second,y.second);}
        auto y=insertion(p,b,a[u],pc,pt);keep(p.routeObjective(ar,pc,pt)+y.first,ar,y.second);
    }
    for(std::size_t v=0;v<b.size();++v){p.checkBudget();auto br=b;br.erase(br.begin()+v);
        auto x=insertion(p,a,b[v],pc,pt);keep(x.first+p.routeObjective(br,pc,pt),x.second,br);}
    return result;
}
}
