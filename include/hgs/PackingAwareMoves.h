#pragma once
#include "Params.h"
namespace hrlh_hgs_moves {
struct MovePreview {bool valid=false;std::vector<int> first,second;};

MovePreview preview(int kind,const std::vector<int>& a,const std::vector<int>& b,
                    int u,int v,bool same);
struct SwapStarPreview {double delta=std::numeric_limits<double>::infinity();
    std::vector<int> first,second;};
SwapStarPreview best_swap_star(Params& params,const std::vector<int>& a,
    const std::vector<int>& b,double pc,double pt);
}
