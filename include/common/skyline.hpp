#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <list>
#include <iterator>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <iomanip>
#include <climits>
#include <functional>
#ifdef _MSC_VER
#include <intrin.h>
#endif

using namespace std;

static constexpr int INF = 1000000007;

static thread_local const std::function<void()>* g_skyline_stop_callback = nullptr;

struct SkylineStopCallbackScope {
    const std::function<void()>* previous = nullptr;
    explicit SkylineStopCallbackScope(const std::function<void()>& cb)
        : previous(g_skyline_stop_callback) {
        if (cb) g_skyline_stop_callback = &cb;
    }
    ~SkylineStopCallbackScope() { g_skyline_stop_callback = previous; }
};

static inline void check_skyline_stop() {
    if (g_skyline_stop_callback && *g_skyline_stop_callback) {
        (*g_skyline_stop_callback)();
    }
}

static long long py_round_nonneg(double x){
    double fl_d=floor(x); long long fl=(long long)fl_d; double frac=x-fl_d;
    if(frac>0.5) return fl+1;
    if(frac<0.5) return fl;
    return (fl%2==0) ? fl : fl+1;
}

struct Item {
    int id{};
    int length{};
    int width{};
    int min_dim{};
};

struct Placement {
    vector<int> ids;
    int w = -1;
    int l = -1;
    int mode = 0;
    bool found = false;
};

struct Rect {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    vector<int> ids;
};

struct Segment {
    int id = -1;
    int x = 0;
    int width = 0;
    int y = 0;
    bool alive = true;
};

struct MinGapResult {
    int min_id = -1;
    int left_id = -1;
    int right_id = -1;
    int hl = INF;
    int hr = INF;
    int gap_width = 0;
    int height_constraint = 0;
    bool skip = false;
};

class Skyline {
public:
    int next_id = 0;

    vector<Segment> segs;
    list<int> active_segments;
    vector<list<int>::iterator> list_positions;
    vector<int> min_heap;
    vector<size_t> heap_positions;
    unordered_map<int,int> x_to_id;
    unordered_map<int,int> xplusw_to_id;
    map<pair<int,int>,int> ordered_active;
    long long active_width = 0;
    long long active_width_y = 0;

    Skyline() = default;
    Skyline(const Skyline& other)
        : next_id(other.next_id), segs(other.segs),
          active_segments(other.active_segments), list_positions(segs.size()),
          min_heap(other.min_heap), heap_positions(other.heap_positions),
          x_to_id(other.x_to_id), xplusw_to_id(other.xplusw_to_id),
          ordered_active(other.ordered_active), active_width(other.active_width),
          active_width_y(other.active_width_y) {

        for (auto it=active_segments.begin(); it!=active_segments.end(); ++it)
            list_positions[*it]=it;
    }
    Skyline& operator=(const Skyline& other) {
        if (this != &other) { Skyline copy(other); *this=std::move(copy); }
        return *this;
    }

    Skyline(Skyline&& other) : Skyline() { swap_state(other); }
    Skyline& operator=(Skyline&& other) {
        if (this != &other) {
            Skyline moved(std::move(other));
            swap_state(moved);
        }
        return *this;
    }

    bool empty() const { return min_heap.empty(); }

    int add_node(int x, int width, int y) {
        int id = next_id++;
        if ((int)segs.size() <= id) segs.resize(id + 1);
        segs[id] = Segment{id, x, width, y, true};
        list_positions.resize(segs.size());
        heap_positions.resize(segs.size(), no_heap_position);

        auto position=active_segments.end();
        auto left=xplusw_to_id.find(x);
        auto right=x_to_id.find(x+width);
        if (left!=xplusw_to_id.end() && alive(left->second)
            && segs[left->second].x < x) {
            position=std::next(list_positions[left->second]);
        } else if (right!=x_to_id.end() && alive(right->second)
                   && x < segs[right->second].x) {
            position=list_positions[right->second];
        } else {
            auto successor=ordered_active.lower_bound({x,id});
            if (successor!=ordered_active.end()) position=list_positions[successor->second];
        }
        list_positions[id]=active_segments.insert(position,id);
        ordered_active.emplace(make_pair(x,id),id);
        active_width += width;
        active_width_y += 1LL * width * y;
        heap_positions[id]=min_heap.size();
        min_heap.push_back(id);
        sift_up(min_heap.size()-1);
        x_to_id[x] = id;
        xplusw_to_id[x + width] = id;
        return id;
    }

    bool alive(int id) const { return id >= 0 && id < (int)segs.size() && segs[id].alive; }

    void remove_node(int id) {
        if (!alive(id)) return;
        auto &s = segs[id];
        remove_from_heap(id);
        active_segments.erase(list_positions[id]);
        list_positions[id]=list<int>::iterator{};
        ordered_active.erase({s.x,id});
        active_width -= s.width;
        active_width_y -= 1LL * s.width * s.y;
        auto itx = x_to_id.find(s.x);
        if (itx != x_to_id.end() && itx->second == id) x_to_id.erase(itx);
        auto itxw = xplusw_to_id.find(s.x + s.width);
        if (itxw != xplusw_to_id.end() && itxw->second == id) xplusw_to_id.erase(itxw);
        s.alive = false;
    }

    optional<MinGapResult> get_min_node() {
        if (min_heap.empty()) return nullopt;
        int id=min_heap.front();
        auto s = segs[id];
        int left = -1, right = -1;
        auto lit = xplusw_to_id.find(s.x);
        if (lit != xplusw_to_id.end() && alive(lit->second)) left = lit->second;
        auto rit = x_to_id.find(s.x + s.width);
        if (rit != x_to_id.end() && alive(rit->second)) right = rit->second;
        int hl = left >= 0 ? segs[left].y - s.y : INF;
        int hr = right >= 0 ? segs[right].y - s.y : INF;
        remove_node(id);
        return MinGapResult{id, left, right, hl, hr, s.width, 0, false};
    }

    long long upper_area(int limit_y) const {

        return 1LL * limit_y * active_width - active_width_y;
    }

private:
    void swap_state(Skyline& other) noexcept {
        std::swap(next_id,other.next_id);
        segs.swap(other.segs);
        active_segments.swap(other.active_segments);
        list_positions.swap(other.list_positions);
        min_heap.swap(other.min_heap);
        heap_positions.swap(other.heap_positions);
        x_to_id.swap(other.x_to_id);
        xplusw_to_id.swap(other.xplusw_to_id);
        ordered_active.swap(other.ordered_active);
        std::swap(active_width,other.active_width);
        std::swap(active_width_y,other.active_width_y);
    }
    static constexpr size_t no_heap_position=std::numeric_limits<size_t>::max();
    bool heap_less(int a,int b) const {
        const auto& x=segs[a]; const auto& y=segs[b];
        return std::tie(x.y,x.x,x.id)<std::tie(y.y,y.x,y.id);
    }
    void swap_heap(size_t a,size_t b) {
        std::swap(min_heap[a],min_heap[b]);
        heap_positions[min_heap[a]]=a;
        heap_positions[min_heap[b]]=b;
    }
    void sift_up(size_t i) {
        while (i>0) {
            size_t p=(i-1)/2;
            if (!heap_less(min_heap[i],min_heap[p])) break;
            swap_heap(i,p); i=p;
        }
    }
    void sift_down(size_t i) {
        while (2*i+1<min_heap.size()) {
            size_t child=2*i+1;
            if (child+1<min_heap.size() && heap_less(min_heap[child+1],min_heap[child])) ++child;
            if (!heap_less(min_heap[child],min_heap[i])) break;
            swap_heap(i,child); i=child;
        }
    }
    void remove_from_heap(int id) {
        size_t i=heap_positions[id],last=min_heap.size()-1;
        if (i!=last) swap_heap(i,last);
        min_heap.pop_back();
        heap_positions[id]=no_heap_position;
        if (i<min_heap.size()) {
            if (i>0 && heap_less(min_heap[i],min_heap[(i-1)/2])) sift_up(i);
            else sift_down(i);
        }
    }
};

struct SegmentTree {
    int n = 0, sz = 0;
    vector<pair<int,int>> tree;
    SegmentTree() = default;
    explicit SegmentTree(const vector<pair<int,int>>& data) { build(data); }
    void build(const vector<pair<int,int>>& data) {
        sz = (int)data.size();
        n = 1; while (n < max(1, sz)) n <<= 1;
        tree.assign(2*n, {INF, -1});
        for (int i = 0; i < sz; ++i) tree[n+i] = data[i];
        for (int i = n-1; i >= 1; --i) tree[i] = min(tree[2*i], tree[2*i+1]);
    }
    void update(int pos, int val, int id) {
        if (pos < 0 || pos >= sz) return;
        int i = n + pos;
        tree[i] = {val, id};
        for (i >>= 1; i >= 1; i >>= 1) {
            tree[i] = min(tree[2*i], tree[2*i+1]);
            if (i == 1) break;
        }
    }
    pair<int,int> query(int l, int r) const {
        if (l > r || l < 0 || r >= sz || sz == 0) return {INF, -1};
        l += n; r += n;
        pair<int,int> res = {INF, -1};
        while (l <= r) {
            if (l & 1) res = min(res, tree[l++]);
            if (!(r & 1)) res = min(res, tree[r--]);
            l >>= 1; r >>= 1;
        }
        return res;
    }
};

static int first_desc_leq(const vector<int>& a, int val) {
    int lo = 0, hi = (int)a.size();
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (a[mid] <= val) hi = mid;
        else lo = mid + 1;
    }
    return lo;
}
static int first_asc_geq(const vector<tuple<int,int,int>>& a, int val) {
    int lo = 0, hi = (int)a.size();
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (get<0>(a[mid]) >= val) hi = mid;
        else lo = mid + 1;
    }
    return lo;
}

struct DimensionPairHash {
    size_t operator()(const pair<int,int>& key) const noexcept {

        const uint64_t mixed=uint64_t(uint32_t(key.first))*0x9e3779b185ebca87ULL
                            ^ uint32_t(key.second);
        return static_cast<size_t>(mixed^(mixed>>32));
    }
};

class DimensionPairIndex {
public:
    unordered_map<pair<int,int>, vector<int>, DimensionPairHash> data;
    unordered_set<int> used;
    mutable unordered_map<pair<int,int>, size_t, DimensionPairHash> cursors;
    int active = 0;

    void build(const vector<Item>& items) {
        data.clear(); used.clear(); cursors.clear(); active = (int)items.size();
        for (const auto &it : items) {
            pair<int,int> original{it.length, it.width};
            pair<int,int> rev{it.width, it.length};
            pair<int,int> key = data.count(rev) ? rev : original;
            data[key].push_back(it.id);
        }
    }
    optional<int> first(pair<int,int> key) const {
        auto it = data.find(key);
        if (it == data.end()) return nullopt;
        if (!it->second.empty() && !used.count(it->second.front())) return it->second.front();
        auto& p=cursors[key];
        while (p<it->second.size() && used.count(it->second[p])) ++p;
        if (p<it->second.size()) return it->second[p];
        return nullopt;
    }
    void mark_used(int id) {
        if (used.insert(id).second) --active;
    }
};

class EdgeDimensionIndex {
public:
    struct DimInfo {
        vector<int> ids;
        vector<int> other_dims;
        SegmentTree st;
        map<int,int> id_to_index;
    };
    map<int, DimInfo> data;
    map<long long,int> pos;
    map<int, Item> item_map;
    static long long key(int dim, int id) { return (static_cast<long long>(dim) << 32) ^ (unsigned)id; }

    void build(const vector<Item>& items, const map<int,int>& simple_idx) {
        data.clear(); pos.clear(); item_map.clear();
        map<int, vector<tuple<int,int,int>>> groups;
        for (auto &it : items) {
            item_map[it.id] = it;
            unordered_set<int> seen;
            vector<pair<int,int>> dims{{it.length,it.width},{it.width,it.length}};
            for (auto [dim, other] : dims) {
                if (seen.insert(dim).second) groups[dim].push_back({it.id, other, simple_idx.at(it.id)});
            }
        }
        for (auto &kv : groups) {
            int dim = kv.first;
            auto group = kv.second;
            sort(group.begin(), group.end(), [](auto &a, auto &b){
                if (get<1>(a) != get<1>(b)) return get<1>(a) > get<1>(b);
                return get<2>(a) < get<2>(b);
            });
            DimInfo info;
            vector<pair<int,int>> st_data;
            for (int i=0;i<(int)group.size();++i) {
                int id=get<0>(group[i]), other=get<1>(group[i]), orig=get<2>(group[i]);
                info.ids.push_back(id); info.other_dims.push_back(other); info.id_to_index[id]=i;
                st_data.push_back({orig,id});
                pos[key(dim,id)] = i;
            }
            info.st.build(st_data);
            data[dim] = std::move(info);
        }
    }
    bool has_dim(int dim) const { return data.find(dim) != data.end(); }
    optional<int> query_min_index(int dim, int max_other) {
        auto it = data.find(dim); if (it == data.end()) return nullopt;
        DimInfo &info = it->second;
        int p = first_desc_leq(info.other_dims, max_other);
        if (p == (int)info.other_dims.size()) return nullopt;
        auto [val,id] = info.st.query(p, (int)info.ids.size()-1);
        if (val >= INF) return nullopt;
        return id;
    }
    vector<pair<int,int>> query_top_k_indices(int dim, int max_other, int k=20) {
        vector<pair<int,int>> ans;
        auto it = data.find(dim); if (it == data.end()) return ans;
        DimInfo &info = it->second;
        int p = first_desc_leq(info.other_dims, max_other);
        if (p == (int)info.other_dims.size()) return ans;
        vector<tuple<int,int,int>> removed;
        int end = (int)info.ids.size()-1;
        for (int c=0;c<k;++c) {
            auto [val,id] = info.st.query(p,end);
            if (val >= INF || id < 0) break;
            int idx = info.id_to_index[id];
            ans.push_back({id, info.other_dims[idx]});
            info.st.update(idx, INF, id);
            removed.push_back({idx,val,id});
        }
        for (auto [idx,val,id] : removed) info.st.update(idx,val,id);
        return ans;
    }
    void mark_used(int id) {
        auto im = item_map.find(id); if (im == item_map.end()) return;
        vector<int> dims{im->second.length}; if(im->second.width != im->second.length) dims.push_back(im->second.width);
        for (int dim : dims) {
            auto dit = data.find(dim); if (dit == data.end()) continue;
            auto pit = pos.find(key(dim,id)); if (pit == pos.end()) continue;
            int idx = pit->second;
            dit->second.st.update(idx, INF, id);
            pos.erase(pit);
        }
    }
};

class FeasibleFootprintIndex {
public:
    vector<tuple<int,int,int>> sorted_min;
    map<int,bool> used;
    map<int,int> id_to_pos;
    map<int, Item> item_map;
    SegmentTree st;
    int min_ptr = 0;
    int current_min_dim = INF;

    struct Orientation { int width, length, id, priority; };
    vector<Orientation> orientations;
    vector<int> length_order;
    map<int,array<int,2>> orientation_positions;
    SegmentTree orientation_tree;
    size_t invalid_length_prefix = 0;
    bool orientation_index_ready = false;

    void build(const vector<Item>& items, const map<int,int>& simple_idx) {
        sorted_min.clear(); used.clear(); id_to_pos.clear(); item_map.clear();
        min_ptr = 0; current_min_dim = INF;
        orientations.clear(); length_order.clear(); orientation_positions.clear();
        invalid_length_prefix = 0; orientation_index_ready = false;
        orientation_tree = SegmentTree{};
        for (auto &it : items) {
            sorted_min.push_back({it.min_dim, it.id, simple_idx.at(it.id)});
            used[it.id] = false; item_map[it.id] = it;
        }
        sort(sorted_min.begin(), sorted_min.end(), [](auto &a, auto &b){
            if (get<0>(a) != get<0>(b)) return get<0>(a) < get<0>(b);
            return get<2>(a) < get<2>(b);
        });
        vector<pair<int,int>> stdata;
        for (int i=0;i<(int)sorted_min.size();++i) {
            int id=get<1>(sorted_min[i]), simple=get<2>(sorted_min[i]);
            id_to_pos[id]=i; stdata.push_back({simple,id});
        }
        st.build(stdata); find_min();
    }
    void build_orientation_index() {

        for (auto [md,id,priority]:sorted_min) {
            const auto& it=item_map.at(id);
            orientations.push_back({it.width,it.length,it.id,priority});
            orientations.push_back({it.length,it.width,it.id,priority});
        }
        sort(orientations.begin(),orientations.end(),[](const auto& a,const auto& b) {
            return tie(a.width,a.length,a.id)<tie(b.width,b.length,b.id);
        });
        vector<pair<int,int>> oriented_data;
        for (int i=0;i<(int)orientations.size();++i) {
            const auto& o=orientations[i];
            auto [p,added]=orientation_positions.emplace(o.id,array<int,2>{i,-1});
            if (!added) p->second[1]=i;
            length_order.push_back(i);
            oriented_data.push_back({is_used(o.id)?INF:o.priority,o.id});
        }
        sort(length_order.begin(),length_order.end(),[&](int a,int b) {
            if (orientations[a].length!=orientations[b].length)
                return orientations[a].length>orientations[b].length;
            return a<b;
        });
        orientation_tree.build(oriented_data);
        orientation_index_ready = true;
    }
    void find_min() {
        current_min_dim = INF;
        while (min_ptr < (int)sorted_min.size()) {
            int md=get<0>(sorted_min[min_ptr]), id=get<1>(sorted_min[min_ptr]);
            if (!used[id]) { current_min_dim = md; break; }
            ++min_ptr;
        }
    }
    int get_min_dim() const { return current_min_dim; }

    bool check_condition(const Item& item, int A, int B) const {
        int d1=item.length, d2=item.width, other=max(d1,d2);
        if (other > A) return other <= B;
        return d1 <= B || d2 <= B;
    }
    optional<int> query_min_index_with_condition(int A, int B) {
        int short_end=first_asc_geq(sorted_min,A+1);
        if (short_end==0) return nullopt;
        auto [first_priority,first_id]=st.query(0,short_end-1);
        if (first_priority>=INF || first_id<0) return nullopt;
        if (check_condition(item_map.at(first_id),A,B)) return first_id;
        if (!orientation_index_ready) build_orientation_index();

        while (invalid_length_prefix>0 &&
               orientations[length_order[invalid_length_prefix-1]].length<=B) {
            int p=length_order[--invalid_length_prefix];
            const auto& o=orientations[p];
            if (!is_used(o.id)) orientation_tree.update(p,o.priority,o.id);
        }
        while (invalid_length_prefix<length_order.size() &&
               orientations[length_order[invalid_length_prefix]].length>B) {
            int p=length_order[invalid_length_prefix++];
            orientation_tree.update(p,INF,orientations[p].id);
        }
        auto end=upper_bound(orientations.begin(),orientations.end(),A,
            [](int width,const Orientation& o){return width<o.width;});
        if (end==orientations.begin()) return nullopt;
        auto [priority,id]=orientation_tree.query(0,int(end-orientations.begin())-1);
        if (priority>=INF || id<0) return nullopt;
        return id;
    }
    void mark_used(int id) {
        auto it=used.find(id); if (it==used.end() || it->second) return;
        it->second = true;
        int p = id_to_pos[id];
        st.update(p, INF, id);
        if (orientation_index_ready) {
            auto oriented=orientation_positions.find(id);
            if (oriented!=orientation_positions.end())
                for (int pos:oriented->second) if (pos>=0) orientation_tree.update(pos,INF,id);
        }
        if (current_min_dim != INF && min_ptr < (int)sorted_min.size() && get<1>(sorted_min[min_ptr]) == id) find_min();
    }
    bool is_used(int id) const { auto it=used.find(id); return it!=used.end() && it->second; }
};

static optional<vector<pair<int,int>>> solve_subset_sum(const vector<pair<int,int>>& cand, int target) {
    if (target <= 0 || cand.empty()) return nullopt;
    long long sum = 0; for (auto &p: cand) sum += p.second;
    if (sum < target) return nullopt;
    if (sum == target) return cand;
    vector<int> dp(target+1, -1); dp[0] = -2;
    for (int i=0;i<(int)cand.size();++i) {
        int size = cand[i].second;
        for (int j=target;j>=size;--j) {
            if (dp[j-size] != -1 && dp[j] == -1) {
                dp[j] = i;
                if (j == target) break;
            }
        }
        if (dp[target] != -1) break;
    }
    if (dp[target] == -1) return nullopt;
    vector<pair<int,int>> res;
    int cur=target;
    while(cur>0) { int idx=dp[cur]; res.push_back(cand[idx]); cur -= cand[idx].second; }
    return res;
}

struct SolverState {
    vector<Item> items;
    vector<int> key_order;
    map<int, Item> item_map;
    map<int,int> simple_index;
    map<int,int> rotate;
    map<int,int> place;
    DimensionPairIndex D1;
    EdgeDimensionIndex D2;
    FeasibleFootprintIndex D3;
};

static Placement fitness3(int hl, int gap, DimensionPairIndex &D1) {
    Placement r;
    if (hl >= INF) return r;
    if (auto id = D1.first({gap,hl})) { r.ids={*id}; r.w=gap; r.l=hl; r.found=true; return r; }
    if (auto id = D1.first({hl,gap})) { r.ids={*id}; r.w=gap; r.l=hl; r.found=true; return r; }
    return r;
}
static Placement fitness2(int hl,int hr,int gap,DimensionPairIndex &D1,const map<int,int>& simple) {
    auto get=[&](int h)->Placement{ Placement r; if (h>=INF) return r; if(auto id=D1.first({h,gap})) {r.ids={*id}; r.w=gap;r.l=h;r.found=true;return r;} if(auto id=D1.first({gap,h})) {r.ids={*id}; r.w=gap;r.l=h;r.found=true;return r;} return r; };
    Placement a=get(hl), b=get(hr);
    if (a.found && b.found) return simple.at(a.ids[0]) < simple.at(b.ids[0]) ? a : b;
    if (a.found) return a; return b;
}
static Placement compose_vertical_block_to_height(int hl,int hr,int gap,EdgeDimensionIndex& D2,int k=20) {
    vector<pair<int,int>> targets;
    if (hl == hr) { if (hl < INF) targets.push_back({hl,0}); }
    else {
        if (hl < INF) targets.push_back({hl,0}); if (hr < INF) targets.push_back({hr,1});
        sort(targets.begin(), targets.end(), [](auto&a,auto&b){return a.first>b.first;});
    }
    for (auto [target,mode] : targets) {
        auto cand = D2.query_top_k_indices(gap,target,k);
        auto sub = solve_subset_sum(cand,target);
        if (sub) { Placement r; for(auto&p:*sub) r.ids.push_back(p.first); r.w=gap;r.l=target;r.mode=mode;r.found=true; return r; }
    }
    return {};
}
static Placement compose_horizontal_block_to_width(int hl,int hr,int gap,EdgeDimensionIndex& D2,int k=20) {
    vector<int> targets;
    if (hl == hr) { if (hl < INF) targets.push_back(hl); }
    else { if (hl < INF) targets.push_back(hl); if (hr < INF) targets.push_back(hr); sort(targets.rbegin(), targets.rend()); }
    for (int target : targets) {
        auto cand = D2.query_top_k_indices(target,gap,k);
        auto sub = solve_subset_sum(cand,gap);
        if (sub) { Placement r; for(auto&p:*sub) r.ids.push_back(p.first); r.w=gap;r.l=target;r.found=true; return r; }
    }
    return {};
}

static Placement type1_fit10(int gap,int hl,int hr,EdgeDimensionIndex& D2,const map<int,Item>& item_map,const map<int,int>& simple,int hcon) {

    map<int, Placement> cand;
    optional<pair<int,Placement>> item1;
    if (D2.has_dim(gap)) if (auto id=D2.query_min_index(gap,hcon)) {
        auto it=item_map.at(*id); Placement r; r.ids={*id}; r.w=gap; r.l=(it.length==gap?it.width:it.length); r.mode=0; r.found=true; item1=make_pair(*id,r);
    }
    if (hl < INF && D2.has_dim(hl)) if (auto id=D2.query_min_index(hl,gap)) {
        auto it=item_map.at(*id); Placement r; r.ids={*id}; r.w=(it.length==hl?it.width:it.length); r.l=hl; r.mode=0; r.found=true; cand[*id]=r;
    }
    if (hr < INF && D2.has_dim(hr)) if (auto id=D2.query_min_index(hr,gap)) {
        auto it=item_map.at(*id); Placement r; r.ids={*id}; r.w=(it.length==hr?it.width:it.length); r.l=hr; r.mode=1; r.found=true; cand[*id]=r;
    }
    if(item1) cand[item1->first]=item1->second;
    if (cand.empty()) return {};
    int best=-1; for(auto &kv:cand){ if(best<0 || simple.at(kv.first)<simple.at(best)) best=kv.first; }
    return cand[best];
}
static Placement type2_fit10(int gap,int hl,EdgeDimensionIndex& D2,const map<int,Item>& item_map,const map<int,int>& simple,int hcon) {

    Placement fit0, fit1;
    if (D2.has_dim(gap)) if (auto id=D2.query_min_index(gap,hcon)) { auto it=item_map.at(*id); fit1.ids={*id}; fit1.w=gap; fit1.l=(it.length==gap?it.width:it.length); fit1.mode=0; fit1.found=true; }
    if (hl < INF && D2.has_dim(hl)) if (auto id=D2.query_min_index(hl,gap)) { auto it=item_map.at(*id); fit0.ids={*id}; fit0.w=(it.length==hl?it.width:it.length); fit0.l=hl; fit0.mode=0; fit0.found=true; }
    if(!fit0.found && !fit1.found) return {};
    if(fit0.found && fit1.found) {
        int id0=fit0.ids[0], id1=fit1.ids[0];
        if(simple.at(id0) <= simple.at(id1)) return fit0;
        return fit1;
    }
    return fit0.found ? fit0 : fit1;
}
static Placement compose_fallback_block(int hl,int hr,int gap,EdgeDimensionIndex& D2,FeasibleFootprintIndex& D3,const map<int,Item>& item_map,int hcon) {
    auto fid = D3.query_min_index_with_condition(gap,hcon);
    if (!fid) return {};
    auto item=item_map.at(*fid);
    vector<int> heights{max(item.length,item.width), min(item.length,item.width)};
    for (int h : heights) {
        auto cand=D2.query_top_k_indices(h,gap,20);
        auto sub=solve_subset_sum(cand,gap);
        if (sub) { Placement r; for(auto&p:*sub) r.ids.push_back(p.first); r.w=gap; r.l=h; r.mode=0; r.found=true; return r; }
    }
    vector<int> v; for(int h:heights) if(h<=gap) v.push_back(h); sort(v.begin(),v.end());
    for(int h:v) { auto r=compose_vertical_block_to_height(hl,hr,h,D2); if(r.found) return r; }
    return {};
}
static Placement place_fallback_footprint(int chosen_id,int gap,const map<int,Item>& item_map,const map<int,int>& rotate,const map<int,int>& place,int mode,int hcon) {
    Placement r; r.mode=mode;
    auto itm_it = item_map.find(chosen_id);
    if (itm_it == item_map.end()) return r;
    auto item=itm_it->second; int width=item.width, length=item.length;
    int rot=rotate.at(chosen_id);
    if (rot==1) {
        if (length<=gap && width<=hcon) { r.w=length; r.l=width; }
        else { r.w=width; r.l=length; }
    } else {
        if (width<=gap && length<=hcon) { r.w=width; r.l=length; }
        else { r.w=length; r.l=width; }
    }
    if (place.at(chosen_id)==0) r.mode=1;
    r.ids={chosen_id}; r.found=true; return r;
}

static void merge_gap(Skyline& skyline, const MinGapResult& g) {
    const int mid = g.min_id, lid = g.left_id, rid = g.right_id;
    const Segment minseg = skyline.segs[mid];
    bool hasL = skyline.alive(lid), hasR = skyline.alive(rid);
    if (hasL && hasR) {
        auto L=skyline.segs[lid], R=skyline.segs[rid];
        if (L.y < R.y) { skyline.remove_node(lid); skyline.add_node(L.x, L.width + minseg.width, L.y); }
        else if (L.y > R.y) { skyline.remove_node(rid); skyline.add_node(minseg.x, minseg.width + R.width, R.y); }
        else { skyline.remove_node(lid); skyline.remove_node(rid); skyline.add_node(L.x, L.width + minseg.width + R.width, L.y); }
    } else if (hasL) { auto L=skyline.segs[lid]; skyline.remove_node(lid); skyline.add_node(L.x, L.width + minseg.width, L.y); }
    else if (hasR) { auto R=skyline.segs[rid]; skyline.remove_node(rid); skyline.add_node(minseg.x, minseg.width + R.width, R.y); }
}

static bool remaining_area_exceeds_skyline(const Skyline& skyline, int limit_y, long long items_area) {
    return items_area > skyline.upper_area(limit_y);
}

static optional<MinGapResult> process_min_gap(Skyline& skyline, FeasibleFootprintIndex& D3, int length_limit, long long items_area, bool area_pruning) {
    while (true) {
        auto opt = skyline.get_min_node(); if(!opt) return nullopt;
        auto g = *opt;
        g.height_constraint = length_limit - skyline.segs[g.min_id].y;
        if (g.gap_width < D3.get_min_dim()) {
            merge_gap(skyline,g);
            if (area_pruning && remaining_area_exceeds_skyline(skyline, length_limit, items_area)) { g.skip=true; return g; }
        } else return g;
    }
}

static long long place_item(SolverState& st, Skyline& skyline, const MinGapResult& g, const Placement& p, vector<int>& order, long long items_area, bool enable_area=true, vector<Rect>* placed_rects=nullptr) {
    for (int id : p.ids) {
        order.push_back(id);
        st.D1.mark_used(id); st.D2.mark_used(id); st.D3.mark_used(id);
    }
    if (enable_area) items_area -= 1LL * p.w * p.l;
    const Segment minseg = skyline.segs[g.min_id];
    int item_top_y = minseg.y + p.l;
    int rem_w = minseg.width - p.w;
    bool left_place = (p.mode == 0);
    int left_x = left_place ? minseg.x : (minseg.x + rem_w);
    int bottom_y = minseg.y;
    if (placed_rects != nullptr) {
        Rect r; r.x=left_x; r.y=bottom_y; r.w=p.w; r.h=p.l; r.ids=p.ids;
        placed_rects->push_back(std::move(r));
    }
    if (rem_w > 0) {
        int x_remain = left_place ? minseg.x + p.w : minseg.x;
        skyline.add_node(x_remain, rem_w, minseg.y);
    }
    bool hasL=skyline.alive(g.left_id), hasR=skyline.alive(g.right_id);
    if (hasL && hasR && item_top_y == skyline.segs[g.left_id].y && item_top_y == skyline.segs[g.right_id].y && rem_w == 0) {
        auto L=skyline.segs[g.left_id], R=skyline.segs[g.right_id]; skyline.remove_node(g.left_id); skyline.remove_node(g.right_id); skyline.add_node(L.x, L.width + p.w + R.width, L.y);
    } else if (hasL && item_top_y == skyline.segs[g.left_id].y && rem_w > 0 && left_place) {
        auto L=skyline.segs[g.left_id]; skyline.remove_node(g.left_id); skyline.add_node(L.x, L.width + p.w, L.y);
    } else if (hasR && item_top_y == skyline.segs[g.right_id].y && rem_w > 0 && !left_place) {
        auto R=skyline.segs[g.right_id]; skyline.remove_node(g.right_id); skyline.add_node(minseg.x + rem_w, p.w + skyline.segs[g.right_id].width, R.y);
    } else {
        int x_above = left_place ? minseg.x : minseg.x + rem_w;
        skyline.add_node(x_above, p.w, item_top_y);
    }
    return items_area;
}

struct SolveResult { bool feasible=false; int L=0; vector<int> order; vector<int> loaded; vector<Rect> placed_rects; Skyline skyline; };

static Skyline rebuild_skyline_from_rects_cpp(int W, const vector<Rect>& rects) {
    Skyline out;
    if (rects.empty()) { out.add_node(0,W,0); return out; }
    vector<int> xs; xs.push_back(0); xs.push_back(W);
    struct Ev { int x; int typ; int h; };
    vector<Ev> events;
    for (const auto& r: rects) {
        int x=max(0,min(W,r.x));
        int xr=max(0,min(W,r.x+r.w));
        if (xr<=x) continue;
        int top=r.y+r.h;
        xs.push_back(x); xs.push_back(xr);
        events.push_back({x,1,top});
        events.push_back({xr,0,top});
    }
    sort(xs.begin(), xs.end()); xs.erase(unique(xs.begin(), xs.end()), xs.end());
    sort(events.begin(), events.end(), [](const Ev&a,const Ev&b){ if(a.x!=b.x) return a.x<b.x; return a.typ<b.typ; });
    priority_queue<int> pq; pq.push(0);
    map<int,int> removed;
    int ei=0, ne=(int)events.size();
    vector<tuple<int,int,int>> segs;
    for(size_t i=0;i+1<xs.size();++i){
        int a=xs[i], b=xs[i+1]; if(b<=a) continue;
        while(ei<ne && events[ei].x<=a){
            auto e=events[ei++];
            if(e.typ==1) pq.push(e.h);
            else removed[e.h]++;
        }
        while(!pq.empty()){
            int h=pq.top(); auto it=removed.find(h);
            if(it!=removed.end() && it->second>0){ pq.pop(); if(--it->second==0) removed.erase(it); }
            else break;
        }
        int h=pq.empty()?0:pq.top();
        segs.emplace_back(a,b-a,h);
    }
    vector<tuple<int,int,int>> merged;
    for(auto [x,w,y]:segs){
        if(!merged.empty() && get<2>(merged.back())==y && get<0>(merged.back())+get<1>(merged.back())==x){
            get<1>(merged.back()) += w;
        } else merged.emplace_back(x,w,y);
    }
    for(auto [x,w,y]:merged) out.add_node(x,w,y);
    if(out.empty()) out.add_node(0,W,0);
    return out;
}

template<class RNG>
static Placement choose_placement(int gap, int hl, int hr, int W, int hcon, SolverState& st, RNG& rng){
    Placement p; int mode=0;
    auto try_f3=[&](){ return fitness3(hl,gap,st.D1); };
    auto try_f2=[&](){ return fitness2(hl,hr,gap,st.D1,st.simple_index); };
    auto try_f1dp=[&](){ return compose_vertical_block_to_height(hl,hr,gap,st.D2); };
    auto try_f0dp=[&](){ return compose_horizontal_block_to_width(hl,hr,gap,st.D2); };
    auto try_t1=[&](){ return type1_fit10(gap,hl,hr,st.D2,st.item_map,st.simple_index,hcon); };
    auto try_t2=[&](){ return type2_fit10(gap,hl,st.D2,st.item_map,st.simple_index,hcon); };
    auto try_negdp=[&](){ return compose_fallback_block(hl,hr,gap,st.D2,st.D3,st.item_map,hcon); };

    auto try_neg=[&](){
        int qgap = (gap==W?W:gap);
        auto __id = st.D3.query_min_index_with_condition(qgap,hcon);
        if(!__id) { Placement r; r.mode=mode; return r; }
        (void)rng.random();
        return place_fallback_footprint(*__id,qgap,st.item_map,st.rotate,st.place,mode,hcon);
    };

    if (hl != hr && gap != W) {
        p=try_f2();
        if(!p.found) p=try_f1dp();
        if(!p.found) p=try_f0dp();
        if(!p.found) p=try_t1();

        if(!p.found) p=try_negdp();
        if(!p.found) p=try_neg();
    } else if (hl == hr && gap != W) {
        p=try_f3();
        if(!p.found) p=try_f1dp();
        if(!p.found) p=try_f0dp();
        if(!p.found) p=try_t2();

        if(!p.found) p=try_negdp();
        if(!p.found) p=try_neg();
    } else if (gap == W) {
        p=try_negdp();
        if(!p.found) p=try_neg();
    }
    return p;
}

template<class RNG>
static SolveResult construct_skyline(SolverState& st, int W, Skyline skyline, int initial_L, int length_limit, long long total_area, RNG& rng, bool area_pruning=false, vector<Rect>* placed_rects=nullptr, const std::function<void()>& stop_callback = {}) {
    vector<int> order;
    int L = initial_L; long long curr_area = total_area;
    bool feasible = true;
    while (st.D1.active > 0) {
        check_skyline_stop();
        if (stop_callback) stop_callback();
        Placement p;
        auto optg = process_min_gap(skyline, st.D3, length_limit, curr_area, area_pruning);
        if (!optg) { feasible=false; break; }
        auto g=*optg;
        if (g.skip) break;
        int gap = g.gap_width, hcon = g.height_constraint, hl = g.hl, hr = g.hr;
        p = choose_placement(gap,hl,hr,W,hcon,st,rng);
        if (stop_callback) stop_callback();
        if (p.found && p.l > hcon) p.found=false;
        if (p.found) {
            const Segment minseg = skyline.segs[g.min_id];
            int currL = minseg.y + p.l;
            if (currL > length_limit) break;
            L = max(L, currL);
            curr_area = place_item(st, skyline, g, p, order, curr_area, true, placed_rects);
        } else if (!p.found && gap == W) {
            break;
        } else {
            merge_gap(skyline,g);
            if (area_pruning && remaining_area_exceeds_skyline(skyline, length_limit, curr_area)) break;
        }
    }
    vector<int> loaded = order;

    if ((int)order.size() != (int)st.simple_index.size()) {
        feasible=false;
        for (int pid : st.key_order) if (!st.D3.is_used(pid)) order.push_back(pid);
    }
    SolveResult sr; sr.feasible=feasible; sr.L=L; sr.order=std::move(order); sr.loaded=std::move(loaded); sr.skyline=std::move(skyline);
    if(placed_rects) sr.placed_rects=*placed_rects;
    return sr;
}

template<class RNG>
static SolveResult construct_fresh_skyline(SolverState& st, int W, int length_limit, long long total_area, RNG& rng, bool area_pruning=false, vector<Rect>* placed_rects=nullptr, const std::function<void()>& stop_callback = {}) {
    if (stop_callback) stop_callback();
    Skyline skyline; skyline.add_node(0,W,0);
    return construct_skyline(st,W,std::move(skyline),0,length_limit,total_area,rng,area_pruning,placed_rects,stop_callback);
}

static bool all_items_same_size(const vector<Item>& items){

    if(items.empty()) return true;
    int a=min(items[0].length, items[0].width), b=max(items[0].length, items[0].width);
    for(const auto& it: items){
        if(min(it.length,it.width)!=a || max(it.length,it.width)!=b) return false;
    }
    return true;
}

static vector<Item> sort_items_by_method(const vector<Item>& items, int method){
    vector<Item> sorted=items;
    stable_sort(sorted.begin(), sorted.end(), [&](const Item& a, const Item& b){
        long long ka=0,kb=0;
        switch(method){
            case 0: ka=1LL*a.length*a.width; kb=1LL*b.length*b.width; break;
            case 1: ka=1LL*a.length*a.length + 1LL*a.width*a.width; kb=1LL*b.length*b.length + 1LL*b.width*b.width; break;
            case 2: ka=max(a.length,a.width); kb=max(b.length,b.width); break;
            case 3: ka=2LL*(a.length+a.width); kb=2LL*(b.length+b.width); break;
            case 4: ka=a.length; kb=b.length; break;
            default: ka=a.width; kb=b.width; break;
        }
        if(ka!=kb) return ka>kb;
        return a.id<b.id;
    });
    return sorted;
}

struct TypeRec {
    int w=0,l=0,d1=0,d2=0;
    vector<long long> profits;
};

struct EvalPlacedRect {
    int type_idx = -1;
    bool is_optional = false;
    int rank_in_type = -1;
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    vector<int> core_ids;
};

struct SkylineEvalOptions {
    bool run_static_precheck = true;
};

struct EvalRes {
    bool feasible=false;
    int bin_count=0;
    long long score=0;
    vector<int> loaded_mandatory_count_by_type;
    vector<int> loaded_optional_count_by_type;
    vector<EvalPlacedRect> placed_rects;
    bool static_lb_infeasible=false;
    string static_lb_reason;
};

static vector<EvalPlacedRect> convert_rects_to_eval_placed(
    const vector<Rect>& rects,
    const vector<TypeRec>& types,
    const vector<int>& id_type,
    bool split_by_type_mandatory_count,
    bool fixed_optional_flag)
{
    vector<EvalPlacedRect> out;
    vector<int> seen(types.size(), 0);
    for (const auto& r : rects) {
        if (r.ids.empty()) continue;
        int id = r.ids.front();
        int ti = (id >= 0 && id < (int)id_type.size()) ? id_type[id] : -1;
        EvalPlacedRect pr;
        pr.type_idx = ti;
        pr.x = r.x; pr.y = r.y; pr.w = r.w; pr.h = r.h;
        pr.core_ids = r.ids;
        if (ti >= 0 && ti < (int)types.size()) {
            pr.rank_in_type = seen[ti]++;
            pr.is_optional = split_by_type_mandatory_count ? (pr.rank_in_type >= types[ti].d1) : fixed_optional_flag;
        } else {
            pr.rank_in_type = -1;
            pr.is_optional = fixed_optional_flag;
        }
        out.push_back(std::move(pr));
    }
    return out;
}

static vector<int> count_loaded_by_type(const vector<int>& loaded, const vector<int>& id_type, size_t n_types) {
    vector<int> cnt(n_types, 0);
    for (int id : loaded) {
        if (id >= 0 && id < (int)id_type.size()) {
            int ti = id_type[id];
            if (ti >= 0 && ti < (int)n_types) cnt[ti]++;
        }
    }
    return cnt;
}

struct PyRandomRng {
    static constexpr int N = 624;
    static constexpr int M = 397;
    static constexpr uint32_t MATRIX_A = 0x9908b0dfU;
    static constexpr uint32_t UPPER_MASK = 0x80000000U;
    static constexpr uint32_t LOWER_MASK = 0x7fffffffU;
    array<uint32_t, N> mt{};
    int index = N + 1;

    explicit PyRandomRng(uint64_t seed=1) { seed_int(seed); }
    explicit PyRandomRng(const string& seed_text) { seed_from_decimal_string(seed_text); }

    void init_genrand(uint32_t s) {
        mt[0] = s & 0xffffffffU;
        for (index = 1; index < N; ++index) {
            mt[index] = (uint32_t)(1812433253U * (mt[index-1] ^ (mt[index-1] >> 30)) + index);
            mt[index] &= 0xffffffffU;
        }
    }

    void init_by_array(const vector<uint32_t>& init_key) {
        init_genrand(19650218U);
        int i = 1, j = 0;
        int key_length = (int)init_key.size();
        int k = (N > key_length ? N : key_length);
        for (; k; --k) {
            uint32_t prev = mt[i-1];
            mt[i] = (uint32_t)((mt[i] ^ ((prev ^ (prev >> 30)) * 1664525U)) + init_key[j] + (uint32_t)j);
            mt[i] &= 0xffffffffU;
            ++i; ++j;
            if (i >= N) { mt[0] = mt[N-1]; i = 1; }
            if (j >= key_length) j = 0;
        }
        for (k = N - 1; k; --k) {
            uint32_t prev = mt[i-1];
            mt[i] = (uint32_t)((mt[i] ^ ((prev ^ (prev >> 30)) * 1566083941U)) - (uint32_t)i);
            mt[i] &= 0xffffffffU;
            ++i;
            if (i >= N) { mt[0] = mt[N-1]; i = 1; }
        }
        mt[0] = 0x80000000U;
    }

    static string trim_seed_string(string s) {
        size_t a = 0, b = s.size();
        while (a < b && isspace((unsigned char)s[a])) ++a;
        while (b > a && isspace((unsigned char)s[b-1])) --b;
        return s.substr(a, b-a);
    }

    static vector<uint32_t> decimal_abs_to_le32(string s) {
        s = trim_seed_string(s);
        if (s.empty()) return {0U};
        if (s[0] == '+' || s[0] == '-') s.erase(s.begin());
        size_t p = 0;
        while (p < s.size() && s[p] == '0') ++p;
        s = (p == s.size()) ? string("0") : s.substr(p);
        for (char c : s) if (!isdigit((unsigned char)c)) throw runtime_error("invalid seed integer");
        vector<uint32_t> limbs;
        if (s == "0") return {0U};
        const uint64_t BASE = 4294967296ULL;
        while (s != "0") {
            string q; q.reserve(s.size());
            uint64_t rem = 0;
            for (char c : s) {
                uint64_t cur = rem * 10ULL + (uint64_t)(c - '0');
                uint64_t digit = cur / BASE;
                rem = cur % BASE;
                if (!q.empty() || digit != 0) q.push_back((char)('0' + digit));
            }
            limbs.push_back((uint32_t)rem);
            s = q.empty() ? string("0") : q;
        }
        return limbs;
    }

    void seed_int(uint64_t seed) {
        vector<uint32_t> key;
        uint64_t x = seed;
        if (x == 0) key.push_back(0U);
        else {
            while (x) { key.push_back((uint32_t)(x & 0xffffffffU)); x >>= 32; }
        }
        init_by_array(key);
    }

    void seed_from_decimal_string(const string& seed_text) {
        vector<uint32_t> key = decimal_abs_to_le32(seed_text);
        init_by_array(key);
    }

    uint32_t genrand_uint32() {
        static const uint32_t mag01[2] = {0x0U, MATRIX_A};
        uint32_t y;
        if (index >= N) {
            int kk;
            if (index == N + 1) init_genrand(5489U);
            for (kk = 0; kk < N - M; ++kk) {
                y = (mt[kk] & UPPER_MASK) | (mt[kk+1] & LOWER_MASK);
                mt[kk] = mt[kk+M] ^ (y >> 1) ^ mag01[y & 0x1U];
            }
            for (; kk < N - 1; ++kk) {
                y = (mt[kk] & UPPER_MASK) | (mt[kk+1] & LOWER_MASK);
                mt[kk] = mt[kk+(M-N)] ^ (y >> 1) ^ mag01[y & 0x1U];
            }
            y = (mt[N-1] & UPPER_MASK) | (mt[0] & LOWER_MASK);
            mt[N-1] = mt[M-1] ^ (y >> 1) ^ mag01[y & 0x1U];
            index = 0;
        }
        y = mt[index++];
        y ^= (y >> 11);
        y ^= (y << 7) & 0x9d2c5680U;
        y ^= (y << 15) & 0xefc60000U;
        y ^= (y >> 18);
        return y;
    }

    double random() {
        uint32_t a = genrand_uint32() >> 5;
        uint32_t b = genrand_uint32() >> 6;
        return ((double)a * 67108864.0 + (double)b) * (1.0 / 9007199254740992.0);
    }
    double uniform(double a, double b) { return a + (b - a) * random(); }

    uint64_t getrandbits(int k) {
        if (k <= 0) return 0;
        if (k <= 32) return (uint64_t)(genrand_uint32() >> (32 - k));
        uint64_t result = 0;
        int bits = 0;
        while (k > 0) {
            int take = min(32, k);
            uint32_t word = genrand_uint32();
            if (take < 32) word >>= (32 - take);
            result |= ((uint64_t)word) << bits;
            bits += take;
            k -= take;
        }
        return result;
    }

    int bit_length_u64(uint64_t n) {
        if (n == 0) return 0;
#if defined(_MSC_VER)
        unsigned long idx = 0;
        _BitScanReverse64(&idx, n);
        return (int)idx + 1;
#elif defined(__GNUC__) || defined(__clang__)
        return 64 - __builtin_clzll(n);
#else
        int bits = 0;
        while (n) { ++bits; n >>= 1; }
        return bits;
#endif
    }

    int randrange(int n) {
        if (n <= 0) return 0;
        int k = bit_length_u64((uint64_t)n);
        uint64_t r;
        do { r = getrandbits(k); } while (r >= (uint64_t)n);
        return (int)r;
    }
};

static int py_sample_setsize(int k) {
    int setsize = 21;
    if (k > 5) {
        int x = 3 * k;
        int p = 1;
        while (p < x) p <<= 2;
        setsize += p;
    }
    return setsize;
}

template<class RNG>
static vector<int> rng_sample_ids_exact(const vector<int>& population, int k, RNG& rng){
    const int n = (int)population.size();
    if (k < 0 || k > n) {
        throw runtime_error("Sample larger than population or is negative");
    }
    vector<int> result;
    result.reserve(k);
    if (n <= py_sample_setsize(k)) {
        vector<int> pool = population;
        for (int i = 0; i < k; ++i) {
            check_skyline_stop();
            int j = rng.randrange(n - i);
            result.push_back(pool[j]);
            pool[j] = pool[n - i - 1];
        }
    } else {
        unordered_set<int> selected;
        selected.reserve((size_t)k * 2 + 1);
        for (int i = 0; i < k; ++i) {
            check_skyline_stop();
            int j = rng.randrange(n);
            while (selected.find(j) != selected.end()) j = rng.randrange(n);
            selected.insert(j);
            result.push_back(population[j]);
        }
    }
    return result;
}

template<class RNG>
static vector<int> rng_sample2_ids(const vector<int>& ids, RNG& rng){
    return rng_sample_ids_exact(ids, 2, rng);
}

static bool rotation_equiv_size(const Item& a, const Item& b){
    return (a.length==b.length && a.width==b.width) || (a.length==b.width && a.width==b.length);
}

static pair<int,int> canonical_size_key(const Item& item){
    return {std::min(item.length, item.width), std::max(item.length, item.width)};
}

template<class RNG>
static void swap_two_items_exact(map<int,int>& dict, const vector<int>& item_ids, const map<int,Item>& item_map, RNG& rng){

    if(item_ids.size() < 2) return;

    auto non_equiv = [&](int a, int b) {
        auto ia = item_map.find(a);
        auto ib = item_map.find(b);
        return ia != item_map.end() && ib != item_map.end()
            && !rotation_equiv_size(ia->second, ib->second);
    };

    bool have_key = false;
    pair<int,int> first_key{0, 0};
    bool has_non_equiv_pair = false;
    for(int id : item_ids){
        check_skyline_stop();
        auto it = item_map.find(id);
        if(it == item_map.end()) continue;
        auto key = canonical_size_key(it->second);
        if(!have_key){
            first_key = key;
            have_key = true;
        } else if(key != first_key){
            has_non_equiv_pair = true;
            break;
        }
    }
    if(!has_non_equiv_pair) return;

    while(true){
        check_skyline_stop();
        auto smp = rng_sample2_ids(item_ids, rng);
        int id1 = smp[0], id2 = smp[1];
        if(non_equiv(id1, id2)){
            swap(dict[id1], dict[id2]);
            return;
        }
    }
}

template<class RNG>
static map<int,int> swap_items_by_ratio_exact(const map<int,int>& original, const vector<int>& item_ids, double swap_ratio, const map<int,Item>& item_map, RNG& rng){
    map<int,int> out = original;
    int total=(int)item_ids.size();
    if(total < 2) return out;
    int target_swap_items = max(2, (int)py_round_nonneg(total * swap_ratio));
    target_swap_items = min(target_swap_items, total);
    int target_swap_pairs = target_swap_items / 2;
    for(int i=0;i<target_swap_pairs;++i) { check_skyline_stop(); swap_two_items_exact(out, item_ids, item_map, rng); }
    return out;
}

template<class RNG>
static map<int,int> random_rotate_items_exact(const map<int,int>& original, const vector<int>& ids, double ratio, RNG& rng){
    map<int,int> out = original;
    vector<int> shuffled = ids;
    for(int i=(int)shuffled.size()-1;i>0;--i){ int j=rng.randrange(i+1); swap(shuffled[i], shuffled[j]); }
    int cnt = (int)py_round_nonneg((double)shuffled.size() * ratio);
    cnt=max(0,min(cnt,(int)shuffled.size()));
    for(int i=0;i<cnt;++i) out[shuffled[i]] = 1 - out[shuffled[i]];
    return out;
}

template<class RNG>
static map<int,int> random_rotate_items_exact_uniform(const map<int,int>& original, const vector<int>& ids, RNG& rng){
    double ratio = rng.uniform(0.0, 1.0);
    return random_rotate_items_exact(original, ids, ratio, rng);
}

static vector<int> ids_from_items(const vector<Item>& items){
    vector<int> ids; ids.reserve(items.size());
    for(const auto& it:items) ids.push_back(it.id);
    return ids;
}

static map<int,int> simple_index_from_ids(const vector<int>& ids){
    map<int,int> mp;
    for(int i=0;i<(int)ids.size();++i) mp[ids[i]]=i;
    return mp;
}

static map<int,Item> item_map_from_items(const vector<Item>& items){
    map<int,Item> mp;
    for(auto it:items) mp[it.id]=it;
    return mp;
}

static SolverState make_state_exact(const vector<Item>& all_items, const map<int,int>& simple_index, const vector<int>& key_order, const map<int,int>& rotate, const map<int,int>& place){
    SolverState st;
    st.items = all_items;
    st.key_order = key_order;
    st.simple_index = simple_index;
    for(auto& it: st.items){
        it.min_dim = min(it.length,it.width);
        st.item_map[it.id] = it;
    }
    st.rotate = rotate;
    st.place = place;
    st.D1.build(st.items);
    st.D2.build(st.items, st.simple_index);
    st.D3.build(st.items, st.simple_index);
    return st;
}

static map<int,int> default_rotate_from_items(const vector<Item>& items){
    map<int,int> out;
    for(const auto& it:items) out[it.id] = (it.length > it.width ? 1 : 0);
    return out;
}
static map<int,int> default_place_from_ids(const vector<int>& ids){
    map<int,int> out;
    for(int id:ids) out[id] = 1;
    return out;
}

template<class RNG>
static SolveResult construct_skyline_from_empty_floor(SolverState& st, int W, int length_limit, long long total_area, RNG& rng, bool area_pruning=false, vector<Rect>* placed_rects=nullptr, const std::function<void()>& stop_callback = {}){
    return construct_fresh_skyline(st,W,length_limit,total_area,rng,area_pruning,placed_rects,stop_callback);
}

template<class RNG>
static SolveResult construct_skyline_on_contour(SolverState& st, int W, Skyline skyline, int initial_L, int length_limit, long long total_area, RNG& rng, bool area_pruning=false, vector<Rect>* placed_rects=nullptr, const std::function<void()>& stop_callback = {}){
    return construct_skyline(st,W,std::move(skyline),initial_L,length_limit,total_area,rng,area_pruning,placed_rects,stop_callback);
}

struct ItemLBExact { int w=0; int h=0; };
struct PieceLBExact { int L=0; int W=0; };

static int ceil_div_ll(long long a, long long b){
    if(b<=0) throw runtime_error("ceil_div denominator must be positive");
    if(a>=0) return (int)((a + b - 1) / b);
    return (int)(- ((-a) / b));
}

static int best_subset_sum_excluding_exact(const vector<int>& weights, const vector<int>& alt_weights, int cap, int exclude_idx){
    if(cap<=0) return 0;
    long long total_others=0;
    int n=(int)weights.size();
    for(int j=0;j<n;++j){
        if(j==exclude_idx) continue;
        total_others += max(weights[j], alt_weights[j]);
    }
    if(total_others <= cap) return (int)total_others;
    vector<unsigned char> dp(cap+1,0), ndp(cap+1,0);
    dp[0]=1;
    for(int j=0;j<n;++j){
        if(j==exclude_idx) continue;
        int w1=weights[j], w2=alt_weights[j];
        int s1=(w1>0 && w1<=cap)?w1:0;
        int s2=(w2>0 && w2<=cap)?w2:0;
        if(!s1 && !s2) continue;
        ndp=dp;
        if(s1){ for(int v=0; v+s1<=cap; ++v) if(dp[v]) ndp[v+s1]=1; }
        if(s2){ for(int v=0; v+s2<=cap; ++v) if(dp[v]) ndp[v+s2]=1; }
        dp.swap(ndp);
        if(dp[cap]) return cap;
    }
    for(int v=cap; v>=0; --v) if(dp[v]) return v;
    return 0;
}

static int preprocess_strip_width_with_rotation_exact(const vector<PieceLBExact>& pieces, int initial_width){
    vector<int> weights, alt;
    weights.reserve(pieces.size()); alt.reserve(pieces.size());
    for(const auto& p: pieces){ weights.push_back(p.L); alt.push_back(p.W); }
    int best_width = best_subset_sum_excluding_exact(weights, alt, initial_width, -1);
    return (best_width < initial_width) ? best_width : initial_width;
}

static vector<PieceLBExact> expand_short_edges_by_best_other_exact(const vector<PieceLBExact>& pieces, int new_width, int master_size){
    if(new_width<=0 || master_size<=0) throw runtime_error("invalid width in lower-bound preprocessing");
    vector<PieceLBExact> updated = pieces;
    int n=(int)updated.size();
    for(int i=0;i<n;++i){
        auto p=updated[i];
        if(p.L <= new_width && p.W <= new_width) continue;
        int current_short = min(p.L, p.W);
        int cap = master_size - current_short;
        int best_other=0;
        if(cap>0){
            vector<int> weights, alt;
            weights.reserve(n); alt.reserve(n);
            for(const auto& q: updated){ weights.push_back(q.L); alt.push_back(q.W); }
            best_other = best_subset_sum_excluding_exact(weights, alt, cap, i);
        }
        int new_short = master_size - best_other;
        if(p.L <= p.W) updated[i] = {new_short, p.W};
        else updated[i] = {p.L, new_short};
    }
    return updated;
}

static pair<vector<ItemLBExact>, int> preprocess_items_for_lower_bound_exact(const vector<ItemLBExact>& items, int strip_width){
    if(items.empty()) return {items, strip_width};
    vector<PieceLBExact> pieces; pieces.reserve(items.size());
    for(const auto& it: items) { check_skyline_stop(); pieces.push_back({it.w, it.h}); }
    int processed_width = preprocess_strip_width_with_rotation_exact(pieces, strip_width);
    vector<PieceLBExact> processed_pieces = expand_short_edges_by_best_other_exact(pieces, processed_width, processed_width);
    vector<ItemLBExact> out; out.reserve(items.size());
    for(const auto& p: processed_pieces) { check_skyline_stop(); out.push_back({p.L, p.W}); }
    return {out, processed_width};
}

static int area_lower_bound_exact(const vector<ItemLBExact>& items, int W){
    if(W<=0) throw runtime_error("Strip width W must be positive");
    long long area=0; for(const auto& it:items) { check_skyline_stop(); area += 1LL*it.w*it.h; }
    return ceil_div_ll(area, W);
}

static int single_item_height_lower_bound_exact(const vector<ItemLBExact>& items, int W, bool allow_rotate=true){
    int max_h=0;
    for(const auto& it: items){
        check_skyline_stop();
        int item_min_h=0;
        if(allow_rotate){
            bool can_fit_w = it.w <= W;
            bool can_fit_h = it.h <= W;
            if(!can_fit_w && !can_fit_h) throw runtime_error("item larger than strip width in both orientations");
            int long_side=max(it.w,it.h), short_side=min(it.w,it.h);
            item_min_h = (long_side <= W) ? short_side : long_side;
        } else {
            if(it.w > W) throw runtime_error("item width exceeds strip width");
            item_min_h = it.h;
        }
        max_h=max(max_h,item_min_h);
    }
    return max_h;
}

static long long dff1_num_over_alpha_exact(int alpha, int W, int w){
    long long num=1LL*(alpha+1)*w;
    if(num % W == 0) return 1LL*w*alpha;
    return (num / W) * 1LL * W;
}
static int dff2_val_exact(int alpha, int W, int w){
    int r=W-alpha;
    if(w > r) return W;
    if(w >= alpha) return w;
    return 0;
}
static int dff3_val_exact(int alpha, int W, int w){
    if(2*w > W) return 2 * (W/alpha - (W-w)/alpha);
    if(2*w == W) return W/alpha;
    return 2 * (w/alpha);
}

static vector<int> dff2_breakpoints_exact(const vector<ItemLBExact>& items, int W, bool allow_rotate=true){
    set<int> vals;
    for(const auto& it:items){
        check_skyline_stop();
        vector<int> sides;
        if(allow_rotate) sides={it.w,it.h}; else sides={it.w};
        for(int side:sides){
            int alpha=min(side,W-side);
            if(1<=alpha && alpha<=W/2) vals.insert(alpha);
        }
    }
    return vector<int>(vals.begin(), vals.end());
}

static int dff1_induced_lower_bound_exact(const vector<ItemLBExact>& items, int W, bool allow_rotate=true, int cutoff=-1){
    int best=0;
    if(W<=2) return best;
    const int max_alpha=min(20,W-2);
    for(int alpha=1; alpha<=max_alpha; ++alpha){
        check_skyline_stop();
        long long sum_num=0;
        for(const auto& it:items){
            vector<long long> cand;
            if(!allow_rotate){
                if(it.w > W) throw runtime_error("item width exceeds strip width");
                cand.push_back(dff1_num_over_alpha_exact(alpha,W,it.w) * 1LL * it.h);
            } else {
                if(it.w <= W) cand.push_back(dff1_num_over_alpha_exact(alpha,W,it.w) * 1LL * it.h);
                if(it.h <= W && it.h != it.w) cand.push_back(dff1_num_over_alpha_exact(alpha,W,it.h) * 1LL * it.w);
                if(cand.empty()) throw runtime_error("item infeasible even with rotation");
            }
            sum_num += *min_element(cand.begin(), cand.end());
        }
        const int bound=ceil_div_ll(sum_num, 1LL*alpha*W);
        if(cutoff>=0 && bound>cutoff) return bound;
        best=max(best,bound);
    }
    return best;
}

static int dff2_induced_lower_bound_exact(const vector<ItemLBExact>& items, int W, const vector<int>& alphas, bool allow_rotate=true, int cutoff=-1){
    int best=0;
    for(int alpha: alphas){
        check_skyline_stop();
        if(alpha<1 || alpha>W/2) continue;
        long long sum=0;
        for(const auto& it:items){
            vector<long long> cand;
            if(!allow_rotate){
                if(it.w > W) throw runtime_error("item width exceeds strip width");
                cand.push_back(1LL*dff2_val_exact(alpha,W,it.w)*it.h);
            } else {
                if(it.w <= W) cand.push_back(1LL*dff2_val_exact(alpha,W,it.w)*it.h);
                if(it.h <= W && it.h != it.w) cand.push_back(1LL*dff2_val_exact(alpha,W,it.h)*it.w);
                if(cand.empty()) throw runtime_error("item infeasible even with rotation");
            }
            sum += *min_element(cand.begin(), cand.end());
        }
        const int bound=ceil_div_ll(sum,W);
        if(cutoff>=0 && bound>cutoff) return bound;
        best=max(best,bound);
    }
    return best;
}

static int dff3_induced_lower_bound_exact(const vector<ItemLBExact>& items, int W, bool allow_rotate=true, int cutoff=-1){
    int best=0;
    for(int alpha=1; alpha<=W/2; ++alpha){
        check_skyline_stop();
        int denom=dff3_val_exact(alpha,W,W);
        if(denom<=0) continue;
        long long sum=0;
        for(const auto& it:items){
            vector<long long> cand;
            if(!allow_rotate){
                if(it.w > W) throw runtime_error("item width exceeds strip width");
                cand.push_back(1LL*dff3_val_exact(alpha,W,it.w)*it.h);
            } else {
                if(it.w <= W) cand.push_back(1LL*dff3_val_exact(alpha,W,it.w)*it.h);
                if(it.h <= W && it.h != it.w) cand.push_back(1LL*dff3_val_exact(alpha,W,it.h)*it.w);
                if(cand.empty()) throw runtime_error("item infeasible even with rotation");
            }
            sum += *min_element(cand.begin(), cand.end());
        }
        const int bound=ceil_div_ll(sum,denom);
        if(cutoff>=0 && bound>cutoff) return bound;
        best=max(best,bound);
    }
    return best;
}

static tuple<vector<ItemLBExact>,int,int> gcd_normalize_lower_bound_space_exact(
    const vector<ItemLBExact>& items, int lifted_W, int vehicle_floor_length){
    if(lifted_W<=0 || vehicle_floor_length<=0)
        throw runtime_error("invalid dimensions in lower-bound GCD normalization");
    int divisor=std::gcd(lifted_W,vehicle_floor_length);
    for(const auto& it:items){
        divisor=std::gcd(divisor,it.w);
        divisor=std::gcd(divisor,it.h);
    }
    divisor=max(1,divisor);
    vector<ItemLBExact> normalized;
    normalized.reserve(items.size());
    for(const auto& it:items){
        check_skyline_stop();
        normalized.push_back({it.w/divisor,it.h/divisor});
    }
    return {normalized,lifted_W/divisor,vehicle_floor_length/divisor};
}

static bool partition_lifting_exceeds_H_exact(const vector<ItemLBExact>& items, int lb_area, int H){
    if(lb_area>H) return true;
    vector<unsigned char> reach(H+1,0), next;
    reach[0]=1;
    map<pair<int,int>, int> groups;
    for(const auto& it:items){
        check_skyline_stop();
        int d1=min(it.w,it.h), d2=max(it.w,it.h);
        groups[{d1,d2}]++;
    }
    for(auto &kv:groups){
        check_skyline_stop();
        int d1=kv.first.first, d2=kv.first.second, cnt=kv.second;
        for(int c=0;c<cnt;++c){
            check_skyline_stop();
            next=reach;
            for(int v=0; v<=H; ++v) { check_skyline_stop(); if(reach[v]){
                if(v+d1<=H) next[v+d1]=1;
                if(d2!=d1 && v+d2<=H) next[v+d2]=1;
            }}
            reach.swap(next);
        }
    }
    for(int v=lb_area; v<=H; ++v) { check_skyline_stop(); if(reach[v]) return false; }
    return true;
}

static bool mandatory_lb_infeasible_cpp(const vector<Item>& mandatory_items, int W, int L, string* reason=nullptr){

    if(mandatory_items.empty()) return false;
    vector<ItemLBExact> items; items.reserve(mandatory_items.size());
    for(const auto& it:mandatory_items) { check_skyline_stop(); items.push_back({it.length, it.width}); }
    try{
        auto pre = preprocess_items_for_lower_bound_exact(items, W);
        auto normalized = gcd_normalize_lower_bound_space_exact(pre.first,pre.second,L);
        const auto& processed_items = std::get<0>(normalized);
        const int processed_W = std::get<1>(normalized);
        const int processed_L = std::get<2>(normalized);
        int lb_area = area_lower_bound_exact(processed_items, processed_W);
        if(lb_area > processed_L) { if(reason) *reason = "area lower bound exceeds vehicle length"; return true; }
        int lb_single = single_item_height_lower_bound_exact(processed_items, processed_W, true);
        if(lb_single > processed_L) { if(reason) *reason = "single item height lower bound exceeds vehicle length"; return true; }
        int lb_dff1 = dff1_induced_lower_bound_exact(processed_items, processed_W, true, processed_L);
        if(lb_dff1 > processed_L) { if(reason) *reason = "DFF1 lower bound exceeds vehicle length"; return true; }
        vector<int> dff2_alphas = dff2_breakpoints_exact(processed_items, processed_W, true);
        int lb_dff2 = dff2_induced_lower_bound_exact(processed_items, processed_W, dff2_alphas, true, processed_L);
        if(lb_dff2 > processed_L) { if(reason) *reason = "DFF2 lower bound exceeds vehicle length"; return true; }
        int lb_dff3 = dff3_induced_lower_bound_exact(processed_items, processed_W, true, processed_L);
        if(lb_dff3 > processed_L) { if(reason) *reason = "DFF3 lower bound exceeds vehicle length"; return true; }
        int lb_final = max({lb_area, lb_single, lb_dff1, lb_dff2, lb_dff3});
        bool lifted = partition_lifting_exceeds_H_exact(processed_items, lb_final, processed_L);
        if(lifted && reason) *reason = "partition lifting lower bound exceeds vehicle length";
        return lifted;
    } catch(const exception&){
        if(reason) *reason = "lower bound preprocessing exception";
        return true;
    }
}

struct BinEvalData {
    vector<Item> mandatory_items;
    vector<Item> optional_items;
    vector<Item> merged_items;
    vector<int> id_type;
    vector<int> mandatory_id_type;
    vector<int> optional_id_type;
    long long profit_upper_bound=0;
    long long mandatory_area=0;
    long long optional_area=0;
    long long merged_area=0;
};

static long long optional_profit_upper_bound(const vector<TypeRec>& types, const vector<int>& type_idx){
    long long upper=0;
    for(int ti:type_idx) upper += accumulate(types[ti].profits.begin(), types[ti].profits.end(), 0LL);
    return upper;
}

static BinEvalData build_bin_eval_data(const vector<TypeRec>& types, const vector<int>& type_idx){
    BinEvalData d;
    d.profit_upper_bound = optional_profit_upper_bound(types,type_idx);
    d.id_type.push_back(-1); d.mandatory_id_type.push_back(-1); d.optional_id_type.push_back(-1);
    int iid=1;
    for(int ti:type_idx){
        for(int k=0;k<types[ti].d1;++k){ Item it; it.id=iid++; it.length=types[ti].w; it.width=types[ti].l; it.min_dim=min(it.length,it.width); d.mandatory_items.push_back(it); d.merged_items.push_back(it); d.id_type.push_back(ti); d.mandatory_id_type.push_back(ti); d.mandatory_area += 1LL*it.length*it.width; d.merged_area += 1LL*it.length*it.width; }
    }
    for(int ti:type_idx){
        for(int k=0;k<types[ti].d2;++k){ Item it; it.id=iid++; it.length=types[ti].w; it.width=types[ti].l; it.min_dim=min(it.length,it.width); d.optional_items.push_back(it); d.merged_items.push_back(it); d.id_type.push_back(ti); if((int)d.optional_id_type.size()<=it.id) d.optional_id_type.resize(it.id+1,-1); d.optional_id_type[it.id]=ti; d.optional_area += 1LL*it.length*it.width; d.merged_area += 1LL*it.length*it.width; }
    }
    return d;
}

static long long score_from_loaded_counts(const vector<TypeRec>& types, const vector<int>& type_idx, const vector<int>& cnt, bool subtract_mandatory){
    long long score=0;
    for(int ti:type_idx){
        int opt_loaded = cnt[ti];
        if(subtract_mandatory) opt_loaded=max(0, opt_loaded - types[ti].d1);
        opt_loaded=min(opt_loaded, (int)types[ti].profits.size());
        for(int i=0;i<opt_loaded;++i) score += types[ti].profits[i];
    }
    return score;
}

template<class RNG>
static EvalRes solve_skyline_packing_cpp(
    const vector<TypeRec>& types, const vector<int>& type_idx, const vector<Item>& remaining_items,
    const vector<int>& id_type, int W, int H, long long item_total_area,
    int mutation_iters, double mutation_swap_count, RNG& rng, const std::function<void()>& stop_callback = {}){
    EvalRes best;
    best.bin_count = 1;
    best.loaded_mandatory_count_by_type.assign(types.size(), 0);
    best.loaded_optional_count_by_type.assign(types.size(), 0);
    if(remaining_items.empty()) { best.feasible=true; return best; }
    bool same=all_items_same_size(remaining_items);
    int method_count=same?1:6;
    const int length_limit=H;
    const long long profit_upper_bound=optional_profit_upper_bound(types,type_idx);
    const bool stop_at_profit_upper_bound=profit_upper_bound>0 &&
        any_of(type_idx.begin(),type_idx.end(),[&](int ti){return types[ti].d2>0;});
    bool full_feasible=false;

    for(int method=0; method<method_count; ++method){
        if (stop_callback) stop_callback();
        vector<Item> all_items=sort_items_by_method(remaining_items, method);
        vector<int> sorted_ids = ids_from_items(all_items);
        map<int,Item> item_map=item_map_from_items(all_items);
        map<int,int> simple_index = simple_index_from_ids(sorted_ids);
        vector<int> simple_key_order = sorted_ids;
        map<int,int> rotate = default_rotate_from_items(all_items);
        map<int,int> place = default_place_from_ids(sorted_ids);
        SolverState st;
        for(int iter=0; iter<=mutation_iters; ++iter){
            if (stop_callback) stop_callback();
            if(iter==0){
                simple_index = simple_index_from_ids(sorted_ids);
                simple_key_order = sorted_ids;
                st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);
            } else {
                if(same){
                    if(iter==1){
                        for(auto& kv: rotate) kv.second ^= 1;
                    } else {
                        rotate = random_rotate_items_exact_uniform(rotate, sorted_ids, rng);
                        place = random_rotate_items_exact_uniform(place, sorted_ids, rng);
                    }
                    st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);
                } else {
                    (void)rng.random();
                    simple_index = swap_items_by_ratio_exact(simple_index, sorted_ids, mutation_swap_count, item_map, rng);
                    rotate = random_rotate_items_exact_uniform(rotate, sorted_ids, rng);
                    place = random_rotate_items_exact_uniform(place, sorted_ids, rng);
                    st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);
                }
            }

            vector<Rect> placed;
            auto res=construct_skyline_from_empty_floor( st, W, length_limit, item_total_area, rng, false, &placed, stop_callback);
            vector<int> cnt=count_loaded_by_type(res.loaded, id_type, types.size());
            bool mandatory_feasible=true;
            for(int ti:type_idx) if(cnt[ti] < types[ti].d1) mandatory_feasible=false;
            long long candidate_score = 0;
            if(res.feasible){
                full_feasible=true;
                for(int ti:type_idx) candidate_score += accumulate(types[ti].profits.begin(), types[ti].profits.end(), 0LL);
            } else if(mandatory_feasible) {
                candidate_score=score_from_loaded_counts(types,type_idx,cnt,true);
            }
            if(res.feasible || mandatory_feasible){
                bool improve = !best.feasible || candidate_score > best.score ||
                               (candidate_score == best.score && placed.size() > best.placed_rects.size());
                if(improve){
                    best.feasible = true;
                    best.score = candidate_score;
                    best.loaded_mandatory_count_by_type.assign(types.size(), 0);
                    best.loaded_optional_count_by_type.assign(types.size(), 0);
                    for(size_t ti=0; ti<cnt.size(); ++ti){
                        int mand = std::min(cnt[ti], types[ti].d1);
                        int opt = std::max(0, cnt[ti] - types[ti].d1);
                        best.loaded_mandatory_count_by_type[ti] = mand;
                        best.loaded_optional_count_by_type[ti] = opt;
                    }
                    best.placed_rects = convert_rects_to_eval_placed(placed, types, id_type, true, true);
                }
                if(stop_at_profit_upper_bound && best.score>=profit_upper_bound) return best;
            }
        }
        if(full_feasible) break;
    }
    return best;
}

template<class RNG>
static EvalRes eval_bin_mandatory_only(
    const vector<TypeRec>& types,
    const vector<int>& type_idx,
    int W,
    int H,
    int mutation_iters,
    double mutation_swap_count,
    RNG& rng,
    const std::function<void()>& stop_callback = {},
    const SkylineEvalOptions& options = {}) {
    if (stop_callback) stop_callback();
    EvalRes best;
    best.bin_count = 1;
    best.loaded_mandatory_count_by_type.assign(types.size(), 0);
    best.loaded_optional_count_by_type.assign(types.size(), 0);

    vector<Item> mandatory_items;
    vector<int> mandatory_id_type(1, -1);
    long long mandatory_area = 0;
    int iid = 1;
    for (int ti : type_idx) {
        for (int k = 0; k < types[ti].d1; ++k) {
            Item it;
            it.id = iid++;
            it.length = types[ti].w;
            it.width = types[ti].l;
            it.min_dim = min(it.length, it.width);
            mandatory_items.push_back(it);
            mandatory_id_type.push_back(ti);
            mandatory_area += 1LL * it.length * it.width;
        }
    }

    if (mandatory_items.empty()) {
        best.feasible = true;
        best.bin_count = 0;
        return best;
    }

    if (options.run_static_precheck) {
        string reason;
        if (mandatory_lb_infeasible_cpp(mandatory_items, W, H, &reason)) {
            EvalRes out{false, 1, 0};
            out.loaded_mandatory_count_by_type.assign(types.size(), 0);
            out.loaded_optional_count_by_type.assign(types.size(), 0);
            out.static_lb_infeasible = reason != "lower bound preprocessing exception";
            out.static_lb_reason = reason;
            return out;
        }
    }

    const bool same = all_items_same_size(mandatory_items);
    const int method_count = same ? 1 : 6;
    for (int method = 0; method < method_count; ++method) {
        if (stop_callback) stop_callback();
        vector<Item> all_items = sort_items_by_method(mandatory_items, method);
        vector<int> sorted_ids = ids_from_items(all_items);
        map<int, Item> item_map = item_map_from_items(all_items);
        map<int, int> simple_index = simple_index_from_ids(sorted_ids);
        vector<int> simple_key_order = sorted_ids;
        map<int, int> rotate = default_rotate_from_items(all_items);
        map<int, int> place = default_place_from_ids(sorted_ids);
        SolverState st;

        for (int iter = 0; iter <= mutation_iters; ++iter) {
            if (stop_callback) stop_callback();
            if (iter == 0) {
                simple_index = simple_index_from_ids(sorted_ids);
                simple_key_order = sorted_ids;
            } else {
                if (same) {
                    if (iter == 1) {
                        for (auto& kv : rotate) kv.second ^= 1;
                    } else {
                        rotate = random_rotate_items_exact_uniform(rotate, sorted_ids, rng);
                        place = random_rotate_items_exact_uniform(place, sorted_ids, rng);
                    }
                } else {
                    (void)rng.random();
                    simple_index = swap_items_by_ratio_exact(simple_index, sorted_ids, mutation_swap_count, item_map, rng);
                    rotate = random_rotate_items_exact_uniform(rotate, sorted_ids, rng);
                    place = random_rotate_items_exact_uniform(place, sorted_ids, rng);
                }
            }

            st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);

            vector<Rect> placed;
            auto res = construct_skyline_from_empty_floor(
                st, W, H, mandatory_area, rng, true, &placed, stop_callback);
            vector<int> mand_cnt = count_loaded_by_type(res.loaded, mandatory_id_type, types.size());
            bool mandatory_feasible = res.feasible;
            if (!mandatory_feasible) {
                mandatory_feasible = true;
                for (int ti : type_idx) {
                    if (mand_cnt[ti] < types[ti].d1) {
                        mandatory_feasible = false;
                        break;
                    }
                }
            }
            if (mandatory_feasible) {
                best.feasible = true;
                best.bin_count = 1;
                best.score = 0;
                best.loaded_mandatory_count_by_type = std::move(mand_cnt);
                best.loaded_optional_count_by_type.assign(types.size(), 0);
                best.placed_rects = convert_rects_to_eval_placed(placed, types, mandatory_id_type, false, false);
                return best;
            }
        }
    }

    return best;
}

template<class RNG>
static EvalRes solve_skyline_packing_2_cpp(
    const vector<TypeRec>& types, const vector<int>& type_idx, const vector<Item>& mandatory_items,
    const vector<Item>& optional_items, const vector<int>& mandatory_id_type, const vector<int>& optional_id_type,
    int W, int H, long long mandatory_area, long long optional_area,
    int mutation_iters, double mutation_swap_count, RNG& rng, const std::function<void()>& stop_callback = {}){
    if(mandatory_items.empty()) throw invalid_argument("solve_skyline_packing_2_cpp requires at least one mandatory item");
    bool same_p1=all_items_same_size(mandatory_items);
    bool same_p2=all_items_same_size(optional_items);
    int method_count=(same_p1 && same_p2)?1:6;
    const int length_limit=H;
    int best_mandatory_L=H;
    const long long profit_upper_bound=optional_profit_upper_bound(types,type_idx);
    const bool stop_at_profit_upper_bound=!optional_items.empty() && profit_upper_bound>0;
    EvalRes best;
    best.bin_count = 1;
    best.loaded_mandatory_count_by_type.assign(types.size(), 0);
    best.loaded_optional_count_by_type.assign(types.size(), 0);

    for(int method=0; method<method_count; ++method){
        if (stop_callback) stop_callback();
        vector<Item> all_items=sort_items_by_method(mandatory_items, method);
        vector<Item> all_items_2=sort_items_by_method(optional_items, method);
        vector<int> sorted_ids = ids_from_items(all_items);
        vector<int> sorted_ids_2 = ids_from_items(all_items_2);
        map<int,Item> item_map=item_map_from_items(all_items);
        map<int,Item> item_map_2=item_map_from_items(all_items_2);
        map<int,int> simple_index = simple_index_from_ids(sorted_ids);
        vector<int> simple_key_order = sorted_ids;
        map<int,int> best_simple_index = simple_index;
        vector<int> best_key_order = simple_key_order;
        map<int,int> rotate = default_rotate_from_items(all_items);
        map<int,int> place = default_place_from_ids(sorted_ids);
        map<int,int> simple_index_p2 = simple_index_from_ids(sorted_ids_2);
        vector<int> simple_key_order_p2 = sorted_ids_2;
        map<int,int> rotate_p2 = default_rotate_from_items(all_items_2);
        map<int,int> place_p2 = default_place_from_ids(sorted_ids_2);
        vector<int> new_sorted_item_ids = sorted_ids;
        SolverState st;
        SolverState st2;
        int p2_attempt_count=0;
        for(int iter=0; iter<=mutation_iters; ++iter){
            if (stop_callback) stop_callback();
            if(iter==0){
                simple_index = simple_index_from_ids(sorted_ids);
                simple_key_order = sorted_ids;
                st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);
            } else {
                if(same_p1){
                    simple_index = best_simple_index;
                    simple_key_order = best_key_order;
                    if(iter==1){
                        for(auto& kv: rotate) kv.second ^= 1;
                    } else {
                        rotate = random_rotate_items_exact_uniform(rotate, new_sorted_item_ids, rng);
                        place = random_rotate_items_exact_uniform(place, new_sorted_item_ids, rng);
                    }
                    st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);
                } else {
                    (void)rng.random();
                    simple_index = swap_items_by_ratio_exact(best_simple_index, new_sorted_item_ids, mutation_swap_count, item_map, rng);
                    simple_key_order = best_key_order;
                    rotate = random_rotate_items_exact_uniform(rotate, new_sorted_item_ids, rng);
                    place = random_rotate_items_exact_uniform(place, new_sorted_item_ids, rng);
                    st = make_state_exact(all_items, simple_index, simple_key_order, rotate, place);
                }
            }

            vector<Rect> placed_p1;
            auto res1=construct_skyline_from_empty_floor( st, W, length_limit, mandatory_area, rng, true, &placed_p1, stop_callback);
            new_sorted_item_ids = res1.order;
            if(res1.feasible){
                if(res1.L < best_mandatory_L){
                    best_mandatory_L=res1.L;
                    if(iter>0) { best_simple_index = simple_index_from_ids(new_sorted_item_ids); best_key_order = new_sorted_item_ids; }
                }
                if(same_p2){
                    if(p2_attempt_count==0){
                        simple_index_p2 = simple_index_from_ids(sorted_ids_2);
                        simple_key_order_p2 = sorted_ids_2;
                    } else {
                        if(p2_attempt_count==1){
                            for(auto& kv: rotate_p2) kv.second ^= 1;
                        } else {
                            rotate_p2 = random_rotate_items_exact_uniform(rotate_p2, sorted_ids_2, rng);
                            place_p2 = random_rotate_items_exact_uniform(place_p2, sorted_ids_2, rng);
                        }
                    }
                    st2 = make_state_exact(all_items_2, simple_index_p2, simple_key_order_p2, rotate_p2, place_p2);
                } else {
                    if(iter==0){
                        simple_index_p2 = simple_index_from_ids(sorted_ids_2);
                        simple_key_order_p2 = sorted_ids_2;
                        st2 = make_state_exact(all_items_2, simple_index_p2, simple_key_order_p2, rotate_p2, place_p2);
                    } else {
                        (void)rng.random();
                        simple_index_p2 = swap_items_by_ratio_exact(simple_index_p2, sorted_ids_2, mutation_swap_count, item_map_2, rng);
                        rotate_p2 = random_rotate_items_exact_uniform(rotate_p2, sorted_ids_2, rng);
                        place_p2 = random_rotate_items_exact_uniform(place_p2, sorted_ids_2, rng);
                        st2 = make_state_exact(all_items_2, simple_index_p2, simple_key_order_p2, rotate_p2, place_p2);
                    }
                }
                long long current_score=0;
                vector<int> mand_cnt=count_loaded_by_type(res1.loaded, mandatory_id_type, types.size());
                vector<int> opt_cnt(types.size(),0);
                vector<EvalPlacedRect> curr_placed = convert_rects_to_eval_placed(placed_p1, types, mandatory_id_type, false, false);
                if(!optional_items.empty()){
                    Skyline skyline2 = rebuild_skyline_from_rects_cpp(W, placed_p1);
                    vector<Rect> placed_p2;
                    auto res2=construct_skyline_on_contour( st2, W, std::move(skyline2), res1.L, H, optional_area, rng, false, &placed_p2, stop_callback);
                    ++p2_attempt_count;
                    opt_cnt=count_loaded_by_type(res2.loaded, optional_id_type, types.size());
                    current_score=score_from_loaded_counts(types,type_idx,opt_cnt,false);
                    auto opt_rects=convert_rects_to_eval_placed(placed_p2, types, optional_id_type, false, true);
                    curr_placed.insert(curr_placed.end(), opt_rects.begin(), opt_rects.end());
                }
                bool improve = !best.feasible || current_score > best.score ||
                               (current_score == best.score && curr_placed.size() > best.placed_rects.size());
                if(improve){
                    best.feasible = true;
                    best.score = current_score;
                    best.loaded_mandatory_count_by_type = mand_cnt;
                    best.loaded_optional_count_by_type = opt_cnt;
                    best.placed_rects = std::move(curr_placed);
                }
                if(stop_at_profit_upper_bound && best.score>=profit_upper_bound) return best;
            }
        }
    }
    return best;
}

template<class RNG>
static EvalRes evaluate_two_stage_skyline(const vector<TypeRec>& types, const vector<int>& type_idx, int W, int H, int mutation_iters, double mutation_swap_count, RNG& rng, const std::function<void()>& stop_callback = {}, const SkylineEvalOptions& options = {}){
    if (stop_callback) stop_callback();
    auto d=build_bin_eval_data(types,type_idx);
    if(options.run_static_precheck){
        string reason;
        if(mandatory_lb_infeasible_cpp(d.mandatory_items,W,H,&reason)) {
            EvalRes out{false,1,0};
            out.static_lb_infeasible = reason != "lower bound preprocessing exception";
            out.static_lb_reason = reason;
            return out;
        }
    }
    auto er1=solve_skyline_packing_2_cpp(types,type_idx,d.mandatory_items,d.optional_items,d.mandatory_id_type,d.optional_id_type,W,H,d.mandatory_area,d.optional_area,mutation_iters,mutation_swap_count,rng,stop_callback);
    if(!er1.feasible) return er1;
    EvalRes best = er1;
    if(er1.score < d.profit_upper_bound){
        auto er2=solve_skyline_packing_cpp(types,type_idx,d.merged_items,d.id_type,W,H,d.merged_area,mutation_iters,mutation_swap_count,rng,stop_callback);
        if(er2.feasible && er2.score > best.score) best = er2;
    }
    best.feasible = true;
    best.bin_count = 1;
    return best;
}

struct StdRng {
    std::mt19937_64 eng;
    explicit StdRng(uint64_t seed=1) : eng(seed) {}
    double random(){ return std::generate_canonical<double, 53>(eng); }
    double uniform(double a, double b){ return a + (b-a)*random(); }
    int randrange(int n){ if(n<=0) return 0; std::uniform_int_distribution<int> dist(0,n-1); return dist(eng); }
};
