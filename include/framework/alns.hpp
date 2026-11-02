#pragma once
#include "routing_problem.hpp"
#include <array>
#include <map>

namespace hrlh::alns {
enum class Destroy { Shaw, Random, Worst, Cluster, NeighbourGraph, Overlap, InnerRoute, Intersection, TourPair };
enum class Repair { Greedy, Regret2, Regret3 };
struct State {
    RoutePlan routes;
    std::vector<int> missing;
};
struct AlnsConfig {

    int iterations = 1000000000;
    int no_improvement = 1000000000;
    int segment_length = 100;
    double reaction = .8;
    std::array<double,3> rewards{{50,10,5}};
    double minimum_removed_fraction = .04;
    double maximum_removed_fraction = .4;
    std::array<double,4> shaw_weights{{6,3,2,6}};
    double determinism = 6;
    double cooling = .99975;
    double initial_relative_worsening = .05;
    double initial_acceptance = .5;
};
void validate_config(const AlnsConfig&);
bool sa_accept(double current,double candidate,double temperature,std::mt19937_64&);
class AdaptiveSelection {
public:
    explicit AdaptiveSelection(const AlnsConfig&);
    std::pair<std::size_t,std::size_t> select(std::mt19937_64&) const;
    void record(std::size_t destroy_index,std::size_t repair_index,double reward);
    void update_segment();

private:
    double reaction;
    std::array<double,9> dw,ds{};
    std::array<double,3> rw,rs{};
    std::array<int,9> dc{};
    std::array<int,3> rc{};
};
void validate_state(const routing::Problem&, const State&);
double objective(const routing::Problem&, const State&);

class NeighbourGraphHistory {
public:
    void observe(const routing::Problem&, const State&);
    double edge_weight(int from, int to) const;
    double removal_score(const routing::Problem&, const Route&, std::size_t position) const;
private:
    std::map<std::pair<int,int>,double> weights;
};
State destroy(const routing::Problem&, const State&, Destroy, std::size_t count,
              std::mt19937_64&, const AlnsConfig& = {},
              const NeighbourGraphHistory* history = nullptr,
              const std::function<void()>& stop = {});

State repair(const routing::Problem&, State, Repair, const routing::SearchCallbacks&);
void run(const routing::Problem&, const RoutePlan&, const std::vector<RoutePlan>&,
         const routing::SearchCallbacks&, std::uint64_t,
         const AlnsConfig& = {});
}
