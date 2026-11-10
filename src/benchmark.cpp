#include "pathfinding.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Case { int side; int percent; };
int parse_int(const char* s) {
    int v{}; auto e = s + std::char_traits<char>::length(s);
    auto [p, ec] = std::from_chars(s,e,v);
    if (ec != std::errc{} || p != e) throw std::invalid_argument("Invalid integer");
    return v;
}
// Integer-threshold sampling is reproducible for a given mt19937 stream across STL implementations.
pf::Grid make_grid(int side, int percent, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<std::uint8_t> blocked(static_cast<std::size_t>(side)*side);
    const std::uint64_t threshold = (std::uint64_t{1} << 32) * percent / 100;
    for (auto& cell : blocked) cell = static_cast<std::uint64_t>(rng()) < threshold;
    blocked.front() = blocked.back() = 0;
    return pf::Grid(side, side, std::move(blocked));
}
// Independent BFS oracle: unit-cost, four-neighbor grid.
int bfs_optimal(const pf::Grid& grid, pf::Point start, pf::Point goal) {
    std::vector<int> distance(grid.cells(), -1);
    std::queue<pf::Point> queue;
    distance[grid.index(start)] = 0;
    queue.push(start);
    constexpr pf::Point moves[] = {{1,0},{-1,0},{0,1},{0,-1}};
    while (!queue.empty()) {
        auto p = queue.front(); queue.pop();
        if (p == goal) return distance[grid.index(p)];
        for (auto d : moves) {
            pf::Point q{p.x+d.x,p.y+d.y};
            if (!grid.free(q) || distance[grid.index(q)] != -1) continue;
            distance[grid.index(q)] = distance[grid.index(p)] + 1;
            queue.push(q);
        }
    }
    return -1;
}
void check_path(const pf::Grid& grid, const pf::Result& result, pf::Point start, pf::Point goal) {
    if (!result.found) return;
    if (result.path.empty() || !(result.path.front()==start) || !(result.path.back()==goal))
        throw std::runtime_error("Invalid path endpoints");
    for (std::size_t i=0;i<result.path.size();++i) {
        if (!grid.free(result.path[i])) throw std::runtime_error("Path crosses obstacle");
        if (i) {
            auto a=result.path[i-1], b=result.path[i];
            if (std::abs(a.x-b.x)+std::abs(a.y-b.y)!=1) throw std::runtime_error("Invalid path step");
        }
    }
    if (result.cost != static_cast<double>(result.path.size()-1))
        throw std::runtime_error("Reported cost differs from path");
}
}
int main(int argc, char** argv) {
    try {
        // Usage: benchmark [output.csv] [maps_per_configuration] [timed_repetitions] [max_side]
        const std::string output = argc>1 ? argv[1] : "results/benchmark.csv";
        const int maps = argc>2 ? parse_int(argv[2]) : 10;
        const int repeats = argc>3 ? parse_int(argv[3]) : 5;
        const int max_side = argc>4 ? parse_int(argv[4]) : 500;
        if (argc>5 || maps<1 || repeats<1 || max_side<10 || max_side>2000)
            throw std::invalid_argument("Usage: benchmark [output.csv] [maps>=1] [repetitions>=1] [max_side=10..2000]");
        const std::vector<int> sides{50,100,250,500};
        const std::vector<int> percents{10,20,30};
        const std::vector<double> weights{1.0,1.1,1.25,1.5,2.0,3.0};
        auto parent=std::filesystem::path(output).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::ofstream csv(output);
        if (!csv) throw std::runtime_error("Cannot create CSV");
        csv << "side,obstacle_percent,map_id,seed,reachable,optimal_cost,weight,repeat,found,cost,relative_error,elapsed_ms,expanded,generated,peak_open,estimated_peak_bytes\n";
        csv << std::fixed << std::setprecision(8);
        std::size_t cases=0, rows=0, unreachable=0;
        for (int side : sides) {
            if (side>max_side) continue;
            for (int percent : percents) for (int map_id=0; map_id<maps; ++map_id) {
                // Distinct deterministic seeds for all cases; no rejection sampling or survivorship bias.
                const std::uint32_t seed = static_cast<std::uint32_t>(20261110u + side*10000u + percent*100u + map_id);
                auto grid=make_grid(side,percent,seed);
                pf::Point start{0,0}, goal{side-1,side-1};
                int optimal=bfs_optimal(grid,start,goal);
                if (optimal<0) ++unreachable;
                ++cases;
                for (double weight : weights) {
                    // Warm-up on identical input; not included in timing rows.
                    auto warmup=pf::astar(grid,start,goal,weight);
                    check_path(grid,warmup,start,goal);
                    for (int rep=0;rep<repeats;++rep) {
                        auto r=pf::astar(grid,start,goal,weight);
                        check_path(grid,r,start,goal);
                        if (r.found != (optimal>=0)) throw std::runtime_error("Reachability disagrees with BFS");
                        if (weight==1.0 && r.found && r.cost!=optimal) throw std::runtime_error("A* disagrees with BFS");
                        if (r.found && (r.cost<optimal || r.cost>weight*optimal+1e-9))
                            throw std::runtime_error("Weighted A* violated cost bound");
                        csv << side << ',' << percent << ',' << map_id << ',' << seed << ','
                            << (optimal>=0) << ',';
                        if (optimal>=0) csv << optimal;
                        csv << ',' << weight << ',' << rep << ',' << r.found << ',';
                        if (r.found) csv << r.cost;
                        csv << ',';
                        if (r.found && optimal>0) csv << (r.cost-optimal)/optimal;
                        csv << ',' << r.elapsed_ms << ',' << r.expanded << ',' << r.generated << ','
                            << r.peak_open << ',' << r.estimated_peak_bytes << '\n';
                        ++rows;
                    }
                }
                std::cout << "Completed " << side << "x" << side << " density=" << percent
                          << "% map=" << map_id+1 << "/" << maps << " reachable=" << (optimal>=0) << '\n';
            }
        }
        std::cout << "Saved " << rows << " rows across " << cases << " maps ("
                  << unreachable << " unreachable) to " << output << '\n';
    } catch (const std::exception& e) {
        std::cerr << "Benchmark error: " << e.what() << '\n'; return 1;
    }
}
