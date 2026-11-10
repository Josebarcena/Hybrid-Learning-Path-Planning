#include "pathfinding.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace pf {
namespace {
struct Item { double key; int id; };
struct Order { bool operator()(const Item& a, const Item& b) const {
    return a.key == b.key ? a.id > b.id : a.key > b.key;
}};
double h(Point a, Point b) { return std::abs(a.x-b.x)+std::abs(a.y-b.y); }
}
AnytimeResult ara_star(const Grid& grid, Point start, Point goal,
                      double initial_weight, double decrement,
                      double time_limit_ms, std::size_t memory_limit_bytes) {
    if (!std::isfinite(initial_weight) || initial_weight < 1 ||
        !std::isfinite(decrement) || decrement <= 0 ||
        !std::isfinite(time_limit_ms) || time_limit_ms < 0)
        throw std::invalid_argument("Invalid ARA* parameters");
    if (!grid.free(start) || !grid.free(goal)) throw std::invalid_argument("Invalid endpoints");
    const auto begin = std::chrono::steady_clock::now();
    const int n = static_cast<int>(grid.cells());
    const int source = grid.index(start), target = grid.index(goal);
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> g(n, inf);
    std::vector<int> parent(n, -1);
    std::vector<std::uint8_t> in_open(n, 0), closed(n, 0), incons(n, 0);
    std::priority_queue<Item, std::vector<Item>, Order> open;
    AnytimeResult out;
    auto elapsed = [&] { return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-begin).count(); };
    auto memory = [&] { return g.capacity()*sizeof(double) + parent.capacity()*sizeof(int)
        + (in_open.capacity()+closed.capacity()+incons.capacity())*sizeof(std::uint8_t)
        + open.size()*sizeof(Item); };
    std::size_t peak_memory = 0;
    auto push = [&](int id, double weight) {
        in_open[id] = 1;
        open.push({g[id]+weight*h(grid.point(id),goal),id});
        peak_memory = std::max(peak_memory,memory());
    };
    auto path = [&] {
        std::vector<Point> p;
        for (int id=target, steps=0; id!=-1 && steps<=n; id=parent[id],++steps)
            p.push_back(grid.point(id));
        std::reverse(p.begin(),p.end());
        return p;
    };
    g[source] = 0;
    double weight = initial_weight;
    push(source, weight);
    constexpr Point moves[] = {{1,0},{-1,0},{0,1},{0,-1}};
    bool exhausted = false;
    while (true) {
        // ImprovePath: expand until the current solution is no worse than min OPEN key.
        while (!open.empty()) {
            if (time_limit_ms > 0 && elapsed() >= time_limit_ms) break;
            if (memory_limit_bytes > 0 && peak_memory > memory_limit_bytes) break;
            Item top = open.top();
            if (!in_open[top.id] || std::abs(top.key-(g[top.id]+weight*h(grid.point(top.id),goal)))>1e-8) {
                open.pop(); continue;
            }
            if (g[target] <= top.key) break;
            open.pop();
            in_open[top.id] = 0;
            closed[top.id] = 1;
            ++out.expanded;
            Point p = grid.point(top.id);
            for (auto d : moves) {
                Point q{p.x+d.x,p.y+d.y};
                if (!grid.free(q)) continue;
                int id = grid.index(q);
                if (g[top.id]+1 >= g[id]) continue;
                g[id] = g[top.id]+1;
                parent[id] = top.id;
                if (!closed[id]) push(id,weight);
                else incons[id] = 1;
            }
        }
        out.elapsed_ms = elapsed();
        out.estimated_peak_bytes = peak_memory;
        if (std::isfinite(g[target])) {
            out.found = true;
            auto current_path = path();
            out.cost = static_cast<double>(current_path.size()-1);
            // Keep a snapshot per completed phase; intermediate partial phases are not certified.
            const bool interrupted = (time_limit_ms>0 && elapsed()>=time_limit_ms)
                || (memory_limit_bytes>0 && peak_memory>memory_limit_bytes);
            if (!interrupted) {
                if (out.improvements.empty() || out.improvements.back().cost != out.cost || weight == 1.0)
                    out.improvements.push_back({weight,out.cost,elapsed(),out.expanded,peak_memory,std::move(current_path)});
            }
        }
        if (weight == 1.0 && !(time_limit_ms>0 && elapsed()>=time_limit_ms)
            && !(memory_limit_bytes>0 && peak_memory>memory_limit_bytes)) {
            out.optimality_proven = true;
            break;
        }
        if ((time_limit_ms>0 && elapsed()>=time_limit_ms) ||
            (memory_limit_bytes>0 && peak_memory>memory_limit_bytes)) break;
        if (open.empty()) {
            bool has_incons = std::any_of(incons.begin(),incons.end(),[](auto v){return v!=0;});
            if (!has_incons) {
                // No better states remain; the incumbent (or lack of a route) is final.
                out.optimality_proven = true;
                exhausted = true;
                break;
            }
        }
        weight = std::max(1.0, weight-decrement);
        // Rebuild OPEN from active entries and INCONS, dropping stale heap records.
        std::priority_queue<Item,std::vector<Item>,Order> next;
        for (int id=0;id<n;++id) {
            if (in_open[id] || incons[id]) {
                in_open[id]=1;
                next.push({g[id]+weight*h(grid.point(id),goal),id});
            }
            incons[id]=0;
            closed[id]=0;
        }
        open.swap(next);
        peak_memory=std::max(peak_memory,memory());
    }
    (void)exhausted;
    out.elapsed_ms = elapsed();
    out.estimated_peak_bytes = peak_memory;
    return out;
}
}
