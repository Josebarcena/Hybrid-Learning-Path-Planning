#include "pathfinding.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace pf {
Grid::Grid(int w, int h, std::vector<std::uint8_t> blocked)
    : width_(w), height_(h), blocked_(std::move(blocked)) {
    if (w <= 0 || h <= 0 || blocked_.size() != static_cast<std::size_t>(w) * h)
        throw std::invalid_argument("Invalid grid dimensions");
}
Grid Grid::from_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) throw std::runtime_error("Cannot open map: " + filename);
    std::string line;
    std::vector<std::uint8_t> cells;
    int w = -1, h = 0;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (w < 0) w = static_cast<int>(line.size());
        if (static_cast<int>(line.size()) != w) throw std::runtime_error("Non-rectangular map");
        for (char c : line) {
            if (c != '.' && c != '#') throw std::runtime_error("Map must contain only . and #");
            cells.push_back(c == '#' ? 1 : 0);
        }
        ++h;
    }
    return Grid(w, h, std::move(cells));
}
bool Grid::valid(Point p) const { return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_; }
bool Grid::free(Point p) const { return valid(p) && !blocked_[index(p)]; }

namespace {
struct Entry { double f, g; int id; };
struct Greater {
    bool operator()(const Entry& a, const Entry& b) const {
        return std::tie(a.f, a.g, a.id) > std::tie(b.f, b.g, b.id);
    }
};
double manhattan(Point a, Point b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }
}
Result astar(const Grid& grid, Point start, Point goal, double weight) {
    if (!std::isfinite(weight) || weight < 1.0) throw std::invalid_argument("weight must be finite and >= 1");
    if (!grid.free(start) || !grid.free(goal)) throw std::invalid_argument("Start/goal outside grid or blocked");
    const auto begin = std::chrono::steady_clock::now();
    const int n = static_cast<int>(grid.cells());
    const int source = grid.index(start), target = grid.index(goal);
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> g(n, inf);
    std::vector<int> parent(n, -1);
    std::vector<std::uint8_t> closed(n, 0);
    std::priority_queue<Entry, std::vector<Entry>, Greater> open;
    Result out;
    g[source] = 0;
    open.push({weight * manhattan(start, goal), 0.0, source});
    out.generated = 1;
    out.peak_open = 1;
    constexpr Point directions[] = {{1,0},{-1,0},{0,1},{0,-1}};
    while (!open.empty()) {
        const Entry current = open.top(); open.pop();
        if (current.g != g[current.id] || closed[current.id]) continue;
        closed[current.id] = 1;
        ++out.expanded;
        if (current.id == target) {
            out.found = true;
            out.cost = g[target];
            for (int at = target; at != -1; at = parent[at]) out.path.push_back(grid.point(at));
            std::reverse(out.path.begin(), out.path.end());
            break;
        }
        Point p = grid.point(current.id);
        for (Point d : directions) {
            Point q{p.x + d.x, p.y + d.y};
            if (!grid.free(q)) continue;
            int id = grid.index(q);
            double tentative = g[current.id] + 1.0;
            if (tentative >= g[id]) continue;
            g[id] = tentative;
            parent[id] = current.id;
            closed[id] = 0; // permits reopening if needed
            open.push({tentative + weight * manhattan(q, goal), tentative, id});
            ++out.generated;
            out.peak_open = std::max(out.peak_open, open.size());
        }
    }
    out.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    out.estimated_peak_bytes = g.capacity() * sizeof(double) + parent.capacity() * sizeof(int)
        + closed.capacity() * sizeof(std::uint8_t) + out.peak_open * sizeof(Entry)
        + out.path.capacity() * sizeof(Point);
    return out;
}
}
