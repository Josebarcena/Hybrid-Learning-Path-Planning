#include "pathfinding.hpp"
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 6 || argc > 7) {
        std::cerr << "Usage: pathfinder MAP start_x start_y goal_x goal_y [weight>=1]\n";
        return 2;
    }
    try {
        auto grid = pf::Grid::from_file(argv[1]);
        pf::Point start{std::stoi(argv[2]), std::stoi(argv[3])};
        pf::Point goal{std::stoi(argv[4]), std::stoi(argv[5])};
        double weight = argc == 7 ? std::stod(argv[6]) : 1.0;
        auto r = pf::astar(grid, start, goal, weight);
        std::cout << std::fixed << std::setprecision(4)
                  << "algorithm=" << (weight == 1.0 ? "A*" : "Weighted A*") << '\n'
                  << "found=" << r.found << '\n'
                  << "cost=" << r.cost << '\n'
                  << "elapsed_ms=" << r.elapsed_ms << '\n'
                  << "expanded=" << r.expanded << '\n'
                  << "generated=" << r.generated << '\n'
                  << "peak_open=" << r.peak_open << '\n'
                  << "estimated_peak_bytes=" << r.estimated_peak_bytes << '\n';
        if (r.found) {
            std::cout << "path=";
            for (const auto& p : r.path) std::cout << '(' << p.x << ',' << p.y << ')';
            std::cout << '\n';
        }
        return r.found ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 2;
    }
}
