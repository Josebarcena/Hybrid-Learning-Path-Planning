#include "pathfinding.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
int main(int argc, char** argv) {
    try {
        if (argc<6 || argc>10) throw std::invalid_argument("Usage: pathfinding_anytime map sx sy gx gy [weight=3] [decrement=0.5] [time_ms=0] [memory_bytes=0]");
        auto grid=pf::Grid::from_file(argv[1]);
        pf::Point s{std::stoi(argv[2]),std::stoi(argv[3])},g{std::stoi(argv[4]),std::stoi(argv[5])};
        auto r=pf::ara_star(grid,s,g,argc>6?std::stod(argv[6]):3.0,
            argc>7?std::stod(argv[7]):0.5,argc>8?std::stod(argv[8]):0.0,
            argc>9?std::stoull(argv[9]):0);
        std::cout << std::fixed << std::setprecision(4)
                  << "found=" << r.found << " optimality_proven=" << r.optimality_proven
                  << " cost=" << r.cost << " elapsed_ms=" << r.elapsed_ms
                  << " expanded=" << r.expanded << " estimated_peak_bytes=" << r.estimated_peak_bytes << '\n';
        for (auto& step:r.improvements)
            std::cout << "weight=" << step.weight << " cost=" << step.cost
                      << " elapsed_ms=" << step.elapsed_ms << " expanded=" << step.expanded << '\n';
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
