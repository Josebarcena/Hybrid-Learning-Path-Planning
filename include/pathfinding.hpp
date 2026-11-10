#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <string>
#include <vector>

namespace pf {
struct Point { int x{}, y{}; bool operator==(const Point&) const = default; };
class Grid {
public:
    static Grid from_file(const std::string& filename);
    Grid(int width, int height, std::vector<std::uint8_t> blocked);
    int width() const { return width_; }
    int height() const { return height_; }
    bool valid(Point p) const;
    bool free(Point p) const;
    int index(Point p) const { return p.y * width_ + p.x; }
    Point point(int index) const { return {index % width_, index / width_}; }
    std::size_t cells() const { return blocked_.size(); }
private:
    int width_, height_;
    std::vector<std::uint8_t> blocked_;
};
struct Result {
    bool found = false;
    double cost = 0.0;
    std::vector<Point> path;
    std::size_t expanded = 0;
    std::size_t generated = 0;
    std::size_t peak_open = 0;
    double elapsed_ms = 0.0;
    std::size_t estimated_peak_bytes = 0; // tracked algorithm buffers, not process RSS
};
Result astar(const Grid& grid, Point start, Point goal, double weight = 1.0);
}

namespace pf {
struct AnytimeSnapshot {
    double weight = 1.0;
    double cost = 0.0;
    double elapsed_ms = 0.0;
    std::size_t expanded = 0;
    std::size_t estimated_peak_bytes = 0;
    std::vector<Point> path;
};
struct AnytimeResult {
    bool found = false;
    bool optimality_proven = false;
    double cost = 0.0;
    double elapsed_ms = 0.0;
    std::size_t expanded = 0;
    std::size_t estimated_peak_bytes = 0;
    std::vector<AnytimeSnapshot> improvements;
};
// ARA*: reuse g-values, OPEN and INCONS across decreasing heuristic weights.
// time_limit_ms <= 0 means unlimited; memory_limit_bytes == 0 means unlimited.
// Memory limit is checked at phase boundaries; not a hard allocator cap.
AnytimeResult ara_star(const Grid& grid, Point start, Point goal,
                       double initial_weight = 3.0, double decrement = 0.5,
                       double time_limit_ms = 0.0, std::size_t memory_limit_bytes = 0);
}

namespace pf {
struct PhaseObservation {
    double weight = 1.0;
    double cost = 0.0;
    double elapsed_ms = 0.0;
    std::size_t expanded = 0;
    std::size_t estimated_peak_bytes = 0;
    bool found = false;
    bool optimality_proven = false;
    bool finished = false;
    // Instrumentation: cumulative algorithm time excludes caller/console delays.
    double phase_ms = 0.0;
    double rebuild_ms = 0.0;
    double search_ms = 0.0;
    std::size_t rebuild_candidates = 0;
    std::size_t phase_expanded = 0;
    double admissible_lower_bound = 0.0; // min(g+h) on OPEN/INCONS, capped by incumbent
    double certified_gap = 0.0; // (incumbent-LB)/incumbent; 1 if no solution
};
// Persistent ARA* search. One call performs one complete refinement phase.
class AnytimeSession {
public:
    AnytimeSession(const Grid& grid, Point start, Point goal, double initial_weight = 3.0);
    ~AnytimeSession();
    AnytimeSession(const AnytimeSession&) = delete;
    AnytimeSession& operator=(const AnytimeSession&) = delete;
    AnytimeSession(AnytimeSession&&) noexcept;
    AnytimeSession& operator=(AnytimeSession&&) noexcept;
    PhaseObservation advance(double next_weight); // must be <= current weight and >= 1
    PhaseObservation observe() const;
    std::vector<Point> best_path() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
