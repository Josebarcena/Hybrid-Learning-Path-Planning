#include "pathfinding.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
namespace pf {
namespace {
struct Entry { double key; int id; };
struct Compare { bool operator()(Entry a, Entry b) const { return a.key==b.key ? a.id>b.id : a.key>b.key; } };
double heuristic(Point a, Point b) { return std::abs(a.x-b.x)+std::abs(a.y-b.y); }
}
struct AnytimeSession::Impl {
    const Grid& grid;
    Point goal;
    int source, target, n;
    double weight;
    std::vector<double> g;
    std::vector<int> parent;
    struct PathNode { int cell; int previous; };
    std::vector<PathNode> history;
    std::vector<int> path_ref;
    std::vector<std::uint8_t> active, closed, incons;
    std::vector<int> active_ids, incons_ids, closed_ids;
    std::priority_queue<Entry,std::vector<Entry>,Compare> open;
    double accumulated_ms=0.0, last_phase_ms=0.0, last_rebuild_ms=0.0, last_search_ms=0.0;
    std::size_t last_candidates=0, last_expanded=0;
    std::size_t expanded=0, peak=0;
    bool proven=false, finished=false, first=true;
    Impl(const Grid& gr,Point s,Point t,double w):grid(gr),goal(t),source(gr.index(s)),target(gr.index(t)),n(static_cast<int>(gr.cells())),weight(w),g(n,std::numeric_limits<double>::infinity()),parent(n,-1),path_ref(n,-1),active(n),closed(n),incons(n) {
        g[source]=0; history.push_back({source,-1}); path_ref[source]=0; push(source);
    }
    double elapsed() const { return accumulated_ms; }
    std::size_t memory() const { return g.capacity()*sizeof(double)+parent.capacity()*sizeof(int)+(active.capacity()+closed.capacity()+incons.capacity())+open.size()*sizeof(Entry)+history.capacity()*sizeof(PathNode)+path_ref.capacity()*sizeof(int)+active_ids.capacity()*sizeof(int)+incons_ids.capacity()*sizeof(int)+closed_ids.capacity()*sizeof(int); }
    void push(int id) { if(!active[id]) active_ids.push_back(id); active[id]=1; open.push({g[id]+weight*heuristic(grid.point(id),goal),id}); peak=std::max(peak,memory()); }
    void clean() {
        while(!open.empty()) {
            auto e=open.top();
            if(active[e.id] && std::abs(e.key-(g[e.id]+weight*heuristic(grid.point(e.id),goal)))<1e-8) break;
            open.pop();
        }
    }
    PhaseObservation observation() const {
        double lb=g[target];
        for(int id:active_ids) if(active[id]) lb=std::min(lb,g[id]+heuristic(grid.point(id),goal));
        for(int id:incons_ids) if(incons[id]) lb=std::min(lb,g[id]+heuristic(grid.point(id),goal));
        if(!std::isfinite(lb)) lb=0.0;
        double gap=std::isfinite(g[target]) && g[target]>0 ? std::clamp((g[target]-lb)/g[target],0.0,1.0):1.0;
        return {weight,g[target],elapsed(),expanded,peak,std::isfinite(g[target]),proven,finished,last_phase_ms,last_rebuild_ms,last_search_ms,last_candidates,last_expanded,lb,gap};
    }
    void rebuild(double w) {
        weight=w;
        std::priority_queue<Entry,std::vector<Entry>,Compare> next;
        last_candidates=0;
        for(int id:active_ids) {
            if(active[id]) {
                next.push({g[id]+weight*heuristic(grid.point(id),goal),id});
                ++last_candidates;
            }
        }
        active_ids.clear();
        for(int id:incons_ids) {
            if(incons[id] && !active[id]) {
                active[id]=1;
                next.push({g[id]+weight*heuristic(grid.point(id),goal),id});
                ++last_candidates;
            }
            incons[id]=0;
        }
        incons_ids.clear();
        // Retain only live IDs for subsequent phases.
        auto copy=next;
        while(!copy.empty()) { active_ids.push_back(copy.top().id); copy.pop(); }
        for(int id:closed_ids) closed[id]=0;
        closed_ids.clear();
        open.swap(next); peak=std::max(peak,memory());
    }
    PhaseObservation step(double w) {
        if(finished) return observation();
        if(!std::isfinite(w)||w<1||w>weight) throw std::invalid_argument("Weight must be in [1,current]");
        using Clock=std::chrono::steady_clock;
        const auto phase_start=Clock::now();
        if(!first) rebuild(w);
        else { last_candidates=0; }
        const auto search_start=Clock::now();
        const auto expanded_before=expanded;
        first=false;
        constexpr Point dirs[]={{1,0},{-1,0},{0,1},{0,-1}};
        while(true) {
            clean();
            if(open.empty() || g[target]<=open.top().key) break;
            auto e=open.top();open.pop();active[e.id]=0;closed[e.id]=1;closed_ids.push_back(e.id);++expanded;
            Point p=grid.point(e.id);
            for(auto d:dirs) {
                Point q{p.x+d.x,p.y+d.y};
                if(!grid.free(q)) continue;
                int id=grid.index(q);
                if(g[e.id]+1>=g[id]) continue;
                g[id]=g[e.id]+1;parent[id]=e.id;history.push_back({id,path_ref[e.id]});path_ref[id]=static_cast<int>(history.size()-1);peak=std::max(peak,memory());
                if(!closed[id]) push(id); else if(!incons[id]) { incons[id]=1; incons_ids.push_back(id); }
            }
        }
        clean();
        bool pending=!incons_ids.empty();
        if(w==1.0 || (open.empty()&&!pending)) { proven=true;finished=true; }
        const auto phase_end=Clock::now();
        last_rebuild_ms=std::chrono::duration<double,std::milli>(search_start-phase_start).count();
        last_search_ms=std::chrono::duration<double,std::milli>(phase_end-search_start).count();
        last_phase_ms=last_rebuild_ms+last_search_ms;
        accumulated_ms+=last_phase_ms;
        last_expanded=expanded-expanded_before;
        return observation();
    }
    std::vector<Point> path() const {
        std::vector<Point> out;
        if(!std::isfinite(g[target])) return out;
        for(int ref=path_ref[target],steps=0;ref!=-1 && steps<=n;ref=history[ref].previous,++steps) out.push_back(grid.point(history[ref].cell));
        std::reverse(out.begin(),out.end());
        if(out.empty()||grid.index(out.front())!=source) throw std::logic_error("Invalid parent chain");
        return out;
    }
};
AnytimeSession::AnytimeSession(const Grid& g,Point s,Point t,double w) {
    if(!std::isfinite(w)||w<1||!g.free(s)||!g.free(t)) throw std::invalid_argument("Invalid session arguments");
    impl_=std::make_unique<Impl>(g,s,t,w);
}
AnytimeSession::~AnytimeSession()=default;
AnytimeSession::AnytimeSession(AnytimeSession&&) noexcept=default;
AnytimeSession& AnytimeSession::operator=(AnytimeSession&&) noexcept=default;
PhaseObservation AnytimeSession::advance(double w){return impl_->step(w);}
PhaseObservation AnytimeSession::observe() const{return impl_->observation();}
std::vector<Point> AnytimeSession::best_path() const{return impl_->path();}
}
