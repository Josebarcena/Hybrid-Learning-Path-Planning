#include "pathfinding.hpp"
#include <iostream>
#include <iomanip>
int main(int argc,char**argv) {
    try {
        auto grid=pf::Grid::from_file(argc>1?argv[1]:"maps/example.txt");
        pf::AnytimeSession session(grid,{0,0},{grid.width()-1,grid.height()-1},3.0);
        std::cout<<"weight,cost,elapsed_ms,expanded,optimality_proven,phase_ms,rebuild_ms,search_ms,rebuild_candidates,phase_expanded\n";
        for(double w:{3.0,2.5,2.0,1.5,1.0}) {
            auto s=session.advance(w);
            std::cout<<std::fixed<<std::setprecision(4)<<s.weight<<','<<s.cost<<','<<s.elapsed_ms<<','<<s.expanded<<','<<s.optimality_proven<<','<<s.phase_ms<<','<<s.rebuild_ms<<','<<s.search_ms<<','<<s.rebuild_candidates<<','<<s.phase_expanded<<'\n';
            if(s.finished) break;
        }
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
