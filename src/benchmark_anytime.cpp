#include "pathfinding.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
int integer(const char* s) { int v{}; auto e=s+std::char_traits<char>::length(s); auto [p,ec]=std::from_chars(s,e,v); if(ec!=std::errc{}||p!=e) throw std::invalid_argument("Invalid integer"); return v; }
pf::Grid make_grid(int side,int percent,std::uint32_t seed) {
 std::mt19937 rng(seed); std::vector<std::uint8_t> cells(static_cast<std::size_t>(side)*side);
 const std::uint64_t threshold=(std::uint64_t{1}<<32)*percent/100;
 for(auto& c:cells) c=static_cast<std::uint64_t>(rng())<threshold;
 cells.front()=cells.back()=0; return {side,side,std::move(cells)};
}
int oracle(const pf::Grid& grid,pf::Point start,pf::Point goal) {
 std::vector<int> d(grid.cells(),-1); std::queue<pf::Point> q; d[grid.index(start)]=0; q.push(start);
 constexpr pf::Point moves[]={{1,0},{-1,0},{0,1},{0,-1}};
 while(!q.empty()) {auto p=q.front();q.pop();if(p==goal)return d[grid.index(p)];for(auto m:moves){pf::Point n{p.x+m.x,p.y+m.y};if(!grid.free(n)||d[grid.index(n)]!=-1)continue;d[grid.index(n)]=d[grid.index(p)]+1;q.push(n);}}
 return -1;
}
void validate_path(const pf::Grid& grid,const std::vector<pf::Point>& path,double cost,pf::Point start,pf::Point goal) {
 if(path.empty()||!(path.front()==start)||!(path.back()==goal)||cost!=static_cast<double>(path.size()-1))throw std::runtime_error("Invalid path endpoints or cost: size="+std::to_string(path.size())+" cost="+std::to_string(cost)+" start="+(path.empty()?"empty":std::to_string(path.front().x)+","+std::to_string(path.front().y)));
 for(std::size_t i=0;i<path.size();++i){if(!grid.free(path[i]))throw std::runtime_error("Path crosses obstacle");if(i&&std::abs(path[i].x-path[i-1].x)+std::abs(path[i].y-path[i-1].y)!=1)throw std::runtime_error("Nonadjacent path");}
}
void validate_anytime(const pf::Grid& grid,const pf::AnytimeResult& r,int optimal,pf::Point start,pf::Point goal) {
 if(r.found!=(optimal>=0))throw std::runtime_error("ARA* reachability mismatch: use an unlimited run for certification");
 if(r.found){if(r.cost<optimal||r.cost>3.0*optimal+1e-9)throw std::runtime_error("ARA* invalid cost bound");if(r.optimality_proven&&r.cost!=optimal)throw std::runtime_error("ARA* falsely certified optimality");}
 double prev=std::numeric_limits<double>::infinity();double last_t=-1;std::size_t last_exp=0;
 for(const auto& s:r.improvements){validate_path(grid,s.path,s.cost,start,goal);if(s.cost>prev||s.elapsed_ms<last_t||s.expanded<last_exp||s.cost<optimal)throw std::runtime_error("Invalid anytime progression");prev=s.cost;last_t=s.elapsed_ms;last_exp=s.expanded;}
}
}
int main(int argc,char** argv) {
 try {
  // benchmark_anytime [output.csv] [maps_per_configuration] [repetitions] [max_side]
  const std::string output=argc>1?argv[1]:"results/anytime.csv";
  int maps=argc>2?integer(argv[2]):10,reps=argc>3?integer(argv[3]):5,max_side=argc>4?integer(argv[4]):500;
  if(argc>5||maps<1||reps<1||max_side<10||max_side>2000)throw std::invalid_argument("Usage: pathfinding_benchmark_anytime [output.csv] [maps>=1] [repetitions>=1] [max_side=10..2000]");
  auto parent=std::filesystem::path(output).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
  std::ofstream csv(output);if(!csv)throw std::runtime_error("Cannot open output CSV");
  csv<<"side,obstacle_percent,map_id,seed,reachable,optimal_cost,algorithm,weight_or_budget_ms,repeat,phase,found,optimality_proven,cost,relative_error,elapsed_ms,expanded,estimated_peak_bytes\n";
  csv<<std::fixed<<std::setprecision(8);
  std::size_t rows=0,cases=0,unreachable=0;
  for(int side:{50,100,250,500}) {if(side>max_side)continue;
   for(int percent:{10,20,30})for(int map_id=0;map_id<maps;++map_id){
    auto seed=static_cast<std::uint32_t>(20261110u+side*10000u+percent*100u+map_id);
    auto grid=make_grid(side,percent,seed);pf::Point start{0,0},goal{side-1,side-1};int optimal=oracle(grid,start,goal);++cases;if(optimal<0)++unreachable;
    auto row=[&](const std::string& algorithm,double weight,int rep,int phase,bool found,bool proven,double cost,double ms,std::size_t expanded,std::size_t mem){
     csv<<side<<','<<percent<<','<<map_id<<','<<seed<<','<<(optimal>=0)<<',';if(optimal>=0)csv<<optimal;
     csv<<','<<algorithm<<','<<weight<<','<<rep<<','<<phase<<','<<found<<','<<proven<<',';
     if(found)csv<<cost;csv<<',';if(found&&optimal>0)csv<<(cost-optimal)/optimal;
     csv<<','<<ms<<','<<expanded<<','<<mem<<'\n';++rows;
    };
    // Warm-up both implementations, excluded from measurements.
    (void)pf::astar(grid,start,goal);
    (void)pf::ara_star(grid,start,goal,3.0,0.5);
    for(int rep=0;rep<reps;++rep){
     auto a=pf::astar(grid,start,goal);
     if(a.found!=(optimal>=0)||(a.found&&a.cost!=optimal))throw std::runtime_error("A* oracle mismatch");
     if(a.found)validate_path(grid,a.path,a.cost,start,goal);
     row("AStar",1.0,rep,0,a.found,true,a.cost,a.elapsed_ms,a.expanded,a.estimated_peak_bytes);
     auto r=pf::ara_star(grid,start,goal,3.0,0.5);
     validate_anytime(grid,r,optimal,start,goal);
     if(!r.optimality_proven)throw std::runtime_error("Unlimited ARA* did not certify completion");
     for(std::size_t i=0;i<r.improvements.size();++i){auto& s=r.improvements[i];row("ARAStar",s.weight,rep,static_cast<int>(i+1),true,s.weight==1.0&&r.optimality_proven&&s.cost==optimal,s.cost,s.elapsed_ms,s.expanded,s.estimated_peak_bytes);}
     row("ARAStar",1.0,rep,0,r.found,r.optimality_proven,r.cost,r.elapsed_ms,r.expanded,r.estimated_peak_bytes);
     // Time-budget runs are independent, with their own warm-up and a separate
     // row type. Not finding a route before the deadline is not an error.
     for(double budget:{0.1,1.0,5.0,10.0}) {
      auto limited=pf::ara_star(grid,start,goal,3.0,0.5,budget);
      if(limited.found && (optimal<0||limited.cost<optimal||limited.cost>3.0*optimal+1e-9))
       throw std::runtime_error("Invalid budgeted ARA* result");
      if(limited.optimality_proven && limited.found && limited.cost!=optimal)
       throw std::runtime_error("Invalid budgeted optimality claim");
      row("ARAStarBudget",budget,rep,0,limited.found,limited.optimality_proven,limited.cost,limited.elapsed_ms,limited.expanded,limited.estimated_peak_bytes);
     }
    }
    std::cout<<"Completed "<<side<<"x"<<side<<" density="<<percent<<"% map="<<map_id+1<<"/"<<maps<<" reachable="<<(optimal>=0)<<'\n';
   }
  }
  std::cout<<"Saved "<<rows<<" rows across "<<cases<<" maps ("<<unreachable<<" unreachable) to "<<output<<'\n';
 }catch(const std::exception& e){std::cerr<<"Benchmark error: "<<e.what()<<'\n';return 1;}
}
