#include "pathfinding.hpp"
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif
static std::size_t peak_rss_bytes(){
#ifdef _WIN32
 PROCESS_MEMORY_COUNTERS_EX c{};c.cb=sizeof(c);
 if(GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&c),sizeof(c)))return c.PeakWorkingSetSize;
 return 0;
#else
 rusage u{};if(getrusage(RUSAGE_SELF,&u)!=0)return 0;
#ifdef __APPLE__
 return static_cast<std::size_t>(u.ru_maxrss);
#else
 return static_cast<std::size_t>(u.ru_maxrss)*1024;
#endif
#endif
}
int main(int argc,char** argv){
 const int side=argc>1?std::atoi(argv[1]):500;
 const int density=argc>2?std::atoi(argv[2]):20;
 const unsigned seed=argc>3?static_cast<unsigned>(std::strtoul(argv[3],nullptr,10)):20261110u;
 if(side<2||side>2000||density<0||density>95){std::cerr<<"Usage: memory_probe [side 2..2000] [density 0..95] [seed]\n";return 2;}
 std::mt19937 rng(seed);std::vector<std::uint8_t> b(static_cast<std::size_t>(side)*side);
 for(auto& cell:b)cell=static_cast<std::uint8_t>(rng()%100<static_cast<unsigned>(density));
 b.front()=b.back()=0;
 pf::Grid grid(side,side,std::move(b));
 pf::AnytimeSession session(grid,{0,0},{side-1,side-1},3.0);
 std::cout<<"weight,found,cost,estimated_peak_bytes,process_peak_rss_bytes,expanded\n";
 for(double w:{3.0,2.5,2.0,1.5,1.0}){
  const auto o=session.advance(w);
  std::cout<<std::fixed<<std::setprecision(1)<<w<<','<<o.found<<','<<o.cost<<','<<o.estimated_peak_bytes<<','<<peak_rss_bytes()<<','<<o.expanded<<'\n';
 }
}
