#include "pathfinding.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int ACTIONS=3, STATES=4*3*3*3*3*3;
using Clock=std::chrono::steady_clock;
using QTable=std::array<std::array<double,ACTIONS>,STATES>;
struct MapCase {pf::Grid grid; int side,density,optimal;};
int bucket(double v,std::initializer_list<double> cuts){int k=0;for(double c:cuts)if(v>c)++k;return k;}
struct Agent {
 QTable q{}; std::mt19937 rng{20261110};
 int choose(int s,double epsilon){
  std::uniform_real_distribution<double> d(0,1);
  if(d(rng)<epsilon)return static_cast<int>(rng()%ACTIONS);
  int a=0;for(int j=1;j<ACTIONS;++j)if(q[s][j]>q[s][a]+1e-12)a=j;
  return a;
 }
 void update(int s,int a,double reward,int next,bool terminal){
  constexpr double alpha=.12,gamma=.95;
  double future=terminal?0:*std::max_element(q[next].begin(),q[next].end());
  q[s][a]+=alpha*(reward+gamma*future-q[s][a]);
 }
};
MapCase make_case(int side,int density,std::uint32_t seed){
 std::mt19937 rng(seed);std::vector<std::uint8_t> blocks(static_cast<std::size_t>(side)*side);
 const std::uint64_t threshold=(std::uint64_t{1}<<32)*density/100;
 for(auto& b:blocks)b=static_cast<std::uint64_t>(rng())<threshold;
 blocks.front()=blocks.back()=0;
 pf::Grid grid(side,side,std::move(blocks));
 auto ref=pf::astar(grid,{0,0},{side-1,side-1});
 return {std::move(grid),side,density,ref.found?static_cast<int>(ref.cost):-1};
}
// All state features are available at decision time; no optimal-cost oracle.
int state(const pf::PhaseObservation& o,double prev_cost,int side,int ablation){
 int w=bucket(o.weight,{1.01,1.5,2.25});
 int stretch=!o.found?2:bucket(o.cost/std::max(1,2*(side-1)),{1.05,1.25});
 int improvement=!std::isfinite(prev_cost)?2:(o.cost<prev_cost-1e-9?1:0);
 int activity=bucket(static_cast<double>(o.phase_expanded),{0,static_cast<double>(side)});
 int time=bucket(o.elapsed_ms,{0.1,1.0});
 int gap=ablation==1?0:bucket(o.certified_gap,{0.01,0.05});
 return (((((w*3+stretch)*3+improvement)*3+activity)*3+time)*3+gap);
}
struct Trace {int step=0,state=0,action=0;double weight=0,cost=0,lower_bound=0,gap=0,after_cost=0,after_gap=0,phase_ms=0;std::size_t expanded=0;};
struct Outcome {std::vector<Trace> trace;bool found=false,proven=false;double cost=0,planner_ms=0,decision_ms=0,wall_ms=0;std::size_t planner_bytes=0,total_bytes=0,expanded=0;int decisions=0,stops=0,small=0,large=0;};
// 0 learned; 1 first solution; 2 jump to one; 3 fixed decrement; 4 bounded-quality baseline.
Outcome run(const MapCase& m,Agent* agent,int policy,bool training,double epsilon,int ablation){
 const auto wall_start=Clock::now();
 pf::AnytimeSession session(m.grid,{0,0},{m.side-1,m.side-1},3.0);
 auto o=session.advance(3.0);
 double prev=std::numeric_limits<double>::infinity();
 struct Transition{int s,a;};std::vector<Transition> transitions;
 Outcome out;
 while(!o.finished && out.decisions<10){
  const auto decision_start=Clock::now();
  int s=state(o,prev,m.side,ablation),a=0;
  if(policy==0)a=agent->choose(s,training?epsilon:0);
  else if(policy==1)a=0;
  else if(policy==2)a=2;
  else if(policy==3)a=1;
  else { // Conservative deterministic quality proxy, not a proof of optimality.
   const double manhattan=2.0*(m.side-1);
   a=(o.found && o.cost<=1.05*manhattan)?0:2;
  }
  if(!o.found&&a==0)a=2;
  if(o.weight<=1.0000001)a=0;
  out.decision_ms+=std::chrono::duration<double,std::milli>(Clock::now()-decision_start).count();
  if(training&&policy==0)transitions.push_back({s,a});
  Trace tr;tr.step=out.decisions;tr.state=s;tr.action=a;tr.weight=o.weight;tr.cost=o.cost;tr.lower_bound=o.admissible_lower_bound;tr.gap=o.certified_gap;
  ++out.decisions;
  if(a==0){++out.stops;tr.after_cost=o.cost;tr.after_gap=o.certified_gap;out.trace.push_back(tr);break;}
  if(a==1)++out.small;else ++out.large;
  prev=o.cost;
  const double decrement=policy==3?.5:(a==1?.25:1.0);
  o=session.advance(std::max(1.0,o.weight-decrement));
  tr.after_cost=o.cost;tr.after_gap=o.certified_gap;tr.phase_ms=o.phase_ms;tr.expanded=o.phase_expanded;out.trace.push_back(tr);
 }
 out.found=o.found;out.proven=o.optimality_proven;out.cost=o.found?o.cost:0;
 out.planner_ms=o.elapsed_ms;out.planner_bytes=o.estimated_peak_bytes;out.expanded=o.expanded;
 out.total_bytes=out.planner_bytes+(policy==0?sizeof(QTable):0);
 out.wall_ms=std::chrono::duration<double,std::milli>(Clock::now()-wall_start).count();
 if(training&&policy==0){
  const double relative=(m.optimal>0&&out.found)?(out.cost-m.optimal)/m.optimal:
                        (m.optimal<0&&!out.found?0:1.0);
  // Normalized per map size; terminal reward and action penalty are independent of reference timing noise.
  const double tnorm=out.planner_ms/std::max(0.01,static_cast<double>(m.side*m.side)/100000.0);
  const double mnorm=static_cast<double>(out.total_bytes)/(m.side*m.side*16.0);
  const double utility=-(6.0*relative+0.3*std::log1p(tnorm)+0.15*mnorm+
                         (!out.found&&m.optimal>=0?10.0:0.0));
  // Dense marginal feedback: improving a certified gap is valuable; extra work costs time.
  // Oracle optimal cost remains restricted to the final training reward.
  for(int k=static_cast<int>(transitions.size())-1;k>=0;--k){
   bool terminal=k+1==static_cast<int>(transitions.size());
   const auto& tr=out.trace[static_cast<std::size_t>(k)];
   const double marginal=ablation==2?0.0:2.0*(tr.gap-tr.after_gap)-0.2*std::log1p(tr.phase_ms/std::max(.01,static_cast<double>(m.side*m.side)/100000.0))-.025;
   agent->update(transitions[k].s,transitions[k].a,(terminal?utility:0.0)+marginal,
                 terminal?0:transitions[k+1].s,terminal);
  }
 }
 return out;
}
}
int main(int argc,char** argv){
 try{
  const std::string output=argc>1?argv[1]:"results/rl_v10.csv";
  const int train=argc>2?std::stoi(argv[2]):40,eval=argc>3?std::stoi(argv[3]):20;
  const int replicate=argc>4?std::stoi(argv[4]):0;
  const bool include_500=argc>5?std::stoi(argv[5])!=0:false;
  const int ablation=argc>6?std::stoi(argv[6]):0; // 0=full, 1=no_gap, 2=no_marginal
  if(train<1||eval<1||train>1000||eval>1000||replicate<0||replicate>10000||(ablation<0||ablation>2)||argc>7)throw std::invalid_argument("Usage: pathfinding_rl_benchmark [csv] [train_per_config] [eval_per_config] [replicate] [include_500_0_or_1] [ablation_0_full_1_no_gap_2_no_marginal]");
  Agent agent;
  const std::array<int,3> train_sides{50,100,250},densities{10,20,30};
  std::vector<int> eval_sides{50,100,250};
  if(include_500)eval_sides.push_back(500);
  int episodes=0;
  for(int epoch=0;epoch<3;++epoch)for(int side:train_sides)for(int density:densities)for(int i=0;i<train;++i){
   auto m=make_case(side,density,static_cast<std::uint32_t>(1000000+side*10000+density*1000+i+replicate*100000003u));
   double epsilon=std::max(.05,.35-.30*episodes/static_cast<double>(27*train));
   run(m,&agent,0,true,epsilon,ablation);++episodes;
  }
  auto parent=std::filesystem::path(output).parent_path();
  if(!parent.empty())std::filesystem::create_directories(parent);
  std::ofstream csv(output);if(!csv)throw std::runtime_error("Cannot open output");
  const auto trace_path=std::filesystem::path(output).parent_path()/(std::filesystem::path(output).stem().string()+"_trace.csv");
  std::ofstream trace(trace_path);if(!trace)throw std::runtime_error("Cannot open trace");
  trace<<"replicate,side,density,map_id,policy,step,state,action,weight,cost,lower_bound,certified_gap,after_cost,after_gap,phase_ms,phase_expanded\n";
  trace<<std::fixed<<std::setprecision(8);
  csv<<"replicate,side,density,map_id,seed,policy,reachable,optimal_cost,found,cost,relative_error,planner_ms,decision_ms,wall_ms,planner_peak_bytes,estimated_total_bytes,expanded,decisions,stop_actions,small_actions,large_actions,optimality_proven\n";
  csv<<std::fixed<<std::setprecision(8);
  const std::array<const char*,5> names{ablation==1?"q_learning_no_gap":(ablation==2?"q_learning_no_marginal":"q_learning"),"stop_first","jump_to_one","fixed_half","quality_proxy"};
  for(int side:eval_sides)for(int density:densities)for(int i=0;i<eval;++i){
   const auto seed=static_cast<std::uint32_t>(9000000+side*10000+density*1000+i+replicate*100000003u);
   auto m=make_case(side,density,seed);
   for(int p=0;p<5;++p){
    auto r=run(m,&agent,p,false,0,ablation);
    for(const auto& tr:r.trace) trace<<replicate<<','<<side<<','<<density<<','<<i<<','<<names[p]<<','<<tr.step<<','<<tr.state<<','<<tr.action<<','<<tr.weight<<','<<tr.cost<<','<<tr.lower_bound<<','<<tr.gap<<','<<tr.after_cost<<','<<tr.after_gap<<','<<tr.phase_ms<<','<<tr.expanded<<'\n';
    if(r.found&&(m.optimal<0||r.cost<m.optimal-1e-8||r.cost>3.0*m.optimal+1e-8))throw std::runtime_error("Invalid path cost bound");
    if(r.proven&&r.found&&r.cost!=m.optimal)throw std::runtime_error("False optimality proof");
    csv<<replicate<<','<<side<<','<<density<<','<<i<<','<<seed<<','<<names[p]<<','<<(m.optimal>=0)<<',';
    if(m.optimal>=0)csv<<m.optimal;
    csv<<','<<r.found<<',';if(r.found)csv<<r.cost;
    csv<<',';if(r.found&&m.optimal>0)csv<<(r.cost-m.optimal)/m.optimal;
    csv<<','<<r.planner_ms<<','<<r.decision_ms<<','<<r.wall_ms<<','<<r.planner_bytes<<','<<r.total_bytes<<','<<r.expanded<<','<<r.decisions<<','<<r.stops<<','<<r.small<<','<<r.large<<','<<r.proven<<'\n';
   }
  }
  std::cout<<"Trained "<<episodes<<" episodes; evaluated "<<eval_sides.size()*3*eval<<" maps ("<<eval_sides.size()*15*eval<<" rows). Saved "<<output<<" and "<<trace_path.string()<<'\n';
 }catch(const std::exception& e){std::cerr<<"RL benchmark error: "<<e.what()<<'\n';return 1;}
}
