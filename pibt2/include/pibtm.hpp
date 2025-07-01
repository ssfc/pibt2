//
// Created by take_ on 25-6-29.
//

#ifndef PIBTM_HPP
#define PIBTM_HPP

#include "solver.hpp"

class PIBTM : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:

  struct Direction {
    int x = 0;
    int y = 0;
  };

  int small_region_column = 25; // (errand点)(小)每个region所占的列数
  int small_region_row = 25; // (errand点)(小)每个region所占的行数
  // (历史统计)地图有几列small region
  int num_small_region_column;
  // (历史统计)地图有几行small region
  int num_small_region_row;


  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker

    int agent_direction_x; // agent起点到终点的向量x投影
    int agent_direction_y; // agent起点到终点的向量x投影

    int mainstream_inner_product; // the inner product between start-goal and mainstream vector
    double mainstream_cos; // the cosine angle between start-goal and mainstream vector

    int agent_region; // agent所属的region
    int region_mainstream_inner_product; // the inner product between start-goal and region mainstream vector
  };
  using Agents = std::vector<Agent*>;

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  // option
  bool disable_dist_init = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);

  // main
  void run();

public:
  PIBTM(MAPF_Instance* _P);
  ~PIBTM() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};


#endif  // PIBTM_HPP
