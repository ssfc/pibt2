//
// Created by take_ on 25-7-13.
//

#ifndef REPULSIVE_HPP
#define REPULSIVE_HPP

#include "solver.hpp"

#include <vector>

using namespace std;


class PIBTR : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:

  // 斥力场
  vector<vector<int>> field;

  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker
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
  PIBTR(MAPF_Instance* _P);
  ~PIBTR() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
#endif  // REPULSIVE_HPP
