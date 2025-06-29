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

  struct Point {
    int x, y;
  };

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
  PIBTM(MAPF_Instance* _P);
  ~PIBTM() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};


#endif  // PIBTM_HPP
