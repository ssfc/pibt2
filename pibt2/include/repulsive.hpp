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

  void set_field(Node* _v);
  void print_field()
  {
    cout << "field: " << endl;
    for(auto const& row : field)
    {
      for(auto const& ele : row)
      {
        cout << ele << " ";
      }
      cout << endl;
    }
    cout << endl;
  }

  // compare two nodes
  bool compare_nodes(Node* const v, Node* const u, Agent* ai)
  {
    int d_v = pathDist(ai->id, v);
    int d_u = pathDist(ai->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    // 优先选择不被占用的node
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    // 如果 v 未被占用且 u 被占用，返回 true，v 优先；
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;

    // 如果占用状态相同，返回 false（默认顺序）。
    return false;
  };
};
#endif  // REPULSIVE_HPP
