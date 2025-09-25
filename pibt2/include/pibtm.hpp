//
// Created by take_ on 25-6-29.
//

#ifndef PIBTM_HPP
#define PIBTM_HPP

#include "solver.hpp"

#include <vector>


using namespace std;


// 草，原来已经到达目标点的agent还可以避让。
class PIBTM : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

  void print_flow_field()
  {
    cout << "field: " << endl;
    for(auto const& row : window_flow)
    {
      for(auto const& cell : row)
      {
        for(auto const& dir : cell)
        {
          cout << dir << " ";
        }
        cout << ", ";
      }
      cout << endl;
    }
    cout << endl;
  }

  static int delta_to_index(int delta_x, int delta_y)
  {
    // 0向右
    if(delta_x == 1 && delta_y == 0)
    {
      return 0;
    }

    // 1向左
    if(delta_x == -1 && delta_y == 0)
    {
      return 1;
    }

    // 2向下
    if(delta_x == 0 && delta_y == 1)
    {
      return 2;
    }

    // 3向上
    if(delta_x == 0 && delta_y == -1)
    {
      return 3;
    }

    return -1;
  }

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

  // 流量场, 有四个方向, 0右, 1左, 2下, 3上
  vector<vector<vector<int>>> window_flow;

  // 流量场, 有四个方向, 0右, 1左, 2下, 3上
  vector<vector<vector<int>>> current_flow;

  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int curr_time = 0; // 当前时间
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker

    int agent_direction_x; // agent起点到终点的向量x投影
    int agent_direction_y; // agent起点到终点的向量x投影

    int mainstream_inner_product; // the inner product between start-goal and mainstream vector
    double mainstream_cos; // the cosine angle between start-goal and mainstream vector

    int agent_region; // agent所属的region
    int region_mainstream_inner_product; // the inner product between start-goal and region mainstream vector
    double region_mainstream_cos; // the cosine angle between start-goal and region mainstream vector
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

  static bool compare_time(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。80%概率给优先级, 不能完全不给随机; 如果完全找不到路再降一些
    if (a->elapsed != b->elapsed)
    {
      // 在MAPF语境下, 大部分时候是相等的，除非到达终点
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  static bool compare_mainstream(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。80%概率给优先级, 不能完全不给随机; 如果完全找不到路再降一些
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 初始距离越远，优先级越高。80%概率给优先级, 不能完全不给随机
    // 内积的方法能否完全刻画agent和主流的距离
    // use initial distance
    if (a->mainstream_inner_product != b->mainstream_inner_product)
    {
      return a->mainstream_inner_product > b->mainstream_inner_product;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  static bool compare_anti_mainstream(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。80%概率给优先级, 不能完全不给随机; 如果完全找不到路再降一些
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 初始距离越远，优先级越高。80%概率给优先级, 不能完全不给随机
    // 内积的方法能否完全刻画agent和主流的距离
    // use initial distance
    if (a->mainstream_inner_product != b->mainstream_inner_product)
    {
      return a->mainstream_inner_product < b->mainstream_inner_product;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  // compare priority of agents: inner production + timestep
  static bool compare_addition(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 内积与时间之和越大，优先级越高。
    // use initial distance
    //*
    if (a->mainstream_inner_product + a->elapsed != b->mainstream_inner_product + b->elapsed)
    {
      return a->mainstream_inner_product + a->elapsed > b->mainstream_inner_product + b->elapsed;
    }
    //*/

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };


  // compare priority of agents
  static bool compare_cos(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->mainstream_cos != b->mainstream_cos)
    {
      return a->mainstream_cos > b->mainstream_cos;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  // compare priority of agents
  static bool compare_anti_cos(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->mainstream_cos != b->mainstream_cos)
    {
      return a->mainstream_cos < b->mainstream_cos;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  // compare priority of agents
  static bool compare_region_inner(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 越符合区域主流，优先级越高。
    // use initial distance
    if (a->region_mainstream_inner_product != b->region_mainstream_inner_product)
    {
      return a->region_mainstream_inner_product > b->region_mainstream_inner_product;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  // compare priority of agents
  static bool compare_anti_region_inner(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 越符合区域主流，优先级越高。
    // use initial distance
    if (a->region_mainstream_inner_product != b->region_mainstream_inner_product)
    {
      return a->region_mainstream_inner_product < b->region_mainstream_inner_product;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  static bool compare_region_cos(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 越符合区域主流，优先级越高。
    // use initial distance
    if (a->region_mainstream_cos != b->region_mainstream_cos)
    {
      return a->region_mainstream_cos > b->region_mainstream_cos;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  static bool compare_anti_region_cos(const Agent* a, const Agent* b)
  {
    // 优先比较: 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed)
    {
      return a->elapsed > b->elapsed;
    }

    // 次级比较: 越符合区域主流，优先级越高。
    // use initial distance
    if (a->region_mainstream_cos != b->region_mainstream_cos)
    {
      return a->region_mainstream_cos < b->region_mainstream_cos;
    }

    // 次级比较: 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d)
    {
      return a->init_d > b->init_d;
    }

    // 最后比较: 0-1随机数
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

public:
  PIBTM(MAPF_Instance* _P);
  ~PIBTM() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};


#endif  // PIBTM_HPP
