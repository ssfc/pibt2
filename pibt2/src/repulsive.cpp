//
// Created by take_ on 25-7-13.
//
#include "../include/repulsive.hpp"

#include <iostream>

using namespace std;


const std::string PIBTR::SOLVER_NAME = "PIBTR";

// 初始化 PIBT 路径规划求解器实例。
PIBTR::PIBTR(MAPF_Instance* _P)
    : MAPF_Solver(_P),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBTR::SOLVER_NAME;
}

void PIBTR::run()
{
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    // 已耗时越多，优先级越高（先被决策）。
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    // 初始距离越远，优先级越高。
    // use initial distance
    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    return a->tie_breaker > b->tie_breaker; // 随机值作为打破平手的最后手段。
  };

  Agents A; // 用于存储全部Agent指针

  Grid* grid = reinterpret_cast<Grid*>(G);
  cout << "map num row: " << grid->getHeight() << endl;
  cout << "map num column: " << grid->getWidth() << endl;
  cout << "agent num: " << A.size() << endl;

  field.resize(grid->getHeight(), vector<int>(grid->getWidth(), 0));

  // initialize
  for (int i = 0; i < P->getNum(); ++i)
  {
    Node* s = P->getStart(i); // 起点
    Node* g = P->getGoal(i); // 终点
    int d = disable_dist_init ? 0 : pathDist(i);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    A.push_back(a);
    occupied_now[s->id] = a;



  }
  solution.add(P->getConfigStart());



  // main loop
  int timestep = 0;
  while (true)
  {

    if(timestep % 1 == 0)
    // if(timestep % 10 == 0)
    {
      info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);
    }

    // planning
    std::sort(A.begin(), A.end(), compare); // 按优先级排序
    for (auto a : A)
    {
      // if the agent has next location, then skip
      if (a->v_next == nullptr)
      {
        // 为 agent 决策下一步目标
        // determine its next location
        funcPIBT(a);
      }
    }

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (auto a : A)
    {
      // clear
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      occupied_next[a->v_next->id] = nullptr;
      // set next location
      config[a->id] = a->v_next; // 更新位置
      occupied_now[a->v_next->id] = a; // 标记新位置被占
      // 检查是否全部到达目标
      // check goal condition
      check_goal_cond &= (a->v_next == a->g);

      // 若到终点，重置；否则步数+1
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      // 更新当前节点
      a->v_now = a->v_next;
      // 重置下一步变量
      a->v_next = nullptr;
    }

    // 路径记录
    // update plan
    solution.add(config);

    ++timestep;

    // 全部到达终点
    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // 超时或步数过多则退出
    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  // 4. 内存回收
  // memory clear
  for (auto a : A) delete a;
}

// 为单个智能体（ai）在下一步寻找一个合适的位置，尽可能避免与其它智能体冲突，同时能处理优先级冲突与递归回溯。
bool PIBTR::funcPIBT(Agent* ai, Agent* aj)
{
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(ai->id, v);
    int d_u = pathDist(ai->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue;
    if (aj != nullptr && u == aj->v_now) continue;

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)) continue;  // replanning
    }
    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

// 根据命令行参数设置 PIBT 算法的内部参数。
void PIBTR::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

// 在命令行输出PIBT算法的帮助信息，主要用于指导用户如何使用可用的命令行参数。
void PIBTR::printHelp()
{
  std::cout << PIBTR::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}

void PIBTR::set_field(Node* _v)
{
  // 顶点位置的斥力场
  field[_v->pos.y][_v->pos.x]++;
  // 顶点周边cell也放上力场
  if(_v->pos.y -1 >= 0)
  {
    field[_v->pos.y -1][_v->pos.x]++;
  }

  if(_v->pos.y +1 < field.size())
  {
    field[_v->pos.y +1][_v->pos.x]++;
  }

  
}