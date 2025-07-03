//
// Created by take_ on 25-6-29.
//

#include "../include/pibtm.hpp"

#include <cmath>
#include <iostream>

using namespace std;


const std::string PIBTM::SOLVER_NAME = "PIBTM";

// 初始化 PIBT 路径规划求解器实例。
PIBTM::PIBTM(MAPF_Instance* _P)
    : MAPF_Solver(_P),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBTM::SOLVER_NAME;
}

void PIBTM::run()
{

  Agents A; // 用于存储全部Agent指针

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

  Grid* grid = reinterpret_cast<Grid*>(G);
  cout << "map num row: " << grid->getHeight() << endl;
  cout << "map num column: " << grid->getWidth() << endl;
  cout << "agent num: " << A.size() << endl;

  /*
  cout << "agent 0 curr: " << A[0]->v_now->pos.x << " " << A[0]->v_now->pos.y << endl;
  cout << "agent 0 goal: " << A[0]->g->pos.x << " " << A[0]->g->pos.y << endl;

  int agent_0_direction_x = A[0]->g->pos.x - A[0]->v_now->pos.x;
  int agent_0_direction_y = A[0]->g->pos.y - A[0]->v_now->pos.y;
  cout << "agent 0 vector: " << agent_0_direction_x << " " << agent_0_direction_y << endl;
   */

  // 计算mainstream: 把所有向量累加起来
  int mainstream_x = 0;
  int mainstream_y = 0;
  for (int i = 0; i < P->getNum(); ++i)
  {
    A[i]->agent_direction_x = A[i]->g->pos.x - A[i]->v_now->pos.x;
    A[i]->agent_direction_y = A[i]->g->pos.y - A[i]->v_now->pos.y;

    mainstream_x += A[i]->agent_direction_x;
    mainstream_y += A[i]->agent_direction_y;
  }

  cout << "mainstream direction: " << mainstream_x << " " << mainstream_y << endl;

  // 计算所有agent向量和mainstream向量的内积
  for (int i = 0; i < P->getNum(); ++i)
  {
    A[i]->mainstream_inner_product = A[i]->agent_direction_x * mainstream_x +
                                     A[i]->agent_direction_y * mainstream_y;
    A[i]->mainstream_cos = (A[i]->agent_direction_x * mainstream_x +
                            A[i]->agent_direction_y * mainstream_y) /
                           (sqrt(A[i]->agent_direction_x * A[i]->agent_direction_x +
                                 A[i]->agent_direction_y * A[i]->agent_direction_y) *
                           sqrt(mainstream_x * mainstream_x +
                                mainstream_y * mainstream_y));
  }

  cout << "A[0] cos: " << A[0]->mainstream_cos << endl;

  // 地图有几列region
  num_small_region_column = std::ceil((double)grid->getWidth() / small_region_column);
  // 地图有几行region
  num_small_region_row = std::ceil((double)grid->getHeight() / small_region_row);

  cout << "num small region column: " << num_small_region_column << endl;
  cout << "num small region row: " << num_small_region_row << endl;

  // 将map分为若干区域, 统计每个区域agent的数量
  vector<int> region_agent_num(num_small_region_column * num_small_region_row, 0);
  vector<Direction> region_mainstreams(num_small_region_row * num_small_region_column); // 区域里的主流
  cout << "region num: " << region_mainstreams.size() << endl;

  // 计算region mainstream: 把区域内所有向量累加起来
  for(int i = 0; i < P->getNum(); i++)
  {
    int agent_region_x = A[i]->v_now->pos.x / small_region_column;
    int agent_region_y = A[i]->v_now->pos.y / small_region_row;
    int agent_region = agent_region_y * num_small_region_column + agent_region_x;
    A[i]->agent_region = agent_region;

    region_agent_num[agent_region]++;
    region_mainstreams[agent_region].x += A[i]->agent_direction_x;
    region_mainstreams[agent_region].y += A[i]->agent_direction_y;
  }

  /*
    cout << "region agent num: ";
    for(int i : region_agent_num)
    {
        cout << i << " ";
    }
    cout << endl;
  //*/

  /*
  cout << "region mainstreams: ";
  for(auto i : region_mainstreams)
  {
    cout << i.x << " " << i.y << " ";
  }
  cout << endl;
   */

  for (int i = 0; i < P->getNum(); ++i)
  {
    A[i]->region_mainstream_inner_product =
        A[i]->agent_direction_x * region_mainstreams[A[i]->agent_region].x +
           A[i]->agent_direction_y * region_mainstreams[A[i]->agent_region].y;

    A[i]->region_mainstream_cos = (A[i]->agent_direction_x * region_mainstreams[A[i]->agent_region].x +
                                   A[i]->agent_direction_y * region_mainstreams[A[i]->agent_region].y) /
                           (sqrt(A[i]->agent_direction_x * A[i]->agent_direction_x +
                                 A[i]->agent_direction_y * A[i]->agent_direction_y) *
                            sqrt(region_mainstreams[A[i]->agent_region].x * region_mainstreams[A[i]->agent_region].x +
                                  region_mainstreams[A[i]->agent_region].y * region_mainstreams[A[i]->agent_region].y));
  }

  cout << "A[0] region mainstream inner product: " << A[0]->region_mainstream_inner_product << endl;



  // main loop
  int timestep = 0;
  while (true)
  {

    if(timestep % 50 == 0)
    {
      info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

      // 刷新mainstream: 把所有向量累加起来
      mainstream_x = 0;
      mainstream_y = 0;
      for (int i = 0; i < P->getNum(); ++i)
      {
        A[i]->agent_direction_x = A[i]->g->pos.x - A[i]->v_now->pos.x;
        A[i]->agent_direction_y = A[i]->g->pos.y - A[i]->v_now->pos.y;

        mainstream_x += A[i]->agent_direction_x;
        mainstream_y += A[i]->agent_direction_y;
      }

      // cout << "mainstream direction: " << mainstream_x << " " << mainstream_y << endl;

      // 计算所有agent向量和mainstream向量的内积
      for (int i = 0; i < P->getNum(); ++i)
      {
        A[i]->mainstream_inner_product = A[i]->agent_direction_x * mainstream_x +
                                         A[i]->agent_direction_y * mainstream_y;

        A[i]->mainstream_cos = (A[i]->agent_direction_x * mainstream_x +
                                A[i]->agent_direction_y * mainstream_y) /
                               (sqrt(A[i]->agent_direction_x * A[i]->agent_direction_x +
                                     A[i]->agent_direction_y * A[i]->agent_direction_y) *
                                sqrt(mainstream_x * mainstream_x +
                                     mainstream_y * mainstream_y));
      }

      // 将map分为若干区域, 更新每个区域agent的数量
      std::fill(region_agent_num.begin(), region_agent_num.end(), 0);
      // 区域里的主流
      std::fill(region_mainstreams.begin(), region_mainstreams.end(), Direction{0,0});
      // cout << "region num: " << region_mainstreams.size() << endl;

      // 计算region mainstream: 把区域内所有向量累加起来
      for(int i = 0; i < P->getNum(); i++)
      {
        int agent_region_x = A[i]->v_now->pos.x / small_region_column;
        int agent_region_y = A[i]->v_now->pos.y / small_region_row;
        int agent_region = agent_region_y * num_small_region_column + agent_region_x;
        A[i]->agent_region = agent_region;

        region_agent_num[agent_region]++;
        region_mainstreams[agent_region].x += A[i]->agent_direction_x;
        region_mainstreams[agent_region].y += A[i]->agent_direction_y;
      }

      /*
      cout << "region agent num: ";
      for(int i : region_agent_num)
      {
          cout << i << " ";
      }
      cout << endl;
        //*/

      /*
      cout << "region mainstreams: ";
      for(auto i : region_mainstreams)
      {
        cout << i.x << " " << i.y << " ";
      }
      cout << endl;
       //*/

      for (int i = 0; i < P->getNum(); ++i)
      {
        A[i]->region_mainstream_inner_product =
            A[i]->agent_direction_x * region_mainstreams[A[i]->agent_region].x +
            A[i]->agent_direction_y * region_mainstreams[A[i]->agent_region].y;

        A[i]->region_mainstream_cos = (A[i]->agent_direction_x * region_mainstreams[A[i]->agent_region].x +
                                       A[i]->agent_direction_y * region_mainstreams[A[i]->agent_region].y) /
                                      (sqrt(A[i]->agent_direction_x * A[i]->agent_direction_x +
                                            A[i]->agent_direction_y * A[i]->agent_direction_y) *
                 sqrt(region_mainstreams[A[i]->agent_region].x * region_mainstreams[A[i]->agent_region].x +
                        region_mainstreams[A[i]->agent_region].y * region_mainstreams[A[i]->agent_region].y));
      }

      cout << "A[0] region mainstream inner product: " << A[0]->region_mainstream_inner_product << endl;

    }

    // planning
    // std::sort(A.begin(), A.end(), compare_time); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_mainstream); // 按mainstream优先级排序
    // std::sort(A.begin(), A.end(), compare_anti_mainstream); // 按mainstream一定概率的优先级排序
    // std::sort(A.begin(), A.end(), compare_cos); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_anti_cos); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_addition); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_region_inner); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_anti_region_inner); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_region_cos); // 按优先级排序
    // std::sort(A.begin(), A.end(), compare_anti_region_cos); // 按优先级排序


    if(timestep % 6 < 2)
    {
      std::sort(A.begin(), A.end(), compare_mainstream); // 按mainstream优先级排序
    }
    else
    {
      std::sort(A.begin(), A.end(), compare_anti_mainstream); // 按mainstream优先级排序
    }

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

      // update agent current time
      a->curr_time = timestep;
      // cout << "agent curr time: " << a->curr_time << endl;

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
bool PIBTM::funcPIBT(Agent* ai, Agent* aj)
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
void PIBTM::setParams(int argc, char* argv[])
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
void PIBTM::printHelp()
{
  std::cout << PIBTM::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
