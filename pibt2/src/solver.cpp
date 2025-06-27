#include "../include/solver.hpp"

#include <fstream>
#include <iomanip> // 持流输入输出（IO）时的数据格式控制，也就是“输入输出格式化”的意思。


// 传入的 Problem 对象 _P 初始化 MinimumSolver 类的各项成员变量。
MinimumSolver::MinimumSolver(Problem* _P)
    : solver_name(""),
      G(_P->getG()),
      MT(_P->getMT()),
      max_timestep(_P->getMaxTimestep()),
      max_comp_time(_P->getMaxCompTime()),
      solved(false),
      comp_time(0),
      verbose(false),
      log_short(false)
{
}

// 组织和管理整个求解过程的三个关键步骤，保证算法执行的完整性和流程性。
void MinimumSolver::solve()
{
  start();
  exec();
  end();
}

// 该函数记录算法或求解器开始运行的时间点。
void MinimumSolver::start() { t_start = Time::now(); }

// 在求解器（solver）运行结束时，记录并保存其总共消耗的计算时间（以毫秒为单位）。
void MinimumSolver::end() { comp_time = getSolverElapsedTime(); }

// 获取并返回自求解器（solver）开始运行以来所经过的时间（单位：毫秒）。
// -------------------------------
// utilities for time
// -------------------------------
int MinimumSolver::getSolverElapsedTime() const
{
  return getElapsedTime(t_start);
}

// 计算并返回剩余可用的计算时间。
int MinimumSolver::getRemainedTime() const
{
  return std::max(0, max_comp_time - getSolverElapsedTime());
}

// 检测求解器的运行时间是否已达到或超过允许的最大计算时间。
bool MinimumSolver::overCompTime() const
{
  return getSolverElapsedTime() >= max_comp_time;
}

// 检查 verbose 标志（通常用于控制调试或日志输出的详细等级）。
// -------------------------------
// utilities for debug
// -------------------------------
void MinimumSolver::info() const
{
  if (verbose) std::cout << std::endl;
}

// 报告错误并中止程序执行。
void MinimumSolver::halt(const std::string& msg) const
{
  std::cout << "error@" << solver_name << ": " << msg << std::endl;
  this->~MinimumSolver();
  std::exit(1);
}

// 向控制台输出警告信息。
void MinimumSolver::warn(const std::string& msg) const
{
  std::cout << "warn@ " << solver_name << ": " << msg << std::endl;
}

// 初始化一个多智能体路径规划（MAPF）求解器对象，并为后续的求解操作做好准备。
// -----------------------------------------------
// base class with utilities
// -----------------------------------------------

MAPF_Solver::MAPF_Solver(MAPF_Instance* _P)
    : MinimumSolver(_P),
      P(_P),
      LB_soc(0),
      LB_makespan(0),
      distance_table(_P->getNum(),
                     std::vector<int>(G->getNodesSize(), max_timestep)),
      distance_table_p(nullptr)
{
}

// 析构函数
MAPF_Solver::~MAPF_Solver() {}

// 执行多智能体路径规划（MAPF）求解器的初始化和主流程。
// -------------------------------
// main
// -------------------------------
void MAPF_Solver::exec()
{
  // create distance table
  if (distance_table_p == nullptr)
  {
    info("  pre-processing, create distance table by BFS");
    createDistanceTable();
    preprocessing_comp_time = getSolverElapsedTime();
    info("  done, elapsed: ", preprocessing_comp_time);
  }

  run();
}

// 计算多智能体路径规划（MAPF）问题的两个常用下界
// -------------------------------
// utilities for problem instance
// -------------------------------
void MAPF_Solver::computeLowerBounds()
{
  LB_soc = 0;
  LB_makespan = 0;

  for (int i = 0; i < P->getNum(); ++i) {
    int d = pathDist(i);
    LB_soc += d;
    if (d > LB_makespan) LB_makespan = d;
  }
}

// 获取所有智能体最短路径之和（SOC，Sum Of Costs）的下界。
int MAPF_Solver::getLowerBoundSOC()
{
  if (LB_soc == 0) computeLowerBounds();
  return LB_soc;
}

// 获取所有智能体“最短最大路径长度”（Makespan）的下界。
int MAPF_Solver::getLowerBoundMakespan()
{
  if (LB_makespan == 0) computeLowerBounds();
  return LB_makespan;
}

// 将计划（Plan）格式的数据转换为路径（Paths）格式，便于后续处理和分析。
// -------------------------------
// utilities for solution representation
Paths MAPF_Solver::planToPaths(const Plan& plan)
{
  int num_agents = plan.get(0).size();
  Paths paths(num_agents);
  int makespan = plan.getMakespan();
  for (int i = 0; i < num_agents; ++i) {
    Path path;
    for (int t = 0; t <= makespan; ++t) {
      path.push_back(plan.get(t, i));
    }
    paths.insert(i, path);
  }
  return paths;
}

// 将每个智能体的独立行走路径（Paths）转换成一个整体的时间步进计划（Plan），即还原为以时间为主序的多智能体位置排布。
Plan MAPF_Solver::pathsToPlan(const Paths& paths)
{
  Plan plan;
  if (paths.empty()) return plan;
  int makespan = paths.getMakespan();
  int num_agents = paths.size();
  for (int t = 0; t <= makespan; ++t) {
    Config c;
    for (int i = 0; i < num_agents; ++i) {
      c.push_back(paths.get(i, t));
    }
    plan.add(c);
  }
  return plan;
}

// 将一个字符串参数列表（通常用于命令行参数）传递并设置到指定的 MAPF 求解器（solver）实例中，便于通过参数灵活配置求解器的行为。
// -------------------------------
// utilities for solver options
// -------------------------------
void MAPF_Solver::setSolverOption(std::shared_ptr<MAPF_Solver> solver,
                                  const std::vector<std::string>& option)
{
  if (option.empty()) return;
  const int argc = option.size() + 1;
  char* argv[argc];
  for (int i = 1; i < argc; ++i) {
    char* tmp = const_cast<char*>(option[i - 1].c_str());
    argv[i] = tmp;
  }
  solver->setParams(argc, argv);
}

// 打印结果
// -------------------------------
// print
// -------------------------------
void MAPF_Solver::printResult()
{
  std::cout << "solved=" << solved << ", solver=" << std::right << std::setw(8)
            << solver_name << ", comp_time(ms)=" << std::right << std::setw(8)
            << getCompTime() << ", soc=" << std::right << std::setw(6)
            << solution.getSOC() << " (LB=" << std::right << std::setw(6)
            << getLowerBoundSOC() << ")"
            << ", makespan=" << std::right << std::setw(4)
            << solution.getMakespan() << " (LB=" << std::right << std::setw(6)
            << getLowerBoundMakespan() << ")" << std::endl;
}

void MinimumSolver::printHelpWithoutOption(const std::string& solver_name)
{
  std::cout << solver_name << "\n"
            << "  (no option)" << std::endl;
}

// -------------------------------
// log
// -------------------------------
void MAPF_Solver::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);
  makeLogSolution(log);
  log.close();
}

void MAPF_Solver::makeLogBasicInfo(std::ofstream& log)
{
  Grid* grid = reinterpret_cast<Grid*>(P->getG());
  log << "instance=" << P->getInstanceFileName() << "\n";
  log << "agents=" << P->getNum() << "\n";
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "solver=" << solver_name << "\n";
  log << "solved=" << solved << "\n";
  log << "soc=" << solution.getSOC() << "\n";
  log << "lb_soc=" << getLowerBoundSOC() << "\n";
  log << "makespan=" << solution.getMakespan() << "\n";
  log << "lb_makespan=" << getLowerBoundMakespan() << "\n";
  log << "comp_time=" << getCompTime() << "\n";
  log << "preprocessing_comp_time=" << preprocessing_comp_time << "\n";
}

void MAPF_Solver::makeLogSolution(std::ofstream& log)
{
  if (log_short) return;
  log << "starts=";
  for (int i = 0; i < P->getNum(); ++i) {
    Node* v = P->getStart(i);
    log << "(" << v->pos.x << "," << v->pos.y << "),";
  }
  log << "\ngoals=";
  for (int i = 0; i < P->getNum(); ++i) {
    Node* v = P->getGoal(i);
    log << "(" << v->pos.x << "," << v->pos.y << "),";
  }
  log << "\n";
  log << "solution=\n";
  for (int t = 0; t <= solution.getMakespan(); ++t) {
    log << t << ":";
    auto c = solution.get(t);
    for (auto v : c) {
      log << "(" << v->pos.x << "," << v->pos.y << "),";
    }
    log << "\n";
  }
}

// 返回第 i 个智能体(Agent)从当前位置到节点 s 的距离，具体来说，是通过查表获取两点间的预计算距离。
// -------------------------------
// distance
// -------------------------------
int MAPF_Solver::pathDist(const int i, Node* const s) const
{
  if (distance_table_p != nullptr) {
    return distance_table_p->operator[](i)[s->id];
  }
  return distance_table[i][s->id];
}

// 获取编号为 i 的智能体（agent）从其起点位置出发的距离信息。
int MAPF_Solver::pathDist(const int i) const
{
  return pathDist(i, P->getStart(i));
}

// 为每一个智能体（agent）构建一张距离表，用于记录该智能体的目标节点（goal）到图中所有其他节点的最短距离。
// 方法是以目标节点为起点，使用广度优先搜索（BFS）遍历图，从而填充距离表。
void MAPF_Solver::createDistanceTable()
{
  for (int i = 0; i < P->getNum(); ++i) {
    // breadth first search
    std::queue<Node*> OPEN;
    Node* n = P->getGoal(i);
    OPEN.push(n);
    distance_table[i][n->id] = 0;
    while (!OPEN.empty()) {
      n = OPEN.front();
      OPEN.pop();
      const int d_n = distance_table[i][n->id];
      for (auto m : n->neighbor) {
        const int d_m = distance_table[i][m->id];
        if (d_n + 1 >= d_m) continue;
        distance_table[i][m->id] = d_n + 1;
        OPEN.push(m);
      }
    }
  }
}

// 构造函数，实现了对 A* 搜索节点各成员变量的初始化。
// -------------------------------
// utilities for getting path
// -------------------------------
MinimumSolver::AstarNode::AstarNode(Node* _v, int _g, int _f, AstarNode* _p)
    : v(_v), g(_g), f(_f), p(_p), name(getName(_v, _g))
{
}

// 根据传入的节点指针 _v 和一个代价值 _g，生成一个字符串用于标识该节点。
std::string MinimumSolver::AstarNode::getName(Node* _v, int _g)
{
  return std::to_string(_v->id) + "-" + std::to_string(_g);
}

// 空间-时间A*路径规划的主算法，其核心目标是在存在具有时序约束（比如动态障碍物或时间窗口）的图中，从起点s搜索到终点g的一条可行路径。
Path MinimumSolver::getPathBySpaceTimeAstar(
    Node* const s, Node* const g, AstarHeuristics& fValue,
    CompareAstarNode& compare, CheckAstarFin& checkAstarFin,
    CheckInvalidAstarNode& checkInvalidAstarNode, const int time_limit)
{
  auto t_start = Time::now();

  AstarNodes GC;  // garbage collection
  auto createNewNode = [&GC](Node* v, int g, int f, AstarNode* p) {
    AstarNode* new_node = new AstarNode(v, g, f, p);
    GC.push_back(new_node);
    return new_node;
  };

  // OPEN and CLOSE list
  std::priority_queue<AstarNode*, AstarNodes, CompareAstarNode> OPEN(compare);
  std::unordered_map<std::string, bool> CLOSE;

  // initial node
  AstarNode* n = createNewNode(s, 0, 0, nullptr);
  n->f = fValue(n);
  OPEN.push(n);

  // main loop
  bool invalid = true;
  while (!OPEN.empty()) {
    // check time limit
    if (time_limit > 0 && getElapsedTime(t_start) > time_limit) break;

    // minimum node
    n = OPEN.top();
    OPEN.pop();

    // check CLOSE list
    if (CLOSE.find(n->name) != CLOSE.end()) continue;
    CLOSE[n->name] = true;

    // check goal condition
    if (checkAstarFin(n)) {
      invalid = false;
      break;
    }

    // expand
    Nodes C = n->v->neighbor;
    C.push_back(n->v);
    for (auto u : C) {
      int g_cost = n->g + 1;
      AstarNode* m = createNewNode(u, g_cost, 0, n);
      m->f = fValue(m);
      // already searched?
      if (CLOSE.find(m->name) != CLOSE.end()) continue;
      // check constraints
      if (checkInvalidAstarNode(m)) continue;
      OPEN.push(m);
    }
  }

  Path path;
  if (!invalid) {  // success
    while (n != nullptr) {
      path.push_back(n->v);
      n = n->p;
    }
    std::reverse(path.begin(), path.end());
  }

  // free
  for (auto p : GC) delete p;

  return path;
}

// 用于A*算法节点优先级比较的lambda表达式
MinimumSolver::CompareAstarNode MinimumSolver::compareAstarNodeBasic =
    [](AstarNode* a, AstarNode* b) {
      if (a->f != b->f) return a->f > b->f;
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

// 优先级规则下单一智能体寻路的主要逻辑
Path MAPF_Solver::getPrioritizedPath(
    const int id, const Paths& paths, const int time_limit,
    const int upper_bound,
    const std::vector<std::tuple<Node*, int>>& constraints,
    CompareAstarNode& compare, const bool manage_path_table)
{
  Node* const s = P->getStart(id);
  Node* const g = P->getGoal(id);
  const int ideal_dist = pathDist(id);
  const int makespan = paths.getMakespan();

  // max timestep that another agent uses the goal
  int max_constraint_time = 0;
  for (int t = makespan; t >= ideal_dist; --t) {
    for (int i = 0; i < P->getNum(); ++i) {
      if (i != id && !paths.empty(i) && paths.get(i, t) == g) {
        max_constraint_time = t;
        break;
      }
    }
    if (max_constraint_time > 0) break;
  }

  // setup functions

  /*
   * Note: greedy f-value is indeed a good choice but sacrifice completeness.
   * > return pathDist(id, n->v)
   * Since prioritized planning itself returns sub-optimal solutions,
   * the underlying pathfinding is not limited to optimal sub-solution.
   * c.f., classical f-value: n->g + pathDist(id, n->v)
   */
  AstarHeuristics fValue;
  if (ideal_dist > max_constraint_time) {
    fValue = [&](AstarNode* n) { return n->g + pathDist(id, n->v); };
  } else {
    // when someone occupies its goal
    fValue = [&](AstarNode* n) {
      return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
    };
  }

  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return n->v == g && n->g > max_constraint_time;
  };

  // update PATH_TABLE
  if (manage_path_table) updatePathTable(paths, id);

  // fast collision checking
  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    if (upper_bound != -1 && m->g > upper_bound) return true;

    if (makespan > 0) {
      if (m->g > makespan) {
        if (PATH_TABLE[makespan][m->v->id] != NIL) return true;
      } else {
        // vertex conflict
        if (PATH_TABLE[m->g][m->v->id] != NIL) return true;
        // swap conflict
        if (PATH_TABLE[m->g][m->p->v->id] != NIL &&
            PATH_TABLE[m->g - 1][m->v->id] == PATH_TABLE[m->g][m->p->v->id])
          return true;
      }
    }

    // check additional constraints
    for (auto c : constraints) {
      const int t = std::get<1>(c);
      if (m->v == std::get<0>(c) && (t == -1 || t == m->g)) return true;
    }
    return false;
  };

  auto p = getPathBySpaceTimeAstar(s, g, fValue, compare, checkAstarFin,
                                   checkInvalidAstarNode, time_limit);

  // clear used path table
  if (manage_path_table) clearPathTable(paths);

  return p;
}

// 根据所有智能体（除了当前规划的智能体）已有的路径，更新全局的时空占据表
void MAPF_Solver::updatePathTable(const Paths& paths, const int id)
{
  const int makespan = paths.getMakespan();
  const int num_agents = paths.size();
  const int nodes_size = G->getNodesSize();
  // extend PATH_TABLE
  while ((int)PATH_TABLE.size() < makespan + 1)
    PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
  // update locations
  for (int i = 0; i < num_agents; ++i) {
    if (i == id || paths.empty(i)) continue;
    auto p = paths.get(i);
    for (int t = 0; t <= makespan; ++t) PATH_TABLE[t][p[t]->id] = i;
  }
}

// 除（重置）全局路径表 PATH_TABLE，让其中记录的已占用信息全部恢复为空（即未被任何智能体占用的状态）。
void MAPF_Solver::clearPathTable(const Paths& paths)
{
  const int makespan = paths.getMakespan();
  const int num_agents = paths.size();
  for (int i = 0; i < num_agents; ++i) {
    if (paths.empty(i)) continue;
    auto p = paths.get(i);
    for (int t = 0; t <= makespan; ++t) PATH_TABLE[t][p[t]->id] = NIL;
  }
}

// 在不清空已有数据的情况下，将某个智能体的新路径增量地更新到全局时空占用表（PATH_TABLE）中。
void MAPF_Solver::updatePathTableWithoutClear(const int id, const Path& p,
                                              const Paths& paths)
{
  if (p.empty()) return;

  const int makespan = PATH_TABLE.size() - 1;
  const int nodes_size = G->getNodesSize();
  const int p_makespan = p.size() - 1;

  // extend PATH_TABLE
  if (p_makespan > makespan) {
    while ((int)PATH_TABLE.size() < p_makespan + 1)
      PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
    for (int i = 0; i < P->getNum(); ++i) {
      if (paths.empty(i)) continue;
      auto v_id = paths.get(i, makespan)->id;
      for (int t = makespan + 1; t <= p_makespan; ++t) PATH_TABLE[t][v_id] = i;
    }
  }

  // register new path
  for (int t = 0; t <= p_makespan; ++t) PATH_TABLE[t][p[t]->id] = id;
  if (makespan > p_makespan) {
    auto v_id = p[p_makespan]->id;
    for (int t = p_makespan + 1; t <= makespan; ++t) PATH_TABLE[t][v_id] = id;
  }
}

// 初始化 MAPD_Solver 类的各项成员变量，为后续多智能体路径调度（MAPD）任务的求解做准备。
//-----------------------------------------------------
// MAPD Solver
MAPD_Solver::MAPD_Solver(MAPD_Instance* _P, bool _use_distance_table)
    : MinimumSolver(_P),
      P(_P),
      use_distance_table(_use_distance_table),
      preprocessing_comp_time(0),
      distance_table(G->getNodesSize(),
                     std::vector<int>(G->getNodesSize(), G->getNodesSize()))
{
}

MAPD_Solver::~MAPD_Solver() {}

// 成前期必要的数据准备（如距离表的生成）以及整体求解流程的调度。
void MAPD_Solver::solve()
{
  // create distance table
  if (use_distance_table) {
    auto t_s = Time::now();
    info("  pre-processing, create distance table by Floyd-Warshall");
    createDistanceTable();
    preprocessing_comp_time = getElapsedTime(t_s);
    info("  done, elapsed: ", preprocessing_comp_time);
  }

  start();
  exec();
  end();
}

// 调用同一个类的 run() 方法。
void MAPD_Solver::exec() { run(); }

// 计算两个节点之间的最短路径距离，并根据是否使用预处理好的距离表选择不同的计算方式。
int MAPD_Solver::pathDist(Node* const s, Node* const g) const
{
  if (use_distance_table) return distance_table[s->id][g->id];
  return G->pathDist(s, g);
}

void MAPD_Solver::createDistanceTable()
{
  const int nodes_num = G->getNodesSize();
  for (int i = 0; i < nodes_num; ++i) {
    auto u = G->getNode(i);
    if (u == nullptr) continue;
    for (auto v : u->neighbor) {
      distance_table[i][v->id] = 1;
    }
    distance_table[i][i] = 0;
  }

  // main loop
  for (int k = 0; k < nodes_num; ++k) {
    for (int i = 0; i < nodes_num; ++i) {
      for (int j = 0; j < nodes_num; ++j) {
        distance_table[i][j] = std::min(
            distance_table[i][j], distance_table[i][k] + distance_table[k][j]);
      }
    }
  }
}

float MAPD_Solver::getTotalServiceTime()
{
  if (!solved) return false;
  auto tasks = P->getClosedTasks();
  return std::accumulate(tasks.begin(), tasks.end(), 0, [](float acc, Task* a) {
    return acc + a->timestep_finished - a->timestep_appear;
  });
}

float MAPD_Solver::getAverageServiceTime()
{
  return getTotalServiceTime() / P->getClosedTasks().size();
}

void MAPD_Solver::printResult()
{
  std::cout << "solved=" << solved << ", solver=" << std::right << std::setw(8)
            << solver_name << ", comp_time(ms)=" << std::right << std::setw(8)
            << getCompTime() << " (pre=" << std::right << std::setw(8)
            << preprocessing_comp_time << ")"
            << ", makespan=" << std::right << std::setw(4)
            << solution.getMakespan() << ", service time (ave)=" << std::right
            << std::setw(6) << getAverageServiceTime() << std::endl;
}

void MAPD_Solver::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);
  makeLogSolution(log);
  log.close();
}

void MAPD_Solver::makeLogBasicInfo(std::ofstream& log)
{
  Grid* grid = reinterpret_cast<Grid*>(P->getG());
  log << "instance=" << P->getInstanceFileName() << "\n";
  log << "agents=" << P->getNum() << "\n";
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "solver=" << solver_name << "\n";
  log << "solved=" << solved << "\n";
  log << "service_time=" << getAverageServiceTime() << "\n";
  log << "makespan=" << solution.getMakespan() << "\n";
  log << "comp_time=" << getCompTime() << "\n";
  log << "preprocessing_comp_time=" << preprocessing_comp_time << "\n";
}

void MAPD_Solver::makeLogSolution(std::ofstream& log)
{
  if (log_short) return;

  log << "starts=";
  for (int i = 0; i < P->getNum(); ++i) {
    Node* v = P->getStart(i);
    log << "(" << v->pos.x << "," << v->pos.y << "),";
  }
  log << "\n";
  log << "task=\n";
  for (auto task : P->getClosedTasks()) {
    log << task->id << ":" << task->loc_pickup->id << "->"
        << task->loc_delivery->id << ","
        << "appear=" << task->timestep_appear << ","
        << "finished=" << task->timestep_finished << "\n";
  }
  log << "solution=\n";
  for (int t = 0; t <= solution.getMakespan(); ++t) {
    log << t << ":";
    auto c = solution.get(t);
    for (int i = 0; i < (int)P->getNum(); ++i) {
      auto v = c[i];
      auto u = hist_targets[t][i];
      auto task = hist_tasks[t][i];
      log << "(" << v->pos.x << "," << v->pos.y << ")->"
          << "(" << u->pos.x << "," << u->pos.y
          << "):" << ((task == nullptr) ? Task::NIL : task->id) << ",";
    }
    log << "\n";
  }
}
