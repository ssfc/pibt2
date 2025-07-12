#include "../include/problem.hpp"

#include <fstream>
#include <regex>
#include <sstream>

#include "../include/util.hpp"

using namespace std;


// Problem 类的构造函数，用于初始化一个多智能体路径规划问题实例。
Problem::Problem(std::string _instance, Graph* _G, std::mt19937* _MT,
                 Config _config_s, Config _config_g, int _num_agents,
                 int _max_timestep, int _max_comp_time)
    : instance(_instance),
      G(_G),
      MT(_MT),
      config_s(_config_s),
      config_g(_config_g),
      num_agents(_num_agents),
      max_timestep(_max_timestep),
      max_comp_time(_max_comp_time){};

// 获取第 i 个智能体的起始节点。
Node* Problem::getStart(int i) const
{
  if (!(0 <= i && i < (int)config_s.size())) halt("invalid index");
  return config_s[i];
}

// 获取第 i 个智能体的目标节点。
Node* Problem::getGoal(int i) const
{
  if (!(0 <= i && i < (int)config_g.size())) halt("invalid index");
  return config_g[i];
}

// 在Problem类中发生严重错误时，输出错误信息并终止程序。
void Problem::halt(const std::string& msg) const
{
  std::cout << "error@Problem: " << msg << std::endl;
  this->~Problem();
  std::exit(1);
}

// 在控制台输出一条警告信息，用于提醒用户有一些非致命的问题或异常情况发生。
void Problem::warn(const std::string& msg) const
{
  std::cout << "warn@Problem: " << msg << std::endl;
}

// 根据一个实例文件（通常是多智能体路径规划的环境与请求配置）来初始化问题实例。
// -------------------------------------------
// MAPF
MAPF_Instance::MAPF_Instance(const std::string& _instance)
    : Problem(_instance), instance_initialized(true)
{
  // read instance file
  // 尝试用传入的 instance 文件名打开配置信息文件。
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::smatch results;
  // 定义用于识别不同配置项的正则表达式
  // 每条正则都用来匹配配置文件中的具体信息，比如地图文件、代理数量、随机种子等。
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_random_problem = std::regex(R"(random_problem=(\d+))");
  std::regex r_well_formed = std::regex(R"(well_formed=(\d+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+),(\d+),(\d+))");

  bool read_scen = true;
  bool well_formed = false;
  // 5. 逐行读取配置文件
  while (getline(file, line))
  {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    // comment
    if (std::regex_match(line, results, r_comment))
    {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map))
    {
      G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents))
    {
      num_agents = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed))
    {
      MT = new std::mt19937(std::stoi(results[1].str()));
      continue;
    }
    // skip reading initial/goal nodes
    if (std::regex_match(line, results, r_random_problem))
    {
      if (std::stoi(results[1].str()))
      {
        read_scen = false;
        config_s.clear();
        config_g.clear();

        struct Agent {
          int start_x, start_y;
          int goal_x, goal_y;
        };

        // cout << "random problem: " << results[1] << endl;
        // 从这里开始，读入scen的内容
        std::ifstream infile("../scene/random-32-32-10-random-1.scen");
        if (!infile) {
          std::cerr << "Cannot open scene!" << std::endl;
        }
        std::string line;
        std::vector<Agent> agents;

        // 跳过第一行 'version 1'
        std::getline(infile, line);

        while (std::getline(infile, line)) {
          std::istringstream iss(line);
          int bucket, map_w, map_h, start_x, start_y, goal_x, goal_y;
          std::string map_name;
          double opt_length;

          // 依次读取列
          if (!(iss >> bucket >> map_name >> map_w >> map_h
                >> start_x >> start_y >> goal_x >> goal_y >> opt_length)) {
            continue; // 跳过无效行
          }
          Agent agent{start_x, start_y, goal_x, goal_y};
          agents.push_back(agent);

          if(agents.size() > 3)
          {
            break;
          }
        }

        cout << "read scene end" << endl;

        // 输出检验
        for (size_t i = 0; i < agents.size(); ++i) {
          std::cout << "Agent " << i << ": Start(" << agents[i].start_x << ", " << agents[i].start_y
                    << "), Goal(" << agents[i].goal_x << ", " << agents[i].goal_y << ")" << std::endl;
        }
      }
      continue;
    }
    //
    if (std::regex_match(line, results, r_well_formed))
    {
      if (std::stoi(results[1].str())) well_formed = true;
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep))
    {
      max_timestep = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time))
    {
      max_comp_time = std::stoi(results[1].str());
      continue;
    }
    // read initial/goal nodes
    // 读取初始/目标点（起点终点）
    if (std::regex_match(line, results, r_sg) && read_scen &&
        (int)config_s.size() < num_agents) // 如果已经从scen里面读够了，这里就不运行了
    {
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      int x_g = std::stoi(results[3].str());
      int y_g = std::stoi(results[4].str());

      if (!G->existNode(x_s, y_s))
      {
        halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
             ") does not exist, invalid scenario");
      }

      if (!G->existNode(x_g, y_g))
      {
        halt("goal node (" + std::to_string(x_g) + ", " + std::to_string(y_g) +
             ") does not exist, invalid scenario");
      }

      Node* s = G->getNode(x_s, y_s);
      Node* g = G->getNode(x_g, y_g);
      config_s.push_back(s);
      config_g.push_back(g);
    }
  }

  // set default value not identified params
  if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
  if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;

  // check starts/goals
  if (num_agents <= 0) halt("invalid number of agents");
  const int config_s_size = config_s.size();
  if (!config_s.empty() && num_agents > config_s_size) {
    warn("given starts/goals are not sufficient\nrandomly create instances");
  }
  if (num_agents > config_s_size) {
    if (well_formed) {
      setWellFormedInstance();
    } else {
      setRandomStartsGoals();
    }
  }

  // trimming
  config_s.resize(num_agents);
  config_g.resize(num_agents);
}

// 通过已有的 MAPF_Instance 实例 P 以及新的配置参数，来创建一个新的 MAPF_Instance 对象。
MAPF_Instance::MAPF_Instance(MAPF_Instance* P, Config _config_s,
                             Config _config_g, int _max_comp_time,
                             int _max_timestep)
    : Problem(P->getInstanceFileName(), P->getG(), P->getMT(), _config_s,
              _config_g, P->getNum(), _max_timestep, _max_comp_time),
      instance_initialized(false)
{
}

// MAPF_Instance 类的拷贝构造器（带部分自定义参数），其主要功能是用一个已存在的 MAPF_Instance 对象 P 来初始化新的 MAPF_Instance 实例，并允许最大计算时间（_max_comp_time）参数自定义。
MAPF_Instance::MAPF_Instance(MAPF_Instance* P, int _max_comp_time)
    : Problem(P->getInstanceFileName(), P->getG(), P->getMT(),
              P->getConfigStart(), P->getConfigGoal(), P->getNum(),
              P->getMaxTimestep(), _max_comp_time),
      instance_initialized(false)
{
}

// 在MAPF_Instance对象被销毁时，安全地释放其内部动态分配的内存资源，防止内存泄漏。
MAPF_Instance::~MAPF_Instance()
{
  if (instance_initialized) {
    if (G != nullptr) delete G;
    if (MT != nullptr) delete MT;
  }
}

// 为多智能体路径规划问题随机生成每个智能体的起点和终点配置。
void MAPF_Instance::setRandomStartsGoals()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  Grid* grid = reinterpret_cast<Grid*>(G);
  const int N = grid->getWidth() * grid->getHeight();

  // set starts
  std::vector<int> starts(N);
  std::iota(starts.begin(), starts.end(), 0);
  std::shuffle(starts.begin(), starts.end(), *MT);
  int i = 0;
  while (true) {
    while (G->getNode(starts[i]) == nullptr) {
      ++i;
      if (i >= N) halt("number of agents is too large.");
    }
    config_s.push_back(G->getNode(starts[i]));
    if ((int)config_s.size() == num_agents) break;
    ++i;
  }

  // set goals
  std::vector<int> goals(N);
  std::iota(goals.begin(), goals.end(), 0);
  std::shuffle(goals.begin(), goals.end(), *MT);
  int j = 0;
  while (true) {
    while (G->getNode(goals[j]) == nullptr) {
      ++j;
      if (j >= N) halt("set goal, number of agents is too large.");
    }
    // retry
    if (G->getNode(goals[j]) == config_s[config_g.size()]) {
      config_g.clear();
      std::shuffle(goals.begin(), goals.end(), *MT);
      j = 0;
      continue;
    }
    config_g.push_back(G->getNode(goals[j]));
    if ((int)config_g.size() == num_agents) break;
    ++j;
  }
}

// 为多智能体路径规划（MAPF）问题自动生成一组起点和终点，且保证这组起点终点是“well-formed”的，即每对起终点之间存在路径，且在分配过程中不会发生节点冲突，适用于路径规划实验或数据集构造。
/*
 * Note: it is hard to generate well-formed instances
 * with dense situations (e.g., ≥300 agents in arena)
 */
void MAPF_Instance::setWellFormedInstance()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  const int N = G->getNodesSize();
  Nodes prohibited, starts_goals;

  while ((int)config_g.size() < getNum()) {
    while (true) {
      // determine start
      Node* s;
      do {
        s = G->getNode(getRandomInt(0, N - 1, MT));
      } while (s == nullptr || inArray(s, prohibited));

      // determine goal
      Node* g;
      do {
        g = G->getNode(getRandomInt(0, N - 1, MT));
      } while (g == nullptr || g == s || inArray(g, prohibited));

      // ensure well formed property
      auto path = G->getPath(s, g, starts_goals);
      if (!path.empty()) {
        config_s.push_back(s);
        config_g.push_back(g);
        starts_goals.push_back(s);
        starts_goals.push_back(g);
        for (auto v : path) {
          if (!inArray(v, prohibited)) prohibited.push_back(v);
        }
        break;
      }
    }
  }
}

// 生成并保存多智能体路径规划（MAPF）问题的场景文件。
void MAPF_Instance::makeScenFile(const std::string& output_file)
{
  Grid* grid = reinterpret_cast<Grid*>(G);
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "agents=" << num_agents << "\n";
  log << "seed=0\n";
  log << "random_problem=0\n";
  log << "max_timestep=" << max_timestep << "\n";
  log << "max_comp_time=" << max_comp_time << "\n";
  for (int i = 0; i < num_agents; ++i) {
    log << config_s[i]->pos.x << "," << config_s[i]->pos.y << ","
        << config_g[i]->pos.x << "," << config_g[i]->pos.y << "\n";
  }
  log.close();
}

// 根据配置文件内容初始化多智能体动态任务分配（MAPD）问题实例，包括环境（地图）、智能体数量、任务参数、起点选择等，自动处理缺省参数、支持随机补全起点、以及特殊任务相关位置的灵活设定，保证所有参数和数据结构初始化正确合法，适用于后续算法流程。
// -------------------------------------------
// MAPD
MAPD_Instance::MAPD_Instance(const std::string& _instance)
    : Problem(_instance), current_timestep(-1), specify_pickup_deliv_locs(true)
{
  // read instance file
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_task_num = std::regex(R"(task_num=(\d+))");
  std::regex r_task_frequency = std::regex(R"(task_frequency=(.+))");
  std::regex r_specify_pikup_deliv_locs =
      std::regex(R"(specify_pikup_deliv_locs=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+))");

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    // comment
    if (std::regex_match(line, results, r_comment)) {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      num_agents = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed)) {
      MT = new std::mt19937(std::stoi(results[1].str()));
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep)) {
      max_timestep = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time)) {
      max_comp_time = std::stoi(results[1].str());
      continue;
    }
    // set the number of tasks
    if (std::regex_match(line, results, r_task_num)) {
      task_num = std::stoi(results[1].str());
      continue;
    }
    // set task frequency
    if (std::regex_match(line, results, r_task_frequency)) {
      task_frequency = std::stof(results[1].str());
      continue;
    }
    // set task frequency
    if (std::regex_match(line, results, r_specify_pikup_deliv_locs)) {
      specify_pickup_deliv_locs = (bool)std::stoi(results[1].str());
      continue;
    }
    // read initial nodes
    if (std::regex_match(line, results, r_sg) &&
        (int)config_s.size() < num_agents) {
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      if (!G->existNode(x_s, y_s)) {
        halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
             ") does not exist, invalid scenario");
      }

      Node* s = G->getNode(x_s, y_s);
      config_s.push_back(s);
    }
  }

  // set default value not identified params
  if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
  if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;
  if (task_frequency == 0) task_frequency = DEFAULT_TASK_FREQUENCY;
  if (task_num == 0) task_num = DEFAULT_TASK_NUM;

  if (specify_pickup_deliv_locs) setupSpetialNodes();
  if (LOCS_PICKUP.empty()) {
    LOCS_PICKUP = G->getV();
    LOCS_DELIVERY = G->getV();
  }

  // check starts
  if (num_agents <= 0) halt("invalid number of agents");
  if (num_agents > (int)config_s.size()) {
    if (!config_s.empty()) {
      warn("given starts are not sufficient\nrandomly create instances");
      config_s.clear();
    }

    // set starts
    std::vector<int> starts(G->getNodesSize());
    std::iota(starts.begin(), starts.end(), 0);
    if (specify_pickup_deliv_locs && !LOCS_NONTASK_ENDPOINTS.empty()) {
      starts.clear();
      for (auto v : LOCS_NONTASK_ENDPOINTS) starts.push_back(v->id);
    }
    std::shuffle(starts.begin(), starts.end(), *MT);

    int i = 0;
    while (true) {
      while (G->getNode(starts[i]) == nullptr) {
        ++i;
        if (i >= (int)starts.size()) halt("number of agents is too large.");
      }

      config_s.push_back(G->getNode(starts[i]));
      if ((int)config_s.size() == num_agents) break;
      ++i;
    }
  }

  // trimming
  config_s.resize(num_agents);

  // initialize
  update();
}

// 负责释放动态分配到堆上的任务对象内存，防止内存泄漏
MAPD_Instance::~MAPD_Instance()
{
  for (auto task : TASKS_OPEN) delete task;
  for (auto task : TASKS_CLOSED) delete task;
}

// 从地图的扩展配置文件（一般为 .pd 文件）中读取节点类型并初始化任务点配置
void MAPD_Instance::setupSpetialNodes()
{
  Grid* grid = reinterpret_cast<Grid*>(G);

  // read instance file
#ifdef _MAPDIR_
  std::ifstream file(_MAPDIR_ + grid->getMapFileName() + ".pd");
#else
  std::ifstream file(grid->getMapFileName() + ".pd");
#endif
  if (!file) return;

  std::string line, s;
  std::smatch results;
  std::regex r_obj = std::regex(R"([@T])");
  std::regex r_pickup = std::regex(R"([psa])");  // pickup loc.
  std::regex r_deliv = std::regex(R"([dsa])");   // delivery loc.
  std::regex r_end = std::regex(R"([ea])");      // end loc.

  const int width = grid->getWidth();

  int y = 0;
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if ((int)line.size() != width) halt("pd format is invalid");

    for (int x = 0; x < width; ++x) {
      if (!G->existNode(x, y)) continue;

      auto v = G->getNode(x, y);
      s = line[x];
      bool flg_endpoints = false;
      if (std::regex_match(s, results, r_pickup)) {
        LOCS_PICKUP.push_back(v);
        flg_endpoints = true;
      }
      if (std::regex_match(s, results, r_deliv)) {
        LOCS_DELIVERY.push_back(v);
        flg_endpoints = true;
      }
      if (std::regex_match(s, results, r_end)) {
        LOCS_NONTASK_ENDPOINTS.push_back(v);
        flg_endpoints = true;
      }
      if (flg_endpoints) {
        LOCS_ENDPOINTS.push_back(v);
      }
    }
    ++y;
  }
}

void MAPD_Instance::update()
{
  // check finished tasks
  auto itr = TASKS_OPEN.begin();
  while (itr != TASKS_OPEN.end()) {
    auto task = *itr;

    // not at delivery location
    if (task->loc_current != task->loc_delivery) {
      ++itr;
      continue;
    }

    task->timestep_finished = current_timestep + 1;
    TASKS_CLOSED.push_back(task);

    // remove from OPEN list
    itr = TASKS_OPEN.erase(itr);
  }

  // create new tasks
  int created_task_num = int(TASKS_OPEN.size() + TASKS_CLOSED.size());
  if (created_task_num < task_num) {
    int new_task_num = (int)task_frequency;
    if (task_frequency < 1 && getRandomFloat(0, 1, MT) < task_frequency) {
      new_task_num = 1;
    }
    new_task_num = std::min(new_task_num, task_num - created_task_num);
    for (int i = 0; i < new_task_num; ++i) {
      Node *p, *d;
      do {
        p = randomChoose(LOCS_PICKUP, MT);
        d = randomChoose(LOCS_DELIVERY, MT);
      } while (p == d);
      TASKS_OPEN.push_back(new Task(p, d, current_timestep + 1));
    }
  }

  // update timestep
  ++current_timestep;
};
