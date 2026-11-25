// compute_bench: Nav2 controller의 한 주기 계산 시간(computeVelocityCommands)을 잰다.
// 속도 명령은 어디에도 보내지 않는다: 돌려받은 Twist는 버리고, 끝에 그래프의 모든 publisher를 적어 cmd_vel이 있으면 실패한다.
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "pode_rl_controller/onnx_policy.hpp"
#include "pode_rl_controller/policy_spec.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using Clock = std::chrono::steady_clock;
using Params = std::map<std::string, rclcpp::ParameterValue>;

// ---- 고정 장면: v7 경기장 가운데 통로(진열대 사이 y 0.40~0.75 m) ----
constexpr double kRobotX = 0.40;
constexpr double kRobotY = 0.58;
constexpr double kRobotV = 0.10;
constexpr int kScanRays = 640;
const char * kMapFrame = "map";
const char * kBaseFrame = "base_footprint";
const char * kLaserFrame = "bench_laser";

struct Args
{
  std::string nav2_params;
  std::string cases;
  std::string map;
  std::string out = ".";
  std::string only;
  int n = 600;
  int warmup = 100;
};

Args parse_args(const std::vector<std::string> & argv)
{
  Args a;
  for (size_t i = 1; i < argv.size(); ++i) {
    const std::string & k = argv[i];
    if (i + 1 >= argv.size()) {
      throw std::invalid_argument("missing value for " + k);
    }
    const std::string & v = argv[++i];
    if (k == "--nav2-params") {a.nav2_params = v;} else if (k == "--cases") {a.cases = v;} else if (
      k == "--map") {a.map = v;} else if (k == "--out") {a.out = v;} else if (k == "--only") {
      a.only = v;
    } else if (k == "--n") {a.n = std::stoi(v);} else if (k == "--warmup") {
      a.warmup = std::stoi(v);
    } else {
      throw std::invalid_argument("unknown argument " + k);
    }
  }
  if (a.nav2_params.empty() || a.map.empty()) {
    throw std::invalid_argument(
            "usage: compute_bench --nav2-params FILE --map MAP_YAML [--cases FILE] [--out DIR] "
            "[--n 600] [--warmup 100] [--only CASE]");
  }
  if (a.cases.empty()) {
    a.cases = ament_index_cpp::get_package_share_directory("pode_compute_bench") +
      "/config/cases.yaml";
  }
  return a;
}

// ---- ROS 파라미터 파일에서 노드 하나의 값 ----
Params node_params(const std::string & file, const std::string & node)
{
  const auto all = rclcpp::parameter_map_from_yaml_file(file);
  for (const auto & [name, params] : all) {
    if (name == "/" + node || name == node) {
      Params out;
      for (const auto & p : params) {
        out[p.get_name()] = p.get_parameter_value();
      }
      return out;
    }
  }
  throw std::runtime_error(file + ": no parameters for node " + node);
}

Params with_prefix(const Params & from, const std::string & prefix, const std::string & to = "")
{
  Params out;
  for (const auto & [k, v] : from) {
    if (k.rfind(prefix + ".", 0) == 0) {
      out[to + k.substr(prefix.size() + 1)] = v;
    }
  }
  return out;
}

void put_param(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  const rclcpp::ParameterValue & value)
{
  if (node->has_parameter(name)) {
    node->set_parameter(rclcpp::Parameter(name, value));
  } else {
    node->declare_parameter(name, value);
  }
}

double cpu_seconds(const rusage & u)
{
  return u.ru_utime.tv_sec + u.ru_utime.tv_usec * 1e-6 + u.ru_stime.tv_sec +
         u.ru_stime.tv_usec * 1e-6;
}

geometry_msgs::msg::Quaternion yaw_quat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation = yaw_quat(yaw);
  return t;
}

// ---- 고정 경로: 통로를 따라 x 1.75 m까지 간 뒤 아래(남쪽)로 꺾어 y 0.15 m까지, 2.5 cm 간격 ----
nav_msgs::msg::Path make_plan(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = kMapFrame;
  path.header.stamp = stamp;
  auto add = [&](double x, double y, double yaw) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation = yaw_quat(yaw);
      path.poses.push_back(p);
    };
  const double step = 0.025;
  for (double x = 0.30; x < 1.75; x += step) {
    add(x, kRobotY, 0.0);
  }
  for (double y = kRobotY; y >= 0.15 - 1e-9; y -= step) {
    add(1.75, y, -M_PI / 2.0);
  }
  return path;
}

// ---- 합성 라이다: 640 광선, 0.25~0.65 m 사이로 천천히 바뀌는 거리 ----
sensor_msgs::msg::LaserScan make_scan(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = kLaserFrame;
  s.header.stamp = stamp;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / kScanRays;
  s.angle_max = s.angle_min + s.angle_increment * (kScanRays - 1);
  s.range_min = 0.05;
  s.range_max = 8.0;
  s.ranges.resize(kScanRays);
  for (int i = 0; i < kScanRays; ++i) {
    s.ranges[i] = static_cast<float>(0.45 + 0.2 * std::sin(i * 2.0 * M_PI / 160.0));
  }
  return s;
}

struct Result
{
  std::string name;
  std::string kind;
  std::string plugin;
  double rate_hz = 0.0;
  std::string batch_size;
  std::string time_steps;
  std::string model_dt;
  std::vector<double> ms;
  int errors = 0;
  std::string first_error;
  int overruns = 0;
  double wall_s = 0.0;
  double proc_user_s = 0.0;
  double proc_sys_s = 0.0;
  double thread_cpu_s = 0.0;
};

std::string param_text(const Params & p, const std::string & key)
{
  const auto it = p.find(key);
  return it == p.end() ? "" : rclcpp::to_string(it->second);
}

// ---- warmup + n번 부른다. 주기마다 그 경우의 rate로 쉬고, 측정 구간만 CPU 시간을 잰다 ----
template<typename Step>
void run_loop(Result & r, const Args & args, Step step, const std::function<void()> & after_warmup)
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / r.rate_hz));
  rusage p0{}, t0{};
  Clock::time_point wall0;
  auto next = Clock::now();
  for (int i = 0; i < args.warmup + args.n; ++i) {
    if (i == args.warmup) {
      after_warmup();
      next = Clock::now();
      getrusage(RUSAGE_SELF, &p0);
      getrusage(RUSAGE_THREAD, &t0);
      wall0 = Clock::now();
    }
    const auto a = Clock::now();
    try {
      step();
    } catch (const std::exception & e) {
      if (i >= args.warmup && r.errors++ == 0) {
        r.first_error = e.what();
      }
    }
    const auto b = Clock::now();
    if (i >= args.warmup) {
      r.ms.push_back(std::chrono::duration<double, std::milli>(b - a).count());
      r.overruns += (b - a) > period ? 1 : 0;
    }
    next += period;
    if (next < b) {
      next = b;
    }
    std::this_thread::sleep_until(next);
  }
  rusage p1{}, t1{};
  getrusage(RUSAGE_SELF, &p1);
  getrusage(RUSAGE_THREAD, &t1);
  r.wall_s = std::chrono::duration<double>(Clock::now() - wall0).count();
  r.proc_user_s = p1.ru_utime.tv_sec - p0.ru_utime.tv_sec +
    (p1.ru_utime.tv_usec - p0.ru_utime.tv_usec) * 1e-6;
  r.proc_sys_s = p1.ru_stime.tv_sec - p0.ru_stime.tv_sec +
    (p1.ru_stime.tv_usec - p0.ru_stime.tv_usec) * 1e-6;
  r.thread_cpu_s = cpu_seconds(t1) - cpu_seconds(t0);
}

// ---- 그래프의 모든 publisher(이 도메인에서 보이는 것 전부) ----
std::set<std::string> graph_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  std::set<std::string> out;
  for (const auto & [topic, types] : node->get_topic_names_and_types()) {
    for (const auto & info : node->get_publishers_info_by_topic(topic)) {
      std::string ns = info.node_namespace();
      out.insert(topic + " <- " + ns + (ns == "/" ? "" : "/") + info.node_name());
    }
  }
  return out;
}

int run(const Args & args)
{
  auto logger = rclcpp::get_logger("compute_bench");

  const Params nav2 = node_params(args.nav2_params, "controller_server");
  const Params local = node_params(args.nav2_params, "local_costmap/local_costmap");
  const Params bench = node_params(args.cases, "compute_bench");

  // ---- 노드: controller_server와 같은 값(controller_frequency, goal checker 등)을 먼저 선언 ----
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("compute_bench");
  for (const auto & [k, v] : nav2) {
    put_param(node, k, v);
  }

  // ---- costmap: 실물 local_costmap 값(3 m 굴림 창, 2.5 cm, 발자국, inflation)에 정적 지도 층 ----
  Params cmp = local;
  cmp["global_frame"] = rclcpp::ParameterValue(kMapFrame);
  cmp["robot_base_frame"] = rclcpp::ParameterValue(kBaseFrame);
  cmp["plugins"] = rclcpp::ParameterValue(std::vector<std::string>{"static_layer", "inflation_layer"});
  cmp["map_topic"] = rclcpp::ParameterValue("/bench_map");
  cmp["static_layer.plugin"] = rclcpp::ParameterValue("nav2_costmap_2d::StaticLayer");
  cmp["static_layer.map_topic"] = rclcpp::ParameterValue("/bench_map");
  cmp["static_layer.map_subscribe_transient_local"] = rclcpp::ParameterValue(true);
  std::vector<rclcpp::Parameter> cm;
  for (const auto & [k, v] : cmp) {
    cm.emplace_back(k, v);
  }
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    rclcpp::NodeOptions().parameter_overrides(cm));

  // ---- TF: 플러그인에는 고정 변환만 든 버퍼, costmap에는 /tf_static ----
  const auto robot_tf = make_tf(kMapFrame, kBaseFrame, kRobotX, kRobotY, 0.0);
  const auto laser_tf = make_tf(kBaseFrame, kLaserFrame, 0.0, 0.0, M_PI);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface()));
  tf->setUsingDedicatedThread(true);
  tf->setTransform(robot_tf, "compute_bench", true);
  tf->setTransform(laser_tf, "compute_bench", true);
  tf2_ros::StaticTransformBroadcaster tf_pub(node);
  tf_pub.sendTransform({robot_tf, laser_tf});

  nav_msgs::msg::OccupancyGrid map;
  if (nav2_map_server::loadMapFromYaml(args.map, map) != nav2_map_server::LOAD_MAP_SUCCESS) {
    RCLCPP_FATAL(logger, "cannot load map %s", args.map.c_str());
    return 1;
  }
  map.header.frame_id = kMapFrame;
  map.header.stamp = node->now();
  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/bench_map", rclcpp::QoS(1).transient_local().reliable());
  map_pub->on_activate();
  map_pub->publish(map);
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "bench_scan", rclcpp::SensorDataQoS());
  scan_pub->on_activate();
  // 라이다처럼 10 Hz로 합성 스캔을 보낸다(RL 플러그인만 받는다)
  auto scan_timer = node->create_wall_timer(
    std::chrono::milliseconds(100), [&]() {scan_pub->publish(make_scan(node->now()));});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  std::thread spinner([&exec]() {exec.spin();});
  costmap->configure();
  costmap->activate();
  auto costmap_thread = std::make_unique<nav2_util::NodeThread>(costmap);

  // ---- 지도가 costmap에 들어올 때까지(로봇 옆 진열대 칸이 치명 비용) ----
  const auto deadline = Clock::now() + std::chrono::seconds(20);
  bool ready = false;
  while (Clock::now() < deadline && !ready) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    unsigned int mx, my;
    auto * c = costmap->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(c->getMutex()));
    ready = c->worldToMap(1.0, 0.774, mx, my) &&
      c->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
  if (!ready) {
    RCLCPP_FATAL(logger, "static map did not reach the costmap within 20 s");
    return 1;
  }
  // 측정 중에는 costmap을 갱신하지 않는다(로봇이 서 있어 창이 같다). 갱신 비용은 이 표에 없다
  costmap->pause();

  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  pluginlib::ClassLoader<nav2_core::GoalChecker> gc_loader("nav2_core", "nav2_core::GoalChecker");
  auto goal_checker = gc_loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  goal_checker->initialize(node, "general_goal_checker", costmap);

  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = kRobotV;

  std::vector<std::string> names = bench.at("cases").get<std::vector<std::string>>();
  if (!args.only.empty()) {
    names = {args.only};
  }
  std::vector<Result> results;
  std::set<std::string> publishers;
  for (const auto & name : names) {
    const Params own = with_prefix(bench, name);
    if (own.empty()) {
      throw std::runtime_error("no case " + name + " in " + args.cases);
    }
    Result r;
    r.name = name;
    r.kind = own.count("bench_kind") ? own.at("bench_kind").get<std::string>() : "controller";
    r.rate_hz = own.at("bench_rate_hz").get<double>();
    auto collect = [&]() {
        for (const auto & p : graph_publishers(node)) {
          publishers.insert(p);
        }
      };

    if (r.kind == "onnx") {
      const std::string dir = ament_index_cpp::get_package_share_directory("pode_rl_controller") +
        "/policies/" + own.at("policy").get<std::string>();
      const auto spec = pode_rl_controller::load_policy_spec(dir + "/policy_spec.json");
      pode_rl_controller::OnnxPolicy policy(
        dir + "/policy.onnx", spec.obs_dim, spec.act_dim,
        static_cast<int>(own.at("intra_op_threads").get<int64_t>()));
      std::vector<float> obs(spec.obs_dim);
      for (int i = 0; i < spec.obs_dim; ++i) {
        obs[i] = 0.5f + 0.4f * static_cast<float>(std::sin(i * 0.7));
      }
      r.plugin = "OnnxPolicy " + own.at("policy").get<std::string>();
      RCLCPP_INFO(logger, "case %s: %s, obs %d", name.c_str(), r.plugin.c_str(), spec.obs_dim);
      run_loop(r, args, [&]() {policy.run(obs);}, collect);
    } else {
      // ---- nav2_params.yaml의 원본 값 + cases.yaml의 덮개를 <경우 이름>.* 로 선언 ----
      Params p;
      const std::string source = own.count("bench_source") ?
        own.at("bench_source").get<std::string>() : "";
      if (!source.empty()) {
        p = with_prefix(nav2, source);
        if (p.empty()) {
          throw std::runtime_error("no controller " + source + " in " + args.nav2_params);
        }
      }
      for (const auto & [k, v] : own) {
        if (k.rfind("bench_", 0) != 0) {
          p[k] = v;
        }
      }
      for (const auto & [k, v] : p) {
        put_param(node, name + "." + k, v);
      }
      put_param(node, "controller_frequency", rclcpp::ParameterValue(r.rate_hz));
      r.plugin = p.at("plugin").get<std::string>();
      r.batch_size = param_text(p, "batch_size");
      r.time_steps = param_text(p, "time_steps");
      r.model_dt = param_text(p, "model_dt");
      RCLCPP_INFO(
        logger, "case %s: %s from %s at %.1f Hz", name.c_str(), r.plugin.c_str(),
        source.empty() ? "(cases.yaml)" : source.c_str(), r.rate_hz);

      auto controller = loader.createSharedInstance(r.plugin);
      controller->configure(node, name, tf, costmap);
      controller->activate();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      controller->setPlan(make_plan(node->now()));
      goal_checker->reset();
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = kMapFrame;
      pose.pose.position.x = kRobotX;
      pose.pose.position.y = kRobotY;
      pose.pose.orientation = yaw_quat(0.0);
      run_loop(
        r, args, [&]() {
          pose.header.stamp = node->now();
          const auto cmd = controller->computeVelocityCommands(pose, velocity, goal_checker.get());
          (void)cmd;
        }, collect);
      controller->deactivate();
      controller->cleanup();
    }
    const double mean = r.ms.empty() ? 0.0 :
      std::accumulate(r.ms.begin(), r.ms.end(), 0.0) / r.ms.size();
    RCLCPP_INFO(
      logger, "case %s: %zu calls, mean %.3f ms, errors %d%s%s", name.c_str(), r.ms.size(), mean,
      r.errors, r.errors ? ", first: " : "", r.first_error.c_str());
    results.push_back(r);
  }

  // ---- 결과: samples.csv(호출마다), runs.csv(경우마다), publishers.txt ----
  std::ofstream samples(args.out + "/samples.csv");
  samples << "case,iter,ms\n";
  std::ofstream runs(args.out + "/runs.csv");
  runs << "case,kind,plugin,rate_hz,batch_size,time_steps,model_dt,n,errors,overruns,wall_s,"
    "proc_user_s,proc_sys_s,thread_cpu_s,first_error\n";
  for (const auto & r : results) {
    for (size_t i = 0; i < r.ms.size(); ++i) {
      samples << r.name << ',' << i << ',' << r.ms[i] << '\n';
    }
    std::string err = r.first_error;
    std::replace(err.begin(), err.end(), ',', ';');
    std::replace(err.begin(), err.end(), '\n', ' ');
    runs << r.name << ',' << r.kind << ',' << r.plugin << ',' << r.rate_hz << ',' <<
      r.batch_size << ',' << r.time_steps << ',' << r.model_dt << ',' << r.ms.size() << ',' <<
      r.errors << ',' << r.overruns << ',' << r.wall_s << ',' << r.proc_user_s << ',' <<
      r.proc_sys_s << ',' << r.thread_cpu_s << ',' << err << '\n';
  }
  std::ofstream pubs(args.out + "/publishers.txt");
  int cmd_vel = 0;
  for (const auto & p : publishers) {
    pubs << p << '\n';
    cmd_vel += p.find("cmd_vel") != std::string::npos ? 1 : 0;
  }
  RCLCPP_INFO(
    logger, "%zu publishers in the graph, %d on a cmd_vel topic (list in publishers.txt)",
    publishers.size(), cmd_vel);

  samples.close();
  runs.close();
  pubs.close();
  scan_timer->cancel();
  costmap_thread.reset();
  costmap->deactivate();
  costmap->cleanup();
  exec.cancel();
  spinner.join();
  int failed = 0;
  for (const auto & r : results) {
    failed += r.errors > 0 || r.ms.empty() ? 1 : 0;
  }
  if (cmd_vel > 0) {
    RCLCPP_FATAL(logger, "a cmd_vel publisher was in the graph during the run");
  }
  const int code = cmd_vel > 0 ? 3 : (failed > 0 ? 2 : 0);
  // 옵션 생성자로 만든 Costmap2DROS는 소멸자에서 SIGSEGV가 난다(Jazzy). 결과를 다 쓴 뒤라 소멸 없이 끝낸다
  rclcpp::shutdown();
  std::fflush(nullptr);
  std::_Exit(code);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    code = run(parse_args(rclcpp::remove_ros_arguments(argc, argv)));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("compute_bench"), "%s", e.what());
  }
  rclcpp::shutdown();
  return code;
}
