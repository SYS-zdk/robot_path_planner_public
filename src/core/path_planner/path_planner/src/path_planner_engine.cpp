/**
 * @file: path_planner_engine.cpp
 * @brief: Contains the path planner core engine class (shared by global planner plugins)
 * @author: ZhangDingkun
 * @date: 2024.06.08
 * @version: 1.0
 */
#include <tf2/utils.h>
#include <cmath>

#include <common/rpp_scope.hpp>

// planning pipeline (concrete algorithm is injected by each plugin package)
#include "path_planner/path_planner_engine.h"

// path processor
#include "path_planner/path_simplify/rdp_path_simplifier.h"

// optimizer
#include "trajectory_planner/trajectory_optimization/optimizer_core.h"

#include "common/util/log.h"

namespace rpp
{
namespace path_planner
{
namespace
{
}  // namespace

/**
 * @brief Construct a new Graph Planner object
 */
PathPlannerEngine::PathPlannerEngine()
  : initialized_(false)
  , g_planner_(nullptr)
  , auto_safety_corridor_(true)
  , use_safety_corridor_(false)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
PathPlannerEngine::PathPlannerEngine(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : PathPlannerEngine()
{
  initialize(name, costmap_ros);
}

void PathPlannerEngine::initializeWithPlanner(std::string instance_name, costmap_2d::Costmap2DROS* costmapRos,
                                              const std::string& planner_name, PLANNER_TYPE planner_type,
                                              std::shared_ptr<PathPlanner> planner)
{
  forced_planner_name_ = planner_name;
  planner_name_ = planner_name;
  planner_type_ = planner_type;
  g_planner_ = std::move(planner);
  initialize(instance_name, costmapRos);
}

/**
 * @brief Planner initialization
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void PathPlannerEngine::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  costmap_ros_ = costmapRos;
  initialize(name);
}

void PathPlannerEngine::initializeWithFixedPlanner(std::string name, costmap_2d::Costmap2DROS* costmapRos,
                                                   const std::string& fixed_planner_name)
{
  forced_planner_name_ = fixed_planner_name;
  initialize(name, costmapRos);
}

void PathPlannerEngine::initializeWithFixedPlanner(std::string name, const std::string& fixed_planner_name)
{
  forced_planner_name_ = fixed_planner_name;
  initialize(name);
}

/**
 * @brief Planner initialization
 * @param name     planner name
 * @param costmap  costmap pointer
 * @param frame_id costmap frame ID
 */
void PathPlannerEngine::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // Record plugin instance name for parameter namespace (e.g., ~/<instance>/optimizer_name).
    instance_name_ = name;

    // initialize ROS node (parameters)
    ros::NodeHandle private_nh("~/" + name);

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    double obstacle_factor;

    private_nh.param("default_tolerance", tolerance_, 0.0);                  // error tolerance
    private_nh.param("outline_map", is_outline_, false);                     // whether outline the map or not
    private_nh.param("obstacle_factor", obstacle_factor, 0.5);               // obstacle factor
    factor_ = obstacle_factor;
    private_nh.param("expand_zone", is_expand_, false);                      // whether publish expand zone or not
    private_nh.param("show_safety_corridor", show_safety_corridor_, false);  // whether visualize safety corridor
    private_nh.param("auto_safety_corridor", auto_safety_corridor_, true);   // auto toggle per planner (toolchain)
  
    // planner name (injected planners should set forced_planner_name_ and g_planner_ before initialize())
    if (!forced_planner_name_.empty())
    {
      planner_name_ = forced_planner_name_;
    }
    else
    {
      private_nh.param("planner_name", planner_name_, (std::string) "a_star");
    }

    // If a plugin injected a concrete planner instance, we skip internal planner construction here.
    if (!g_planner_)
    {
      R_ERROR << "PathPlannerEngine: no planner instance provided. "
                 "Global planner plugins must call initializeWithPlanner() and inject a concrete planner.";
    }

    if (!g_planner_)
    {
      R_ERROR << "Failed to construct global planner for planner_name: " << planner_name_;
      initialized_ = false;
      return;
    }

    // Apply obstacle factor to planner base
    g_planner_->setFactor(factor_);

    R_INFO << "Using path planner: " << planner_name_;

    // pass costmap information to planner (required)
    g_planner_->setFactor(factor_);

    // ========== 根据全局规划方法配置工具链和优化器 ==========
    // 在这一步中，根据不同的规划器类型来选择性启用RDP、优化器和走廊约束
    configurePlannerToolchain();
  }
  else
  {
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
}

/**
 * @brief plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
bool PathPlannerEngine::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully, else false
 */
bool PathPlannerEngine::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_ || !g_planner_)
  {
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }

  // Reset last debug snapshot
  debug_data_ = DebugData{};

  auto* costmap = g_planner_->getCostMap();
  if (!costmap)
  {
    R_ERROR << "Costmap is null in global planner.";
    return false;
  }

  // start thread mutex
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

  // Costmap can be resized after plugin construction (e.g. when static map arrives).
  // Refresh planner cached map dimensions before any world2Map/grid operations.
  g_planner_->syncCostmapInfo();

  // Diagnostics: log call time
  ros::Time now = ros::Time::now();
  R_INFO << "makePlan() called at " << now.toSec();

  // Standard planning flow (caching/throttle/concurrency guard removed)
  // clear existing plan
  plan.clear();

  // judege whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_)
  {
    R_ERROR << "The goal pose passed to this planner must be in the " << frame_id_ << " frame. It is instead in the "
            << goal.header.frame_id << " frame.";
    return false;
  }

  if (start.header.frame_id != frame_id_)
  {
    R_ERROR << "The start pose passed to this planner must be in the " << frame_id_ << " frame. It is instead in the "
            << start.header.frame_id << " frame.";
    return false;
  }

  // get goal and start node coordinate tranform from world to costmap
  double wx = start.pose.position.x, wy = start.pose.position.y;
  double g_start_x, g_start_y, g_goal_x, g_goal_y;
  if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y))
  {
    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    const double res = costmap->getResolution();
    const double max_x = origin_x + costmap->getSizeInCellsX() * res;
    const double max_y = origin_y + costmap->getSizeInCellsY() * res;
    R_WARN << "The robot's start position is off the global costmap. Planning will always fail. "
              "start_world=(" << wx << "," << wy << ") "
              "costmap_origin=(" << origin_x << "," << origin_y << ") "
              "costmap_size=(" << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY() << ") "
              "res=" << res << " "
              "world_bounds_x=[" << origin_x << "," << max_x << "] "
              "world_bounds_y=[" << origin_y << "," << max_y << "]";
    return false;
  }
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y))
  {
    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    const double res = costmap->getResolution();
    const double max_x = origin_x + costmap->getSizeInCellsX() * res;
    const double max_y = origin_y + costmap->getSizeInCellsY() * res;
    R_WARN << "The robot's goal position is off the global costmap. Planning will always fail. "
              "goal_world=(" << wx << "," << wy << ") "
              "costmap_origin=(" << origin_x << "," << origin_y << ") "
              "costmap_size=(" << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY() << ") "
              "res=" << res << " "
              "world_bounds_x=[" << origin_x << "," << max_x << "] "
              "world_bounds_y=[" << origin_y << "," << max_y << "]";
    return false;
  }

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();

  // calculate path
  PathPlanner::Points3d origin_path;
  PathPlanner::Points3d expand;
  bool path_found = false;

  auto start_time_all = std::chrono::high_resolution_clock::now();
  auto start_time_plan = start_time_all;
  // planning
  path_found = g_planner_->plan({ g_start_x, g_start_y, tf2::getYaw(start.pose.orientation) },
                                { g_goal_x, g_goal_y, tf2::getYaw(goal.pose.orientation) }, origin_path, expand);

  auto end_time_plan = std::chrono::high_resolution_clock::now();
  debug_data_.planning_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time_plan - start_time_plan).count();
  debug_data_.path_found = path_found;
  debug_data_.origin_path_map = origin_path;
  debug_data_.expand_map = expand;

  R_INFO << "[makePlan] planner=" << planner_name_ << ": origin_path(map units) size=" << origin_path.size();
                          

  // convert path to ros plan
  if (path_found)
  {
    if (_getPlanFromPath(origin_path, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);

      R_INFO << "[makePlan] ros plan size(after _getPlanFromPath + goal)=" << plan.size();

      // path process
      PathPlanner::Points3d origin_plan, prune_plan;
      for (const auto& pt : plan)
      {
        origin_plan.emplace_back(pt.pose.position.x, pt.pose.position.y);
      }

      R_INFO << "[makePlan] origin_plan(world) size=" << origin_plan.size();

      debug_data_.origin_plan_world = origin_plan;

      // 根据配置决定是否应用RDP
      if (pruner_)
      {
        pruner_->process(origin_plan, prune_plan);
        R_DEBUG << "[makePlan] RDP applied: " << origin_plan.size() << " -> " << prune_plan.size() << " points";
      }
      else
      {
        prune_plan = origin_plan;
        R_DEBUG << "[makePlan] RDP skipped (disabled for " << planner_name_ << ")";
      }

      R_INFO << "[makePlan] prune_plan(world) size=" << prune_plan.size();

      debug_data_.prune_plan_world = prune_plan;

      // optimization
      if (optimizer_name_ != "")
      {
        PathPlanner::Points3d path_opt;
        {
          // ✅ 关键修复：先执行优化，再获取结果
          if (optimizer_->run(prune_plan))
          {
            rpp::common::structure::Trajectory3d traj;
            if (optimizer_->getTrajectory(traj))
            {
              R_INFO << "[makePlan] optimizer trajectory size=" << traj.position.size();
              for (const auto& pt : traj.position)
              {
                path_opt.emplace_back(pt.x(), pt.y(), 0.0);
              }
            }
            else
            {
              R_WARN << "Optimizer finished but trajectory is unavailable, fallback to pruned path";
              path_opt = prune_plan;
            }
          }
          else
          {
            R_ERROR << "Optimizer failed to run, using pruned path instead";
            path_opt = prune_plan;
          }

          if (path_opt.empty())
          {
            R_WARN << "Optimizer produced empty path, fallback to pruned path";
            path_opt = prune_plan;
          }

          debug_data_.optimized_plan_world = path_opt;

          plan.clear();
          for (const auto& pt : path_opt)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_id_;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;  // 默认无旋转
            plan.push_back(pose);
          }
          plan.push_back(goalCopy);
        }
      }
      else
      {
        // 如果禁用优化，但启用了RDP，需要用 prune_plan 更新 plan
        if (pruner_)
        {
          R_INFO << "[makePlan] Optimization disabled, using RDP path: " << prune_plan.size() << " points";
          plan.clear();
          for (const auto& pt : prune_plan)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_id_;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
          }
          plan.push_back(goalCopy);
        }
        // 否则 plan 保持为原始规划路径（已在第440行设置）
      }

      // planner-specific debug snapshots (no publishing here)
      g_planner_->appendDebugData(debug_data_);
    }
    else
    {
      R_ERROR << "Failed to get a plan from path when a legal path was found. This shouldn't happen.";
    }
  }
  else
  {
    R_ERROR << "Failed to get a path. planner=" << planner_name_
            << ", start=(" << start.pose.position.x << ", " << start.pose.position.y << ")"
            << ", goal=(" << goal.pose.position.x << ", " << goal.pose.position.y << ")";
  }
  auto end_time_all = std::chrono::high_resolution_clock::now();
  debug_data_.total_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time_all - start_time_all).count();
  debug_data_.has_total_time = true;

  return !plan.empty();
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool PathPlannerEngine::_getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }
  plan.clear();

  for (const auto& pt : path)
  {
    double wx, wy;
    g_planner_->map2World(pt.x(), pt.y(), wx, wy);

    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);

  }

  return !plan.empty();
}

/**
 * @brief 根据全局规划方法配置对应的工具链
 * 
 * 为不同的全局规划器分别配置：
 *  1. RDP路径剪枝 (pruner_)
 *  2. 轨迹优化器 (optimizer_)
 *  3. 安全走廊约束 (use_safety_corridor_)
 * 
 * 例如：
 *  - A*: RDP=ON + LBFGS优化 + 走廊=OFF
 *  - Hybrid A*: RDP=OFF + LBFGS优化 + 走廊=OFF
 *  - Sunshine: RDP=OFF + LBFGS优化 + 走廊=OFF
 *  - 采样规划器: RDP=OFF + 无优化 + 走廊=OFF
 */
void PathPlannerEngine::configurePlannerToolchain()
{
  ros::NodeHandle private_nh("~/" + instance_name_);

  auto apply_safety_corridor_vis = [&](bool enable_by_toolchain) {
    if (auto_safety_corridor_)
    {
      show_safety_corridor_ = enable_by_toolchain;
    }
    R_INFO << "  ✓ Safety Corridor VIS " << (show_safety_corridor_ ? "ENABLED" : "DISABLED")
           << (auto_safety_corridor_ ? " (auto)" : " (manual)");
  };

  // 优先检查是否有外部（launch/arg）强制设置的 optimizer 参数。
  // 如果存在，则根据该参数直接初始化对应的优化器，并在后续分支中避免被覆盖。
  bool optimizer_forced = false;
  std::string forced_optimizer;
  // New (preferred): per-plugin namespace, e.g. /move_base/NagPlanner/optimizer_name
  private_nh.param("optimizer_name", forced_optimizer, std::string(""));
  // Backward compatibility: legacy PathPlanner namespace
  if (forced_optimizer.empty())
  {
    ros::NodeHandle root_nh;
    root_nh.param("/move_base/PathPlanner/optimizer_name", forced_optimizer, std::string(""));
  }
  if (!forced_optimizer.empty())
  {
    R_INFO << "[PlannerToolchain] Forced optimizer param detected: " << forced_optimizer;
    if (forced_optimizer == "lbfgs" || forced_optimizer == "lbfgs_optimizer")
    {
      int max_iter;
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 200);
      private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 1.4);
      private_nh.param("/move_base/Optimizer/k_max", k_max, 0.2);
      private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
      private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 10.0);
      private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 10.0);
        optimizer_ = std::make_shared<rpp::trajectory_optimization::LBFGSOptimizer>(
          costmap_ros_, max_iter, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "lbfgs";
      R_INFO << "  ✓ Optimizer (forced): lbfgs";
      optimizer_forced = true;
    }
    else if (forced_optimizer == "splinetrajectory" || forced_optimizer == "splinetrajectory_optimizer" ||
             forced_optimizer == "spline_trajectory" || forced_optimizer == "spline_trajectory_optimizer")
    {
      int sample_points;
      double vel_max, acc_max;
      private_nh.param("/move_base/Optimizer/sample_points", sample_points, 120);
      private_nh.param("/move_base/Optimizer/vel_max", vel_max, 1.0);
      private_nh.param("/move_base/Optimizer/acc_max", acc_max, 2.0);

        optimizer_ = std::make_shared<rpp::trajectory_optimization::SplineTrajectoryOptimizer>(
          costmap_ros_, sample_points, vel_max, acc_max);
      optimizer_name_ = "splinetrajectory_optimizer";
      R_INFO << "  ✓ Optimizer (forced): splinetrajectory_optimizer";
      optimizer_forced = true;
    }
    else if (forced_optimizer == "minco" || forced_optimizer == "minco_spline_optimizer" ||
             forced_optimizer == "minco_spline" || forced_optimizer == "minco_spline_opt")
    {
      int sample_points;
      double vel_max, acc_max;
      private_nh.param("/move_base/Optimizer/sample_points", sample_points, 120);
      private_nh.param("/move_base/Optimizer/vel_max", vel_max, 1.0);
      private_nh.param("/move_base/Optimizer/acc_max", acc_max, 2.0);

        optimizer_ = std::make_shared<rpp::trajectory_optimization::MincoSplineOptimizer>(
          costmap_ros_, sample_points, vel_max, acc_max);
      optimizer_name_ = "minco_spline_optimizer";
      R_INFO << "  ✓ Optimizer (forced): minco_spline_optimizer";
      optimizer_forced = true;
    }
    else if (forced_optimizer == "none" || forced_optimizer == "disabled")
    {
      // 明确通过参数要求禁用优化器
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer (forced): DISABLED";
      optimizer_forced = true;
    }
    else
    {
      R_WARN << "[PlannerToolchain] Unknown forced optimizer: " << forced_optimizer;
    }
  }
  if (planner_name_ == "a_star" || planner_name_ == "dijkstra" || planner_name_ == "gbfs")
  {
    // ========== A* 及其变种 ==========
    R_INFO << "[PlannerToolchain] Configuring for A* variant...";

    // 启用RDP路径剪枝
    pruner_ = std::make_shared<RDPPathSimplifier>(0.22);  //lbfgs 0.035 
    R_INFO << "  ✓ RDP Path Processor ENABLED (threshold=0.22)";

    if (!optimizer_forced)
    {
      // Default to LBFGS smoothing (minimum-snap implementation removed from this repository).
      int max_iter;
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 200);
      private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 1.4);
      private_nh.param("/move_base/Optimizer/k_max", k_max, 0.2);
      private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
      private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 10.0);
      private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 10.0);
      optimizer_ = std::make_shared<rpp::trajectory_optimization::LBFGSOptimizer>(
        costmap_ros_, max_iter, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "lbfgs";
      R_INFO << "  ✓ Optimizer: lbfgs";
    }

    // 走廊约束（当前未接入优化器约束项，保持关闭）
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor CONSTRAINT DISABLED";

    // Safety corridor visualization is disabled by default in this repo variant.
    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "hybrid_astar")
  {
    // ========== Hybrid A* ==========
    R_INFO << "[PlannerToolchain] Configuring for Hybrid A*...";

    // ✅ FIX: 启用RDP路径剪枝，但使用较小的delta（0.1m）
    // 原问题：混合A*输出225个密集点，导致LBFGS有450个优化变量，Line Search失败
    // 解决方案：用RDP将225个点简化为80-120个关键点，LBFGS优化200-240个变量，正好在最优范围内
    pruner_ = std::make_shared<RDPPathSimplifier>(0.03);
    R_INFO << "  ✓ RDP Path Processor ENABLED (threshold=0.1m)";

    if (!optimizer_forced)
    {
      // 启用LBFGS优化器 (默认)
      int max_iter;
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 200);
      private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 1.4);
      private_nh.param("/move_base/Optimizer/k_max", k_max, 0.2);
      private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
      private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 10.0);
      private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 10.0);
      optimizer_ = std::make_shared<rpp::trajectory_optimization::LBFGSOptimizer>(
        costmap_ros_, max_iter, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "lbfgs";
      R_INFO << "  ✓ Optimizer: lbfgs";
    }

    // 禁用安全走廊 (混合A*本身已保证动力学可行性)
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "sunshine")
  {
    // ========== Sunshine规划器 ==========
    R_INFO << "[PlannerToolchain] Configuring for Sunshine...";

    // 禁用RDP
    pruner_ = nullptr;
    R_INFO << "  ✓ RDP Path Processor DISABLED";

    if (!optimizer_forced)
    {
      // Default to LBFGS smoothing (conjugate-gradient implementation removed from this repository).
      int max_iter;
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 200);
      private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 1.4);
      private_nh.param("/move_base/Optimizer/k_max", k_max, 0.2);
      private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
      private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 10.0);
      private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 10.0);
      optimizer_ = std::make_shared<rpp::trajectory_optimization::LBFGSOptimizer>(
        costmap_ros_, max_iter, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "lbfgs";
      R_INFO << "  ✓ Optimizer: lbfgs";
    }

    // 禁用安全走廊
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "rhcf")
  {
    // ========== RHCF (Voronoi Random Walk) ==========
    R_INFO << "[PlannerToolchain] Configuring for RHCF...";

    // RHCF 输出通常是 Voronoi 骨架上的折线（cell-by-cell），点数偏多且转折生硬。
    // 使用 RDP 先做关键点提取，再交给优化器做平滑/避障，可得到更“可执行”的全局参考路径。
    pruner_ = std::make_shared<RDPPathSimplifier>(0.05);
    R_INFO << "  ✓ RDP Path Processor ENABLED (threshold=0.05m)";

    if (!optimizer_forced)
    {
      // 默认启用 LBFGS 平滑优化（与 Hybrid A* 的默认保持一致，便于复用现有参数）
      int max_iter;
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 200);
      private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 1.4);
      private_nh.param("/move_base/Optimizer/k_max", k_max, 0.2);
      private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
      private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 10.0);
      private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 10.0);
      optimizer_ = std::make_shared<rpp::trajectory_optimization::LBFGSOptimizer>(
        costmap_ros_, max_iter, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "lbfgs";
      R_INFO << "  ✓ Optimizer: lbfgs";
    }

    // 禁用安全走廊：全局路径仅作为参考，局部规划器负责时域/动力学可行性
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    // RHCF 默认不启用走廊可视化（否则 Voronoi cell-by-cell 路径会生成大量 corridor 段）
    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "nag")
  {
    // ========== NAG (Neighborhood-Augmented Graph) ==========
    R_INFO << "[PlannerToolchain] Configuring for NAG...";

    // 为贴近论文中 geodesic 搜索过程，默认不做额外剪枝与轨迹优化
    pruner_ = nullptr;
    R_INFO << "  ✓ RDP Path Processor DISABLED";

    // Default: keep NAG output as-is (geodesic polyline) unless the user explicitly forces an optimizer.
    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer DISABLED";
    }
    else
    {
      // optimizer_ / optimizer_name_ have been set in the forced-optimizer section above.
      R_INFO << "  ✓ Optimizer (forced): " << optimizer_name_;
    }

    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "rrt" || planner_name_ == "rrt_star" || planner_name_ == "informed_rrt" ||
           planner_name_ == "bdrp")
  {
    // ========== 采样规划器 (RRT系列) ==========
    R_INFO << "[PlannerToolchain] Configuring for sampling planner (" << planner_name_ << ")...";

    // 禁用RDP (采样规划器输出已经是点集，无需进一步剪枝)
    pruner_ = nullptr;
    R_INFO << "  ✓ RDP Path Processor DISABLED";

    // 禁用优化器 (采样规划器通常已内置路径优化)
    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer DISABLED";
    }

    // 禁用安全走廊
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "voronoi" || planner_name_ == "reachability")
  {
    // ========== 其他图搜索规划器 ==========
    R_INFO << "[PlannerToolchain] Configuring for graph planner (" << planner_name_ << ")...";

    // 禁用RDP
    pruner_ = nullptr;
    R_INFO << "  ✓ RDP Path Processor DISABLED";

    // 禁用优化器
    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer DISABLED";
    }

    // 禁用安全走廊
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "parallel_curves")
  {
    // ========== Parallel Curves ==========
    R_INFO << "[PlannerToolchain] Configuring for Parallel Curves...";

    // Keep native parallel-curves graph result untouched by default.
    // Segment densification is already handled by planner parameter line_splits.
    pruner_ = nullptr;
    R_INFO << "  ✓ RDP Path Processor DISABLED";

    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer DISABLED";
    }

    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "itp")
  {
    // ========== ITP (Interactive Topological Planner) ==========
    R_INFO << "[PlannerToolchain] Configuring for ITP (Interactive Topological Planner)...";

    // ITP 输出是一条由拓扑图插值生成的参考折线；默认不做额外剪枝与轨迹优化。
    pruner_ = nullptr;
    R_INFO << "  ✓ RDP Path Processor DISABLED";

    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer DISABLED";
    }

    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else if (planner_name_ == "rolling_circle_center" || planner_name_ == "rcc")
  {
    // ========== Rolling Circle Center (RCC) ==========
    R_INFO << "[PlannerToolchain] Configuring for Rolling Circle Center (RCC)...";

    // RCC 输出通常是密集折线，建议轻度 RDP 提取关键点（默认可关/由用户覆盖）
    pruner_ = std::make_shared<RDPPathSimplifier>(0.05);
    R_INFO << "  ✓ RDP Path Processor ENABLED (threshold=0.05m)";

    // 默认不强制启用优化器，用户可通过 /move_base/<planner_instance>/optimizer_name 覆盖
    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
      R_INFO << "  ✓ Optimizer DISABLED";
    }

    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";

    apply_safety_corridor_vis(false);
  }
  else
  {
    R_WARN << "[PlannerToolchain] Unknown planner (" << planner_name_ << "), using minimal config";
    pruner_ = nullptr;
    if (!optimizer_forced)
    {
      optimizer_ = nullptr;
      optimizer_name_ = "";
    }
    use_safety_corridor_ = false;

    apply_safety_corridor_vis(false);
  }
}

const PathPlannerEngine::DebugData& PathPlannerEngine::debugData() const
{
  return debug_data_;
}

bool PathPlannerEngine::isInitialized() const
{
  return initialized_;
}

const std::string& PathPlannerEngine::frameId() const
{
  return frame_id_;
}

const std::string& PathPlannerEngine::plannerName() const
{
  return planner_name_;
}

PathPlannerEngine::PLANNER_TYPE PathPlannerEngine::plannerType() const
{
  return planner_type_;
}

bool PathPlannerEngine::isExpandEnabled() const
{
  return is_expand_;
}

bool PathPlannerEngine::shouldShowSafetyCorridor() const
{
  return show_safety_corridor_;
}

costmap_2d::Costmap2DROS* PathPlannerEngine::costmapRos() const
{
  return costmap_ros_;
}

PathPlanner* PathPlannerEngine::planner() const
{
  return g_planner_.get();
}

}  // namespace path_planner
}  // namespace rpp