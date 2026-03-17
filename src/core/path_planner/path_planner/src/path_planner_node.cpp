
/**
 * @file: path_planner_node.cpp
 * @brief: Contains the path planner ROS wrapper class
 * @author: ZhangDingkun
 * @date: 2024.06.08
 * @version: 1.0
 */
#include <tf2/utils.h>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

// graph-based planner
#include "path_planner/path_planner_node.h"
#include "path_planner/graph_planner/astar_planner.h"
#include "path_planner/graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"
#include "path_planner/graph_planner/reachability_planner.h"
#include "path_planner/sample_planner/sunshine_planner.h"

// path processor
#include "path_planner/path_prune/ramer_douglas_peucker.h"

// optimizer
#include "trajectory_planner/trajectory_optimization/optimizer_core.h"

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/geometry/polygon2d.h"
#include "common/safety_corridor/convex_safety_corridor.h"

PLUGINLIB_EXPORT_CLASS(rpp::path_planner::PathPlannerNode, nav_core::BaseGlobalPlanner)

namespace rpp
{
namespace path_planner
{
using Visualizer = rpp::common::util::Visualizer;

namespace
{
}  // namespace

/**
 * @brief Construct a new Graph Planner object
 */
PathPlannerNode::PathPlannerNode()
  : initialized_(false)
  , g_planner_(nullptr)
  , use_safety_corridor_(false)
  , is_initial_plan_recorded_(false)
  , is_initial_plan_recorded2_(false)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
PathPlannerNode::PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : PathPlannerNode()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Planner initialization
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void PathPlannerNode::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  costmap_ros_ = costmapRos;
  initialize(name);
}

/**
 * @brief Planner initialization
 * @param name     planner name
 * @param costmap  costmap pointer
 * @param frame_id costmap frame ID
 */
void PathPlannerNode::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
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
  
    // planner name
    private_nh.param("planner_name", planner_name_, (std::string) "a_star");
    if (planner_name_ == "a_star")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "dijkstra")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_, true);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "gbfs")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_, false, true);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "hybrid_astar")
      {
        double goal_tolerance, cost_penalty, curve_sample_ratio, minimum_turning_radius, non_straight_penalty,
            change_penalty, reverse_penalty, retrospective_penalty, lookup_table_dim, analytic_expansion_ratio,
            analytic_expansion_max_length;
        double path_point_spacing;
        double voronoi_heuristic_weight, voronoi_cost_weight, voronoi_alpha, voronoi_d0_max;
        int dim_3_size, max_iterations, max_approach_iterations, motion_model;
        bool traverse_unknown, cache_obstacle_heuristic, downsample_obstacle_heuristic;
        // 2: Dubins, 3: Reeds-Shepp. Default to RS so reversing maneuvers are allowed.
        private_nh.param(planner_name_ + "/motion_model", motion_model, 2);
        private_nh.param(planner_name_ + "/goal_tolerance", goal_tolerance, 0.125);
        private_nh.param(planner_name_ + "/dim_3_size", dim_3_size, 1);
        private_nh.param(planner_name_ + "/max_iterations", max_iterations, 10000);
        private_nh.param(planner_name_ + "/max_approach_iterations", max_approach_iterations, 1000);
        private_nh.param(planner_name_ + "/traverse_unknown", traverse_unknown, true);
        private_nh.param(planner_name_ + "/cost_penalty", cost_penalty, 1.0);
        private_nh.param(planner_name_ + "/cache_obstacle_heuristic", cache_obstacle_heuristic, false);
        // Used by Dubins/Reeds-Shepp curve generation (analytic expansion + heuristic lookup).
        // Smaller step -> denser sampling.
        private_nh.param(planner_name_ + "/curve_sample_ratio", curve_sample_ratio, 0.2);
        // Output path downsampling spacing in meters (applied to consecutive points).
        // ✅ Hybrid A* 的path_point_spacing应恢复为默认值0.5m
        // removeNearDuplicateXYInPlace会删除间距<10cells的点，削减到100个左右
        // 然后通过configurePlannerToolchain()中的RDP(0.1m)再简化，达到最优密度
        private_nh.param(planner_name_ + "/path_point_spacing", path_point_spacing, 0.3);
        private_nh.param(planner_name_ + "/minimum_turning_radius", minimum_turning_radius, 0.4);
        private_nh.param(planner_name_ + "/non_straight_penalty", non_straight_penalty, 1.0);
        private_nh.param(planner_name_ + "/change_penalty", change_penalty, 1.0);
        private_nh.param(planner_name_ + "/reverse_penalty", reverse_penalty, 1.0);
        private_nh.param(planner_name_ + "/retrospective_penalty", retrospective_penalty, 0.0);
        private_nh.param(planner_name_ + "/lookup_table_dim", lookup_table_dim, 10.0);
        private_nh.param(planner_name_ + "/analytic_expansion_ratio", analytic_expansion_ratio, 3.5);
        private_nh.param(planner_name_ + "/analytic_expansion_max_length", analytic_expansion_max_length, 3.0);
        private_nh.param(planner_name_ + "/downsample_obstacle_heuristic", downsample_obstacle_heuristic, true);
        private_nh.param(planner_name_ + "/voronoi_heuristic_weight", voronoi_heuristic_weight, 0.0);
        private_nh.param(planner_name_ + "/voronoi_cost_weight", voronoi_cost_weight, 0.3); 
        private_nh.param(planner_name_ + "/voronoi_alpha", voronoi_alpha, 1.0);
        private_nh.param(planner_name_ + "/voronoi_d0_max", voronoi_d0_max, 2.0);  // 3.0 米

        HybridSearchInfo info;
        const double resolution = costmap_ros_->getCostmap() ? costmap_ros_->getCostmap()->getResolution() : 0.0;
        info.motion_model = motion_model;
        // Internally, Hybrid A* uses map cell units for distances.
        info.goal_tolerance = (resolution > 0.0) ? (goal_tolerance / resolution) : goal_tolerance;
        info.dim_3_size = dim_3_size;
        info.max_iterations = max_iterations;
        info.max_approach_iterations = max_approach_iterations;
        info.traverse_unknown = traverse_unknown;
        info.cost_penalty = cost_penalty;
        info.cache_obstacle_heuristic = cache_obstacle_heuristic;
        info.curve_sample_ratio = curve_sample_ratio;
        info.minimum_turning_radius = (resolution > 0.0) ? (minimum_turning_radius / resolution) : minimum_turning_radius;
        info.non_straight_penalty = non_straight_penalty;
        info.change_penalty = change_penalty;
        info.reverse_penalty = reverse_penalty;
        info.retrospective_penalty = retrospective_penalty;
        info.lookup_table_dim = lookup_table_dim;
        info.analytic_expansion_ratio = analytic_expansion_ratio;
        info.analytic_expansion_max_length = analytic_expansion_max_length;
        info.downsample_obstacle_heuristic = downsample_obstacle_heuristic;
        info.path_min_separation = (resolution > 0.0) ? (path_point_spacing / resolution) : path_point_spacing;
        info.voronoi_heuristic_weight = voronoi_heuristic_weight;
        info.voronoi_cost_weight = voronoi_cost_weight;
        info.voronoi_alpha = voronoi_alpha;
        info.voronoi_d0_max = voronoi_d0_max;

         R_INFO << "Hybrid A*: curve_sample_ratio(step_cells)=" << info.curve_sample_ratio
           << ", path_point_spacing(m)=" << path_point_spacing
           << ", path_min_separation(cells)=" << info.path_min_separation;
        g_planner_ = std::make_shared<HybridAStarPathPlanner>(costmap_ros_, obstacle_factor, info);
        planner_type_ = GRAPH_PLANNER;
      }
      else if (planner_name_ == "sunshine")
      {
        SunshinePathPlanner::Params params;
        private_nh.param(planner_name_ + "/max_iterations", params.max_iterations, 20000);
        private_nh.param(planner_name_ + "/theta_step", params.theta_step, 0.17453292519943295);
        private_nh.param(planner_name_ + "/length_step", params.length_step, 1.0);
        private_nh.param(planner_name_ + "/length_diff_threshold", params.length_diff_threshold, 8.0);
        private_nh.param(planner_name_ + "/forward_distance", params.forward_distance, 6.0);
        private_nh.param(planner_name_ + "/max_ray_length", params.max_ray_length, 200.0);
        private_nh.param(planner_name_ + "/enable_bidirectional_opt", params.enable_bidirectional_opt, true);
        private_nh.param(planner_name_ + "/optimize_iterations", params.optimize_iterations, 5);

        g_planner_ = std::make_shared<SunshinePathPlanner>(costmap_ros_, obstacle_factor, params);
        planner_type_ = SAMPLE_PLANNER;
      }
      else if (planner_name_ == "reachability")
      {
        double min_reachability, cost_weight, reachability_weight, step_base_cost;
        private_nh.param(planner_name_ + "/min_reachability", min_reachability, 0.0);
        private_nh.param(planner_name_ + "/cost_weight", cost_weight, 1.0);
        private_nh.param(planner_name_ + "/reachability_weight", reachability_weight, 5.0);
        private_nh.param(planner_name_ + "/step_base_cost", step_base_cost, 1.0);
        g_planner_ = std::make_shared<ReachabilityPathPlanner>(costmap_ros_, min_reachability, cost_weight,
                                                              reachability_weight, step_base_cost);
        planner_type_ = GRAPH_PLANNER;
      }
    else if (planner_name_ == "voronoi" || planner_name_ == "rhcf" || planner_name_ == "rrt" ||
             planner_name_ == "rrt_star" || planner_name_ == "informed_rrt" || planner_name_ == "bdrp")
    {
      const std::string requested_planner = planner_name_;
      planner_name_ = "a_star";
      R_WARN << "Planner '" << requested_planner
             << "' is not available in this public build; falling back to '" << planner_name_ << "'.";
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else
    {
      R_ERROR << "Unknown planner name: " << planner_name_;
    }

    if (!g_planner_)
    {
      R_ERROR << "Failed to construct global planner for planner_name: " << planner_name_;
      initialized_ = false;
      return;
    }

    R_INFO << "Using path planner: " << planner_name_;

    // pass costmap information to planner (required)
    g_planner_->setFactor(factor_);

    // ========== 根据全局规划方法配置工具链和优化器 ==========
    // 在这一步中，根据不同的规划器类型来选择性启用RDP、优化器和走廊约束
    configurePlannerToolchain();

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);//1
    prune_plan_pub_ = private_nh.advertise<nav_msgs::Path>("prune_plan", 1); ////新增对比看原始路径与剪枝
    plan_opt_pub_ = private_nh.advertise<nav_msgs::Path>("plan_opt", 1);//1
    points_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("key_points", 1);
    lines_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("safety_corridor", 1);
    tree_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("random_tree", 1);
    particles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("particles", 1);
    initial_plan_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("initial_plan", 1000);//新增初始路径数据，对于A*要1000
    initial_plan_pub2_ = private_nh.advertise<geometry_msgs::PoseStamped>("initial_plan2", 100);//新增初始路径数据，对于剪枝不需要1000
    plan_time_pub_ = private_nh.advertise<std_msgs::Float32>("plan_time", 1000);//新增轨迹规划时间发布

    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);

    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &PathPlannerNode::makePlanService, this);

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
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
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
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_ || !g_planner_)
  {
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }

  auto* costmap = g_planner_->getCostMap();
  if (!costmap)
  {
    R_ERROR << "Costmap is null in global planner.";
    return false;
  }

  // start thread mutex
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
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
    R_WARN << "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot "
              "has been properly localized?";
    return false;
  }
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y))
  {
    R_WARN << "The robot's goal position is off the global costmap. Planning will always fail, are you sure the robot "
              "has been properly localized?";
    return false;
  }

  // visualization
  const auto& visualizer = rpp::common::util::VisualizerPtr::Instance();

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();

  // calculate path
  PathPlanner::Points3d origin_path;
  PathPlanner::Points3d expand;
  bool path_found = false;

  auto start_time1 = std::chrono::high_resolution_clock::now(); // 记录规划开始时间    
  // planning
  path_found = g_planner_->plan({ g_start_x, g_start_y, tf2::getYaw(start.pose.orientation) },
                                { g_goal_x, g_goal_y, tf2::getYaw(goal.pose.orientation) }, origin_path, expand);

  R_INFO << "[makePlan] planner=" << planner_name_ << ": origin_path(map units) size=" << origin_path.size();
                          

  // convert path to ros plan
  if (path_found)
  {
          // 统计规划时间
          auto end_time1 = std::chrono::high_resolution_clock::now(); // 记录规划结束时间
          auto planning_time1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - start_time1); // 计算规划时间
          double planning_time_ms1 = planning_time1.count(); 
          // // 创建并发布ROS消息
          std_msgs::Float32 msg;
          msg.data = planning_time_ms1;  // 自动转换double到float
          plan_time_pub_.publish(msg);
          // std::cout << "OPT_Planning time: " << planning_time_ms1 << " astar_ms" << std::endl;
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
      // publishInitialPlan(origin_plan); //Astar

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
      publishInitialPlan2(prune_plan);

      // optimization
      if (optimizer_name_ != "")
      {
        PathPlanner::Points3d path_opt;
        {
          auto end_time2 = std::chrono::high_resolution_clock::now(); // 记录规划结束时间
          
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
          }
          else
          {
            R_ERROR << "Optimizer failed to run, using pruned path instead";
            path_opt = prune_plan;
          }

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
          // 在优化完成后调用publishInitialPlan函数
          publishInitialPlan(path_opt); //opt
          visualizer->publishPlan(path_opt, plan_opt_pub_, frame_id_);
          // 统计规划时间
          auto planning_time2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time2 - start_time1); // 计算规划时间
          double planning_time_ms2 = planning_time2.count(); // 转换为毫秒
          // 创建并发布ROS消息
          std_msgs::Float32 msg;
          msg.data = planning_time_ms2;  // 自动转换double到float
          plan_time_pub_.publish(msg);
          std::cout << "OPT_Planning time: " << planning_time_ms2 << " opt_ms" << std::endl;
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
      
      // publish visulization plan
      if (is_expand_)
      {
        if (planner_type_ == GRAPH_PLANNER)
        {
          // publish expand zone
          visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), expand_pub_, frame_id_);
        }
        else if (planner_type_ == SAMPLE_PLANNER)
        {
          // publish expand tree
          Visualizer::Lines2d tree_lines;
          for (const auto& node : expand)
          {
            // using theta to record parent id element
            if (node.theta() != 0)
            {
              int px_i, py_i;
              double px_d, py_d, x_d, y_d;
              g_planner_->index2Grid(node.theta(), px_i, py_i);
              g_planner_->map2World(px_i, py_i, px_d, py_d);
              g_planner_->map2World(node.x(), node.y(), x_d, y_d);
              tree_lines.emplace_back(
                  std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ x_d, y_d }, { px_d, py_d }));
            }
          }

          // 未选中的采样轨迹：统一颜色
          visualizer->publishLines2d(tree_lines, tree_pub_, frame_id_, "tree", Visualizer::DARK_GREEN, 0.05);

          // 选中的采样轨迹(输出 origin_path 对应的边)：用另一种颜色高亮
          if (origin_path.size() >= 2)
          {
            Visualizer::Lines2d selected_lines;
            selected_lines.reserve(origin_path.size() - 1);
            for (size_t i = 1; i < origin_path.size(); ++i)
            {
              double x0, y0, x1, y1;
              g_planner_->map2World(origin_path[i - 1].x(), origin_path[i - 1].y(), x0, y0);
              g_planner_->map2World(origin_path[i].x(), origin_path[i].y(), x1, y1);
              selected_lines.emplace_back(
                  std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ x0, y0 }, { x1, y1 }));
            }
            visualizer->publishLines2d(selected_lines, tree_pub_, frame_id_, "tree_selected", Visualizer::RED, 0.08);
          }
        }
        else
        {
          R_WARN << "Unknown planner type.";
        }
      }

      visualizer->publishPlan(origin_plan, plan_pub_, frame_id_);
      visualizer->publishPoints(prune_plan, points_pub_, frame_id_, "key_points", Visualizer::PURPLE, 0.15);
      // visualizer->publishPlan(prune_plan, prune_plan_pub_, frame_id_); //新增对比看原始路径与剪枝


      // safety corridor
      if (show_safety_corridor_)
      {
        std::vector<rpp::common::geometry::Polygon2d> polygons;
        rpp::AckermannConfig ackermann_cfg;
        ackermann_cfg.wheelbase = 0.5278;//0.445; //2.5 //0.5278
        ackermann_cfg.max_steer_angle = 0.4143; //0.6 //0.4143
        ackermann_cfg.track_width = 0.5908; //0.360; //1.8 //0.5908
        auto safety_corridor = std::make_unique<rpp::common::safety_corridor::ConvexSafetyCorridor>(costmap_ros_, 0.6, ackermann_cfg); //原0.5
        // auto safety_corridor = std::make_unique<rpp::common::safety_corridor::ConvexSafetyCorridor>(costmap_ros_, 0.5);

        safety_corridor->decompose(prune_plan, polygons);

        Visualizer::Lines2d lines;
        for (const auto& polygon : polygons)
        {
          for (int i = 0; i < polygon.num_points(); i++)
          {
            const auto& pt = polygon.points()[i];
            const auto& next_pt = polygon.points()[polygon.next(i)];
            lines.emplace_back(std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ pt.x(), pt.y() },
                                                                                        { next_pt.x(), next_pt.y() }));
          }
        }
        visualizer->publishLines2d(lines, lines_pub_, frame_id_, "safety_corridor", Visualizer::BLUE, 0.1);
      }
    }
    else
    {
      R_ERROR << "Failed to get a plan from path when a legal path was found. This shouldn't happen.";
    }
  }
  else
  {
    R_ERROR << "Failed to get a path.";
  }
  return !plan.empty();
}

/**
 * @brief Regeister planning service
 * @param req  request from client
 * @param resp response from server
 * @return true
 */
bool PathPlannerNode::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool PathPlannerNode::_getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan)
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

void PathPlannerNode::publishInitialPlan(const PathPlanner::Points3d& plan)
{
    // 只有在尚未记录初始轨迹时才执行记录和发布
    if (!is_initial_plan_recorded_)
    {
        // 清空之前的记录（虽然这是第一次，但保持逻辑一致性）
        initial_plan_.clear();

        // 将路径点存储到initial_plan_中
        for (const auto& pt : plan)
        {
            initial_plan_.push_back(pt);
        }

        // 发布初始轨迹点
        for (const auto& pt : initial_plan_)
        {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id_;
            msg.pose.position.x = pt.x();
            msg.pose.position.y = pt.y();
            msg.pose.position.z = 0.0;
            msg.pose.orientation.w = 1.0;
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;

            initial_plan_pub_.publish(msg);
        }

        // 设置标志变量，表示已经记录了初始轨迹
        is_initial_plan_recorded_ = true;
    }
}
void PathPlannerNode::publishInitialPlan2(const PathPlanner::Points3d& plan)
{
    // 只有在尚未记录初始轨迹时才执行记录和发布
    if (!is_initial_plan_recorded2_)
    {
        // 清空之前的记录（虽然这是第一次，但保持逻辑一致性）
        initial_plan2_.clear();

        // 将路径点存储到initial_plan_中
        for (const auto& pt : plan)
        {
            initial_plan2_.push_back(pt);
        }

        // 发布初始轨迹点
        for (const auto& pt : initial_plan2_)
        {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id_;
            msg.pose.position.x = pt.x();
            msg.pose.position.y = pt.y();
            msg.pose.position.z = 0.0;
            msg.pose.orientation.w = 1.0;
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;

            initial_plan_pub2_.publish(msg);
        }

        // 设置标志变量，表示已经记录了初始轨迹
        is_initial_plan_recorded2_ = true;
    }
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
 *  - A*: RDP=ON + minimum_snap优化 + 走廊=ON
 *  - Hybrid A*: RDP=OFF + LBFGS优化 + 走廊=OFF
 *  - Sunshine: RDP=OFF + conjugate梯度优化 + 走廊=OFF
 *  - 采样规划器: RDP=OFF + 无优化 + 走廊=OFF
 */
void PathPlannerNode::configurePlannerToolchain()
{
  ros::NodeHandle private_nh("~/" + planner_name_);

  // 优先检查是否有外部（launch/arg）强制设置的 optimizer 参数。
  // 如果存在，则根据该参数直接初始化对应的优化器，并在后续分支中避免被覆盖。
  bool optimizer_forced = false;
  std::string forced_optimizer;
  private_nh.param("/move_base/PathPlanner/optimizer_name", forced_optimizer, std::string(""));
  if (!forced_optimizer.empty())
  {
    R_INFO << "[PlannerToolchain] Forced optimizer param detected: " << forced_optimizer;
    if (forced_optimizer == "conjugate_gradient")
    {
      int max_iter;
      double alpha, obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;

      // Choose defaults depending on upstream planner
      // If planner is a_star, use values from conjugate_gradient_params.yaml (more conservative)
      // else use the larger/default values intended for other planners
      if (planner_name_ == "a_star")
      {
        // defaults matching conjugate_gradient_params.yaml
        private_nh.param("/move_base/Optimizer/max_iter", max_iter, 100);
        private_nh.param("/move_base/Optimizer/alpha", alpha, 0.1);
        private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 0.3);
        private_nh.param("/move_base/Optimizer/k_max", k_max, 0.15);
        private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 0.5);
        private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 0.25);
        private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 0.25);
        R_INFO << "[PlannerToolchain] Using conjugate_gradient defaults for A* (from conjugate_gradient_params.yaml)";
      }
      else
      {
        // defaults for non-A* planners (larger/looser defaults)
        private_nh.param("/move_base/Optimizer/max_iter", max_iter, 100);
        private_nh.param("/move_base/Optimizer/alpha", alpha, 1.0);
        private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 2.0);
        private_nh.param("/move_base/Optimizer/k_max", k_max, 4.0);
        private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
        private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 2.0);
        private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 4.0);
        R_INFO << "[PlannerToolchain] Using conjugate_gradient defaults for non-A* planner";
      }

        optimizer_ = std::make_shared<rpp::trajectory_optimization::CGOptimizer>(
          costmap_ros_, max_iter, alpha, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "conjugate_gradient";
      R_INFO << "  ✓ Optimizer (forced): conjugate_gradient";
      optimizer_forced = true;
    }
    else if (forced_optimizer == "lbfgs" || forced_optimizer == "lbfgs_optimizer")
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
    else if (forced_optimizer == "minimumsnap_optimizer" || forced_optimizer == "minimumsnap")
    {
      int max_iter;
      double vel_max, acc_max, jerk_max;
      rpp::AckermannConfig ackermann_cfg;
      ackermann_cfg.wheelbase = 0.5278;
      ackermann_cfg.max_steer_angle = 0.4143;
      ackermann_cfg.track_width = 0.5908;
      double safety_range = 0.6;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 100);
      private_nh.param("/move_base/Optimizer/vel_max", vel_max, 1.0);
      private_nh.param("/move_base/Optimizer/acc_max", acc_max, 2.0);
      private_nh.param("/move_base/Optimizer/jerk_max", jerk_max, 4.0);
        optimizer_ = std::make_shared<rpp::trajectory_optimization::MinimumsnapOptimizer>(
          costmap_ros_, max_iter, vel_max, acc_max, jerk_max, safety_range, ackermann_cfg);
      optimizer_name_ = "minimumsnap_optimizer";
      R_INFO << "  ✓ Optimizer (forced): minimumsnap_optimizer";
      optimizer_forced = true;
    }
    else if (forced_optimizer == "splinetrajectory" || forced_optimizer == "splinetrajectory_optimizer" ||
             forced_optimizer == "spline_trajectory" || forced_optimizer == "spline_trajectory_optimizer")
    {
      // NOTE: SplineTrajectoryOptimizer implementation is intentionally not shipped in the public repo.
      // Keep move_base/pluginlib loadable by falling back to an available optimizer.
      R_WARN << "[PlannerToolchain] forced_optimizer='" << forced_optimizer
             << "' is not available in this public build; falling back to lbfgs.";

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
    else if (forced_optimizer == "minco" || forced_optimizer == "minco_spline_optimizer" ||
             forced_optimizer == "minco_spline" || forced_optimizer == "minco_spline_opt")
    {
      // NOTE: MincoSplineOptimizer implementation is intentionally not shipped in the public repo.
      // Keep move_base/pluginlib loadable by falling back to an available optimizer.
      R_WARN << "[PlannerToolchain] forced_optimizer='" << forced_optimizer
             << "' is not available in this public build; falling back to lbfgs.";

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
    pruner_ = std::make_shared<RDPPathProcessor>(0.22);
    R_INFO << "  ✓ RDP Path Processor ENABLED (threshold=0.22)";

    if (!optimizer_forced)
    {
      // // 启用minimum_snap优化器
      int max_iter;
      double vel_max, acc_max, jerk_max;
      rpp::AckermannConfig ackermann_cfg;
      ackermann_cfg.wheelbase = 0.5278;
      ackermann_cfg.max_steer_angle = 0.4143;
      ackermann_cfg.track_width = 0.5908;
      double safety_range = 0.6;

      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 100);
      private_nh.param("/move_base/Optimizer/vel_max", vel_max, 1.0);
      private_nh.param("/move_base/Optimizer/acc_max", acc_max, 2.0);
      private_nh.param("/move_base/Optimizer/jerk_max", jerk_max, 4.0);
      optimizer_ = std::make_shared<rpp::trajectory_optimization::MinimumsnapOptimizer>(
        costmap_ros_, max_iter, vel_max, acc_max, jerk_max, safety_range, ackermann_cfg);
      optimizer_name_ = "minimumsnap_optimizer";
      R_INFO << "  ✓ Optimizer: minimumsnap_optimizer";
    }

    // 启用安全走廊
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor ENABLED";
  }
  else if (planner_name_ == "hybrid_astar")
  {
    // ========== Hybrid A* ==========
    R_INFO << "[PlannerToolchain] Configuring for Hybrid A*...";

    // ✅ FIX: 启用RDP路径剪枝，但使用较小的delta（0.1m）
    // 原问题：混合A*输出225个密集点，导致LBFGS有450个优化变量，Line Search失败
    // 解决方案：用RDP将225个点简化为80-120个关键点，LBFGS优化200-240个变量，正好在最优范围内
    pruner_ = std::make_shared<RDPPathProcessor>(0.03);
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
      // 启用conjugate梯度优化器
      int max_iter;
      double alpha, obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      private_nh.param("/move_base/Optimizer/max_iter", max_iter, 100);
      private_nh.param("/move_base/Optimizer/alpha", alpha, 1.0);
      private_nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max, 2.0);
      private_nh.param("/move_base/Optimizer/k_max", k_max, 4.0);
      private_nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 1.0);
      private_nh.param("/move_base/Optimizer/w_smooth", w_smooth, 2.0);
      private_nh.param("/move_base/Optimizer/w_curvature", w_curvature, 4.0);
      optimizer_ = std::make_shared<rpp::trajectory_optimization::CGOptimizer>(
        costmap_ros_, max_iter, alpha, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
      optimizer_name_ = "conjugate_gradient";
      R_INFO << "  ✓ Optimizer: conjugate_gradient";
    }

    // 禁用安全走廊
    use_safety_corridor_ = false;
    R_INFO << "  ✓ Safety Corridor DISABLED";
  }
  else if (planner_name_ == "rhcf")
  {
    // ========== RHCF (Voronoi Random Walk) ==========
    R_INFO << "[PlannerToolchain] Configuring for RHCF...";

    // RHCF 输出通常是 Voronoi 骨架上的折线（cell-by-cell），点数偏多且转折生硬。
    // 使用 RDP 先做关键点提取，再交给优化器做平滑/避障，可得到更“可执行”的全局参考路径。
    pruner_ = std::make_shared<RDPPathProcessor>(0.05);
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
  }
}

}  // namespace path_planner
}  // namespace rpp