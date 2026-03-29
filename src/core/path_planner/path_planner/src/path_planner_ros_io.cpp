#include "path_planner/path_planner_ros_io.h"

#include <cmath>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "common/util/visualizer.h"
#include "common/geometry/polygon2d.h"

namespace rpp
{
namespace path_planner
{
using Visualizer = rpp::common::util::Visualizer;

void PathPlannerRosIO::initialize(const std::string& instance_name, PathPlannerEngine* engine)
{
  if (initialized_)
  {
    ROS_WARN("PathPlannerRosIO: already initialized (instance=%s)", instance_name_.c_str());
    return;
  }

  instance_name_ = instance_name;
  engine_ = engine;

  ros::NodeHandle private_nh("~/" + instance_name_);

  std::string io_namespace;
  private_nh.param("io_namespace", io_namespace, std::string("PathPlanner"));
  io_nh_ = io_namespace.empty() ? ros::NodeHandle("~") : ros::NodeHandle("~/" + io_namespace);

  plan_pub_ = io_nh_.advertise<nav_msgs::Path>("plan", 1, true);
  prune_plan_pub_ = io_nh_.advertise<nav_msgs::Path>("prune_plan", 1, true);
  plan_opt_pub_ = io_nh_.advertise<nav_msgs::Path>("plan_opt", 1, true);
  points_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("key_points", 1);
  lines_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("safety_corridor", 1);
  tree_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("random_tree", 1);
  particles_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("particles", 1, true);
  nag_candidates_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("nag_candidates", 1, true);
  nag_cutpoints_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("nag_cutpoints", 1, true);
  nag_cpr_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("nag_cpr", 1, true);
  raystar_candidates_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("non_homotopic_paths", 1, true);
  raystar_poly_obstacles_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("poly_obstacles", 1, true);
  gkvm_landmarks_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("gkvm_landmarks", 1, true);
  gkvm_route_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("gkvm_route", 1, true);
  rcc_marker_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("rcc_debug", 1);
  parallel_curves_marker_pub_ = io_nh_.advertise<visualization_msgs::MarkerArray>("parallel_curves", 1, true);
  initial_plan_pub_ = io_nh_.advertise<geometry_msgs::PoseStamped>("initial_plan", 1000);
  initial_plan_pub2_ = io_nh_.advertise<geometry_msgs::PoseStamped>("initial_plan2", 100);
  plan_time_pub_ = io_nh_.advertise<std_msgs::Float32>("plan_time", 1000);

  expand_pub_ = io_nh_.advertise<nav_msgs::OccupancyGrid>("expand", 1);

  // Use WARN so these lines are still visible when move_base is configured with rosconsole>=WARN.
  ROS_WARN("PathPlannerRosIO: instance=~/%s io_namespace=%s", instance_name_.c_str(),
           io_namespace.empty() ? std::string("<empty>").c_str() : io_namespace.c_str());
  ROS_WARN("PathPlannerRosIO topics: plan=%s plan_opt=%s expand=%s", plan_pub_.getTopic().c_str(),
           plan_opt_pub_.getTopic().c_str(), expand_pub_.getTopic().c_str());

  make_plan_srv_ = io_nh_.advertiseService("make_plan", &PathPlannerRosIO::makePlanService, this);

  initialized_ = true;
}

bool PathPlannerRosIO::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  if (!engine_)
  {
    ROS_ERROR("PathPlannerRosIO: makePlan service called but engine_ is null");
    return false;
  }

  engine_->makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = engine_->frameId();

  publish();
  return true;
}

void PathPlannerRosIO::publish()
{
  if (!initialized_ || !engine_ || !engine_->isInitialized())
  {
    return;
  }

  const auto& dbg = engine_->debugData();
  const auto& frame_id = engine_->frameId();
  const auto& visualizer = rpp::common::util::VisualizerPtr::Instance();

  // Timing
  if (dbg.planning_time_ms > 0.0)
  {
    std_msgs::Float32 msg;
    msg.data = static_cast<float>(dbg.planning_time_ms);
    plan_time_pub_.publish(msg);

    if (!dbg.optimized_plan_world.empty() && dbg.has_total_time && dbg.total_time_ms > 0.0)
    {
      std_msgs::Float32 msg2;
      msg2.data = static_cast<float>(dbg.total_time_ms);
      plan_time_pub_.publish(msg2);
    }
  }

  // Plans
  if (!dbg.origin_plan_world.empty())
  {
    visualizer->publishPlan(dbg.origin_plan_world, plan_pub_, frame_id);
  }

  if (!dbg.optimized_plan_world.empty())
  {
    visualizer->publishPlan(dbg.optimized_plan_world, plan_opt_pub_, frame_id);
  }

  if (!dbg.prune_plan_world.empty())
  {
    visualizer->publishPoints(dbg.prune_plan_world, points_pub_, frame_id, "key_points", Visualizer::PURPLE, 0.15);
    // visualizer->publishPlan(dbg.prune_plan_world, prune_plan_pub_, frame_id);
  }

  // Initial plan records (publish only once)
  if (!is_initial_plan_recorded2_ && !dbg.prune_plan_world.empty())
  {
    initial_plan2_.assign(dbg.prune_plan_world.begin(), dbg.prune_plan_world.end());
    for (const auto& pt : initial_plan2_)
    {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = frame_id;
      msg.pose.position.x = pt.x();
      msg.pose.position.y = pt.y();
      msg.pose.position.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      initial_plan_pub2_.publish(msg);
    }
    is_initial_plan_recorded2_ = true;
  }

  if (!is_initial_plan_recorded_ && !dbg.optimized_plan_world.empty())
  {
    initial_plan_.assign(dbg.optimized_plan_world.begin(), dbg.optimized_plan_world.end());
    for (const auto& pt : initial_plan_)
    {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = frame_id;
      msg.pose.position.x = pt.x();
      msg.pose.position.y = pt.y();
      msg.pose.position.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      initial_plan_pub_.publish(msg);
    }
    is_initial_plan_recorded_ = true;
  }

  // Expand visualization
  if (engine_->isExpandEnabled())
  {
    if (engine_->plannerType() == PathPlannerEngine::GRAPH_PLANNER)
    {
      auto* costmap_ros = engine_->costmapRos();
      if (costmap_ros && costmap_ros->getCostmap())
      {
        visualizer->publishExpandZone(dbg.expand_map, costmap_ros->getCostmap(), expand_pub_, frame_id);
      }
    }
    else if (engine_->plannerType() == PathPlannerEngine::SAMPLE_PLANNER)
    {
      auto* planner = engine_->planner();
      if (planner)
      {
        Visualizer::Lines2d tree_lines;
        for (const auto& node : dbg.expand_map)
        {
          if (node.theta() != 0)
          {
            int px_i, py_i;
            double px_d, py_d, x_d, y_d;
            planner->index2Grid(static_cast<int>(node.theta()), px_i, py_i);
            planner->map2World(px_i, py_i, px_d, py_d);
            planner->map2World(node.x(), node.y(), x_d, y_d);
            tree_lines.emplace_back(
                std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ x_d, y_d }, { px_d, py_d }));
          }
        }
        visualizer->publishLines2d(tree_lines, tree_pub_, frame_id, "tree", Visualizer::DARK_GREEN, 0.05);

        if (dbg.origin_path_map.size() >= 2)
        {
          Visualizer::Lines2d selected_lines;
          selected_lines.reserve(dbg.origin_path_map.size() - 1);
          for (size_t i = 1; i < dbg.origin_path_map.size(); ++i)
          {
            double x0, y0, x1, y1;
            planner->map2World(dbg.origin_path_map[i - 1].x(), dbg.origin_path_map[i - 1].y(), x0, y0);
            planner->map2World(dbg.origin_path_map[i].x(), dbg.origin_path_map[i].y(), x1, y1);
            selected_lines.emplace_back(
                std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ x0, y0 }, { x1, y1 }));
          }
          visualizer->publishLines2d(selected_lines, tree_pub_, frame_id, "tree_selected", Visualizer::RED, 0.08);
        }
      }
    }
  }

  // RCC debug markers
  if (engine_->plannerName() == "rolling_circle_center" || engine_->plannerName() == "rcc")
  {
    auto* planner = engine_->planner();
    auto* costmap_ros = engine_->costmapRos();
    const double resolution =
        (costmap_ros && costmap_ros->getCostmap()) ? costmap_ros->getCostmap()->getResolution() : 0.0;

    if (planner && resolution > 0.0)
    {
      PathPlanner::Points3d circles_world;
      circles_world.reserve(dbg.expand_map.size());

      for (const auto& c : dbg.expand_map)
      {
        double wx, wy;
        planner->map2World(c.x(), c.y(), wx, wy);
        const double r_cells = std::max(0.0, c.theta());
        const double r_m = r_cells * resolution;
        circles_world.emplace_back(wx, wy, r_m);
      }

      std_msgs::ColorRGBA centers_color;
      centers_color.r = 1.0;
      centers_color.g = 0.2;
      centers_color.b = 0.2;
      centers_color.a = 1.0;

      std_msgs::ColorRGBA disks_color;
      disks_color.r = 0.2;
      disks_color.g = 0.9;
      disks_color.b = 0.2;
      disks_color.a = 0.12;

      Visualizer::publishVariableRadiusDisks2d(circles_world, rcc_marker_pub_, frame_id, "rcc", centers_color,
                                              disks_color, 0.06, 0.01, 200, true,
                                              true /* disks_color_gradient */);
    }
  }

  // Parallel Curves debug markers (concentric curves)
  if (!dbg.parallel_curves_circles_world.empty())
  {
    std_msgs::ColorRGBA centers_color;
    centers_color.r = 0.9;
    centers_color.g = 0.2;
    centers_color.b = 0.9;
    centers_color.a = 1.0;

    std_msgs::ColorRGBA disks_color;
    disks_color.r = 0.0;
    disks_color.g = 1.0;
    disks_color.b = 0.0;
    disks_color.a = 0.12;

    Visualizer::publishVariableRadiusDisks2d(dbg.parallel_curves_circles_world, parallel_curves_marker_pub_, frame_id,
                                            "parallel_curves", centers_color, disks_color, 0.06, 0.01, 200, true,
                                            false);
  }

  // RHCF debug markers (published to particles topic, kept for compatibility)
  if (!dbg.rhcf_candidate_paths_world.empty())
  {
    const int best_idx = dbg.rhcf_best_candidate_index;

    visualization_msgs::MarkerArray arr;
    arr.markers.reserve(dbg.rhcf_candidate_paths_world.size());

    auto palette = [](int i) {
      std_msgs::ColorRGBA c;
      c.a = 0.9;
      switch (i % 6)
      {
        case 0:
          c.r = 0.0;
          c.g = 0.8;
          c.b = 1.0;
          break;
        case 1:
          c.r = 0.2;
          c.g = 1.0;
          c.b = 0.2;
          break;
        case 2:
          c.r = 1.0;
          c.g = 0.6;
          c.b = 0.0;
          break;
        case 3:
          c.r = 0.7;
          c.g = 0.2;
          c.b = 1.0;
          break;
        case 4:
          c.r = 1.0;
          c.g = 1.0;
          c.b = 0.2;
          break;
        default:
          c.r = 0.9;
          c.g = 0.2;
          c.b = 0.2;
          break;
      }
      return c;
    };

    for (size_t i = 0; i < dbg.rhcf_candidate_paths_world.size(); ++i)
    {
      const auto& pts = dbg.rhcf_candidate_paths_world[i];
      if (pts.size() < 2)
        continue;

      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = ros::Time::now();
      m.ns = "rhcf_candidates";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.lifetime = ros::Duration(0.0);
      m.scale.x = (static_cast<int>(i) == best_idx) ? 0.08 : 0.04;

      if (static_cast<int>(i) == best_idx)
      {
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
      }
      else
      {
        m.color = palette(static_cast<int>(i));
      }

      m.points.reserve(pts.size());
      for (const auto& p : pts)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.0;
        m.points.push_back(gp);
      }
      arr.markers.push_back(std::move(m));
    }

    if (!arr.markers.empty())
      particles_pub_.publish(arr);
  }

  // NAG debug markers
  if (!dbg.nag_candidate_paths_world.empty() || !dbg.nag_cut_points_world.empty() || !dbg.nag_cpr_cells_world.empty())
  {
    const int best_idx = dbg.nag_best_candidate_index;

    visualization_msgs::MarkerArray arr;
    arr.markers.reserve(dbg.nag_candidate_paths_world.size() + 4);
    visualization_msgs::MarkerArray cand_arr;
    visualization_msgs::MarkerArray cut_arr;
    visualization_msgs::MarkerArray cpr_arr;

    visualization_msgs::Marker clear_m;
    clear_m.header.frame_id = frame_id;
    clear_m.header.stamp = ros::Time::now();
    clear_m.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(clear_m);
    cand_arr.markers.push_back(clear_m);
    cut_arr.markers.push_back(clear_m);
    cpr_arr.markers.push_back(clear_m);

    auto palette = [](int i) {
      std_msgs::ColorRGBA c;
      c.a = 0.92f;
      switch (i % 6)
      {
        case 0:
          c.r = 0.1f;
          c.g = 0.8f;
          c.b = 1.0f;
          break;
        case 1:
          c.r = 0.4f;
          c.g = 1.0f;
          c.b = 0.2f;
          break;
        case 2:
          c.r = 1.0f;
          c.g = 0.7f;
          c.b = 0.2f;
          break;
        case 3:
          c.r = 0.8f;
          c.g = 0.4f;
          c.b = 1.0f;
          break;
        case 4:
          c.r = 1.0f;
          c.g = 1.0f;
          c.b = 0.3f;
          break;
        default:
          c.r = 0.95f;
          c.g = 0.35f;
          c.b = 0.35f;
          break;
      }
      return c;
    };

    for (size_t i = 0; i < dbg.nag_candidate_paths_world.size(); ++i)
    {
      const auto& pts = dbg.nag_candidate_paths_world[i];
      if (pts.size() < 2)
        continue;

      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = ros::Time::now();
      m.ns = "nag_candidates";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.lifetime = ros::Duration(0.0);
      m.scale.x = (static_cast<int>(i) == best_idx) ? 0.09 : 0.05;

      if (static_cast<int>(i) == best_idx)
      {
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
      }
      else
      {
        m.color = palette(static_cast<int>(i));
      }

      m.points.reserve(pts.size());
      for (const auto& p : pts)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.03;
        m.points.push_back(gp);
      }

      arr.markers.push_back(std::move(m));
      cand_arr.markers.push_back(arr.markers.back());
    }

    if (!dbg.nag_cut_points_world.empty())
    {
      visualization_msgs::Marker cp;
      cp.header.frame_id = frame_id;
      cp.header.stamp = ros::Time::now();
      cp.ns = "nag_cutpoints";
      cp.id = 10000;
      cp.type = visualization_msgs::Marker::SPHERE_LIST;
      cp.action = visualization_msgs::Marker::ADD;
      cp.pose.orientation.w = 1.0;
      cp.scale.x = 0.18;
      cp.scale.y = 0.18;
      cp.scale.z = 0.18;
      cp.color.r = 1.0f;
      cp.color.g = 0.2f;
      cp.color.b = 0.9f;
      cp.color.a = 0.95f;

      cp.points.reserve(dbg.nag_cut_points_world.size());
      for (const auto& p : dbg.nag_cut_points_world)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.12;
        cp.points.push_back(gp);
      }
      arr.markers.push_back(std::move(cp));
      cut_arr.markers.push_back(arr.markers.back());
    }

    if (!dbg.nag_cpr_cells_world.empty())
    {
      visualization_msgs::Marker cpr;
      cpr.header.frame_id = frame_id;
      cpr.header.stamp = ros::Time::now();
      cpr.ns = "nag_cpr";
      cpr.id = 10001;
      cpr.type = visualization_msgs::Marker::POINTS;
      cpr.action = visualization_msgs::Marker::ADD;
      cpr.pose.orientation.w = 1.0;
      cpr.scale.x = 0.03;
      cpr.scale.y = 0.03;
      cpr.color.r = 0.95f;
      cpr.color.g = 0.95f;
      cpr.color.b = 0.1f;
      cpr.color.a = 0.65f;

      cpr.points.reserve(dbg.nag_cpr_cells_world.size());
      for (const auto& p : dbg.nag_cpr_cells_world)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.06;
        cpr.points.push_back(gp);
      }
      arr.markers.push_back(std::move(cpr));
      cpr_arr.markers.push_back(arr.markers.back());
    }

    if (!arr.markers.empty())
      particles_pub_.publish(arr);
    nag_candidates_pub_.publish(cand_arr);
    nag_cutpoints_pub_.publish(cut_arr);
    nag_cpr_pub_.publish(cpr_arr);
  }

  // RayStar debug markers (topic names match upstream Ray* for easy RViz reuse)
  if (!dbg.raystar_candidate_paths_world.empty() || !dbg.raystar_poly_obstacles_world.empty())
  {
    visualization_msgs::MarkerArray cand_arr;
    visualization_msgs::MarkerArray obs_arr;

    visualization_msgs::Marker clear_m;
    clear_m.header.frame_id = frame_id;
    clear_m.header.stamp = ros::Time::now();
    clear_m.action = visualization_msgs::Marker::DELETEALL;
    cand_arr.markers.push_back(clear_m);
    obs_arr.markers.push_back(clear_m);

    const int best_idx = dbg.raystar_best_candidate_index;

    auto palette = [](int i) {
      std_msgs::ColorRGBA c;
      c.a = 0.92f;
      switch (i % 6)
      {
        case 0:
          c.r = 0.1f;
          c.g = 0.8f;
          c.b = 1.0f;
          break;
        case 1:
          c.r = 0.4f;
          c.g = 1.0f;
          c.b = 0.2f;
          break;
        case 2:
          c.r = 1.0f;
          c.g = 0.7f;
          c.b = 0.2f;
          break;
        case 3:
          c.r = 0.8f;
          c.g = 0.4f;
          c.b = 1.0f;
          break;
        case 4:
          c.r = 1.0f;
          c.g = 1.0f;
          c.b = 0.3f;
          break;
        default:
          c.r = 0.95f;
          c.g = 0.35f;
          c.b = 0.35f;
          break;
      }
      return c;
    };

    // Candidate paths
    for (size_t i = 0; i < dbg.raystar_candidate_paths_world.size(); ++i)
    {
      const auto& pts = dbg.raystar_candidate_paths_world[i];
      if (pts.size() < 2)
        continue;

      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = ros::Time::now();
      m.ns = "raystar_candidates";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.lifetime = ros::Duration(0.0);
      m.scale.x = (static_cast<int>(i) == best_idx) ? 0.10 : 0.05;

      if (static_cast<int>(i) == best_idx)
      {
        m.color.r = 1.0f;
        m.color.g = 0.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
      }
      else
      {
        m.color = palette(static_cast<int>(i));
      }

      m.points.reserve(pts.size());
      for (const auto& p : pts)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.04;
        m.points.push_back(gp);
      }
      cand_arr.markers.push_back(std::move(m));
    }

    // Poly obstacles (debug): each poly is a closed world polyline
    for (size_t i = 0; i < dbg.raystar_poly_obstacles_world.size(); ++i)
    {
      const auto& poly = dbg.raystar_poly_obstacles_world[i];
      if (poly.size() < 2)
        continue;

      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = ros::Time::now();
      m.ns = "raystar_poly_obstacles";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.lifetime = ros::Duration(0.0);
      m.scale.x = 0.03;
      m.color.r = 0.0f;
      m.color.g = 0.9f;
      m.color.b = 1.0f;
      m.color.a = 0.8f;

      m.points.reserve(poly.size());
      for (const auto& p : poly)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.02;
        m.points.push_back(gp);
      }
      obs_arr.markers.push_back(std::move(m));
    }

    raystar_candidates_pub_.publish(cand_arr);
    raystar_poly_obstacles_pub_.publish(obs_arr);
  }

  // GKVM debug markers (informative landmarks + ordered route)
  const bool has_gkvm = !dbg.gkvm_landmarks_world.empty() || !dbg.gkvm_route_world.empty();
  if (has_gkvm || has_published_gkvm_)
  {
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::MarkerArray landmarks_arr;
    visualization_msgs::MarkerArray route_arr;

    visualization_msgs::Marker clear_m;
    clear_m.header.frame_id = frame_id;
    clear_m.header.stamp = stamp;
    clear_m.action = visualization_msgs::Marker::DELETEALL;
    landmarks_arr.markers.push_back(clear_m);
    route_arr.markers.push_back(clear_m);

    if (!dbg.gkvm_landmarks_world.empty())
    {
      visualization_msgs::Marker lm;
      lm.header.frame_id = frame_id;
      lm.header.stamp = stamp;
      lm.ns = "gkvm_landmarks";
      lm.id = 0;
      lm.type = visualization_msgs::Marker::SPHERE_LIST;
      lm.action = visualization_msgs::Marker::ADD;
      lm.pose.orientation.w = 1.0;
      lm.lifetime = ros::Duration(0.0);
      lm.scale.x = 0.16;
      lm.scale.y = 0.16;
      lm.scale.z = 0.16;
      lm.color.r = 1.0f;
      lm.color.g = 0.0f;
      lm.color.b = 1.0f;
      lm.color.a = 0.9f;

      lm.points.reserve(dbg.gkvm_landmarks_world.size());
      for (const auto& p : dbg.gkvm_landmarks_world)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.12;
        lm.points.push_back(gp);
      }

      landmarks_arr.markers.push_back(std::move(lm));
    }

    if (!dbg.gkvm_route_world.empty())
    {
      std::vector<geometry_msgs::Point> route_pts;
      route_pts.reserve(dbg.gkvm_route_world.size());
      for (const auto& p : dbg.gkvm_route_world)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.03;
        route_pts.push_back(gp);
      }

      visualization_msgs::Marker line;
      line.header.frame_id = frame_id;
      line.header.stamp = stamp;
      line.ns = "gkvm_route";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.lifetime = ros::Duration(0.0);
      line.scale.x = 0.07;
      line.color.r = 1.0f;
      line.color.g = 0.9f;
      line.color.b = 0.1f;
      line.color.a = 0.95f;

      line.points = route_pts;

      visualization_msgs::Marker wp;
      wp.header.frame_id = frame_id;
      wp.header.stamp = stamp;
      wp.ns = "gkvm_route_waypoints";
      wp.id = 1;
      wp.type = visualization_msgs::Marker::SPHERE_LIST;
      wp.action = visualization_msgs::Marker::ADD;
      wp.pose.orientation.w = 1.0;
      wp.lifetime = ros::Duration(0.0);
      wp.scale.x = 0.12;
      wp.scale.y = 0.12;
      wp.scale.z = 0.12;
      wp.color.r = 1.0f;
      wp.color.g = 0.5f;
      wp.color.b = 0.1f;
      wp.color.a = 0.95f;
      wp.points = route_pts;
      for (auto& p : wp.points)
        p.z = 0.07;

      route_arr.markers.push_back(std::move(line));
      route_arr.markers.push_back(std::move(wp));
    }

    gkvm_landmarks_pub_.publish(landmarks_arr);
    gkvm_route_pub_.publish(route_arr);

    has_published_gkvm_ = has_gkvm;
  }

  // Safety corridor visualization
  if (engine_->shouldShowSafetyCorridor())
  {
    // Safety corridor feature is not available in this repository variant.
    // Publish a DELETEALL marker to clear any previously displayed corridor in RViz.
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(m);
    lines_pub_.publish(arr);
  }
}

}  // namespace path_planner
}  // namespace rpp
