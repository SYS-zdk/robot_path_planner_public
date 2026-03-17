
/**
 * *********************************************************
 *
 * @file: ilqr_controller.cpp
 * @brief: Contains the iterative LQR (iLQR) local controller class
 * @author: Zhang Dingkun (generated)
 * @date: 2026-02-04
 * @version: 1.0
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <limits>
#include <queue>

#include "controller/ilqr_controller.h"

#include "costmap_2d/layered_costmap.h"
#include "distance_layer.h"

PLUGINLIB_EXPORT_CLASS(rpp::controller::ILQRController, nav_core::BaseLocalPlanner)

namespace rpp
{
namespace controller
{
namespace
{
inline double sqr(const double x)
{
  return x * x;
}

template <typename T>
inline T clampValue(const T& v, const T& lo, const T& hi)
{
  return std::min(hi, std::max(lo, v));
}

inline double angleDiff(double a, double b)
{
  // a-b in [-pi, pi]
  double d = a - b;
  d = d - 2.0 * M_PI * std::floor((d + M_PI) / (2.0 * M_PI));
  return d;
}
}  // namespace

ILQRController::ILQRController() : Controller()
{
}

ILQRController::ILQRController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : ILQRController()
{
  initialize(name, tf, costmap_ros);
}

ILQRController::~ILQRController() = default;

void ILQRController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("ILQR controller has already been initialized.");
    return;
  }

  initialized_ = true;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

  // set costmap properties for Controller::worldToMap
  setSize(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  setOrigin(costmap->getOriginX(), costmap->getOriginY());
  setResolution(costmap->getResolution());

  ros::NodeHandle nh("~/" + name);

  // base
  nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
  nh.param("rotate_tolerance", rotate_tol_, 0.5);
  nh.param("convert_offset", convert_offset_, 0.0);
  nh.param("base_frame", base_frame_, base_frame_);
  nh.param("map_frame", map_frame_, map_frame_);

  // lookahead
  nh.param("lookahead_time", lookahead_time_, 0.8);
  nh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
  nh.param("max_lookahead_dist", max_lookahead_dist_, 1.2);

  // linear velocity
  nh.param("max_v", max_v_, 0.6);
  nh.param("min_v", min_v_, 0.0);
  nh.param("max_v_inc", max_v_inc_, 0.6);

  // angular velocity
  nh.param("max_w", max_w_, 1.57);
  nh.param("min_w", min_w_, 0.0);
  nh.param("max_w_inc", max_w_inc_, 1.57);

  // iLQR params
  nh.param("horizon_steps", params_.horizon_steps, params_.horizon_steps);
  nh.param("max_iter", params_.max_iter, params_.max_iter);
  nh.param("lambda_init", params_.lambda_init, params_.lambda_init);
  nh.param("lambda_max", params_.lambda_max, params_.lambda_max);

  nh.param("v_ref", params_.v_ref, params_.v_ref);
  nh.param("ds_ref", params_.ds_ref, params_.ds_ref);

  nh.param("w_track", params_.w_track, params_.w_track);
  nh.param("w_heading", params_.w_heading, params_.w_heading);
  nh.param("w_u", params_.w_u, params_.w_u);
  nh.param("w_du", params_.w_du, params_.w_du);
  nh.param("w_obs", params_.w_obs, params_.w_obs);

  nh.param("use_corridor", params_.use_corridor, params_.use_corridor);
  nh.param("corridor_hard_constraint", params_.corridor_hard_constraint, params_.corridor_hard_constraint);
  nh.param("w_corridor", params_.w_corridor, params_.w_corridor);
  nh.param("corridor_safety_range", params_.corridor_safety_range, params_.corridor_safety_range);
  nh.param("corridor_wheelbase", params_.corridor_wheelbase, params_.corridor_wheelbase);
  nh.param("corridor_max_steer_angle", params_.corridor_max_steer_angle, params_.corridor_max_steer_angle);
  nh.param("corridor_track_width", params_.corridor_track_width, params_.corridor_track_width);

  nh.param("obs_cost_scale", params_.obs_cost_scale, params_.obs_cost_scale);
  nh.param("obs_cost_gain", params_.obs_cost_gain, params_.obs_cost_gain);
  nh.param("obs_cost_threshold", params_.obs_cost_threshold, params_.obs_cost_threshold);
  nh.param("obs_collision_cost", params_.obs_collision_cost, params_.obs_collision_cost);

  // Optional ESDF (DistanceLayer) support
  nh.param("use_esdf", use_esdf_, use_esdf_);
  nh.param("esdf_sigma_m", esdf_sigma_m_, esdf_sigma_m_);
  nh.param("esdf_collision_dist_m", esdf_collision_dist_m_, esdf_collision_dist_m_);

  distance_layer_ = nullptr;
  if (costmap_ros_ && costmap_ros_->getLayeredCostmap() && costmap_ros_->getLayeredCostmap()->getPlugins())
  {
    for (auto it = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
         it != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++it)
    {
      costmap_2d::Layer* layer = it->get();
      if (!layer)
        continue;
      if (auto* dl = dynamic_cast<costmap_2d::DistanceLayer*>(layer))
      {
        distance_layer_ = dl;
        break;
      }
    }
  }

  if (use_esdf_ && !distance_layer_)
  {
    ROS_WARN("[ILQRController] use_esdf=true but costmap has no DistanceLayer. "
             "Add {name: distance_layer, type: costmap_2d::DistanceLayer} to local_costmap plugins to enable ESDF.");
  }

  if (params_.use_corridor)
  {
    rpp::AckermannConfig ack_cfg;
    ack_cfg.wheelbase = params_.corridor_wheelbase;
    ack_cfg.max_steer_angle = params_.corridor_max_steer_angle;
    ack_cfg.track_width = params_.corridor_track_width;
    safety_corridor_ = std::make_unique<rpp::common::safety_corridor::ConvexSafetyCorridor>(costmap_ros_,
                                                                                            params_.corridor_safety_range,
                                                                                            ack_cfg);
  }

  std::vector<double> alphas;
  if (nh.getParam("alphas", alphas) && !alphas.empty())
    params_.alphas = alphas;

  double controller_frequency;
  nh.param("/move_base/controller_frequency", controller_frequency, 10.0);
  params_.dt = 1.0 / std::max(1e-3, controller_frequency);

  ros::NodeHandle nh_root("/");
  debug_ref_pub_ = nh_root.advertise<nav_msgs::Path>("ilqr_reference", 1, true);
  corridor_pub_ = nh_root.advertise<visualization_msgs::MarkerArray>("ilqr_corridor", 1, true);
  corridor_pub_ns_ = nh.advertise<visualization_msgs::MarkerArray>("ilqr_corridor", 1, true);
  traj_pub_ = nh_root.advertise<nav_msgs::Path>("ilqr_trajectory", 1, true);

  ROS_INFO_STREAM("[ILQRController] corridor topics: " << corridor_pub_.getTopic() << " , "
                                                     << corridor_pub_ns_.getTopic());

  if (corridor_pub_)
  {
    visualization_msgs::MarkerArray empty;
    corridor_pub_.publish(empty);
  }
  if (corridor_pub_ns_)
  {
    visualization_msgs::MarkerArray empty;
    corridor_pub_ns_.publish(empty);
  }

  ROS_INFO("ILQR controller initialized!");
}

bool ILQRController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("ILQR controller has not been initialized");
    return false;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;

  if (global_plan_.empty())
    return false;

  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

bool ILQRController::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("ILQR controller has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

bool ILQRController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("ILQR controller has not been initialized");
    return false;
  }

  if (global_plan_.empty())
    return false;

  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);

  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);
  if (prune_plan.empty())
    prune_plan = global_plan_;

  const double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  const double L = getLookAheadDistance(vt);

  geometry_msgs::PointStamped lookahead_pt;
  double theta_trj = 0.0;
  double kappa = 0.0;
  getLookAheadPoint(L, robot_pose_map, prune_plan, lookahead_pt, theta_trj, kappa);

  const double theta = tf2::getYaw(robot_pose_map.pose.orientation);

  // goal reached (position) -> rotate to goal yaw
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    const double e_theta = regularizeAngle(goal_rpy_.z() - theta);

    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = std::copysign(std::min(max_w_, std::max(min_w_, std::fabs(e_theta))), e_theta);
    return true;
  }

  // build reference
  std::vector<Eigen::Vector3d> x_ref;
  if (!buildReference(robot_pose_map, prune_plan, x_ref))
  {
    // fallback: pure-pursuit-like command from curvature
    const double v_d = clampValue(params_.v_ref, min_v_, max_v_);
    const double w_d = clampValue(kappa * v_d, -max_w_, max_w_);

    cmd_vel.linear.x = linearRegularization(base_odom, v_d);
    cmd_vel.angular.z = angularRegularization(base_odom, w_d);
    return true;
  }

  if (params_.use_corridor)
  {
    buildCorridorFromReference(x_ref);
    publishCorridorMarkers();
  }

  // publish ref for debug
  {
    nav_msgs::Path ref_msg;
    ref_msg.header.frame_id = map_frame_;
    ref_msg.header.stamp = ros::Time::now();
    ref_msg.poses.reserve(x_ref.size());
    for (const auto& xr : x_ref)
    {
      geometry_msgs::PoseStamped ps;
      ps.header = ref_msg.header;
      ps.pose.position.x = xr.x();
      ps.pose.position.y = xr.y();
      tf2::Quaternion q;
      q.setRPY(0, 0, xr.z());
      ps.pose.orientation = tf2::toMsg(q);
      ref_msg.poses.push_back(ps);
    }
    debug_ref_pub_.publish(ref_msg);
  }

  State x0;
  x0.setZero();
  x0(0) = robot_pose_map.pose.position.x;
  x0(1) = robot_pose_map.pose.position.y;
  x0(2) = theta;
  x0(3) = base_odom.twist.twist.linear.x;
  x0(4) = base_odom.twist.twist.angular.z;

  Trajectory traj;
  if (!solveILQR(x0, x_ref, traj))
  {
    // fallback: pure-pursuit-like command from curvature
    const double v_d = clampValue(params_.v_ref, min_v_, max_v_);
    const double w_d = clampValue(kappa * v_d, -max_w_, max_w_);

    cmd_vel.linear.x = linearRegularization(base_odom, v_d);
    cmd_vel.angular.z = angularRegularization(base_odom, w_d);
    return true;
  }

  Eigen::Vector2d u0 = traj.u.front();
  publishTrajectoryPath(traj);
  if (!std::isfinite(u0.x()) || !std::isfinite(u0.y()))
  {
    const double v_d = clampValue(params_.v_ref, min_v_, max_v_);
    const double w_d = clampValue(kappa * v_d, -max_w_, max_w_);
    cmd_vel.linear.x = linearRegularization(base_odom, v_d);
    cmd_vel.angular.z = angularRegularization(base_odom, w_d);
    return true;
  }
  const double v_cmd = linearRegularization(base_odom, u0.x());
  const double w_cmd = angularRegularization(base_odom, u0.y());

  cmd_vel.linear.x = v_cmd;
  cmd_vel.angular.z = w_cmd;
  return true;
}

bool ILQRController::buildReference(const geometry_msgs::PoseStamped& robot_pose_map,
                                   const std::vector<geometry_msgs::PoseStamped>& prune_plan,
                                   std::vector<Eigen::Vector3d>& x_ref)
{
  if (prune_plan.size() < 2)
    return false;

  const int N = std::max(5, params_.horizon_steps);
  const double ds = std::max(1e-3, params_.ds_ref);

  x_ref.clear();
  x_ref.reserve(N + 1);

  // starting at the robot
  const double x0 = robot_pose_map.pose.position.x;
  const double y0 = robot_pose_map.pose.position.y;
  const double th0 = tf2::getYaw(robot_pose_map.pose.orientation);
  x_ref.emplace_back(x0, y0, th0);

  // walk along prune_plan by arc-length
  size_t seg_i = 0;
  double seg_s = 0.0;
  double target_s = ds;

  // find first segment close to robot (prune_plan[0] is already near robot)
  while (seg_i + 1 < prune_plan.size() && x_ref.size() < static_cast<size_t>(N + 1))
  {
    const double x_a = prune_plan[seg_i].pose.position.x;
    const double y_a = prune_plan[seg_i].pose.position.y;
    const double x_b = prune_plan[seg_i + 1].pose.position.x;
    const double y_b = prune_plan[seg_i + 1].pose.position.y;

    const double seg_len = std::hypot(x_b - x_a, y_b - y_a);
    if (seg_len < 1e-6)
    {
      seg_i++;
      seg_s = 0.0;
      continue;
    }

    while (target_s <= seg_s + seg_len && x_ref.size() < static_cast<size_t>(N + 1))
    {
      const double t = (target_s - seg_s) / seg_len;
      const double x = x_a + t * (x_b - x_a);
      const double y = y_a + t * (y_b - y_a);
      const double th = std::atan2(y_b - y_a, x_b - x_a);
      x_ref.emplace_back(x, y, th);
      target_s += ds;
    }

    seg_s += seg_len;
    seg_i++;
  }

  // pad with the last known reference
  while (x_ref.size() < static_cast<size_t>(N + 1))
    x_ref.push_back(x_ref.back());

  // deduplicate extremely close consecutive points to avoid zero-length segments
  std::vector<Eigen::Vector3d> filtered;
  filtered.reserve(x_ref.size());
  const double min_ds = std::max(1e-4, 0.1 * ds);
  for (const auto& p : x_ref)
  {
    if (filtered.empty())
    {
      filtered.push_back(p);
      continue;
    }
    const auto& last = filtered.back();
    const double dx = p.x() - last.x();
    const double dy = p.y() - last.y();
    if (std::hypot(dx, dy) < min_ds)
      continue;
    filtered.push_back(p);
  }
  if (filtered.size() < 2)
    return false;
  x_ref.swap(filtered);

  while (x_ref.size() < static_cast<size_t>(N + 1))
    x_ref.push_back(x_ref.back());

  return true;
}

bool ILQRController::buildCorridorFromReference(const std::vector<Eigen::Vector3d>& x_ref)
{
  corridor_polygons_.clear();
  if (!params_.use_corridor || !safety_corridor_ || x_ref.size() < 2)
    return false;

  rpp::common::geometry::Points3d pts;
  pts.reserve(x_ref.size());
  const double min_ds = std::max(1e-4, 0.1 * params_.ds_ref);
  for (const auto& xr : x_ref)
  {
    if (!pts.empty())
    {
      const auto& last = pts.back();
      const double dx = xr.x() - last.x();
      const double dy = xr.y() - last.y();
      if (std::hypot(dx, dy) < min_ds)
        continue;
    }
    rpp::common::geometry::Point3d p;
    p.setX(xr.x());
    p.setY(xr.y());
    p.setTheta(xr.z());
    pts.push_back(p);
  }

  if (pts.size() < 2)
    return false;

  return safety_corridor_->decompose(pts, corridor_polygons_);
}

bool ILQRController::isInsideCorridor(const Eigen::Vector2d& p) const
{
  if (!params_.use_corridor || corridor_polygons_.empty())
    return true;

  rpp::common::geometry::Vec2d pt(p.x(), p.y());
  for (const auto& poly : corridor_polygons_)
  {
    if (poly.isPointIn(pt) || poly.isPointOnBoundary(pt))
      return true;
  }
  return false;
}

ILQRController::State ILQRController::stepDynamics(const State& x, const Control& u)
{
  const double v = u.x();
  const double w = u.y();
  const double th = x.z();

  State xn;
  xn.setZero();
  xn(0) = x(0) + v * std::cos(th) * params_.dt;
  xn(1) = x(1) + v * std::sin(th) * params_.dt;
  xn(2) = regularizeAngle(th + w * params_.dt);
  xn(3) = v;
  xn(4) = w;
  return xn;
}

void ILQRController::linearizeDynamics(const State& x, const Control& u, MatA& A, MatB& B)
{
  const double v = u.x();
  const double th = x.z();
  const double dt = params_.dt;

  A.setZero();
  A(0, 0) = 1.0;
  A(1, 1) = 1.0;
  A(2, 2) = 1.0;
  A(0, 2) = -v * std::sin(th) * dt;
  A(1, 2) = v * std::cos(th) * dt;

  B.setZero();
  B(0, 0) = std::cos(th) * dt;
  B(1, 0) = std::sin(th) * dt;
  B(2, 1) = dt;
  B(3, 0) = 1.0;
  B(4, 1) = 1.0;
}

void ILQRController::clampControl(Control& u)
{
  u.x() = clampValue(u.x(), min_v_, max_v_);
  u.y() = clampValue(u.y(), -max_w_, max_w_);
  if (std::fabs(u.y()) < min_w_)
    u.y() = std::copysign(min_w_, u.y());
}

bool ILQRController::isCollision(const Eigen::Vector2d& p)
{
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  int mx = 0, my = 0;
  if (!worldToMap(p.x(), p.y(), mx, my))
    return true;
  const unsigned char c = costmap->getCost(mx, my);
  return c >= static_cast<unsigned char>(params_.obs_cost_threshold);
}

double ILQRController::obstacleCost(const Eigen::Vector2d& p, Eigen::Vector2d* grad)
{
  if (use_esdf_ && distance_layer_)
    return obstacleCostESDF(p, grad);
  return obstacleCostCostmap(p, grad);
}

double ILQRController::obstacleCostESDF(const Eigen::Vector2d& p, Eigen::Vector2d* grad)
{
  int mx = 0, my = 0;
  if (!worldToMap(p.x(), p.y(), mx, my))
  {
    if (grad)
      grad->setZero();
    return params_.obs_collision_cost;
  }

  const double sigma = std::max(1e-6, esdf_sigma_m_);

  boost::unique_lock<boost::mutex> lock(distance_layer_->getMutex());
  const double x_cell = static_cast<double>(mx) + 0.5;
  const double y_cell = static_cast<double>(my) + 0.5;

  const double d2_cells = std::max(0.0, distance_layer_->getDistance(x_cell, y_cell));
  const double d_cells = std::sqrt(d2_cells);
  const double d_m = d_cells * resolution_;

  if (d_m <= std::max(0.0, esdf_collision_dist_m_))
  {
    if (grad)
      grad->setZero();
    return params_.obs_collision_cost;
  }

  // Smooth repulsive cost from ESDF: exp(-d/sigma)
  const double cost = params_.obs_cost_scale * params_.obs_cost_gain * std::exp(-d_m / sigma);

  if (grad)
  {
    double gx2 = 0.0, gy2 = 0.0;
    distance_layer_->getGradient(x_cell, y_cell, gx2, gy2);  // gradient of d^2 in cell coordinates

    const double denom = std::max(1e-6, 2.0 * d_cells);
    const double dd_dx = gx2 / denom;
    const double dd_dy = gy2 / denom;

    // d(cost)/d(x,y) = cost * (-1/sigma) * d(d_m)/d(x,y), and d(d_m)/d(x,y)=d(d_cells)/d(x,y) because res cancels.
    (*grad).x() = cost * (-1.0 / sigma) * dd_dx;
    (*grad).y() = cost * (-1.0 / sigma) * dd_dy;
  }

  return cost;
}

double ILQRController::obstacleCostCostmap(const Eigen::Vector2d& p, Eigen::Vector2d* grad)
{
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  int mx = 0, my = 0;
  if (!worldToMap(p.x(), p.y(), mx, my))
  {
    if (grad)
      grad->setZero();
    return params_.obs_collision_cost;
  }

  const auto getC = [&](int ix, int iy) -> double {
    ix = clampValue(ix, 0, static_cast<int>(costmap->getSizeInCellsX()) - 1);
    iy = clampValue(iy, 0, static_cast<int>(costmap->getSizeInCellsY()) - 1);
    return static_cast<double>(costmap->getCost(ix, iy));
  };

  const double c0 = getC(mx, my);

  // Shape obstacle cost so that approaching inflated/inscribed/lethal zones is strongly discouraged.
  // For collision/no-information (>= threshold), return a large constant penalty.
  double cost = 0.0;
  if (c0 >= params_.obs_cost_threshold)
  {
    cost = params_.obs_collision_cost;
  }
  else
  {
    const double s = clampValue(c0 / 255.0, 0.0, 1.0);
    cost = params_.obs_cost_scale * params_.obs_cost_gain * (s * s);
  }

  if (grad)
  {
    const double cx1 = getC(mx + 1, my);
    const double cx0 = getC(mx - 1, my);
    const double cy1 = getC(mx, my + 1);
    const double cy0 = getC(mx, my - 1);

    const double dc_dx_cell = 0.5 * (cx1 - cx0);
    const double dc_dy_cell = 0.5 * (cy1 - cy0);

    // convert cell gradient to world gradient
    const double inv_res = 1.0 / std::max(1e-6, resolution_);
    // When in collision/no-info cells, use zero gradient (costmap gradient can be ill-defined/saturated).
    if (c0 >= params_.obs_cost_threshold)
    {
      (*grad).setZero();
    }
    else
    {
      (*grad).x() = params_.obs_cost_scale * params_.obs_cost_gain * (dc_dx_cell / 255.0) * inv_res;
      (*grad).y() = params_.obs_cost_scale * params_.obs_cost_gain * (dc_dy_cell / 255.0) * inv_res;
    }
  }

  return cost;
}

double ILQRController::evaluateTrajectoryCost(const Trajectory& traj, const std::vector<Eigen::Vector3d>& x_ref)
{
  const int N = params_.horizon_steps;
  double J = 0.0;

  for (int k = 0; k < N; ++k)
  {
    const auto& x = traj.x[k];
    const auto& u = traj.u[k];
    const auto& xr = x_ref[k];

    const double ex = x(0) - xr.x();
    const double ey = x(1) - xr.y();
    const double eth = angleDiff(x(2), xr.z());

    J += params_.w_track * (ex * ex + ey * ey);
    J += params_.w_heading * (eth * eth);

    J += params_.w_u * (u.x() * u.x() + u.y() * u.y());

    // Smoothness cost in a Markov form: penalize deviation from previous control carried in state.
    const double dv = u.x() - x(3);
    const double dw = u.y() - x(4);
    J += params_.w_du * (dv * dv + dw * dw);

    Eigen::Vector2d g;
    const double oc = obstacleCost(Eigen::Vector2d(x(0), x(1)), &g);
    J += params_.w_obs * oc;

    bool outside = false;
    const double cc = corridorCost(Eigen::Vector2d(x(0), x(1)), nullptr, &outside);
    if (params_.use_corridor && params_.corridor_hard_constraint && outside)
      return std::numeric_limits<double>::infinity();
    J += params_.w_corridor * cc;
  }

  // terminal cost
  {
    const auto& xN = traj.x.back();
    const auto& xrN = x_ref.back();
    const double ex = xN(0) - xrN.x();
    const double ey = xN(1) - xrN.y();
    const double eth = angleDiff(xN(2), xrN.z());
    J += 5.0 * params_.w_track * (ex * ex + ey * ey);
    J += 5.0 * params_.w_heading * (eth * eth);
  }

  return J;
}

double ILQRController::corridorCost(const Eigen::Vector2d& p, Eigen::Vector2d* grad, bool* outside)
{
  if (grad)
    grad->setZero();
  if (outside)
    *outside = false;

  if (!params_.use_corridor || corridor_polygons_.empty())
    return 0.0;

  rpp::common::geometry::Vec2d pt(p.x(), p.y());

  for (const auto& poly : corridor_polygons_)
  {
    if (poly.isPointIn(pt) || poly.isPointOnBoundary(pt))
      return 0.0;
  }

  double min_dist = std::numeric_limits<double>::infinity();
  rpp::common::geometry::Vec2d nearest_best;
  bool found = false;

  for (const auto& poly : corridor_polygons_)
  {
    const auto& pts = poly.points();
    if (pts.size() < 2)
      continue;
    for (size_t i = 0; i < pts.size(); ++i)
    {
      const auto& a = pts[i];
      const auto& b = pts[poly.next(i)];
      rpp::common::geometry::LineSegment2d seg(a, b);
      rpp::common::geometry::Vec2d nearest;
      const double d = seg.distanceTo(pt, &nearest);
      if (d < min_dist)
      {
        min_dist = d;
        nearest_best = nearest;
        found = true;
      }
    }
  }

  if (!found || !std::isfinite(min_dist))
    return 0.0;

  if (outside)
    *outside = true;

  const Eigen::Vector2d diff(p.x() - nearest_best.x(), p.y() - nearest_best.y());
  if (grad)
    *grad = diff;
  return 0.5 * diff.squaredNorm();
}

void ILQRController::publishCorridorMarkers() const
{
  if ((!corridor_pub_ && !corridor_pub_ns_) || corridor_polygons_.empty())
    return;

  visualization_msgs::MarkerArray ma;
  int id = 0;
  ros::Duration marker_life(0.2);

  for (const auto& poly : corridor_polygons_)
  {
    visualization_msgs::Marker outline;
    outline.header.frame_id = map_frame_;
    outline.header.stamp = ros::Time::now();
    outline.ns = "ilqr_corridor";
    outline.id = id++;
    outline.type = visualization_msgs::Marker::LINE_STRIP;
    outline.action = visualization_msgs::Marker::ADD;
    outline.scale.x = 0.03;
    outline.color.r = 0.2;
    outline.color.g = 0.6;
    outline.color.b = 0.2;
    outline.color.a = 0.8;
    outline.lifetime = marker_life;

    visualization_msgs::Marker fill;
    fill.header.frame_id = map_frame_;
    fill.header.stamp = ros::Time::now();
    fill.ns = "ilqr_corridor_filled";
    fill.id = id++;
    fill.type = visualization_msgs::Marker::TRIANGLE_LIST;
    fill.action = visualization_msgs::Marker::ADD;
    fill.color.r = 0.4;
    fill.color.g = 0.8;
    fill.color.b = 0.4;
    fill.color.a = 0.35;
    fill.scale.x = 1.0;
    fill.scale.y = 1.0;
    fill.scale.z = 1.0;
    fill.pose.orientation.w = 1.0;
    fill.lifetime = marker_life;

    std::vector<rpp::common::geometry::Vec2d> pts;
    pts.reserve(poly.points().size());
    for (const auto& v : poly.points())
    {
      rpp::common::geometry::Vec2d p;
      p.setX(v.x());
      p.setY(v.y());
      pts.push_back(p);
    }

    if (pts.size() >= 3)
    {
      double cx = 0.0, cy = 0.0;
      for (const auto& p : pts)
      {
        cx += p.x();
        cy += p.y();
      }
      cx /= static_cast<double>(pts.size());
      cy /= static_cast<double>(pts.size());

      geometry_msgs::Point c;
      c.x = cx;
      c.y = cy;
      c.z = 0.0;

      outline.points.reserve(pts.size() + 1);
      for (const auto& p : pts)
      {
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.0;
        outline.points.push_back(gp);
      }
      outline.points.push_back(outline.points.front());

      for (size_t i = 0; i < pts.size(); ++i)
      {
        const auto& p1 = pts[i];
        const auto& p2 = pts[(i + 1) % pts.size()];
        geometry_msgs::Point g1, g2;
        g1.x = p1.x();
        g1.y = p1.y();
        g1.z = 0.0;
        g2.x = p2.x();
        g2.y = p2.y();
        g2.z = 0.0;
        fill.points.push_back(c);
        fill.points.push_back(g1);
        fill.points.push_back(g2);
      }
    }

    ma.markers.push_back(outline);
    ma.markers.push_back(fill);
  }

  if (corridor_pub_)
    corridor_pub_.publish(ma);
  if (corridor_pub_ns_)
    corridor_pub_ns_.publish(ma);
}

void ILQRController::publishTrajectoryPath(const Trajectory& traj) const
{
  if (!traj_pub_)
    return;

  nav_msgs::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = ros::Time::now();
  path.poses.reserve(traj.x.size());

  for (const auto& x : traj.x)
  {
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = x(0);
    ps.pose.position.y = x(1);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x(2));
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }

  traj_pub_.publish(path);
}

bool ILQRController::solveILQR(const State& x0,
                             const std::vector<Eigen::Vector3d>& x_ref,
                             Trajectory& traj)
{
  const int N = params_.horizon_steps;

  traj.x.assign(N + 1, x0);
  traj.u.assign(N, Control(params_.v_ref, 0.0));

  // initial guess from ref heading
  for (int k = 0; k < N; ++k)
  {
    const double th = x_ref[k].z();
    const double thn = x_ref[k + 1].z();
    const double dth = angleDiff(thn, th);
    Control u(params_.v_ref, dth / params_.dt);
    clampControl(u);
    traj.u[k] = u;
  }

  auto rollout = [&](Trajectory& t) {
    t.x[0] = x0;
    for (int k = 0; k < N; ++k)
      t.x[k + 1] = stepDynamics(t.x[k], t.u[k]);
    t.cost = evaluateTrajectoryCost(t, x_ref);
    if (!std::isfinite(t.cost))
      t.cost = std::numeric_limits<double>::infinity();
  };

  rollout(traj);

  double lambda = params_.lambda_init;

  std::vector<Control> kff(N);
  std::vector<MatK> Kfb(N);

  for (int iter = 0; iter < params_.max_iter; ++iter)
  {
    // Note: do NOT hard-abort on collision; rely on collision penalty in obstacleCost.

    // backward pass
    bool backward_ok = true;

    State Vx = State::Zero();
    MatA Vxx = MatA::Zero();

    // terminal cost derivatives (quadratic)
    {
      const Eigen::Vector3d e(traj.x[N](0) - x_ref[N].x(), traj.x[N](1) - x_ref[N].y(), angleDiff(traj.x[N](2), x_ref[N].z()));
      const double wT = 5.0;
      Vx(0) = 2.0 * wT * params_.w_track * e.x();
      Vx(1) = 2.0 * wT * params_.w_track * e.y();
      Vx(2) = 2.0 * wT * params_.w_heading * e.z();

      Vxx.setZero();
      Vxx(0, 0) = 2.0 * wT * params_.w_track;
      Vxx(1, 1) = 2.0 * wT * params_.w_track;
      Vxx(2, 2) = 2.0 * wT * params_.w_heading;
    }

    for (int k = N - 1; k >= 0; --k)
    {
      const auto& x = traj.x[k];
      const auto& u = traj.u[k];
      const auto& xr = x_ref[k];

      MatA A;
      MatB B;
      linearizeDynamics(x, u, A, B);

      const Eigen::Vector3d e(x(0) - xr.x(), x(1) - xr.y(), angleDiff(x(2), xr.z()));

      // l_x, l_u
      State lx = State::Zero();
      Control lu = Control::Zero();

      lx(0) = 2.0 * params_.w_track * e.x();
      lx(1) = 2.0 * params_.w_track * e.y();
      lx(2) = 2.0 * params_.w_heading * e.z();

      lu.x() = 2.0 * params_.w_u * u.x();
      lu.y() = 2.0 * params_.w_u * u.y();

      // smoothness vs previous control carried in state: w_du * ||u - u_prev||^2
      const double dv = u.x() - x(3);
      const double dw = u.y() - x(4);
      lu.x() += 2.0 * params_.w_du * dv;
      lu.y() += 2.0 * params_.w_du * dw;
      lx(3) += -2.0 * params_.w_du * dv;
      lx(4) += -2.0 * params_.w_du * dw;

      // obstacle gradient
      Eigen::Vector2d g;
      obstacleCost(Eigen::Vector2d(x(0), x(1)), &g);
      lx(0) += params_.w_obs * g.x();
      lx(1) += params_.w_obs * g.y();

      // corridor gradient (soft penalty when outside)
      Eigen::Vector2d cg;
      bool outside = false;
      corridorCost(Eigen::Vector2d(x(0), x(1)), &cg, &outside);
      if (outside)
      {
        lx(0) += params_.w_corridor * cg.x();
        lx(1) += params_.w_corridor * cg.y();
      }

      // l_xx, l_uu, l_ux
      MatA lxx = MatA::Zero();
      lxx(0, 0) = 2.0 * params_.w_track;
      lxx(1, 1) = 2.0 * params_.w_track;
      lxx(2, 2) = 2.0 * params_.w_heading;
      lxx(3, 3) = 2.0 * params_.w_du;
      lxx(4, 4) = 2.0 * params_.w_du;

      if (outside)
      {
        lxx(0, 0) += params_.w_corridor;
        lxx(1, 1) += params_.w_corridor;
      }

      Eigen::Matrix2d luu = Eigen::Matrix2d::Zero();
      luu(0, 0) = 2.0 * params_.w_u + 2.0 * params_.w_du;
      luu(1, 1) = 2.0 * params_.w_u + 2.0 * params_.w_du;

      Eigen::Matrix<double, 2, kNx> lux = Eigen::Matrix<double, 2, kNx>::Zero();
      lux(0, 3) = -2.0 * params_.w_du;
      lux(1, 4) = -2.0 * params_.w_du;

      // Q terms
      const State Qx = lx + A.transpose() * Vx;
      const Control Qu = lu + B.transpose() * Vx;

      const MatA Qxx = lxx + A.transpose() * Vxx * A;
      const Eigen::Matrix<double, 2, kNx> Qux = lux + B.transpose() * Vxx * A;
      Eigen::Matrix2d Quu = luu + B.transpose() * Vxx * B;

      Quu(0, 0) += lambda;
      Quu(1, 1) += lambda;

      Eigen::LLT<Eigen::Matrix2d> llt(Quu);
      if (llt.info() != Eigen::Success)
      {
        backward_ok = false;
        break;
      }

      const Eigen::Matrix2d Quu_inv = llt.solve(Eigen::Matrix2d::Identity());
      kff[k] = -Quu_inv * Qu;
      Kfb[k] = -Quu_inv * Qux;

      if (!kff[k].allFinite() || !Kfb[k].allFinite())
      {
        backward_ok = false;
        break;
      }

      // update V
      Vx = Qx + Kfb[k].transpose() * Quu * kff[k] + Kfb[k].transpose() * Qu + Qux.transpose() * kff[k];
      Vxx = Qxx + Kfb[k].transpose() * Quu * Kfb[k] + Kfb[k].transpose() * Qux + Qux.transpose() * Kfb[k];

      // ensure symmetry
      Vxx = 0.5 * (Vxx + Vxx.transpose());

      if (!Vx.allFinite() || !Vxx.allFinite())
      {
        backward_ok = false;
        break;
      }
    }

    if (!backward_ok)
    {
      lambda = std::min(params_.lambda_max, lambda * 10.0);
      if (lambda >= params_.lambda_max)
        return false;
      continue;
    }

    // forward pass with line search
    bool accepted = false;
    Trajectory best = traj;

    for (double a : params_.alphas)
    {
      Trajectory cand;
      cand.x.assign(N + 1, x0);
      cand.u = traj.u;

      State x = x0;
      for (int k = 0; k < N; ++k)
      {
        const Control du = a * kff[k] + Kfb[k] * (x - traj.x[k]);
        cand.u[k] = traj.u[k] + du;
        clampControl(cand.u[k]);
        x = stepDynamics(x, cand.u[k]);
        cand.x[k + 1] = x;
      }

      cand.cost = evaluateTrajectoryCost(cand, x_ref);

      if (!std::isfinite(cand.cost))
        continue;

      if (cand.cost + 1e-6 < best.cost)
      {
        best = cand;
        accepted = true;
        break;
      }
    }

    if (!accepted)
    {
      lambda = std::min(params_.lambda_max, lambda * 10.0);
      if (lambda >= params_.lambda_max)
        break;
      continue;
    }

    const double improvement = traj.cost - best.cost;
    traj = best;

    lambda = std::max(params_.lambda_init, lambda / 2.0);

    if (!std::isfinite(improvement) || improvement < 1e-3)
      break;
  }

  return true;
}

}  // namespace controller
}  // namespace rpp
