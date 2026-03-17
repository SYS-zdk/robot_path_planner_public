/**
 * @file reachability_dwa.cpp
 * @brief Implementation of reachability-aware trajectory sampling and scoring (core).
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>

#include "controller/reachability_dwa.h"

namespace rpp
{
namespace controller
{
static inline double clampd(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

void ReachabilityDWA::ensureControlSequence()
{
  if (mppi_horizon_steps_ <= 0)
    mppi_horizon_steps_ = 1;

  if (static_cast<int>(u_v_.size()) != mppi_horizon_steps_)
  {
    u_v_.assign(mppi_horizon_steps_, 0.0);
    u_w_.assign(mppi_horizon_steps_, 0.0);
  }
}

void ReachabilityDWA::shiftControlSequence()
{
  if (u_v_.empty())
    return;
  u_v_.erase(u_v_.begin());
  u_w_.erase(u_w_.begin());
  u_v_.push_back(u_v_.empty() ? 0.0 : u_v_.back());
  u_w_.push_back(u_w_.empty() ? 0.0 : u_w_.back());
}

void ReachabilityDWA::refreshReachabilityCache()
{
  reach_cache_valid_ = false;
  reach_field_cache_.clear();
  reach_size_x_ = 0;
  reach_size_y_ = 0;

  if (!costmap_ros_ || !costmap_ros_->getLayeredCostmap() || !costmap_ros_->getLayeredCostmap()->getPlugins())
    return;

  if (!reach_layer_)
  {
    for (auto it = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
         it != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++it)
    {
      auto reach = boost::dynamic_pointer_cast<costmap_2d::LocalReachabilityLayer>(*it);
      if (reach)
      {
        reach_layer_ = reach;
        break;
      }
    }
  }

  if (!reach_layer_)
    return;

  reach_cache_valid_ = reach_layer_->getReachabilityCache(reach_field_cache_, reach_size_x_, reach_size_y_);
}

double ReachabilityDWA::costToPath(double x, double y) const
{
  if (global_plan_.empty())
    return 0.0;

  double best = std::numeric_limits<double>::infinity();
  for (const auto& p : global_plan_)
  {
    const double dx = x - p.pose.position.x;
    const double dy = y - p.pose.position.y;
    const double d = std::hypot(dx, dy);
    if (d < best)
      best = d;
  }
  return std::isfinite(best) ? best : 0.0;
}

base_local_planner::Trajectory ReachabilityDWA::rolloutTrajectory(const geometry_msgs::PoseStamped& start_pose,
                                                                  const std::vector<double>& seq_v,
                                                                  const std::vector<double>& seq_w) const
{
  base_local_planner::Trajectory traj;
  traj.cost_ = 0.0;

  double x = start_pose.pose.position.x;
  double y = start_pose.pose.position.y;
  double th = tf2::getYaw(start_pose.pose.orientation);

  traj.addPoint(x, y, th);
  const int T = std::min(static_cast<int>(seq_v.size()), static_cast<int>(seq_w.size()));
  for (int t = 0; t < T; ++t)
  {
    const double v = seq_v[t];
    const double w = seq_w[t];
    x += v * std::cos(th) * mppi_dt_;
    y += v * std::sin(th) * mppi_dt_;
    th = angles::normalize_angle(th + w * mppi_dt_);
    traj.addPoint(x, y, th);
  }
  return traj;
}

void ReachabilityDWA::reconfigure(reachability_controller::ReachabilityControllerConfig& config)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  // MPPI params
  mppi_num_samples_ = std::max(1, config.mppi_num_samples);
  mppi_horizon_steps_ = std::max(1, config.mppi_horizon_steps);
  mppi_iterations_ = std::max(1, config.mppi_iterations);
  mppi_dt_ = std::max(1e-3, config.mppi_dt);
  mppi_lambda_ = std::max(1e-6, config.mppi_lambda);
  mppi_noise_v_std_ = std::max(0.0, config.mppi_noise_v_std);
  mppi_noise_w_std_ = std::max(0.0, config.mppi_noise_w_std);

  w_.path = std::max(0.0, config.mppi_path_weight);
  w_.goal = std::max(0.0, config.mppi_goal_weight);
  w_.obstacle = std::max(0.0, config.mppi_obstacle_weight);
  w_.control = std::max(0.0, config.mppi_control_weight);
  w_.reachability = std::max(0.0, config.reachability_scale);

  min_reachability_ = clampd(config.min_reachability, 0.0, 1.0);
  reachability_use_min_ = config.reachability_use_min;

  // tie sim period to dt (stop-rotate controller uses it)
  sim_period_ = mppi_dt_;

  noise_v_ = std::normal_distribution<double>(0.0, mppi_noise_v_std_);
  noise_w_ = std::normal_distribution<double>(0.0, mppi_noise_w_std_);

  ensureControlSequence();
}

ReachabilityDWA::ReachabilityDWA(std::string name, base_local_planner::LocalPlannerUtil* planner_util,
                                 costmap_2d::Costmap2DROS* costmap_ros)
  : planner_util_(planner_util)
  , costmap_ros_(costmap_ros)
  , rng_(std::random_device{}())
  , noise_v_(0.0, mppi_noise_v_std_)
  , noise_w_(0.0, mppi_noise_w_std_)
{
  ros::NodeHandle private_nh("~/" + name);

  // If running in nav stack, try to pick up controller_frequency as a reasonable default dt.
  std::string controller_frequency_param_name;
  if (private_nh.searchParam("controller_frequency", controller_frequency_param_name))
  {
    double controller_frequency = 0.0;
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    if (controller_frequency > 0.0)
    {
      sim_period_ = 1.0 / controller_frequency;
      mppi_dt_ = sim_period_;
    }
  }

  ensureControlSequence();
}

// used for visualization only, total_costs are not really total costs
bool ReachabilityDWA::getCellCosts(int cx, int cy, float& path_cost, float& goal_cost, float& occ_cost, float& total_cost)
{
  // for visualization only (not used by MPPI core)
  path_cost = 0.0f;
  goal_cost = 0.0f;
  occ_cost = 0.0f;
  total_cost = 0.0f;
  if (!planner_util_ || !planner_util_->getCostmap())
    return false;
  occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
  total_cost = occ_cost;
  return occ_cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool ReachabilityDWA::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  return planner_util_->setPlan(orig_global_plan);
}

/**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
bool ReachabilityDWA::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
{
  (void)vel;
  if (!planner_util_ || !planner_util_->getCostmap())
    return false;

  // One-step feasibility check (used by stop-rotate controller).
  const double x0 = pos[0];
  const double y0 = pos[1];
  const double th0 = pos[2];
  const double v = vel_samples[0];
  const double w = vel_samples[2];
  const double x1 = x0 + v * std::cos(th0) * mppi_dt_;
  const double y1 = y0 + v * std::sin(th0) * mppi_dt_;

  unsigned int mx = 0, my = 0;
  if (!planner_util_->getCostmap()->worldToMap(x1, y1, mx, my))
    return false;

  const unsigned char c = planner_util_->getCostmap()->getCost(mx, my);
  if (c >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return false;

  if (reach_cache_valid_ && mx < reach_size_x_ && my < reach_size_y_)
  {
    const size_t idx = static_cast<size_t>(mx) + static_cast<size_t>(my) * static_cast<size_t>(reach_size_x_);
    if (idx < reach_field_cache_.size())
    {
      const double r = clampd(reach_field_cache_[idx], 0.0, 1.0);
      if (r + 1e-12 < min_reachability_)
        return false;
    }
  }

  return true;
}

void ReachabilityDWA::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                                              const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                              const std::vector<geometry_msgs::Point>& footprint_spec)
{
  (void)global_pose;
  (void)footprint_spec;
  global_plan_ = new_plan;
}

/*
 * given the current state of the robot, find a good trajectory
 */
base_local_planner::Trajectory ReachabilityDWA::findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                                             const geometry_msgs::PoseStamped& global_vel,
                                                             geometry_msgs::PoseStamped& drive_velocities)
{
  (void)global_vel;
  boost::mutex::scoped_lock l(configuration_mutex_);

  result_traj_.resetPoints();
  result_traj_.cost_ = -1.0;

  if (!planner_util_ || !planner_util_->getCostmap())
    return result_traj_;

  if (global_plan_.empty())
    return result_traj_;

  ensureControlSequence();
  refreshReachabilityCache();

  const auto limits = planner_util_->getCurrentLimits();
  const double v_min = limits.min_vel_x;
  const double v_max = limits.max_vel_x;
  const double w_min = limits.min_vel_theta;
  const double w_max = limits.max_vel_theta;

  const double obs_lethal = static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // MPPI iterations
  const int T = mppi_horizon_steps_;
  const int K = mppi_num_samples_;
  std::vector<double> costs(K, 0.0);
  std::vector<std::vector<double>> eps_v(K, std::vector<double>(T, 0.0));
  std::vector<std::vector<double>> eps_w(K, std::vector<double>(T, 0.0));

  for (int it = 0; it < mppi_iterations_; ++it)
  {
    double min_cost = std::numeric_limits<double>::infinity();
    for (int k = 0; k < K; ++k)
    {
      double x = global_pose.pose.position.x;
      double y = global_pose.pose.position.y;
      double th = tf2::getYaw(global_pose.pose.orientation);

      double min_r = 1.0;
      double sum_r = 0.0;
      unsigned int used_r = 0;

      double total = 0.0;

      for (int t = 0; t < T; ++t)
      {
        const double dv = noise_v_(rng_);
        const double dw = noise_w_(rng_);
        eps_v[k][t] = dv;
        eps_w[k][t] = dw;

        const double v = clampd(u_v_[t] + dv, v_min, v_max);
        const double w = clampd(u_w_[t] + dw, w_min, w_max);

        x += v * std::cos(th) * mppi_dt_;
        y += v * std::sin(th) * mppi_dt_;
        th = angles::normalize_angle(th + w * mppi_dt_);

        unsigned int mx = 0, my = 0;
        if (!planner_util_->getCostmap()->worldToMap(x, y, mx, my))
        {
          total += 1e6;
          continue;
        }

        const unsigned char c = planner_util_->getCostmap()->getCost(mx, my);
        if (static_cast<double>(c) >= obs_lethal)
        {
          total += 1e6;
          continue;
        }
        total += w_.obstacle * (static_cast<double>(c) / 255.0);

        // reachability
        if (reach_cache_valid_ && mx < reach_size_x_ && my < reach_size_y_)
        {
          const size_t idx = static_cast<size_t>(mx) + static_cast<size_t>(my) * static_cast<size_t>(reach_size_x_);
          if (idx < reach_field_cache_.size())
          {
            const double r = clampd(reach_field_cache_[idx], 0.0, 1.0);
            min_r = std::min(min_r, r);
            sum_r += r;
            ++used_r;
            if (r + 1e-12 < min_reachability_)
              total += 1e6;
          }
        }

        total += w_.path * costToPath(x, y);
        total += w_.control * (v * v + 0.1 * w * w) * mppi_dt_;
      }

      // terminal goal cost
      const auto& g = global_plan_.back();
      total += w_.goal * std::hypot(x - g.pose.position.x, y - g.pose.position.y);

      if (used_r > 0)
      {
        const double agg_r = reachability_use_min_ ? min_r : (sum_r / static_cast<double>(used_r));
        total += w_.reachability * (1.0 - clampd(agg_r, 0.0, 1.0));
      }

      costs[k] = total;
      if (total < min_cost)
        min_cost = total;
    }

    // weights
    double denom = 0.0;
    std::vector<double> wk(K, 0.0);
    for (int k = 0; k < K; ++k)
    {
      const double z = (costs[k] - min_cost) / mppi_lambda_;
      const double w = std::exp(-z);
      wk[k] = w;
      denom += w;
    }
    if (denom <= 1e-12)
      denom = 1.0;

    for (int t = 0; t < T; ++t)
    {
      double dv_bar = 0.0;
      double dw_bar = 0.0;
      for (int k = 0; k < K; ++k)
      {
        const double w = wk[k] / denom;
        dv_bar += w * eps_v[k][t];
        dw_bar += w * eps_w[k][t];
      }
      u_v_[t] = clampd(u_v_[t] + dv_bar, v_min, v_max);
      u_w_[t] = clampd(u_w_[t] + dw_bar, w_min, w_max);
    }
  }

  // build output command
  const double v0 = u_v_.empty() ? 0.0 : clampd(u_v_[0], v_min, v_max);
  const double w0 = u_w_.empty() ? 0.0 : clampd(u_w_[0], w_min, w_max);

  drive_velocities.pose.position.x = v0;
  drive_velocities.pose.position.y = 0.0;
  drive_velocities.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, w0);
  tf2::convert(q, drive_velocities.pose.orientation);

  // output trajectory for visualization
  result_traj_ = rolloutTrajectory(global_pose, u_v_, u_w_);
  result_traj_.xv_ = v0;
  result_traj_.yv_ = 0.0;
  result_traj_.thetav_ = w0;
  result_traj_.cost_ = 0.0;

  shiftControlSequence();
  return result_traj_;
}
};  // namespace controller
}  // namespace rpp