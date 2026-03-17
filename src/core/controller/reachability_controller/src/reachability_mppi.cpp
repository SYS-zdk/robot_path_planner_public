/**
 * @file reachability_mppi.cpp
 * @brief MPPI-style rollout and cost evaluation implementation for reachability controller.
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "controller/reachability_dwa.h"

namespace rpp
{
namespace controller
{
static inline double clampd(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

static inline double sqr(double v)
{
  return v * v;
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

void ReachabilityDWA::publishSampledTrajectories(const geometry_msgs::PoseStamped& global_pose,
                                                 const std::vector<double>& costs,
                                                 const std::vector<std::vector<double>>& eps_v,
                                                 const std::vector<std::vector<double>>& eps_w,
                                                 double v_min,
                                                 double v_max,
                                                 double w_min,
                                                 double w_max)
{
  if (!publish_traj_pc_ || !traj_cloud_pub_)
    return;
  if (!planner_util_ || !planner_util_->getCostmap())
    return;
  if (costs.empty() || eps_v.empty() || eps_w.empty())
    return;

  const int K = static_cast<int>(costs.size());
  const int T = static_cast<int>(eps_v[0].size());
  if (K <= 0 || T <= 0)
    return;

  const int stride = std::max(1, traj_viz_stride_);
  const int pts_per_traj = 1 + (T + stride - 1) / stride;

  // Visualization coloring:
  // - Keep intensity channel (for debugging/alternative displays)
  // - Use RGB to enforce exact colors in RViz: samples=light gray, executed=red
  const float sample_intensity = 0.0f;
  const float exec_intensity = 1.0f;

  const uint8_t sample_r = 180, sample_g = 180, sample_b = 180;  // light gray
  const uint8_t exec_r = 255, exec_g = 0, exec_b = 0;            // red

  auto packRGB = [](uint8_t r, uint8_t g, uint8_t b) {
    const uint32_t rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
    float out;
    std::memcpy(&out, &rgb, sizeof(float));
    return out;
  };

  std::vector<int> order(K);
  for (int i = 0; i < K; ++i)
    order[i] = i;
  const int show_k = std::min(K, std::max(1, traj_viz_num_samples_));
  std::partial_sort(order.begin(), order.begin() + show_k, order.end(),
                    [&](int a, int b) { return costs[a] < costs[b]; });

  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = traj_cloud_frame_.empty() ? costmap_ros_->getGlobalFrameID() : traj_cloud_frame_;
  cloud.header.stamp = ros::Time::now();

  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                 "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1,
                                 sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);
  // sampled trajectories + executed trajectory
  cloud_mod.resize(static_cast<size_t>(show_k) * static_cast<size_t>(pts_per_traj) + static_cast<size_t>(pts_per_traj));

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_rgb(cloud, "rgb");
  sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");

  for (int kk = 0; kk < show_k; ++kk)
  {
    const int k = order[kk];
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    double th = tf2::getYaw(global_pose.pose.orientation);

    // first point
    *it_x = static_cast<float>(x);
    *it_y = static_cast<float>(y);
    *it_z = 0.0f;
    *it_rgb = packRGB(sample_r, sample_g, sample_b);
    *it_i = sample_intensity;
    ++it_x;
    ++it_y;
    ++it_z;
    ++it_rgb;
    ++it_i;

    int out_count = 1;
    for (int t = 0; t < T; ++t)
    {
      const double v = clampd(u_v_[t] + eps_v[k][t], v_min, v_max);
      const double w = clampd(u_w_[t] + eps_w[k][t], w_min, w_max);
      x += v * std::cos(th) * mppi_dt_;
      y += v * std::sin(th) * mppi_dt_;
      th = angles::normalize_angle(th + w * mppi_dt_);

      const bool should_output = ((t + 1) % stride == 0) || (t == T - 1);
      if (should_output && out_count < pts_per_traj)
      {
        *it_x = static_cast<float>(x);
        *it_y = static_cast<float>(y);
        *it_z = 0.0f;
        *it_rgb = packRGB(sample_r, sample_g, sample_b);
        *it_i = sample_intensity;
        ++it_x;
        ++it_y;
        ++it_z;
        ++it_rgb;
        ++it_i;
        ++out_count;
      }
    }
  }

  // Append executed (nominal) trajectory using the current control sequence u_v_/u_w_.
  {
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    double th = tf2::getYaw(global_pose.pose.orientation);

    *it_x = static_cast<float>(x);
    *it_y = static_cast<float>(y);
    *it_z = 0.0f;
    *it_rgb = packRGB(exec_r, exec_g, exec_b);
    *it_i = exec_intensity;
    ++it_x;
    ++it_y;
    ++it_z;
    ++it_rgb;
    ++it_i;

    int out_count = 1;
    for (int t = 0; t < T; ++t)
    {
      const double v = clampd(u_v_[t], v_min, v_max);
      const double w = clampd(u_w_[t], w_min, w_max);
      x += v * std::cos(th) * mppi_dt_;
      y += v * std::sin(th) * mppi_dt_;
      th = angles::normalize_angle(th + w * mppi_dt_);

      const bool should_output = ((t + 1) % stride == 0) || (t == T - 1);
      if (should_output && out_count < pts_per_traj)
      {
        *it_x = static_cast<float>(x);
        *it_y = static_cast<float>(y);
        *it_z = 0.0f;
        *it_rgb = packRGB(exec_r, exec_g, exec_b);
        *it_i = exec_intensity;
        ++it_x;
        ++it_y;
        ++it_z;
        ++it_rgb;
        ++it_i;
        ++out_count;
      }
    }
  }

  traj_cloud_pub_.publish(cloud);
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
  w_.lookahead_goal = std::max(0.0, config.mppi_lookahead_weight);
  w_.heading = std::max(0.0, config.mppi_heading_weight);
  w_.path_heading = std::max(0.0, config.mppi_path_heading_weight);
  w_.obstacle = std::max(0.0, config.mppi_obstacle_weight);
  w_.control = std::max(0.0, config.mppi_control_weight);
  w_.smooth = std::max(0.0, config.mppi_smooth_weight);
  w_.forward = std::max(0.0, config.mppi_forward_weight);
  w_.reachability = std::max(0.0, config.reachability_scale);

  mppi_lookahead_dist_ = std::max(0.0, config.mppi_lookahead_dist);
  mppi_update_alpha_ = clampd(config.mppi_update_alpha, 0.0, 1.0);

  min_reachability_ = clampd(config.min_reachability, 0.0, 1.0);
  reachability_use_min_ = config.reachability_use_min;

  // tie sim period to dt (stop-rotate controller uses it)
  sim_period_ = mppi_dt_;

  noise_v_ = std::normal_distribution<double>(0.0, mppi_noise_v_std_);
  noise_w_ = std::normal_distribution<double>(0.0, mppi_noise_w_std_);

  ensureControlSequence();
}

bool ReachabilityDWA::computeLookaheadTarget(const geometry_msgs::PoseStamped& reference_pose, double lookahead_dist,
                                            double& target_x, double& target_y) const
{
  if (global_plan_.empty())
    return false;

  const double rx = reference_pose.pose.position.x;
  const double ry = reference_pose.pose.position.y;

  // Find nearest plan index.
  int nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(global_plan_.size()); ++i)
  {
    const double dx = rx - global_plan_[i].pose.position.x;
    const double dy = ry - global_plan_[i].pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best)
    {
      best = d2;
      nearest = i;
    }
  }

  // March forward along the plan until reaching lookahead_dist.
  double acc = 0.0;
  int idx = nearest;
  for (int i = nearest; i + 1 < static_cast<int>(global_plan_.size()); ++i)
  {
    const double x0 = global_plan_[i].pose.position.x;
    const double y0 = global_plan_[i].pose.position.y;
    const double x1 = global_plan_[i + 1].pose.position.x;
    const double y1 = global_plan_[i + 1].pose.position.y;
    const double seg = std::hypot(x1 - x0, y1 - y0);
    acc += seg;
    idx = i + 1;
    if (acc >= lookahead_dist)
      break;
  }

  target_x = global_plan_[idx].pose.position.x;
  target_y = global_plan_[idx].pose.position.y;
  return true;
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

  private_nh.param("publish_traj_pc", publish_traj_pc_, false);
  private_nh.param("traj_viz_num_samples", traj_viz_num_samples_, 50);
  private_nh.param("traj_viz_stride", traj_viz_stride_, 1);
  private_nh.param("traj_cloud_frame", traj_cloud_frame_, std::string(""));
  if (publish_traj_pc_)
  {
    // Publish on a global topic to make RViz discovery straightforward.
    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("/trajectory_cloud", 1);
  }

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

  if (!reach_cache_valid_)
    refreshReachabilityCache();

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
  // In nav stack configs, min_vel_theta is a minimum *magnitude* threshold, not a signed bound.
  // Use symmetric signed bounds for MPPI sampling to avoid one-direction-only spinning.
  const double w_abs_max = std::abs(limits.max_vel_theta);
  const double w_min = -w_abs_max;
  const double w_max = w_abs_max;

  const double obs_lethal = static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // A fixed lookahead target computed from the current pose and current local plan.
  double look_x = global_plan_.back().pose.position.x;
  double look_y = global_plan_.back().pose.position.y;
  (void)computeLookaheadTarget(global_pose, mppi_lookahead_dist_, look_x, look_y);

  // Path tangent heading near the robot (helps cornering / reduces overshoot).
  double path_heading = tf2::getYaw(global_pose.pose.orientation);
  if (w_.path_heading > 0.0 && global_plan_.size() >= 2)
  {
    int nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(global_plan_.size()); ++i)
    {
      const double dx = global_pose.pose.position.x - global_plan_[i].pose.position.x;
      const double dy = global_pose.pose.position.y - global_plan_[i].pose.position.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best)
      {
        best = d2;
        nearest = i;
      }
    }
    const int i0 = std::min(nearest, static_cast<int>(global_plan_.size()) - 2);
    const double x0 = global_plan_[i0].pose.position.x;
    const double y0 = global_plan_[i0].pose.position.y;
    const double x1 = global_plan_[i0 + 1].pose.position.x;
    const double y1 = global_plan_[i0 + 1].pose.position.y;
    path_heading = std::atan2(y1 - y0, x1 - x0);
  }

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

      double prev_v = 0.0;
      double prev_w = 0.0;

      for (int t = 0; t < T; ++t)
      {
        const double dv = noise_v_(rng_);
        const double dw = noise_w_(rng_);
        eps_v[k][t] = dv;
        eps_w[k][t] = dw;

        const double v = clampd(u_v_[t] + dv, v_min, v_max);
        const double w = clampd(u_w_[t] + dw, w_min, w_max);

        // Smoothness penalty: discourages rapid left-right wobble and jitter.
        if (w_.smooth > 0.0 && t > 0)
          total += w_.smooth * (sqr(v - prev_v) + sqr(w - prev_w));
        prev_v = v;
        prev_w = w;

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

        // Encourage progress along the local plan by steering toward a lookahead point.
        if (w_.lookahead_goal > 0.0)
          total += w_.lookahead_goal * std::hypot(x - look_x, y - look_y);
        if (w_.heading > 0.0)
        {
          const double desired = std::atan2(look_y - y, look_x - x);
          const double e = angles::shortest_angular_distance(th, desired);
          total += w_.heading * sqr(e);
        }

        if (w_.path_heading > 0.0)
        {
          const double e = angles::shortest_angular_distance(th, path_heading);
          total += w_.path_heading * sqr(e);
        }

        total += w_.control * (v * v + w * w) * mppi_dt_;

        // Reduce stop-and-go: penalize low forward speed (relative to v_max).
        if (w_.forward > 0.0 && v_max > 1e-6)
        {
          const double v_pos = std::max(0.0, v);
          const double e = 1.0 - clampd(v_pos / v_max, 0.0, 1.0);
          total += w_.forward * e * e;
        }
      }

      // terminal goal cost (final point of the current local plan)
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
      const double a = mppi_update_alpha_;
      u_v_[t] = clampd(u_v_[t] + a * dv_bar, v_min, v_max);
      u_w_[t] = clampd(u_w_[t] + a * dw_bar, w_min, w_max);
    }
  }

  // Publish visualization once per cycle, after the control sequence has been updated.
  publishSampledTrajectories(global_pose, costs, eps_v, eps_w, v_min, v_max, w_min, w_max);

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
