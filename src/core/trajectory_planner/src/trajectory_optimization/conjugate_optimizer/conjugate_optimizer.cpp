
/***********************************************************
 *
 * @file: conjugate_optimizer.cpp
 * @brief: Trajectory optimization using conjugate gradient methods
 * @author: Yang Haodong
 * @date: 2024-09-30
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "common/math/math_helper.h"
#include "common/util/log.h"

#include "trajectory_planner/trajectory_optimization/conjugate_optimizer/conjugate_optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{
/**
 * @brief Construct a new trajectory optimizer object
 * @param costmap_ros costmap ROS wrapper
 * @param max_iter the maximum iterations for optimization
 * @param alpha learning rate
 * @param obs_dist_max the maximum distance to obstacle (m)
 * @param k_max the maximum curvature
 * @param w_obstacle the weight for obstacle avoidance
 * @param w_smooth the weight for smooth
 * @param w_curvature the weight for curvature
 */
CGOptimizer::CGOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int max_iter, double alpha, double obs_dist_max,
                         double k_max, double w_obstacle, double w_smooth, double w_curvature)
  : Optimizer(costmap_ros)
  , max_iter_(max_iter)
  , alpha_(alpha)
  , obs_dist_max_(obs_dist_max)
  , k_max_(k_max)
  , w_obstacle_(w_obstacle)
  , w_smooth_(w_smooth)
  , w_curvature_(w_curvature)
{
}

/**
 * @brief Running trajectory optimization
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool CGOptimizer::run(const Points3d& waypoints)
{
  path_opt_ = waypoints;
  return optimize(waypoints);
}

bool CGOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

/**
 * @brief Get the optimized trajectory
 * @param traj the trajectory buffer
 * @return true if optimizes successfully, else failed
 */
bool CGOptimizer::getTrajectory(Trajectory3d& traj)
{
  traj.reset(path_opt_.size());

  double t = 0.0;
  const double dt = 0.25;
  for (int i = 0; i < path_opt_.size(); i++)
  {
    const auto& pt = path_opt_[i];
    traj.time.push_back(t);
    traj.position.emplace_back(pt.x(), pt.y(), pt.theta());

    if (i < path_opt_.size() - 1)
    {
      const auto& pt_next = path_opt_[i + 1];
      const double vx = (pt_next.x() - pt.x()) / dt;
      const double vy = (pt_next.y() - pt.y()) / dt;
      traj.velocity.emplace_back(vx, vy);
      if (i < path_opt_.size() - 2)
      {
        const auto& pt_next_next = path_opt_[i + 2];
        const double vx_next = (pt_next_next.x() - pt_next.x()) / dt;
        const double vy_next = (pt_next_next.y() - pt_next.y()) / dt;
        traj.acceletation.emplace_back((vx_next - vx) / dt, (vy_next - vy) / dt);
      }
    }

    t += dt;
  }

  return true;
}

bool CGOptimizer::optimize(const Points3d& waypoints)
{
  // distance map update
  boost::shared_ptr<costmap_2d::DistanceLayer> distance_layer;
  bool is_distance_layer_exist = false;
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    distance_layer = boost::dynamic_pointer_cast<costmap_2d::DistanceLayer>(*layer);
    if (distance_layer)
    {
      is_distance_layer_exist = true;
      break;
    }
  }
  if (!is_distance_layer_exist)
  {
    R_ERROR << "Failed to get a Distance layer for potentional application.";
  }

  int iter = 0;
  while (iter < max_iter_)
  {
    // choose the first three nodes of the path
    for (int i = 2; i < path_opt_.size() - 2; ++i)
    {
      Vec2d xi_c2(path_opt_[i - 2].x(), path_opt_[i - 2].y());
      Vec2d xi_c1(path_opt_[i - 1].x(), path_opt_[i - 1].y());
      Vec2d xi(path_opt_[i].x(), path_opt_[i].y());
      Vec2d xi_p1(path_opt_[i + 1].x(), path_opt_[i + 1].y());
      Vec2d xi_p2(path_opt_[i + 2].x(), path_opt_[i + 2].y());

      Vec2d correction;
      correction = correction + _calObstacleTerm(xi, distance_layer);
      if (!_insideMap((xi - correction).x(), (xi - correction).y()))
        continue;

      correction = correction + _calSmoothTerm(xi_c2, xi_c1, xi, xi_p1, xi_p2);
      if (!_insideMap((xi - correction).x(), (xi - correction).y()))
        continue;

      correction = correction + _calCurvatureTerm(xi_c1, xi, xi_p1);
      if (!_insideMap((xi - correction).x(), (xi - correction).y()))
        continue;

      Vec2d gradient = alpha_ * correction / (w_obstacle_ + w_smooth_ + w_curvature_);
      if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
        gradient = decltype(gradient)();

      xi = xi - gradient;
      path_opt_[i].setX(xi.x());
      path_opt_[i].setY(xi.y());
    }

    iter++;
  }

  return true;
}

/**
 * @brief Static obstacle avoidance term.
 * @param xi the i-th waypoint
 * @return grad_i the i-th obstacle avoidance gradient to waypoint
 */
CGOptimizer::Vec2d CGOptimizer::_calObstacleTerm(const Vec2d& xi,
                                                 const boost::shared_ptr<costmap_2d::DistanceLayer>& dist_layer)
{
  Vec2d gradient;

  // the vector determining where the obstacle is
  unsigned int mx, my;
  costmap_ros_->getCostmap()->worldToMap(xi.x(), xi.y(), mx, my);
  if (_insideMap(mx, my))
  {
    // the distance to the closest obstacle from the current node
    const double resolution = costmap_ros_->getCostmap()->getResolution();
    double obs_dist = dist_layer->getDistance(mx, ny_ - my - 1) * resolution;
    double dx, dy;
    dist_layer->getGradient(mx, ny_ - my - 1, dx, dy);
    Vec2d obs_vec(dx, -dy);
    obs_vec.normalize();
    obs_vec *= 0.1 * resolution;
    // the closest obstacle is closer than desired correct the path for that
    if (obs_dist < obs_dist_max_ && obs_dist > 0)
      gradient = w_obstacle_ * 2.0 * (obs_dist - obs_dist_max_) * obs_vec / obs_dist;
  }
  return gradient;
}

/**
 * @brief Smooth term.
 * @param xim2 the (i-2)-th waypoint
 * @param xim1 the (i-1)-th waypoint
 * @param xi the i-th waypoint
 * @param xip1 the (i+1)-th waypoint
 * @param xip2 the (i+2)-th waypoint
 * @return grad_i the i-th obstacle avoidance gradient to waypoint
 */
CGOptimizer::Vec2d CGOptimizer::_calSmoothTerm(const Vec2d& xi_c2, const Vec2d& xi_c1, const Vec2d& xi,
                                               const Vec2d& xi_p1, const Vec2d& xi_p2)
{
  return w_smooth_ * (xi_c2 - 4.0 * xi_c1 + 6.0 * xi - 4.0 * xi_p1 + xi_p2);
}

/**
 * @brief Curvature term.
 * @param xim1 the (i-1)-th waypoint
 * @param xi the i-th waypoint
 * @param xip1 the (i+1)-th waypoint
 * @return grad_i the i-th obstacle avoidance gradient to waypoint
 */
CGOptimizer::Vec2d CGOptimizer::_calCurvatureTerm(const Vec2d& xi_c1, const Vec2d& xi, const Vec2d& xi_p1)
{
  Vec2d gradient;

  // the curvature needs to be reduced in the world coordinate system to prevent shaking
  double resolution = costmap_ros_->getCostmap()->getResolution();
  Vec2d d_xi = (xi - xi_c1) / (0.1 * resolution);
  Vec2d d_xi_p1 = (xi_p1 - xi) / (0.1 * resolution);

  // orthogonal complements vector
  Vec2d p1, p2;
  auto ort = [](const Vec2d& a, const Vec2d& b) { return a - b * a.innerProd(b) / std::pow(b.length(), 2); };

  // the distance of the vectors
  double abs_dxi = d_xi.length();
  double abs_dxi_p1 = d_xi_p1.length();

  if (abs_dxi > 1e-3 && abs_dxi_p1 > 1e-3)
  {
    // the angular change at the node
    double d_phi = acos(rpp::common::math::clamp(d_xi.innerProd(d_xi_p1) / (abs_dxi * abs_dxi_p1), -1.0, 1.0));
    if (std::fabs(d_phi - 1.0) < rpp::common::math::kMathEpsilon ||
      std::fabs(d_phi + 1.0) < rpp::common::math::kMathEpsilon)
    {
      return gradient;
    }
    double k = d_phi / abs_dxi;

    // if the curvature is smaller then the maximum do nothing
    if (k > k_max_)
    {
      double u = 1.0 / abs_dxi / std::sqrt(1 - std::pow(std::cos(d_phi), 2));
      // calculate the p1 and p2 terms
      p1 = ort(d_xi, -d_xi_p1) / (abs_dxi * abs_dxi_p1);
      p2 = -ort(d_xi_p1, d_xi) / (abs_dxi * abs_dxi_p1);

      // calculate the last terms
      Vec2d s = d_phi * d_xi / std::pow(abs_dxi, 3);
      Vec2d ki = u * (-p1 - p2) - s;
      Vec2d ki_c1 = u * p2 + s;
      Vec2d ki_p1 = u * p1;

      // calculate the gradient
      gradient = w_curvature_ * (0.25 * ki_c1 + 0.5 * ki + 0.25 * ki_p1);

      if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
      {
        R_WARN << "nan values in curvature term";
        return Vec2d();
      }
    }
  }

  return gradient;
}
}  // namespace trajectory_optimization
}  // namespace rpp