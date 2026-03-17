
/***********************************************************
 *
 * @file: conjugate_optimizer.h
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
#ifndef RMP_TRAJECTORY_OPTIMIZATION_CONJUGATE_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_CONJUGATE_OPTIMIZER_H_

#include "distance_layer.h"

#include "common/geometry/vec2d.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{
class CGOptimizer : public Optimizer
{
private:
  using Vec2d = rpp::common::geometry::Vec2d;
  using DistanceField = std::vector<std::vector<double>>;

public:
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
  CGOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int max_iter, double alpha, double obs_dist_max, double k_max,
              double w_obstacle, double w_smooth, double w_curvature);
  ~CGOptimizer() = default;

  /**
   * @brief Running trajectory optimization
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool run(const Points3d& waypoints);
  bool run(const Trajectory3d& traj);

  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  bool getTrajectory(Trajectory3d& traj);

protected:
  /**
   * @brief trajectory optimization executor
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool optimize(const Points3d& waypoints);

private:
  /**
   * @brief Static obstacle avoidance term.
   * @param xi the i-th waypoint
   * @return grad_i the i-th obstacle avoidance gradient to waypoint
   */
  Vec2d _calObstacleTerm(const Vec2d& xi, const boost::shared_ptr<costmap_2d::DistanceLayer>& dist_layer);

  /**
   * @brief Smooth term.
   * @param xim2 the (i-2)-th waypoint
   * @param xim1 the (i-1)-th waypoint
   * @param xi the i-th waypoint
   * @param xip1 the (i+1)-th waypoint
   * @param xip2 the (i+2)-th waypoint
   * @return grad_i the i-th obstacle avoidance gradient to waypoint
   */
  Vec2d _calSmoothTerm(const Vec2d& xi_c2, const Vec2d& xi_c1, const Vec2d& xi, const Vec2d& xi_p1, const Vec2d& xi_p2);

  /**
   * @brief Curvature term.
   * @param xim1 the (i-1)-th waypoint
   * @param xi the i-th waypoint
   * @param xip1 the (i+1)-th waypoint
   * @return grad_i the i-th obstacle avoidance gradient to waypoint
   */
  Vec2d _calCurvatureTerm(const Vec2d& xi_c1, const Vec2d& xi, const Vec2d& xi_p1);

private:
  int max_iter_;         // the maximum iterations for optimization
  double alpha_;         // learning rate
  double obs_dist_max_;  // the maximum distance to obstacle (pixel)
  double k_max_;         // the maximum curvature
  double w_obstacle_;    // the weight for obstacle avoidance
  double w_smooth_;      // the weight for smooth
  double w_curvature_;   // the weight for curvature
  Points3d path_opt_;    // optimized path
};
}  // namespace trajectory_optimization
}  // namespace rpp
#endif