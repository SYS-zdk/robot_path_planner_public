
/***********************************************************
 *
 * @file: lbfgs_optimizer.h
 * @brief: Trajectory optimization using L-BFGS method
 * @author: Zhang Dingkun
 * @date: 2026-02-06
 * @version: 1.0
 *
 * L-BFGS path smoother for Hybrid A* and other planners.
 * Optimizes waypoint (x,y) positions to minimize a weighted
 * combination of obstacle, smoothness, and curvature costs.
 *
 * Reference:
 *   - LBFGS-Lite: https://github.com/ZJU-FAST-Lab/LBFGS-Lite
 *   - hybridAstar_lbfgsSmooth: https://github.com/USE-jx/hybridAstar_lbfgsSmooth
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_LBFGS_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_LBFGS_OPTIMIZER_H_

#include <Eigen/Dense>

#include "distance_layer.h"

#include "common/geometry/vec2d.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{
/**
 * @brief LBFGS-based trajectory optimizer.
 *
 * Takes a path (sequence of waypoints) and optimizes the interior
 * waypoint positions by minimizing:
 *   f(x) = w_obs * f_obstacle + w_smo * f_smooth + w_cur * f_curvature
 *
 * The first and last two waypoints are held fixed to preserve
 * start/goal position and heading constraints.
 *
 * Reference:
 *   - LBFGS-Lite: https://github.com/ZJU-FAST-Lab/LBFGS-Lite
 *   - hybridAstar_lbfgsSmooth: https://github.com/USE-jx/hybridAstar_lbfgsSmooth
 */
class LBFGSOptimizer : public Optimizer
{
private:
  using Vec2d = rpp::common::geometry::Vec2d;

public:
  LBFGSOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int max_iter,
                 double obs_dist_max, double k_max,
                 double w_obstacle, double w_smooth, double w_curvature);
  ~LBFGSOptimizer() = default;

  bool run(const Points3d& waypoints) override;
  bool run(const Trajectory3d& traj) override;
  bool getTrajectory(Trajectory3d& traj) override;

private:
  /**
   * @brief Core optimization routine using L-BFGS
   */
  bool optimize(const Points3d& waypoints);

  /**
   * @brief L-BFGS cost function callback (static, passed to lbfgs_optimize).
   *
   * Computes total cost and gradient using 3 terms:
   *   1. Obstacle cost: distance-field based penalty
   *   2. Smooth cost: 5-point stencil (fourth-order finite difference)
   *   3. Curvature cost: triple-kappa weighted penalty (0.25, 0.5, 0.25)
   */
  static double costFunction(void* instance,
                             const Eigen::VectorXd& x,
                             Eigen::VectorXd& g);

private:
  int max_iter_;
  double obs_dist_max_;
  double k_max_;
  double w_obstacle_;
  double w_smooth_;
  double w_curvature_;
  Points3d path_opt_;

  boost::shared_ptr<costmap_2d::DistanceLayer> distance_layer_;
};

}  // namespace trajectory_optimization
}  // namespace rpp
#endif  // RMP_TRAJECTORY_OPTIMIZATION_LBFGS_OPTIMIZER_H_
