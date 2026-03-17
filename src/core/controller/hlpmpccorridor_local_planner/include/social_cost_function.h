/**
 * @file social_cost_function.h
 * @brief Trajectory cost function that queries SocialLayer costs along a trajectory.
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_LOCAL_PLANNER_SOCIAL_COST_FUNCTION_H_
#define HYBRID_LOCAL_PLANNER_SOCIAL_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>

namespace hlpmpccorridor_local_planner
{

/**
 * @class SocialCostFunction
 * @brief Evaluates trajectories based on social comfort zones around pedestrians
 *
 * This cost function accesses social costs from the SocialLayer without modifying
 * the master_grid, maintaining semantic clarity between true obstacles and social zones.
 * It integrates directly with the HybridPlanner for explicit trajectory evaluation.
 */
class SocialCostFunction : public base_local_planner::TrajectoryCostFunction
{
public:
  SocialCostFunction();

  void setScale(double scale)
  {
    scale_ = scale;
  }

  /**
   * @brief Set the layered costmap to access the social layer
   */
  void setLayeredCostmap(costmap_2d::LayeredCostmap* layered_costmap)
  {
    layered_costmap_ = layered_costmap;
  }

  /**
   * @brief Enable/disable this cost function
   */
  void setEnabled(bool enabled)
  {
    enabled_ = enabled;
  }

  virtual bool prepare();

  /**
   * @brief Score a trajectory based on social cost
   * 
   * Integrates social costs from the SocialLayer along the trajectory path.
   * Higher scores indicate the trajectory passes closer to pedestrians (worse).
   */
  virtual double scoreTrajectory(base_local_planner::Trajectory& traj);

private:
  costmap_2d::LayeredCostmap* layered_costmap_;
  double scale_;
  bool enabled_;
};

}  // namespace hlpmpccorridor_local_planner

#endif
