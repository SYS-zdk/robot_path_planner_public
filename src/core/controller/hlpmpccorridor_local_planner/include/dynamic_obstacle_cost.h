/**
 * @file dynamic_obstacle_cost.h
 * @brief Trajectory cost function for dynamic pedestrians (moving obstacle clearance).
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_LOCAL_PLANNER_DYNAMIC_OBSTACLE_COST_H_
#define HYBRID_LOCAL_PLANNER_DYNAMIC_OBSTACLE_COST_H_

#include <base_local_planner/trajectory_cost_function.h>

namespace hlpmpccorridor_local_planner
{

struct DynamicPed
{
  double x;
  double y;
  double vx;
  double vy;
};

class DynamicObstacleCostFunction : public base_local_planner::TrajectoryCostFunction
{
public:
  DynamicObstacleCostFunction();

  void setScale(double scale)
  {
    scale_ = scale;
  }

  void setRobotRadius(double r)
  {
    robot_radius_ = r;
  }

  void setDynamicPeds(const std::vector<DynamicPed>* peds)
  {
    peds_ = peds;
  }

  void setSimPeriod(double sim_period)
  {
    sim_period_ = sim_period;
  }

  // Additional clearance beyond robot radius (meters). Trajectories whose
  // predicted distance to a moving pedestrian is < robot_radius + safe_distance
  // are rejected (score = -1).
  void setSafeDistance(double d)
  {
    safe_distance_ = d;
  }

  virtual bool prepare();

  virtual double scoreTrajectory(base_local_planner::Trajectory& traj);

private:
  const std::vector<DynamicPed>* peds_;
  double scale_;
  double robot_radius_;
  double sim_period_;
  double safe_distance_;
};

}  // namespace hlpmpccorridor_local_planner

#endif
