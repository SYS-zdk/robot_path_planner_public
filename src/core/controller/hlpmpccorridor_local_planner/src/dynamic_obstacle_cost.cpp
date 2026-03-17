/**
 * @file dynamic_obstacle_cost.cpp
 * @brief Implementation of dynamic obstacle trajectory cost function.
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <cmath>

#include <dynamic_obstacle_cost.h>

namespace hlpmpccorridor_local_planner
{

DynamicObstacleCostFunction::DynamicObstacleCostFunction()
  : peds_(nullptr), scale_(0.0), robot_radius_(0.3), sim_period_(0.05), safe_distance_(0.6)
{
}

bool DynamicObstacleCostFunction::prepare()
{
  return true;
}

double DynamicObstacleCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj)
{
  if (scale_ <= 0.0 || !peds_ || peds_->empty())
    return 0.0;

  double max_cost = 0.0;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
  {
    double px, py, pth;
    traj.getPoint(i, px, py, pth);
    double t = traj.time_delta_ * static_cast<double>(i);

    for (const auto& ped : *peds_)
    {
      double ped_x = ped.x + ped.vx * t;
      double ped_y = ped.y + ped.vy * t;
      double dx = px - ped_x;
      double dy = py - ped_y;
      double dist = std::sqrt(dx * dx + dy * dy);

      double margin = dist - robot_radius_ - safe_distance_;
      if (margin < 0.0)
      {
        return -1.0;
      }

      double c = std::exp(-margin);
      if (c > max_cost)
        max_cost = c;
    }
  }

  return scale_ * max_cost;
}

}  // namespace hlpmpccorridor_local_planner
