
/**
 * *********************************************************
 *
 * @file: safety_corridor.cpp
 * @brief: safety corridor abstract class
 * @author: Yang Haodong
 * @date: 2024-09-22
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/safety_corridor/safety_corridor.h"

namespace rpp
{
namespace common
{
namespace safety_corridor
{
namespace
{
constexpr double obstacle_factor = 0.6;
}
/**
 * @brief Constructor which takes a vector of obstacle points as its barrier.
 * @param obstacle_pts The obstacle points to construct the safety corridor.
 */
SafetyCorridor::SafetyCorridor(const Points3d& obstacle_pts) : costmap_ros_(nullptr)
{
  obs_kd_tree_.build(obstacle_pts);
}
/**
 * @brief Constructor which takes a vector of obstacle points as its barrier.
 * @param costmap_ros costmap ROS wrapper
 */
SafetyCorridor::SafetyCorridor(costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(costmap_ros)
{
  Points3d obstacles;
  auto grid2Index = [&](int x, int y) { return x + costmap_ros_->getCostmap()->getSizeInCellsX() * y; };
  for (int x = 0; x < costmap_ros_->getCostmap()->getSizeInCellsX(); x++)
  {
    for (int y = 0; y < costmap_ros_->getCostmap()->getSizeInCellsY(); y++)
    {
      if (costmap_ros_->getCostmap()->getCharMap()[grid2Index(x, y)] == costmap_2d::LETHAL_OBSTACLE)
      {
        double wx, wy;
        costmap_ros_->getCostmap()->mapToWorld(x, y, wx, wy);
        obstacles.emplace_back(wx, wy);
      }
    }
  }
  obs_kd_tree_.build(obstacles);
}

/**
 * @brief Determine whether a given point is an obstacle on the map.
 * @param point the given point
 * @return true if the given point is obstacle
 */
bool SafetyCorridor::isObstacleInMap(const Point3d& point) const
{
  unsigned int pt_x, pt_y;
  costmap_ros_->getCostmap()->worldToMap(point.x(), point.y(), pt_x, pt_y);
  return (pt_x >= costmap_ros_->getCostmap()->getSizeInCellsX() ||
          pt_y >= costmap_ros_->getCostmap()->getSizeInCellsY() ||
          costmap_ros_->getCostmap()->getCharMap()[pt_x + costmap_ros_->getCostmap()->getSizeInCellsX() * pt_y] >=
              costmap_2d::LETHAL_OBSTACLE * obstacle_factor);
}

bool SafetyCorridor::isObstacleInMap(const Point2i& point) const
{
  return (
      point.x() >= costmap_ros_->getCostmap()->getSizeInCellsX() ||
      point.y() >= costmap_ros_->getCostmap()->getSizeInCellsY() ||
      costmap_ros_->getCostmap()->getCharMap()[point.x() + costmap_ros_->getCostmap()->getSizeInCellsX() * point.y()] >=
          costmap_2d::LETHAL_OBSTACLE * obstacle_factor);
}

}  // namespace safety_corridor
}  // namespace common
}  // namespace rpp