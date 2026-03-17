
/***********************************************************
 *
 * @file: optimizer.cpp
 * @brief: Trajectory optimization
 * @author: Yang Haodong
 * @date: 2023-12-29
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{
/**
 * @brief Construct a new trajectory optimizer object
 * @param costmap_ros costmap ROS wrapper
 */
Optimizer::Optimizer() : costmap_ros_(nullptr), nx_(0), ny_(0), map_size_(0)
{
}

Optimizer::Optimizer(costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(costmap_ros)
  , nx_(costmap_ros->getCostmap()->getSizeInCellsX())
  , ny_(costmap_ros->getCostmap()->getSizeInCellsY())
  , map_size_(costmap_ros->getCostmap()->getSizeInCellsX() * costmap_ros->getCostmap()->getSizeInCellsY())
{
}

/**
 * @brief Judge whether the grid(x, y) is inside the map
 * @param x grid coordinate x
 * @param y grid coordinate y
 * @return true if inside the map else false
 */
bool Optimizer::_insideMap(unsigned int x, unsigned int y)
{
  return (x < nx_ && x >= 0 && y < ny_ && y >= 0) ? true : false;
}

/**
 * @brief Judge whether the grid(x, y) is inside the map
 * @param x world coordinate x
 * @param y world coordinate y
 * @return true if inside the map else false
 */
bool Optimizer::_insideMap(double x, double y)
{
  unsigned int ix, iy;
  costmap_ros_->getCostmap()->worldToMap(x, y, ix, iy);
  return (ix < nx_ && ix >= 0 && iy < ny_ && iy >= 0) ? true : false;
}
}  // namespace trajectory_optimization
}  // namespace rpp