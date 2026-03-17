
/**
 * *********************************************************
 *
 * @file: safety_corridor.h
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
#ifndef RMP_COMMON_SAFETYCORRIDOR_SAFETY_CORRIDOR_H_
#define RMP_COMMON_SAFETYCORRIDOR_SAFETY_CORRIDOR_H_

#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/point.h"
#include "common/structure/kd_tree.h"

namespace rpp
{
namespace common
{
namespace safety_corridor
{
class SafetyCorridor
{
protected:
  using Point2i = rpp::common::geometry::Point2i;
  using Point3d = rpp::common::geometry::Point3d;
  using Points3d = rpp::common::geometry::Points3d;

public:
  /**
   * @brief Empty constructor.
   */
  SafetyCorridor() = default;
  /**
   * @brief Constructor which takes a vector of obstacle points as its barrier.
   * @param obstacle_pts The obstacle points to construct the safety corridor.
   */
  SafetyCorridor(const Points3d& obstacle_pts);
  /**
   * @brief Constructor which takes a vector of obstacle points as its barrier.
   * @param costmap_ros costmap ROS wrapper
   */
  SafetyCorridor(costmap_2d::Costmap2DROS* costmap_ros);
  virtual ~SafetyCorridor() = default;

protected:
  /**
   * @brief Determine whether a given point is an obstacle on the map.
   * @param point the given point
   * @return true if the given point is obstacle
   */
  bool isObstacleInMap(const Point3d& point) const;
  bool isObstacleInMap(const Point2i& point) const;

protected:
  // costmap ROS wrapper
  costmap_2d::Costmap2DROS* costmap_ros_;
  // obstacles kd-tree for quickly indexing
  rpp::common::structure::KDTree<Point3d> obs_kd_tree_;
};

}  // namespace safety_corridor
}  // namespace common
}  // namespace rpp

#endif