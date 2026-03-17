
/**
 * *********************************************************
 *
 * @file: convex_safety_corridor.h
 * @brief: convex safety corridor class
 * @author: Yang Haodong
 * @date: 2025-01-12
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_SAFETYCORRIDOR_CONVEX_SAFETY_CORRIDOR_H_
#define RMP_COMMON_SAFETYCORRIDOR_CONVEX_SAFETY_CORRIDOR_H_

#include "common/geometry/polygon2d.h"
#include "common/geometry/ellipse2d.h"
#include "common/safety_corridor/safety_corridor.h"

#include "common/safety_corridor/ackermann_config.h"

namespace rpp
{
namespace common
{
namespace safety_corridor
{
class ConvexSafetyCorridor : public SafetyCorridor
{
private:
  using Vec2d = rpp::common::geometry::Vec2d;
  using Polygon2d = rpp::common::geometry::Polygon2d;
  using Ellipse2d = rpp::common::geometry::Ellipse2d;

  rpp::AckermannConfig ackermann_config_;

public:
  /**
   * @brief Empty constructor.
   */
  ConvexSafetyCorridor();

  /**
   * @brief Constructor which takes a vector of obstacle points as its barrier.
   * @param obstacle_pts The obstacle points to construct the safety corridor.
   * @param safety_range The safety range to limit polygon space [m]
   */
  ConvexSafetyCorridor(const Points3d& obstacle_pts, double safety_range);

  /**
   * @brief Constructor which takes a vector of obstacle points as its barrier.
   * @param obstacle_pts The obstacle points to construct the safety corridor.
   * @param safety_range The safety range to limit polygon space [m]
   * @param ackermann_cfg Ackermann footprint/config used by corridor geometry
   */
  ConvexSafetyCorridor(const Points3d& obstacle_pts, double safety_range, const rpp::AckermannConfig& ackermann_cfg);

  /**
   * @brief Constructor which takes a vector of obstacle points as its barrier.
   * @param costmap_ros costmap ROS wrapper
   * @param safety_range The safety range to limit polygon space [m]
   */
  ConvexSafetyCorridor(costmap_2d::Costmap2DROS* costmap_ros, double safety_range, const rpp::AckermannConfig& ackermann_cfg);

  virtual ~ConvexSafetyCorridor() = default;

  /**
   * @brief Construct a safety corridor composed of polygons based on waypoints and environmental obstacles。
   * @param waypoints The waypoints
   * @param decomp_polygons The decomposed polygons to construct the safety corridor.
   * @return true if successful construction.
   */
  bool decompose(const Points3d& waypoints, std::vector<Polygon2d>& decomp_polygons) const;

private:
  /**
   * @brief Polygon initialization.
   * @param pf waypoint front
   * @param pr waypoint rear
   * @return initial polygon
   */
  Polygon2d _initPolygon(const Vec2d& pf, const Vec2d& pr) const;

  /**
   * @brief Build ellipse of corridor.
   * @param pf waypoint front
   * @param pr waypoint rear
   * @param obs_points obstacles that inside the initial polygon
   * @return maximum ellipse
   */
  Ellipse2d _buildEllipse(const Vec2d& pf, const Vec2d& pr, const std::vector<Vec2d>& obs_points) const;

  /**
   * @brief Build polygon of corridor.
   * @param ellipse maximum ellipse
   * @param init_polygon initial polygon
   * @param obs_points obstacles that inside the initial polygon
   * @return maximum polygon
   */
  Polygon2d _buildPolygon(const Ellipse2d& ellipse, const Polygon2d& init_polygon,
                          const std::vector<Vec2d>& obs_points) const;

protected:
  // The safety range to limit polygon space [m]
  double safety_range_{ 0.0 };
};
}  // namespace safety_corridor
}  // namespace common
}  // namespace rpp
#endif