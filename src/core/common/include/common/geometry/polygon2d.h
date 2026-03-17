
/**
 * *********************************************************
 *
 * @file: polygon2d.h
 * @brief: 2D polygon
 * @author: Zhang Dingkun
 * @date: 2025-03-07
 * @version: 1.0
 *
 * Copyright (c) 2025, Zhang Dingkun.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_GEOMETRY_POLYGON2D_H_
#define RMP_COMMON_GEOMETRY_POLYGON2D_H_

#include <vector>

#include "common/geometry/vec2d.h"
#include "common/geometry/line_segment2d.h"

namespace rpp
{
namespace common
{
namespace geometry
{
class Polygon2d
{
public:
  /**
   * @brief Empty constructor.
   */
  Polygon2d() = default;

  /**
   * @brief Constructor which takes a vector of points as its vertices.
   * @param points The points to construct the polygon.
   */
  explicit Polygon2d(std::vector<Vec2d> points);

  ~Polygon2d() = default;

  /**
   * @brief Get the vertices of the polygon.
   * @return The vertices of the polygon.
   */
  const std::vector<Vec2d>& points() const;

  /**
   * @brief Get the number of vertices of the polygon.
   * @return The number of vertices of the polygon.
   */
  int num_points() const;

  /**
   * @brief Check if the polygon is convex.
   * @return Whether the polygon is convex or not.
   */
  bool is_convex() const;

  /**
   * @brief Get the area of the polygon.
   * @return The area of the polygon.
   */
  double area() const;

  /**
   * @brief Check if a point is within the polygon.
   * @param point The target point. To check if it is within the polygon.
   * @return Whether a point is within the polygon or not.
   */
  bool isPointIn(const Vec2d& point) const;

  /**
   * @brief Check if a point is on the boundary of the polygon.
   * @param point The target point. To check if it is on the boundary
   *        of the polygon.
   * @return Whether a point is on the boundary of the polygon or not.
   */
  bool isPointOnBoundary(const Vec2d& point) const;

  /**
   * @brief Get the next index of given index.
   * @param at the given index
   * @return next index
   */
  int next(int at) const;

  /**
   * @brief Get the previous index of given index.
   * @param at the given index
   * @return previous index
   */
  int prev(int at) const;

protected:
  void buildFromPoints();

protected:
  std::vector<Vec2d> points_;
  std::vector<LineSegment2d> line_segments_;
  int num_points_ = 0;
  bool is_convex_ = false;
  double area_ = 0.0;
  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
};

}  // namespace geometry
}  // namespace common
}  // namespace rpp
#endif