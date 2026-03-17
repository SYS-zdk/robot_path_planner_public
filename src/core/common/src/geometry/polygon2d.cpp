
/**
 * *********************************************************
 *
 * @file: polygon2d.cpp
 * @brief: geometry: 2D polygon
 * @author: Yang Haodong
 * @date: 2024-9-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>
#include <climits>

#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "common/geometry/polygon2d.h"

namespace rpp
{
namespace common
{
namespace geometry
{
/**
 * @brief Constructor which takes a vector of points as its vertices.
 * @param points The points to construct the polygon.
 */
Polygon2d::Polygon2d(std::vector<Vec2d> points) : points_(std::move(points))
{
  buildFromPoints();
}

/**
 * @brief Get the vertices of the polygon.
 * @return The vertices of the polygon.
 */
const std::vector<Vec2d>& Polygon2d::points() const
{
  return points_;
}

/**
 * @brief Get the number of vertices of the polygon.
 * @return The number of vertices of the polygon.
 */
int Polygon2d::num_points() const
{
  return num_points_;
}

/**
 * @brief Check if the polygon is convex.
 * @return Whether the polygon is convex or not.
 */
bool Polygon2d::is_convex() const
{
  return is_convex_;
}

/**
 * @brief Get the area of the polygon.
 * @return The area of the polygon.
 */
double Polygon2d::area() const
{
  return area_;
}

/**
 * @brief Check if a point is within the polygon.
 * @param point The target point. To check if it is within the polygon.
 * @return Whether a point is within the polygon or not.
 */
bool Polygon2d::isPointIn(const Vec2d& point) const
{
  CHECK_GE(points_.size(), 3U);
  if (isPointOnBoundary(point))
  {
    return true;
  }
  int j = num_points_ - 1;
  int c = 0;
  for (int i = 0; i < num_points_; ++i)
  {
    if ((points_[i].y() > point.y()) != (points_[j].y() > point.y()))
    {
      const double side = rpp::common::math::crossProd(point, points_[i], points_[j]);
      if (points_[i].y() < points_[j].y() ? side > 0.0 : side < 0.0)
      {
        ++c;
      }
    }
    j = i;
  }
  return c & 1;
}

/**
 * @brief Check if a point is on the boundary of the polygon.
 * @param point The target point. To check if it is on the boundary
 *        of the polygon.
 * @return Whether a point is on the boundary of the polygon or not.
 */
bool Polygon2d::isPointOnBoundary(const Vec2d& point) const
{
  CHECK_GE(points_.size(), 3U);
  return std::any_of(line_segments_.begin(), line_segments_.end(),
                     [&](const LineSegment2d& poly_seg) { return poly_seg.isPointIn(point); });
}

void Polygon2d::buildFromPoints()
{
  num_points_ = static_cast<int>(points_.size());
  CHECK_GE(num_points_, 3);

  // Make sure the points are in ccw order.
  area_ = 0.0;
  for (int i = 1; i < num_points_; ++i)
  {
    area_ += rpp::common::math::crossProd(points_[0], points_[i - 1], points_[i]);
  }
  if (area_ < 0)
  {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;
  CHECK_GT(area_, rpp::common::math::kMathEpsilon);

  // Construct line_segments.
  line_segments_.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i)
  {
    line_segments_.emplace_back(points_[i], points_[next(i)]);
  }

  // Check convexity.
  is_convex_ = true;
  for (int i = 0; i < num_points_; ++i)
  {
    if (rpp::common::math::crossProd(points_[prev(i)], points_[i], points_[next(i)]) <=
        -rpp::common::math::kMathEpsilon)
    {
      is_convex_ = false;
      break;
    }
  }

  // Compute aabox.
  min_x_ = points_[0].x();
  max_x_ = points_[0].x();
  min_y_ = points_[0].y();
  max_y_ = points_[0].y();
  for (const auto& point : points_)
  {
    min_x_ = std::min(min_x_, point.x());
    max_x_ = std::max(max_x_, point.x());
    min_y_ = std::min(min_y_, point.y());
    max_y_ = std::max(max_y_, point.y());
  }
}

/**
 * @brief Get the next index of given index.
 * @param at the given index
 * @return next index
 */
int Polygon2d::next(int at) const
{
  return at >= num_points_ - 1 ? 0 : at + 1;
}

/**
 * @brief Get the previous index of given index.
 * @param at the given index
 * @return previous index
 */
int Polygon2d::prev(int at) const
{
  return at == 0 ? num_points_ - 1 : at - 1;
}

}  // namespace geometry
}  // namespace common
}  // namespace rpp