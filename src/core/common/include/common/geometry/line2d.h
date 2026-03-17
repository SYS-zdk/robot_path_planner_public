
/**
 * *********************************************************
 *
 * @file: line2d.h
 * @brief: 2D line 
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
#ifndef RMP_COMMON_GEOMETRY_LINE2D_H_
#define RMP_COMMON_GEOMETRY_LINE2D_H_

#include <vector>

#include "common/geometry/vec2d.h"

namespace rpp
{
namespace common
{
namespace geometry
{
class Line2d
{
public:
  /**
   * @brief Line2d class.
   * @param n normal vector
   * @param d translation vector
   * @note y = n^T(x - d)
   */
  Line2d() = default;
  Line2d(Vec2d n, Vec2d d);
  ~Line2d() = default;

  const Vec2d& n() const;
  const Vec2d& d() const;

  /**
 * @brief Calculate the signed distance to a line, where "+" indicates being on the same side
        as the normal vector, and "-" indicates being on the opposite side.
 * @param p 2d point
 * @return the signed distance to the line
 */
  double distTo(const Vec2d& p) const;

  /**
   * @brief Calculate the vertices of feasible domain composed of several Line2d.
   * @param feasible_domain feasible domain composed of several Line2d
   * @param vertices the vertices of feasible domain
   * @return true if calculate successfully
   */
  static bool getVertexOfFeasibleDomain(const std::vector<Line2d>& feasible_domain, std::vector<Vec2d>& vertices);

private:
  Vec2d n_;
  Vec2d d_;
};
}  // namespace geometry
}  // namespace common
}  // namespace rpp
#endif