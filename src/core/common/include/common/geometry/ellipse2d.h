
/**
 * *********************************************************
 *
 * @file: ellipse2d.h
 * @brief: 2D ellipse
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
#ifndef RMP_COMMON_GEOMETRY_ELLIPSE2D_H_
#define RMP_COMMON_GEOMETRY_ELLIPSE2D_H_

#include <vector>

#include <Eigen/Dense>

#include "common/geometry/vec2d.h"

namespace rpp
{
namespace common
{
namespace geometry
{
class Ellipse2d
{
public:
  /**
   * @brief Ellipse2d class.
   * @param C ellipse matrix (C=RAR^T)
   * @param d translation vector
   */
  Ellipse2d();
  Ellipse2d(Eigen::Matrix2d C, Eigen::Vector2d d);
  ~Ellipse2d() = default;

  const Eigen::Vector2d& d() const;
  const Eigen::Matrix2d& C() const;
  void setC(Eigen::Matrix2d new_C);
  void setd(Eigen::Vector2d new_d);

  /**
   * @brief Determining whether a point lies within an ellipse.
   * @param p 2d point
   * @param include_bound with consideration of boundary conditions
   * @return whether a point lies within an ellipse
   */
  bool isInside(const Vec2d& p, bool include_bound) const;
  /**
   * @brief Calculate the points inside the ellipse.
   * @param p_list 2d points
   * @param inside_list the points inside the ellipse
   * @param include_bound with consideration of boundary conditions
   */
  void insidePoints(const std::vector<Vec2d>& p_list, std::vector<Vec2d>& inside_list,
                    bool include_bound = true) const;
  /**
   * @brief Calculate the closest point to the ellipse.
   * @param p_list 2d points
   * @return the closest point to the ellipse
   */
  Eigen::Vector2d closestPoint(const std::vector<Vec2d>& p_list) const;
  /**
 * @brief Calculate the RELATED distance between a point and an ellipse, where 0 < d < 1 signifies
        the point being inside the ellipse, d = 1 signifies the point lying on the ellipse, and
        d > 1 signifies the point being outside the ellipse.
 * @param p 2d point
 * @return the RELATED distance between a point and an ellipse
 */
  double distTo(const Vec2d& p) const;

private:
  Eigen::Matrix2d C_;
  Eigen::Vector2d d_;
};

}  // namespace geometry
}  // namespace common
}  // namespace rpp
#endif