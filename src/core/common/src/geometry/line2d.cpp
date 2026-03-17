
/**
 * *********************************************************
 *
 * @file: line2d.cpp
 * @brief: geometry: line2d class
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

#include "common/geometry/line2d.h"
#include "common/math/math_helper.h"

namespace rpp
{
namespace common
{
namespace geometry
{
/**
 * @brief Line2d class.
 * @param n normal vector
 * @param d translation vector
 * @note y = n^T(x - d)
 */
Line2d::Line2d(Vec2d n, Vec2d d) : n_(n), d_(d){};

const Vec2d& Line2d::n() const
{
  return n_;
};

const Vec2d& Line2d::d() const
{
  return d_;
};

/**
* @brief Calculate the signed distance to a line, where "+" indicates being on the same side
      as the normal vector, and "-" indicates being on the opposite side.
* @param p 2d point
* @return the signed distance to the line
*/
double Line2d::distTo(const Vec2d& p) const
{
  return n_.innerProd(p - d_);
}

/**
 * @brief Calculate the vertices of feasible domain composed of several Line2d.
 * @param feasible_domain feasible domain composed of several Line2d
 * @param vertices the vertices of feasible domain
 * @return true if calculate successfully
 */
bool Line2d::getVertexOfFeasibleDomain(const std::vector<Line2d>& feasible_domain, std::vector<Vec2d>& vertices)
{
  vertices.clear();
  std::vector<Vec2d> inter_points;

  auto isFeasible = [&](const Vec2d& p) {
    for (const auto& hyper_plane : feasible_domain)
    {
      if (rpp::common::math::large(hyper_plane.distTo(p), 0.0))
      {
        return false;
      }
    }
    return true;
  };

  // Get the intersection points of all Line2d
  int num = static_cast<int>(feasible_domain.size());
  for (int i = 0; i < num; i++)
  {
    for (int j = i + 1; j < num; j++)
    {
      const auto& plane_1 = feasible_domain[i];
      const auto& plane_2 = feasible_domain[j];
      double det = plane_1.n().crossProd(plane_2.n());

      if (std::fabs(det) < rpp::common::math::kMathEpsilon)
      {  // parallel
        continue;
      }
      else
      {  // intersection
        double b_0 = plane_1.n().innerProd(plane_1.d());
        double b_1 = plane_2.n().innerProd(plane_2.d());
        Vec2d inter_pt((plane_2.n().y() * b_0 - plane_1.n().y() * b_1) / det,
                       (plane_1.n().x() * b_1 - plane_2.n().x() * b_0) / det);
        if (isFeasible(inter_pt))
        {
          inter_points.push_back(std::move(inter_pt));
        }
      }
    }
  }

  if (inter_points.size() > 0)
  {
    // Sort vertices
    rpp::common::math::sortPoints(inter_points);
    vertices = std::move(inter_points);
  }

  return true;
}

}  // namespace geometry
}  // namespace common
}  // namespace rpp