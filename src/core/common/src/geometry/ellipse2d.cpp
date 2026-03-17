
/**
 * @file: ellipse2d.cpp
 * @brief: geometry: 2D ellipse
 * @author: Yang Haodong
 * @date: 2024.06.08
 * @version: 1.0
 */

#include <climits>

#include "common/math/math_helper.h"
#include "common/geometry/ellipse2d.h"

namespace rpp
{
namespace common
{
namespace geometry
{
/**
 * @brief Ellipse2d class.
 * @param C ellipse matrix (C=RAR^T)
 * @param d translation vector
 */
Ellipse2d::Ellipse2d() : C_(Eigen::Matrix2d::Zero()), d_(Eigen::Vector2d::Zero()){};
Ellipse2d::Ellipse2d(Eigen::Matrix2d C, Eigen::Vector2d d) : C_(C), d_(d){};

const Eigen::Vector2d& Ellipse2d::d() const
{
  return d_;
}

const Eigen::Matrix2d& Ellipse2d::C() const
{
  return C_;
}

void Ellipse2d::setC(Eigen::Matrix2d new_C)
{
  C_ = new_C;
}
void Ellipse2d::setd(Eigen::Vector2d new_d)
{
  d_ = new_d;
}

/**
 * @brief Determining whether a point lies within an ellipse.
 * @param p 2d point
 * @param include_bound with consideration of boundary conditions
 * @return whether a point lies within an ellipse
 */
bool Ellipse2d::isInside(const Vec2d& p, bool include_bound) const
{
  return include_bound ? rpp::common::math::less(distTo(p), 1.0) || rpp::common::math::equal(distTo(p), 1.0) :
                         rpp::common::math::less(distTo(p), 1.0);
}

/**
 * @brief Calculate the points inside the ellipse.
 * @param p_list 2d points
 * @param inside_list the points inside the ellipse
 * @param include_bound with consideration of boundary conditions
 */
void Ellipse2d::insidePoints(const std::vector<Vec2d>& p_list, std::vector<Vec2d>& inside_list,
                             bool include_bound) const
{
  inside_list.clear();
  for (const auto& p : p_list)
  {
    if (isInside(p, include_bound))
    {
      inside_list.emplace_back(p.x(), p.y());
    }
  }
}

/**
 * @brief Calculate the closest point to the ellipse.
 * @param p_list 2d points
 * @return the closest point to the ellipse
 */
Eigen::Vector2d Ellipse2d::closestPoint(const std::vector<Vec2d>& p_list) const
{
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();
  for (const auto& p : p_list)
  {
    double d = distTo(p);
    if (rpp::common::math::large(min_dist, d))
    {
      min_dist = d;
      closest_point[0] = p.x();
      closest_point[1] = p.y();
    }
  }
  return closest_point;
}

/**
 * @brief Calculate the RELATED distance between a point and an ellipse, where 0 < d < 1 signifies
        the point being inside the ellipse, d = 1 signifies the point lying on the ellipse, and
        d > 1 signifies the point being outside the ellipse.
 * @param p 2d point
 * @return the RELATED distance between a point and an ellipse
 */
double Ellipse2d::distTo(const Vec2d& p) const
{
  Eigen::Vector2d temp(p.x(), p.y());
  auto related_vec = C_.inverse() * (temp - d_); //利用了椭圆的几何特性，通过将点投影到椭圆的主轴上来计算距离
  return related_vec.norm();
}
}  // namespace geometry
}  // namespace common
}  // namespace rpp