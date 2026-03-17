
/**
 * *********************************************************
 *
 * @file: line_segment2d.h
 * @brief: 2D Line segment class
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
#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_helper.h"
#include "common/geometry/line_segment2d.h"

namespace rpp
{
namespace common
{
namespace geometry
{
namespace
{
bool IsWithin(double val, double bound1, double bound2)
{
  if (bound1 > bound2)
  {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - rpp::common::math::kMathEpsilon && val <= bound2 + rpp::common::math::kMathEpsilon;
}

}  // namespace

LineSegment2d::LineSegment2d()
{
  unit_direction_ = Vec2d(1, 0);
}

LineSegment2d::LineSegment2d(const Vec2d& start, const Vec2d& end) : start_(start), end_(end)
{
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ = (length_ <= rpp::common::math::kMathEpsilon ? Vec2d(0, 0) : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.angle();
}

Vec2d LineSegment2d::rotate(const double angle)
{
  Vec2d diff_vec = end_ - start_;
  diff_vec.selfRotate(angle);
  return start_ + diff_vec;
}

double LineSegment2d::length() const
{
  return length_;
}

double LineSegment2d::length_sqr() const
{
  return length_ * length_;
}

double LineSegment2d::distanceTo(const Vec2d& point) const
{
  if (length_ <= rpp::common::math::kMathEpsilon)
  {
    return point.distanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0)
  {
    return hypot(x0, y0);
  }
  if (proj >= length_)
  {
    return point.distanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::distanceTo(const Vec2d& point, Vec2d* const nearest_pt) const
{
  if (length_ <= rpp::common::math::kMathEpsilon)
  {
    *nearest_pt = start_;
    return point.distanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0)
  {
    *nearest_pt = start_;
    return std::hypot(x0, y0);
  }
  if (proj > length_)
  {
    *nearest_pt = end_;
    return point.distanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::distanceSquareTo(const Vec2d& point) const
{
  if (length_ <= rpp::common::math::kMathEpsilon)
  {
    return point.distanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0)
  {
    return x0 * x0 + y0 * y0;
  }
  if (proj >= length_)
  {
    return point.distanceSquareTo(end_);
  }
  return std::pow(x0 * unit_direction_.y() - y0 * unit_direction_.x(), 2);
}

double LineSegment2d::distanceSquareTo(const Vec2d& point, Vec2d* const nearest_pt) const
{
  if (length_ <= rpp::common::math::kMathEpsilon)
  {
    *nearest_pt = start_;
    return point.distanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0)
  {
    *nearest_pt = start_;
    return x0 * x0 + y0 * y0;
  }
  if (proj >= length_)
  {
    *nearest_pt = end_;
    return point.distanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::pow(x0 * unit_direction_.y() - y0 * unit_direction_.x(), 2);
}

bool LineSegment2d::isPointIn(const Vec2d& point) const
{
  if (length_ <= rpp::common::math::kMathEpsilon)
  {
    return std::abs(point.x() - start_.x()) <= rpp::common::math::kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= rpp::common::math::kMathEpsilon;
  }
  const double prod = rpp::common::math::crossProd(point, start_, end_);
  if (std::abs(prod) > rpp::common::math::kMathEpsilon)
  {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) && IsWithin(point.y(), start_.y(), end_.y());
}

double LineSegment2d::projectOntoUnit(const Vec2d& point) const
{
  return unit_direction_.innerProd(point - start_);
}

double LineSegment2d::productOntoUnit(const Vec2d& point) const
{
  return unit_direction_.crossProd(point - start_);
}

bool LineSegment2d::hasIntersect(const LineSegment2d& other_segment) const
{
  Vec2d point;
  return getIntersect(other_segment, &point);
}

bool LineSegment2d::getIntersect(const LineSegment2d& other_segment, Vec2d* const point) const
{
  if (isPointIn(other_segment.start()))
  {
    *point = other_segment.start();
    return true;
  }
  if (isPointIn(other_segment.end()))
  {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.isPointIn(start_))
  {
    *point = start_;
    return true;
  }
  if (other_segment.isPointIn(end_))
  {
    *point = end_;
    return true;
  }
  if (length_ <= rpp::common::math::kMathEpsilon || other_segment.length() <= rpp::common::math::kMathEpsilon)
  {
    return false;
  }
  const double cc1 = rpp::common::math::crossProd(start_, end_, other_segment.start());
  const double cc2 = rpp::common::math::crossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -rpp::common::math::kMathEpsilon)
  {
    return false;
  }
  const double cc3 = rpp::common::math::crossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 = rpp::common::math::crossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -rpp::common::math::kMathEpsilon)
  {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio), start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double LineSegment2d::getPerpendicularFoot(const Vec2d& point, Vec2d* const foot_point) const
{
  if (length_ <= rpp::common::math::kMathEpsilon)
  {
    *foot_point = start_;
    return point.distanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

}  // namespace geometry
}  // namespace common
}  // namespace rpp
