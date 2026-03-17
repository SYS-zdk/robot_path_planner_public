
/**
 * @file dubins_curve.cpp
 * @brief Dubins curve generation implementation.
 * @author Yang Haodong
 * @date 2025-03-07
 * @version 1.0
 */

#include <cmath>
#include <limits>
#include <tuple>

#include "common/geometry/curve/dubins_curve.h"

namespace rpp
{
namespace common
{
namespace geometry
{
namespace
{
enum
{
  DUBINS_L = 0,
  DUBINS_S = 1,
  DUBINS_R = 2,
};

inline double mod2pi(double theta)
{
  return rpp::common::math::mod2pi(theta);
}

inline double clampUnit(double v)
{
  if (v > 1.0)
  {
    return 1.0;
  }
  if (v < -1.0)
  {
    return -1.0;
  }
  return v;
}
}  // namespace

DubinsCurve::DubinsCurve() : Curve(0.1), max_curv_(0.25)
{
}

DubinsCurve::DubinsCurve(double step, double max_curv) : Curve(step), max_curv_(max_curv)
{
}

DubinsCurve::~DubinsCurve() = default;

bool DubinsCurve::run(const Points2d& points, Points3d& path)
{
  if (points.size() < 2)
  {
    return false;
  }

  Points3d poses;
  poses.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    double theta = 0.0;
    if (i + 1 < points.size())
    {
      theta = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
    }
    else
    {
      theta = poses.back().theta();
    }
    poses.emplace_back(points[i].x(), points[i].y(), theta);
  }

  return run(poses, path);
}

bool DubinsCurve::run(const Points3d& points, Points3d& path)
{
  if (points.size() < 2)
  {
    return false;
  }

  path.clear();
  Points3d segment;
  for (size_t i = 0; i + 1 < points.size(); ++i)
  {
    segment.clear();
    if (!generation(points[i], points[i + 1], segment))
    {
      return false;
    }

    if (path.empty())
    {
      path.insert(path.end(), segment.begin(), segment.end());
    }
    else
    {
      // avoid duplicating the connection point
      path.insert(path.end(), segment.begin() + 1, segment.end());
    }
  }

  return !path.empty();
}

void DubinsCurve::LSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);

  const double tmp0 = dist + sa - sb;
  const double p_sq = 2.0 + dist * dist - 2.0 * std::cos(alpha - beta) + 2.0 * dist * (sa - sb);

  if (p_sq < 0.0)
  {
    length = { 0.0, 0.0, 0.0 };
    mode = { DUBINS_S, DUBINS_S, DUBINS_S };
    return;
  }

  const double tmp1 = std::atan2(cb - ca, tmp0);
  const double t = mod2pi(-alpha + tmp1);
  const double p = std::sqrt(p_sq);
  const double q = mod2pi(beta - tmp1);

  length = { t, p, q };
  mode = { DUBINS_L, DUBINS_S, DUBINS_L };
}

void DubinsCurve::RSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);

  const double tmp0 = dist - sa + sb;
  const double p_sq = 2.0 + dist * dist - 2.0 * std::cos(alpha - beta) + 2.0 * dist * (-sa + sb);

  if (p_sq < 0.0)
  {
    length = { 0.0, 0.0, 0.0 };
    mode = { DUBINS_S, DUBINS_S, DUBINS_S };
    return;
  }

  const double tmp1 = std::atan2(ca - cb, tmp0);
  const double t = mod2pi(alpha - tmp1);
  const double p = std::sqrt(p_sq);
  const double q = mod2pi(-beta + tmp1);

  length = { t, p, q };
  mode = { DUBINS_R, DUBINS_S, DUBINS_R };
}

void DubinsCurve::LSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);

  const double p_sq = -2.0 + dist * dist + 2.0 * std::cos(alpha - beta) + 2.0 * dist * (sa + sb);
  if (p_sq < 0.0)
  {
    length = { 0.0, 0.0, 0.0 };
    mode = { DUBINS_S, DUBINS_S, DUBINS_S };
    return;
  }

  const double p = std::sqrt(p_sq);
  const double tmp0 = std::atan2(-ca - cb, dist + sa + sb);
  const double tmp1 = std::atan2(-2.0, p);
  const double t = mod2pi(-alpha + tmp0 - tmp1);
  const double q = mod2pi(-beta + tmp0 - tmp1);

  length = { t, p, q };
  mode = { DUBINS_L, DUBINS_S, DUBINS_R };
}

void DubinsCurve::RSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);

  const double p_sq = -2.0 + dist * dist + 2.0 * std::cos(alpha - beta) - 2.0 * dist * (sa + sb);
  if (p_sq < 0.0)
  {
    length = { 0.0, 0.0, 0.0 };
    mode = { DUBINS_S, DUBINS_S, DUBINS_S };
    return;
  }

  const double p = std::sqrt(p_sq);
  const double tmp0 = std::atan2(ca + cb, dist - sa - sb);
  const double tmp1 = std::atan2(2.0, p);
  const double t = mod2pi(alpha - tmp0 + tmp1);
  const double q = mod2pi(beta - tmp0 + tmp1);

  length = { t, p, q };
  mode = { DUBINS_R, DUBINS_S, DUBINS_L };
}

void DubinsCurve::RLR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);

  const double tmp0 = (6.0 - dist * dist + 2.0 * std::cos(alpha - beta) + 2.0 * dist * (sa - sb)) / 8.0;
  if (std::fabs(tmp0) > 1.0)
  {
    length = { 0.0, 0.0, 0.0 };
    mode = { DUBINS_S, DUBINS_S, DUBINS_S };
    return;
  }

  const double p = mod2pi(2.0 * M_PI - std::acos(clampUnit(tmp0)));
  const double tmp1 = std::atan2(ca - cb, dist - sa + sb);
  const double t = mod2pi(alpha - tmp1 + p / 2.0);
  const double q = mod2pi(alpha - beta - t + p);

  length = { t, p, q };
  mode = { DUBINS_R, DUBINS_L, DUBINS_R };
}

void DubinsCurve::LRL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);

  const double tmp0 = (6.0 - dist * dist + 2.0 * std::cos(alpha - beta) + 2.0 * dist * (-sa + sb)) / 8.0;
  if (std::fabs(tmp0) > 1.0)
  {
    length = { 0.0, 0.0, 0.0 };
    mode = { DUBINS_S, DUBINS_S, DUBINS_S };
    return;
  }

  const double p = mod2pi(2.0 * M_PI - std::acos(clampUnit(tmp0)));
  const double tmp1 = std::atan2(ca - cb, dist + sa - sb);
  const double t = mod2pi(-alpha - tmp1 + p / 2.0);
  const double q = mod2pi(beta - alpha - t + p);

  length = { t, p, q };
  mode = { DUBINS_L, DUBINS_R, DUBINS_L };
}

Point3d DubinsCurve::interpolate(int mode, double length, const Point3d& init_pose)
{
  Point3d new_pose = init_pose;

  if (mode == DUBINS_S)
  {
    new_pose.setX(init_pose.x() + std::cos(init_pose.theta()) * length);
    new_pose.setY(init_pose.y() + std::sin(init_pose.theta()) * length);
    new_pose.setTheta(init_pose.theta());
  }
  else if (mode == DUBINS_L)
  {
    new_pose.setX(init_pose.x() + std::sin(init_pose.theta() + length) - std::sin(init_pose.theta()));
    new_pose.setY(init_pose.y() - std::cos(init_pose.theta() + length) + std::cos(init_pose.theta()));
    new_pose.setTheta(init_pose.theta() + length);
  }
  else if (mode == DUBINS_R)
  {
    new_pose.setX(init_pose.x() - std::sin(init_pose.theta() - length) + std::sin(init_pose.theta()));
    new_pose.setY(init_pose.y() + std::cos(init_pose.theta() - length) - std::cos(init_pose.theta()));
    new_pose.setTheta(init_pose.theta() - length);
  }

  new_pose.setTheta(mod2pi(new_pose.theta()));
  return new_pose;
}

bool DubinsCurve::generation(const Point3d& start, const Point3d& goal, Points3d& path)
{
  if (!(max_curv_ > 0.0))
  {
    return false;
  }

  const double dx = goal.x() - start.x();
  const double dy = goal.y() - start.y();
  const double D = std::hypot(dx, dy);

  const double theta = std::atan2(dy, dx);
  const double alpha = mod2pi(start.theta() - theta);
  const double beta = mod2pi(goal.theta() - theta);
  const double dist = D * max_curv_;

  DubinsLength best_length{ 0.0, 0.0, 0.0 };
  DubinsMode best_mode{ DUBINS_S, DUBINS_S, DUBINS_S };
  double best_cost = std::numeric_limits<double>::infinity();

  for (const auto& solver : dubins_solvers)
  {
    DubinsLength length;
    DubinsMode mode;
    solver(alpha, beta, dist, length, mode);

    // If solver returns all S, treat as invalid (we never output S,S,S)
    if (std::get<0>(mode) == DUBINS_S && std::get<1>(mode) == DUBINS_S && std::get<2>(mode) == DUBINS_S)
    {
      continue;
    }

    _update(length, mode, best_length, best_mode, best_cost);
  }

  if (!std::isfinite(best_cost))
  {
    return false;
  }

  path.clear();
  path.reserve(static_cast<size_t>(std::ceil(best_cost / (step_ * max_curv_))) + 2u);

  const double step_n = step_ * max_curv_;
  Point3d pose_n(0.0, 0.0, alpha);
  path.emplace_back(start.x(), start.y(), mod2pi(start.theta()));

  const std::array<int, 3> modes = { std::get<0>(best_mode), std::get<1>(best_mode), std::get<2>(best_mode) };
  const std::array<double, 3> lens = { std::get<0>(best_length), std::get<1>(best_length), std::get<2>(best_length) };

  for (size_t seg = 0; seg < 3; ++seg)
  {
    double remaining = lens[seg];
    while (remaining > 1e-12)
    {
      const double dl = (step_n > 0.0) ? std::min(step_n, remaining) : remaining;
      pose_n = interpolate(modes[seg], dl, pose_n);
      remaining -= dl;

      const double x_local = pose_n.x() / max_curv_;
      const double y_local = pose_n.y() / max_curv_;
      const double yaw_local = pose_n.theta();

      const double x_global = start.x() + std::cos(theta) * x_local - std::sin(theta) * y_local;
      const double y_global = start.y() + std::sin(theta) * x_local + std::cos(theta) * y_local;
      const double yaw_global = mod2pi(yaw_local + theta);

      path.emplace_back(x_global, y_global, yaw_global);
    }
  }

  // Ensure last pose is the goal pose (numerical stability)
  path.back().setX(goal.x());
  path.back().setY(goal.y());
  path.back().setTheta(mod2pi(goal.theta()));

  return true;
}

void DubinsCurve::setMaxCurv(double max_curv)
{
  CHECK_GT(max_curv, 0.0);
  max_curv_ = max_curv;
}

void DubinsCurve::_update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode,
                          double& best_cost)
{
  const double cost = std::get<0>(length) + std::get<1>(length) + std::get<2>(length);
  if (cost < best_cost)
  {
    best_cost = cost;
    best_length = length;
    best_mode = mode;
  }
}

}  // namespace geometry
}  // namespace common
}  // namespace rpp
