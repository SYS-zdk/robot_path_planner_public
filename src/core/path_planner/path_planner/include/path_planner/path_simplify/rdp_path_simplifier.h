/**
 * @file rdp_path_simplifier.h
 * @brief Ramer–Douglas–Peucker (RDP) polyline simplification.
 *
 * This implementation is original to this repository.
 * It simplifies a 2D polyline (x,y) while preserving endpoints.
 */
#ifndef RPP_PATH_PLANNER_PATH_SIMPLIFY_RDP_PATH_SIMPLIFIER_H_
#define RPP_PATH_PLANNER_PATH_SIMPLIFY_RDP_PATH_SIMPLIFIER_H_

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "common/geometry/point.h"

namespace rpp
{
namespace path_planner
{

namespace detail
{
inline double clamp01(const double t)
{
  if (t < 0.0)
  {
    return 0.0;
  }
  if (t > 1.0)
  {
    return 1.0;
  }
  return t;
}
}  // namespace detail

inline void rdpSimplify2d(const rpp::common::geometry::Points3d& path_in, double epsilon,
                          rpp::common::geometry::Points3d& path_out)
{
  path_out.clear();

  if (path_in.empty())
  {
    return;
  }
  if (path_in.size() <= 2)
  {
    path_out = path_in;
    return;
  }

  const double eps = std::max(0.0, epsilon);

  // Mark points to keep.
  std::vector<char> keep(path_in.size(), 0);
  keep.front() = 1;
  keep.back() = 1;

  // Iterative stack of segments [i,j].
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  stack.reserve(path_in.size());
  stack.emplace_back(0, path_in.size() - 1);

  while (!stack.empty())
  {
    const std::size_t i = stack.back().first;
    const std::size_t j = stack.back().second;
    stack.pop_back();
    if (j <= i + 1)
    {
      continue;
    }

    const auto& a = path_in[i];
    const auto& b = path_in[j];
    const double ax = a.x();
    const double ay = a.y();
    const double bx = b.x();
    const double by = b.y();

    const double vx = bx - ax;
    const double vy = by - ay;
    const double v2 = vx * vx + vy * vy;

    std::size_t max_idx = i;
    double max_dist = -1.0;

    for (std::size_t k = i + 1; k < j; ++k)
    {
      const double px = path_in[k].x();
      const double py = path_in[k].y();

      // Distance from P to segment AB.
      double dist = 0.0;
      if (v2 <= 1e-12)
      {
        dist = std::hypot(px - ax, py - ay);
      }
      else
      {
        const double t = ((px - ax) * vx + (py - ay) * vy) / v2;
        const double tc = detail::clamp01(t);
        const double projx = ax + tc * vx;
        const double projy = ay + tc * vy;
        dist = std::hypot(px - projx, py - projy);
      }

      if (dist > max_dist)
      {
        max_dist = dist;
        max_idx = k;
      }
    }

    if (max_dist > eps)
    {
      keep[max_idx] = 1;
      stack.emplace_back(i, max_idx);
      stack.emplace_back(max_idx, j);
    }
  }

  path_out.reserve(path_in.size());
  for (std::size_t idx = 0; idx < path_in.size(); ++idx)
  {
    if (keep[idx])
    {
      path_out.push_back(path_in[idx]);
    }
  }

  // Ensure at least endpoints exist.
  if (path_out.size() < 2)
  {
    path_out.clear();
    path_out.push_back(path_in.front());
    path_out.push_back(path_in.back());
  }
}

}  // namespace path_planner
}  // namespace rpp

#endif
