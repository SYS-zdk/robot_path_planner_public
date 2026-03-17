
/**
 * @file: sunshine_planner.cpp
 * @brief: Sunshine (Sunlight) sampling-based global planner
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <cmath>
#include <limits>

#include <costmap_2d/cost_values.h>

#include "common/geometry/collision_checker.h"
#include "path_planner/sample_planner/sunshine_planner.h"

namespace rpp
{
namespace path_planner
{
namespace
{
using CollisionChecker = rpp::common::geometry::CollisionChecker;

inline double heuristicXY(int ax, int ay, int bx, int by)
{
  return std::hypot(ax - bx, ay - by);
}

inline int clampInt(int v, int lo, int hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}
}  // namespace

SunshinePathPlanner::SunshinePathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor,
                                         const Params& params)
  : PathPlanner(costmap_ros, obstacle_factor), params_(params)
{
  if (params_.theta_step <= 0.0)
  {
    params_.theta_step = 0.17453292519943295;
  }
  if (params_.length_step <= 0.0)
  {
    params_.length_step = 1.0;
  }
  if (params_.max_iterations < 1)
  {
    params_.max_iterations = 20000;
  }
  if (params_.optimize_iterations < 0)
  {
    params_.optimize_iterations = 0;
  }
}

bool SunshinePathPlanner::inBounds(int x, int y) const
{
  return x >= 0 && y >= 0 && x < nx_ && y < ny_;
}

bool SunshinePathPlanner::isOccupied(int x, int y) const
{
  if (!inBounds(x, y))
  {
    return true;
  }
  const int idx = grid2Index(x, y);
  return costmap_->getCharMap()[idx] >= costmap_2d::LETHAL_OBSTACLE * factor_;
}

bool SunshinePathPlanner::hasLineOfSight(const Node& a, const Node& b) const
{
  return !CollisionChecker::BresenhamCollisionDetection(a, b, [&](const Node& n) { return isOccupied(n.x(), n.y()); });
}

double SunshinePathPlanner::raycastFreeLength(int x0, int y0, double angle) const
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  double traveled = 0.0;
  while (traveled + params_.length_step <= params_.max_ray_length)
  {
    traveled += params_.length_step;
    const int x = static_cast<int>(std::lround(x0 + traveled * c));
    const int y = static_cast<int>(std::lround(y0 + traveled * s));

    if (!inBounds(x, y))
    {
      break;
    }
    if (isOccupied(x, y))
    {
      break;
    }
  }

  return traveled;
}

std::vector<SunshinePathPlanner::Node> SunshinePathPlanner::findTangents(const Node& sun) const
{
  std::vector<Node> candidates;

  // 360-degree ray casting with fixed angular resolution.
  const int n = std::max(8, static_cast<int>(std::ceil(2.0 * M_PI / params_.theta_step)));
  std::vector<double> lengths(static_cast<size_t>(n));
  std::vector<double> angles(static_cast<size_t>(n));

  for (int i = 0; i < n; ++i)
  {
    const double angle = i * (2.0 * M_PI / static_cast<double>(n));
    angles[static_cast<size_t>(i)] = angle;
    lengths[static_cast<size_t>(i)] = raycastFreeLength(sun.x(), sun.y(), angle);
  }

  // Tangent detection: if adjacent ray lengths differ beyond threshold, a corner-like tangent exists.
  for (int i = 0; i < n; ++i)
  {
    const int j = (i + 1) % n;
    const double li = lengths[static_cast<size_t>(i)];
    const double lj = lengths[static_cast<size_t>(j)];

    if (std::fabs(li - lj) < params_.length_diff_threshold)
    {
      continue;
    }

    const int longer = (li >= lj) ? i : j;
    const double l_long = (li >= lj) ? li : lj;
    const double l_short = (li >= lj) ? lj : li;

    // Choose a point in the direction of the longer ray, with a forward bias to avoid getting stuck.
    double desired = l_short + params_.forward_distance;
    desired = std::min(desired, l_long);

    if (desired < 1.0)
    {
      continue;
    }

    const double ang = angles[static_cast<size_t>(longer)];
    const int cx = static_cast<int>(std::lround(sun.x() + desired * std::cos(ang)));
    const int cy = static_cast<int>(std::lround(sun.y() + desired * std::sin(ang)));

    if (!inBounds(cx, cy) || isOccupied(cx, cy))
    {
      continue;
    }

    Node cand(cx, cy);
    cand.set_id(grid2Index(cx, cy));
    candidates.push_back(cand);
  }

  // De-duplicate by id.
  std::unordered_set<int> seen;
  std::vector<Node> unique;
  unique.reserve(candidates.size());
  for (const auto& c : candidates)
  {
    if (seen.insert(c.id()).second)
    {
      unique.push_back(c);
    }
  }

  return unique;
}

bool SunshinePathPlanner::isBrotherCandidate(const Node& sun, const Node& cand) const
{
  const int pid = sun.pid();
  if (pid < 0)
  {
    return false;
  }

  auto it = nodes_.find(pid);
  if (it == nodes_.end())
  {
    return false;
  }

  return hasLineOfSight(it->second, cand);
}

bool SunshinePathPlanner::isOtherSonCandidate(const Node& /*sun*/, const Node& cand, double cand_g) const
{
  // NotOtherSon: reject if any other open node can connect to cand with <= cost.
  for (const auto& kv : nodes_)
  {
    const int other_id = kv.first;
    if (closed_.find(other_id) != closed_.end())
    {
      continue;
    }

    const auto itg = best_g_.find(other_id);
    if (itg == best_g_.end())
    {
      continue;
    }

    const Node& other = kv.second;
    if (other.id() == cand.id())
    {
      continue;
    }

    // Quick lower bound.
    if (itg->second + heuristicXY(other.x(), other.y(), cand.x(), cand.y()) > cand_g + 1e-9)
    {
      continue;
    }

    if (hasLineOfSight(other, cand))
    {
      const double alt = itg->second + heuristicXY(other.x(), other.y(), cand.x(), cand.y());
      if (alt <= cand_g + 1e-9)
      {
        return false;
      }
    }
  }

  return true;
}

std::vector<SunshinePathPlanner::Node> SunshinePathPlanner::backtracePath(int goal_parent_id, const Node& start,
                                                                          const Node& goal) const
{
  std::vector<Node> out;
  out.reserve(256);

  // Goal
  out.push_back(goal);

  int cur = goal_parent_id;
  while (cur >= 0)
  {
    auto it = nodes_.find(cur);
    if (it == nodes_.end())
    {
      break;
    }
    out.push_back(it->second);
    if (cur == start.id())
    {
      break;
    }
    cur = it->second.pid();
  }

  // Ensure start is present.
  if (out.empty() || out.back().id() != start.id())
  {
    out.push_back(start);
  }

  std::reverse(out.begin(), out.end());
  return out;
}

SunshinePathPlanner::Node SunshinePathPlanner::midpointNode(const Node& a, const Node& b)
{
  Node m;
  m.set_x((a.x() + b.x()) / 2);
  m.set_y((a.y() + b.y()) / 2);
  return m;
}

SunshinePathPlanner::Node SunshinePathPlanner::bisectionTowards(const Node& anchor, const Node& from, const Node& to) const
{
  // Find a point on segment [from, to] such that anchor has LOS to that point,
  // pushing as far towards `to` as possible.
  Node lo = from;
  Node hi = to;

  if (!hasLineOfSight(anchor, lo))
  {
    return from;
  }
  if (hasLineOfSight(anchor, hi))
  {
    return hi;
  }

  for (int iter = 0; iter < 12; ++iter)
  {
    Node mid = midpointNode(lo, hi);
    mid.set_x(clampInt(mid.x(), 0, nx_ - 1));
    mid.set_y(clampInt(mid.y(), 0, ny_ - 1));

    if (hasLineOfSight(anchor, mid))
    {
      lo = mid;
    }
    else
    {
      hi = mid;
    }
  }

  return lo;
}

void SunshinePathPlanner::optimizeForward(std::vector<Node>& path) const
{
  if (path.size() < 3)
  {
    return;
  }

  for (size_t i = 0; i + 2 < path.size(); ++i)
  {
    Node& a = path[i];
    Node& b = path[i + 1];
    Node& c = path[i + 2];

    if (hasLineOfSight(a, c))
    {
      b = midpointNode(a, c);
      b.set_x(clampInt(b.x(), 0, nx_ - 1));
      b.set_y(clampInt(b.y(), 0, ny_ - 1));
      continue;
    }

    // If cannot shortcut, push b towards c while keeping LOS(a, b).
    Node new_b = bisectionTowards(a, b, c);
    b = new_b;
  }
}

void SunshinePathPlanner::optimizeBackward(std::vector<Node>& path) const
{
  if (path.size() < 3)
  {
    return;
  }

  for (size_t k = path.size() - 1; k >= 2; --k)
  {
    Node& c = path[k];
    Node& b = path[k - 1];
    Node& a = path[k - 2];

    if (hasLineOfSight(a, c))
    {
      b = midpointNode(a, c);
      b.set_x(clampInt(b.x(), 0, nx_ - 1));
      b.set_y(clampInt(b.y(), 0, ny_ - 1));
    }
    else
    {
      Node new_b = bisectionTowards(c, b, a);
      b = new_b;
    }

    if (k == 2)
    {
      break;
    }
  }
}

void SunshinePathPlanner::optimizePath(std::vector<Node>& path) const
{
  if (!params_.enable_bidirectional_opt || params_.optimize_iterations <= 0)
  {
    return;
  }

  for (int i = 0; i < params_.optimize_iterations; ++i)
  {
    optimizeForward(path);
    optimizeBackward(path);
  }

  // Remove consecutive duplicates
  std::vector<Node> out;
  out.reserve(path.size());
  for (const auto& n : path)
  {
    if (out.empty() || out.back().x() != n.x() || out.back().y() != n.y())
    {
      out.push_back(n);
    }
  }
  path.swap(out);
}

bool SunshinePathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  path.clear();
  expand.clear();

  nodes_.clear();
  closed_.clear();
  best_g_.clear();
  open_ = decltype(open_)();

  Node start_n(static_cast<int>(std::lround(start.x())), static_cast<int>(std::lround(start.y())));
  start_n.set_x(clampInt(start_n.x(), 0, nx_ - 1));
  start_n.set_y(clampInt(start_n.y(), 0, ny_ - 1));
  start_n.set_id(grid2Index(start_n.x(), start_n.y()));
  start_n.set_pid(-1);
  start_n.set_g(0.0);

  Node goal_n(static_cast<int>(std::lround(goal.x())), static_cast<int>(std::lround(goal.y())));
  goal_n.set_x(clampInt(goal_n.x(), 0, nx_ - 1));
  goal_n.set_y(clampInt(goal_n.y(), 0, ny_ - 1));
  goal_n.set_id(grid2Index(goal_n.x(), goal_n.y()));

  if (isOccupied(start_n.x(), start_n.y()) || isOccupied(goal_n.x(), goal_n.y()))
  {
    return false;
  }

  nodes_.insert({ start_n.id(), start_n });
  best_g_.insert({ start_n.id(), 0.0 });

  open_.push({ heuristicXY(start_n.x(), start_n.y(), goal_n.x(), goal_n.y()), start_n.id() });

  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::max();

  for (int iter = 0; iter < params_.max_iterations && !open_.empty(); ++iter)
  {
    const PQItem item = open_.top();
    open_.pop();

    auto it = nodes_.find(item.id);
    if (it == nodes_.end())
    {
      continue;
    }

    Node sun = it->second;
    if (closed_.find(sun.id()) != closed_.end())
    {
      continue;
    }

    closed_.insert(sun.id());
    expand.emplace_back(sun.x(), sun.y(), sun.pid());

    const double sun_g = best_g_[sun.id()];

    // Direct illumination: if sun can see the goal, record a candidate path.
    if (hasLineOfSight(sun, goal_n))
    {
      const double cost = sun_g + heuristicXY(sun.x(), sun.y(), goal_n.x(), goal_n.y());
      if (cost < best_goal_cost)
      {
        best_goal_cost = cost;
        best_goal_parent = sun.id();
      }
    }

    // Find tangent candidates by ray-length discontinuities.
    const auto candidates = findTangents(sun);
    for (auto cand : candidates)
    {
      // Must be directly reachable from current sun.
      if (!hasLineOfSight(sun, cand))
      {
        continue;
      }

      const double edge = heuristicXY(sun.x(), sun.y(), cand.x(), cand.y());
      const double cand_g = sun_g + edge;

      // NotBrother: if parent can directly reach it, connecting via sun is suboptimal.
      if (isBrotherCandidate(sun, cand))
      {
        continue;
      }

      // NotOtherSon: ensure no other open sun offers a cheaper visible connection.
      if (!isOtherSonCandidate(sun, cand, cand_g))
      {
        continue;
      }

      // Accept if it improves the best known g.
      auto itg = best_g_.find(cand.id());
      if (itg != best_g_.end() && itg->second <= cand_g)
      {
        continue;
      }

      cand.set_pid(sun.id());
      cand.set_g(cand_g);
      nodes_[cand.id()] = cand;
      best_g_[cand.id()] = cand_g;

      const double f = cand_g + heuristicXY(cand.x(), cand.y(), goal_n.x(), goal_n.y());
      open_.push({ f, cand.id() });
    }
  }

  if (best_goal_parent < 0)
  {
    return false;
  }

  std::vector<Node> node_path = backtracePath(best_goal_parent, start_n, goal_n);
  optimizePath(node_path);

  // Export as Points3d in costmap cell coordinates.
  for (const auto& n : node_path)
  {
    path.emplace_back(static_cast<double>(n.x()), static_cast<double>(n.y()), 0.0);
  }

  return !path.empty();
}

}  // namespace path_planner
}  // namespace rpp
