
/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner_impl.cpp
 * @brief: Hybrid A* planner implementation (no external system-level config)
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 *
 * ********************************************************
 */

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "path_planner/graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"

namespace rpp
{
namespace path_planner
{
namespace
{
constexpr int kGraphSizeDefault = 100000;

inline uint64_t stateIndexFromPathPose(const rpp::common::geometry::Point3d& pose)
{
  const unsigned int theta_bin = static_cast<unsigned int>(
      rpp::path_planner::NodeHybrid::motion_table.getOrientationBin(pose.theta()));
  return rpp::path_planner::NodeHybrid::getIndex({ pose.x(), pose.y(), theta_bin });
}

inline void removeStateCyclesInPlace(rpp::common::geometry::Points3d& path)
{
  if (path.size() < 3)
  {
    return;
  }

  rpp::common::geometry::Points3d out;
  out.reserve(path.size());
  std::unordered_map<uint64_t, size_t> last;
  last.reserve(path.size());

  auto rebuild_last = [&]() {
    last.clear();
    last.reserve(out.size());
    for (size_t i = 0; i < out.size(); ++i)
    {
      last[stateIndexFromPathPose(out[i])] = i;
    }
  };

  for (const auto& p : path)
  {
    const uint64_t idx = stateIndexFromPathPose(p);

    auto it = last.find(idx);
    if (it != last.end())
    {
      // Found a cycle that returns to an identical state (x, y, theta_bin).
      // Drop the loop segment to prevent local repeated circling.
      out.resize(it->second + 1);
      rebuild_last();
      continue;
    }

    last[idx] = out.size();
    out.push_back(p);
  }

  path.swap(out);
}

inline void removeNearDuplicateXYInPlace(rpp::common::geometry::Points3d& path, double min_separation)
{
  if (path.size() < 2 || !(min_separation > 0.0))
  {
    return;
  }

  rpp::common::geometry::Points3d out;
  out.reserve(path.size());
  out.push_back(path.front());

  // Keep interior points only if they are sufficiently far from the last kept point.
  for (size_t i = 1; i + 1 < path.size(); ++i)
  {
    const auto& prev = out.back();
    const auto& cur = path[i];
    const double dx = cur.x() - prev.x();
    const double dy = cur.y() - prev.y();
    if (std::hypot(dx, dy) >= min_separation)
    {
      out.push_back(cur);
    }
  }

  // Always keep the last point (may overwrite the previous if too close).
  const auto& last = path.back();
  if (out.size() == 1)
  {
    out.push_back(last);
  }
  else
  {
    const double dx = last.x() - out.back().x();
    const double dy = last.y() - out.back().y();
    if (std::hypot(dx, dy) >= min_separation)
    {
      out.push_back(last);
    }
    else
    {
      out.back() = last;
    }
  }

  path.swap(out);
}

inline bool sameGoalApprox(const rpp::common::geometry::Point3d& a, const rpp::common::geometry::Point3d& b)
{
  return a.x() == b.x() && a.y() == b.y() && std::fabs(a.theta() - b.theta()) < 1e-3;
}
}  // namespace

HybridAStarPathPlanner::HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor,
                                               const HybridSearchInfo& info)
  : PathPlanner(costmap_ros, obstacle_factor)
{
  if (!std::isfinite(obstacle_factor) || obstacle_factor <= 0.0 || obstacle_factor > 10.0)
  {
    obstacle_factor = 0.5;
  }
  factor_ = static_cast<float>(obstacle_factor);

  collision_checker_ = std::make_shared<rpp::common::geometry::CollisionChecker>(costmap_ros_, factor_);
  costmap_ = collision_checker_->getCostmapROS()->getCostmap();

  search_info_ = info;
  if (search_info_.motion_model != 2 && search_info_.motion_model != 3)
  {
    search_info_.motion_model = 3;
  }
  if (search_info_.dim_3_size < 4)
  {
    search_info_.dim_3_size = 72;
  }
  if (!std::isfinite(search_info_.heuristic_weight) || search_info_.heuristic_weight <= 0.0)
  {
    search_info_.heuristic_weight = 2.5;
  }

  motion_model_ = (search_info_.motion_model == 2) ? MotionModel::DUBIN : MotionModel::REEDS_SHEPP;

  NodeHybrid::precomputeDistanceHeuristic(motion_model_, search_info_);

  expander_ = std::make_unique<AnalyticExpansion<NodeHybrid>>(motion_model_, search_info_);
  expander_->setCollisionChecker(collision_checker_);

  graph_.reserve(kGraphSizeDefault);
}

bool HybridAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  path.clear();
  expand.clear();

  if (!collision_checker_)
  {
    collision_checker_ = std::make_shared<rpp::common::geometry::CollisionChecker>(costmap_ros_, factor_);
  }
  costmap_ = collision_checker_->getCostmapROS()->getCostmap();
  expander_->setCollisionChecker(collision_checker_);

  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!validityCheckMap(start, m_start_x, m_start_y) || !validityCheckMap(goal, m_goal_x, m_goal_y))
  {
    return false;
  }

  if (!last_path_.empty() && sameGoalApprox(goal_, goal))
  {
    bool is_collision = false;
    auto closest_iter = last_path_.begin();
    double min_dist = std::numeric_limits<double>::max();

    const unsigned int size_x = costmap_->getSizeInCellsX();
    const unsigned int size_y = costmap_->getSizeInCellsY();
    for (auto iter = last_path_.begin(); iter != last_path_.end(); ++iter)
    {
      const double px = iter->x();
      const double py = iter->y();
      if (px < 0.0 || py < 0.0 || px >= static_cast<double>(size_x) || py >= static_cast<double>(size_y))
      {
        is_collision = true;
        break;
      }
      const unsigned int idx = static_cast<unsigned int>(px) + static_cast<unsigned int>(py) * size_x;
      if (collision_checker_->inCollision(idx, search_info_.traverse_unknown))
      {
        is_collision = true;
        break;
      }

      const double dist = std::hypot(px - m_start_x, py - m_start_y);
      if (dist < min_dist)
      {
        min_dist = dist;
        closest_iter = iter;
      }
    }

    if (!is_collision)
    {
      for (auto iter = closest_iter; iter != last_path_.end(); ++iter)
      {
        path.emplace_back(iter->x(), iter->y(), iter->theta());
      }

      // Defensive cleanup: the stitched suffix can contain a small local loop.
      removeStateCyclesInPlace(path);

      // Remove accidental near-duplicate XY points (can happen with analytic expansion sampling
      // or alternating primitives). This prevents downstream consumers from seeing zero-length
      // segments.
      removeNearDuplicateXYInPlace(path, search_info_.path_min_separation);

      last_path_ = path;
      return true;
    }
    last_path_.clear();
  }

  goal_ = goal;

  Points3d path_in_map;
  if (!createPath({ m_start_x, m_start_y, start.theta() }, { m_goal_x, m_goal_y, goal.theta() }, path_in_map, expand))
  {
    return false;
  }

  for (auto iter = path_in_map.rbegin(); iter != path_in_map.rend(); ++iter)
  {
    path.emplace_back(iter->x(), iter->y(), iter->theta());
  }

  // Remove accidental repeated cycles in the final path (typically caused by
  // alternating primitives or certain Reeds-Shepp CCC patterns).
  removeStateCyclesInPlace(path);

  // Remove near-duplicate XY samples (e.g. dense analytic expansion sampling). We operate in
  // map coordinates (cell units). 0.25 means 1/4 cell.
  removeNearDuplicateXYInPlace(path, search_info_.path_min_separation);

  last_path_ = path;
  return true;
}

bool HybridAStarPathPlanner::createPath(const Point3d& start_map, const Point3d& goal_map, Points3d& path_map,
                                       Points3d& expand)
{
  path_map.clear();
  expand.clear();
  best_heuristic_node_ = { std::numeric_limits<float>::max(), 0 };

  if (!costmap_ || !collision_checker_)
  {
    return false;
  }

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  NodeHybrid::initMotionModel(motion_model_, size_x, size_y, search_info_);

  const double start_theta_bin = NodeHybrid::motion_table.getOrientationBin(start_map.theta());
  const double goal_theta_bin = NodeHybrid::motion_table.getOrientationBin(goal_map.theta());
  NodeHybrid::Pose start_pose(start_map.x(), start_map.y(), start_theta_bin);
  NodeHybrid::Pose goal_pose(goal_map.x(), goal_map.y(), goal_theta_bin);
  goal_pose_ = goal_pose;

  if (search_info_.goal_tolerance < 0.001 && isCollisionMap(goal_pose.x(), goal_pose.y()))
  {
    return false;
  }

  clearGraph();
  clearQueue();

  NodeHybrid* start_node = addToGraph(NodeHybrid::getIndex(start_pose));
  NodeHybrid* goal_node = addToGraph(NodeHybrid::getIndex(goal_pose));
  if (!start_node || !goal_node)
  {
    return false;
  }

  start_node->reset();
  goal_node->reset();
  start_node->setPose(start_pose);
  goal_node->setPose(goal_pose);

  NodeHybrid::resetObstacleHeuristic(collision_checker_->getCostmapROS(), static_cast<unsigned int>(start_pose.x()),
                                     static_cast<unsigned int>(start_pose.y()),
                                     static_cast<unsigned int>(goal_pose.x()), static_cast<unsigned int>(goal_pose.y()));

  addToQueue(0.0, start_node->index());
  start_node->setAccumulatedCost(0.0);

  const uint64_t max_index = static_cast<uint64_t>(size_x) * static_cast<uint64_t>(size_y) *
                             static_cast<uint64_t>(search_info_.dim_3_size);
  std::function<bool(const uint64_t&, NodeHybrid*&)>
      neighborGetter = [&, this](const uint64_t& index, NodeHybrid*& neighbor_rtn) -> bool {
    if (index >= max_index)
    {
      return false;
    }
    neighbor_rtn = addToGraph(index);
    return neighbor_rtn != nullptr;
  };

  std::vector<NodeHybrid*> neighbors;
  neighbors.reserve(64);

  int iterations = 0;
  int approach_iterations = 0;
  double analytic_attempt_acc = 0.0;
  const double analytic_ratio =
      (std::isfinite(search_info_.analytic_expansion_ratio) && search_info_.analytic_expansion_ratio > 1.0) ?
          search_info_.analytic_expansion_ratio :
          1.0;
  while (iterations < search_info_.max_iterations && !queue_.empty())
  {
    const uint64_t cur_index = queue_.top().index;
    queue_.pop();

    auto it = graph_.find(cur_index);
    if (it == graph_.end())
    {
      continue;
    }
    NodeHybrid* current = it->second.get();

    expand.emplace_back(current->pose().x(), current->pose().y(),
                        NodeHybrid::motion_table.getAngleFromBin(current->pose().theta()));

    if (current->is_visited())
    {
      continue;
    }
    iterations++;
    current->visited();

    // Attempt analytic expansion at a reduced frequency controlled by analytic_expansion_ratio.
    analytic_attempt_acc += 1.0;
    if (analytic_attempt_acc >= analytic_ratio)
    {
      analytic_attempt_acc -= analytic_ratio;
      NodeHybrid* expansion_result = expander_->tryAnalyticExpansion(current, goal_node);
      if (expansion_result != nullptr)
      {
        current = expansion_result;
      }
    }

    if (current == goal_node)
    {
      return goal_node->backtracePath(path_map);
    }
    else if (best_heuristic_node_.first < search_info_.goal_tolerance)
    {
      approach_iterations++;
      if (approach_iterations >= search_info_.max_approach_iterations)
      {
        auto best_it = graph_.find(best_heuristic_node_.second);
        if (best_it == graph_.end())
        {
          return false;
        }
        return best_it->second->backtracePath(path_map);
      }
    }

    neighbors.clear();
    current->getNeighbors(neighborGetter, collision_checker_, search_info_.traverse_unknown, neighbors);
    for (auto* neighbor : neighbors)
    {
      const double g_cost = current->accumulated_cost() + current->getTraversalCost(neighbor);
      if (g_cost < neighbor->accumulated_cost())
      {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current;
        addToQueue(g_cost + search_info_.heuristic_weight * getHeuristicCost(neighbor), neighbor->index());
      }
    }
  }

  if (best_heuristic_node_.first < search_info_.goal_tolerance)
  {
    auto best_it = graph_.find(best_heuristic_node_.second);
    if (best_it == graph_.end())
    {
      return false;
    }
    return best_it->second->backtracePath(path_map);
  }

  return false;
}

bool HybridAStarPathPlanner::validityCheckMap(const Point3d& p, double& mx, double& my) const
{
  if (!costmap_)
  {
    return false;
  }
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  mx = p.x();
  my = p.y();
  if (mx < 0.0 || my < 0.0 || mx >= static_cast<double>(size_x) || my >= static_cast<double>(size_y))
  {
    return false;
  }
  return true;
}

bool HybridAStarPathPlanner::isCollisionMap(const double& mx, const double& my) const
{
  if (!collision_checker_ || !costmap_)
  {
    return true;
  }
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  if (mx < 0.0 || my < 0.0 || mx >= static_cast<double>(size_x) || my >= static_cast<double>(size_y))
  {
    return true;
  }
  const unsigned int idx = static_cast<unsigned int>(mx) + static_cast<unsigned int>(my) * size_x;
  return collision_checker_->inCollision(idx, search_info_.traverse_unknown);
}

NodeHybrid* HybridAStarPathPlanner::addToGraph(const uint64_t& index)
{
  auto it = graph_.find(index);
  if (it != graph_.end())
  {
    return it->second.get();
  }
  auto node = std::make_unique<NodeHybrid>(index);
  NodeHybrid* ptr = node.get();
  graph_.emplace(index, std::move(node));
  return ptr;
}

void HybridAStarPathPlanner::clearGraph()
{
  Graph g;
  std::swap(graph_, g);
  graph_.reserve(kGraphSizeDefault);
}

void HybridAStarPathPlanner::addToQueue(const double& cost, const uint64_t& index)
{
  queue_.push(QueueItem{ cost, index });
}

void HybridAStarPathPlanner::clearQueue()
{
  Queue q;
  std::swap(queue_, q);
}

float HybridAStarPathPlanner::getHeuristicCost(NodeHybrid* node)
{
  const NodeHybrid::Pose cur_pose = NodeHybrid::getCoords(node->index());
  const double h = NodeHybrid::getHeuristicCost(cur_pose, goal_pose_);
  if (h < best_heuristic_node_.first)
  {
    best_heuristic_node_ = { static_cast<float>(h), node->index() };
  }
  return static_cast<float>(h);
}

}  // namespace path_planner
}  // namespace rpp
