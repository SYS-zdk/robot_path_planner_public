
/**
 * @file: reachability_planner.cpp
 * @brief: Contains the reachability-aware A* planner class
 * @author: Zhang Dingkun
 * @date: 2026.01.27
 * @version: 1.0
 */

#include <queue>
#include <unordered_map>

#include <costmap_2d/cost_values.h>

#include <globalreachability_layer/globalreachability_layer.h>

#include "path_planner/graph_planner/reachability_planner.h"

namespace rpp
{
namespace path_planner
{
ReachabilityPathPlanner::ReachabilityPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double min_reachability,
                                                 double cost_weight, double reachability_weight, double step_base_cost)
  : PathPlanner(costmap_ros, 0.5)
  , min_reachability_(min_reachability)
  , cost_weight_(cost_weight)
  , reachability_weight_(reachability_weight)
  , step_base_cost_(step_base_cost)
{
}

bool ReachabilityPathPlanner::updateReachabilityLayer()
{
  if (reachability_layer_)
    return true;

  bool exist = false;
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    boost::shared_ptr<costmap_2d::GlobalReachabilityLayer> reachability_layer =
        boost::dynamic_pointer_cast<costmap_2d::GlobalReachabilityLayer>(*layer);
    if (reachability_layer)
    {
      reachability_layer_ = reachability_layer;
      exist = true;
      break;
    }
  }

  if (!exist)
    ROS_ERROR_THROTTLE(1.0, "Failed to get globalreachability_layer for reachability planner.");

  return exist;
}

bool ReachabilityPathPlanner::getEnvScoreSnapshot(std::vector<double>& env_score, unsigned int& size_x,
                                                 unsigned int& size_y) const
{
  if (!reachability_layer_)
    return false;
  return reachability_layer_->getEnvScoreCache(env_score, size_x, size_y);
}

double ReachabilityPathPlanner::reachabilityScoreAtCell(unsigned int mx, unsigned int my,
                                                        const std::vector<double>& env_score, unsigned int size_x,
                                                        unsigned int size_y, bool has_env) const
{
  if (!has_env)
    return 1.0;
  if (mx >= size_x || my >= size_y)
    return 1.0;

  const size_t idx = static_cast<size_t>(mx) + static_cast<size_t>(my) * static_cast<size_t>(size_x);
  if (idx >= env_score.size())
    return 1.0;

  const double r = env_score[idx];
  if (r < 0.0)
    return 0.0;
  if (r > 1.0)
    return 1.0;
  return r;
}

bool ReachabilityPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  updateReachabilityLayer();
  std::vector<double> env_score;
  unsigned int env_size_x = 0, env_size_y = 0;
  const bool has_env = getEnvScoreSnapshot(env_score, env_size_x, env_size_y);

  Node start_node(start.x(), start.y());
  Node goal_node(goal.x(), goal.y());
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

  path.clear();
  expand.clear();

  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start_node);

  while (!open_list.empty())
  {
    auto current = open_list.top();
    open_list.pop();

    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    if (current == goal_node)
    {
      const auto& backtrace = _convertClosedListToPath<Node>(closed_list, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
        path.emplace_back(iter->x(), iter->y());
      return true;
    }

    for (const auto& motion : motions)
    {
      auto node_new = current + motion;
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      if ((node_new.id() < 0) || (node_new.id() >= map_size_))
        continue;

      const auto c_new = costmap_->getCharMap()[node_new.id()];
      const auto c_cur = costmap_->getCharMap()[current.id()];

      if (c_new >= costmap_2d::LETHAL_OBSTACLE * factor_ && c_new >= c_cur)
        continue;

      const double r = reachabilityScoreAtCell(static_cast<unsigned int>(node_new.x()),
                                               static_cast<unsigned int>(node_new.y()), env_score, env_size_x,
                                               env_size_y, has_env);
      if (r < min_reachability_)
        continue;

      const double cost_norm = static_cast<double>(c_new) / 255.0;
      const double penalty = cost_weight_ * cost_norm + reachability_weight_ * (1.0 - r);
      const double step = step_base_cost_ * motion.g() * (1.0 + penalty);

      node_new.set_pid(current.id());
      node_new.set_g(current.g() + step);
      node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));

      open_list.push(node_new);
    }
  }

  return false;
}

}  // namespace path_planner
}  // namespace rpp
