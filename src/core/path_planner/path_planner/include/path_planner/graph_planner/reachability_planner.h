
/**
 * *********************************************************
 *
 * @file: reachability_planner.h
 * @brief: Contains the reachability-aware A* planner class
 * @author: Zhang Dingkun
 * @date: 2026-01-27
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_REACHABILITY_PLANNER_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_REACHABILITY_PLANNER_H

#include <cmath>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "path_planner/path_planner.h"

namespace costmap_2d
{
class GlobalReachabilityLayer;
}

namespace rpp
{
namespace path_planner
{
/**
 * @brief A* planner that biases search toward higher reachability score.
 *
 * Reachability source: cached planning field (`env_score`) inside `globalreachability_layer`.
 */
class ReachabilityPathPlanner : public PathPlanner
{
public:
  ReachabilityPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double min_reachability, double cost_weight,
                          double reachability_weight, double step_base_cost);

  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) override;

private:
  bool updateReachabilityLayer();
  bool getEnvScoreSnapshot(std::vector<double>& env_score, unsigned int& size_x, unsigned int& size_y) const;
  double reachabilityScoreAtCell(unsigned int mx, unsigned int my, const std::vector<double>& env_score,
                                 unsigned int size_x, unsigned int size_y, bool has_env) const;

private:
  boost::shared_ptr<costmap_2d::GlobalReachabilityLayer> reachability_layer_;
  double min_reachability_{ 0.0 };
  double cost_weight_{ 1.0 };
  double reachability_weight_{ 5.0 };
  double step_base_cost_{ 1.0 };

  using Node = rpp::common::structure::Node<int>;
  const std::vector<Node> motions = {
    { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };
};

}  // namespace path_planner
}  // namespace rpp

#endif
