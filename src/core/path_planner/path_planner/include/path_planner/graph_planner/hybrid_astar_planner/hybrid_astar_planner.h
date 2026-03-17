
/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.h
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RPP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_H_
#define RPP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/geometry/curve/dubins_curve.h"

#include "path_planner/path_planner.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"

// Depends on HybridSearchInfo (defined in node_hybrid.h)
#include "path_planner/graph_planner/hybrid_astar_planner/analytic_expansion.h"

namespace rpp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class HybridAStarPathPlanner : public PathPlanner
{
public:
  /**
   * @brief Construct a new Hybrid A* object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, const HybridSearchInfo& info);

  /**
   * @brief Destory the Hybrid A* object
   */
  ~HybridAStarPathPlanner() = default;

  /**
   * @brief Hybrid A* implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

private:
  struct QueueItem
  {
    double cost;
    uint64_t index;
  };

  struct QueueItemComparator
  {
    bool operator()(const QueueItem& a, const QueueItem& b) const
    {
      // min-heap by cost
      return a.cost > b.cost;
    }
  };

  using Graph = std::unordered_map<uint64_t, std::unique_ptr<NodeHybrid>>;
  using Queue = std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemComparator>;

  bool createPath(const Point3d& start_map, const Point3d& goal_map, Points3d& path_map, Points3d& expand);
  bool validityCheckMap(const Point3d& p, double& mx, double& my) const;
  bool isCollisionMap(const double& mx, const double& my) const;

  NodeHybrid* addToGraph(const uint64_t& index);
  void clearGraph();
  void addToQueue(const double& cost, const uint64_t& index);
  void clearQueue();
  float getHeuristicCost(NodeHybrid* node);

  MotionModel motion_model_{ MotionModel::UNKNOWN };
  std::unique_ptr<AnalyticExpansion<NodeHybrid>> expander_;

  Graph graph_;
  Queue queue_;

  NodeHybrid::Pose goal_pose_;
  std::pair<float, uint64_t> best_heuristic_node_{ std::numeric_limits<float>::max(), 0 };

  HybridSearchInfo search_info_;

  Point3d goal_;
  Points3d last_path_;
};
}  // namespace path_planner
}  // namespace rpp
#endif