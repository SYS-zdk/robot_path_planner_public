
/**
 * @file: sunshine_planner.h
 * @brief: Sunshine (Sunlight) sampling-based global planner
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_SUNSHINE_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_SUNSHINE_H_

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "path_planner/path_planner.h"

namespace rpp
{
namespace path_planner
{
class SunshinePathPlanner : public PathPlanner
{
private:
  using Node = rpp::common::structure::Node<int>;

  struct PQItem
  {
    double f{ 0.0 };
    int id{ -1 };

    struct Greater
    {
      bool operator()(const PQItem& a, const PQItem& b) const
      {
        return a.f > b.f;
      }
    };
  };

public:
  struct Params
  {
    int max_iterations{ 20000 };

    double theta_step{ 0.17453292519943295 };      // [rad] 10 deg
    double length_step{ 1.0 };                     // [cells]
    double length_diff_threshold{ 8.0 };           // [cells]
    double forward_distance{ 6.0 };                // [cells]
    double max_ray_length{ 200.0 };                // [cells]

    bool enable_bidirectional_opt{ true };
    int optimize_iterations{ 5 };
  };

  SunshinePathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, const Params& params);

  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) override;

private:
  bool inBounds(int x, int y) const;
  bool isOccupied(int x, int y) const;
  bool hasLineOfSight(const Node& a, const Node& b) const;
  double raycastFreeLength(int x0, int y0, double angle) const;

  std::vector<Node> findTangents(const Node& sun) const;

  bool isBrotherCandidate(const Node& sun, const Node& cand) const;
  bool isOtherSonCandidate(const Node& sun, const Node& cand, double cand_g) const;

  std::vector<Node> backtracePath(int goal_parent_id, const Node& start, const Node& goal) const;

  void optimizePath(std::vector<Node>& path) const;
  void optimizeForward(std::vector<Node>& path) const;
  void optimizeBackward(std::vector<Node>& path) const;
  static Node midpointNode(const Node& a, const Node& b);
  Node bisectionTowards(const Node& anchor, const Node& from, const Node& to) const;

private:
  Params params_;

  std::unordered_map<int, Node> nodes_;          // id -> node (pid encodes parent)
  std::unordered_set<int> closed_;               // expanded ids
  std::unordered_map<int, double> best_g_;       // best cost-to-come

  std::priority_queue<PQItem, std::vector<PQItem>, PQItem::Greater> open_;
};

}  // namespace path_planner
}  // namespace rpp

#endif  // RMP_PATH_PLANNER_SAMPLE_PLANNER_SUNSHINE_H_
