/**
 * @file: rhcf_planner.h
 * @brief: Randomized Homotopy Class Finder (RHCF) global planner (Voronoi random walk)
 * @author: ZhangDingkun (integration) / Copilot
 * @date: 2026.03.10
 *
 * This is an original re-implementation inspired by the ideas described in:
 *   https://github.com/srl-freiburg/srl_rhcf_planner
 *
 * Key idea: a (socially-informed) Voronoi diagram implicitly encodes different homotopy classes.
 * RHCF samples K distinct paths by performing biased random walks over the Voronoi skeleton,
 * then returns the best geometric path among those candidates.
 */
#ifndef RPP_RHCF_GLOBAL_PLANNER_RHCF_PLANNER_H_
#define RPP_RHCF_GLOBAL_PLANNER_RHCF_PLANNER_H_

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>

#include "common/structure/node.h"
#include "path_planner/path_planner.h"
#include "voronoi_layer.h"

namespace rpp
{
namespace path_planner
{
class RhcfPathPlanner : public PathPlanner
{
private:
  using Node = rpp::common::structure::Node<int>;

  struct CellInfo
  {
    bool is_voronoi{ false };
    float dist_m{ 0.0f };
  };

  struct Vertex
  {
    int id{ -1 };  // grid2Index(x,y)
    int x{ 0 };
    int y{ 0 };
  };

  struct Edge
  {
    int id{ -1 };
    int a{ -1 };  // endpoint vertex id
    int b{ -1 };

    // polyline cells in direction a -> b (inclusive endpoints)
    std::vector<int> cells;

    double length_m{ 0.0 };
    double cost{ 0.0 };  // geometric cost used for selection (no heuristic)
  };

public:
  struct Params
  {
    // RHCF core
    int K_homotopy_classes{ 5 };
    double discounting_factor{ 0.8 };  // alpha in [0.5, 1.0]; smaller -> more exploratory

    // sampling limits
    int max_random_walks{ 200 };
    int max_vertex_steps_per_walk{ 2000 };

    // cost weights
    double w_length{ 1.0 };
    double w_clearance{ 2.0 };
    double w_costmap{ 0.01 };

    // random-walk bias
    double w_heuristic{ 1.0 };
    // Deprecated (kept for param compatibility; no longer used):
    // earlier versions down-weighted revisits instead of forbidding them.
    double visited_vertex_weight{ 0.1 };

    // Deprecated (kept for param compatibility; no longer used):
    // earlier versions used edge-set Jaccard similarity; current behavior matches upstream
    // more closely by rejecting only identical vertex sequences.
    double edge_similarity_threshold{ 0.7 };

    // connectivity
    bool voronoi_connect_diagonal{ false };  // keep topology cleaner with 4-neighborhood

    // unknown handling
    bool traverse_unknown{ true };

    // rng
    bool use_fixed_seed{ false };
    int fixed_seed{ 1 };
  };

public:
  RhcfPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double circumscribed_radius, const Params& params,
                  double obstacle_factor = 0.5);

  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) override;

  void appendDebugData(DebugData& data) const override
  {
    data.rhcf_candidate_paths_world = last_candidate_paths_world_;
    data.rhcf_best_candidate_index = last_best_candidate_idx_;
  }

  // ---- Debug/visualization helpers (optional) ----
  // Cached from the last successful plan() call.
  const std::vector<Points3d>& getLastCandidatePathsWorld() const
  {
    return last_candidate_paths_world_;
  }
  const std::vector<double>& getLastCandidateCosts() const
  {
    return last_candidate_costs_;
  }
  int getLastBestCandidateIndex() const
  {
    return last_best_candidate_idx_;
  }

private:
  bool updateVoronoi();

  bool isInBounds(int x, int y) const;
  bool isTraversableCell(int x, int y, int cur_id) const;
  bool isSkeletonCell(int x, int y) const;
  int skeletonDegree4(int x, int y) const;

  bool searchToVoronoi(const Node& start, std::vector<int>& out_path_cells, int& out_voronoi_id);

  void buildVoronoiVertexGraph(int v_start_id, int v_goal_id);

  bool runOneRandomWalk(int v_start_id, int v_goal_id, std::unordered_map<uint64_t, double>& inout_directed_weights,
                        std::vector<int>& out_vertex_seq, double& out_cost,
                        std::unordered_set<int>& inout_expand_cells);

  double heuristicM(int v_id, int goal_id) const;

  void appendEdgeCellsDirected(int from_v, int to_v, const Edge& e, std::vector<int>& io_cells) const;
  bool reconstructCellsFromVertexSeq(const std::vector<int>& vertex_seq, std::vector<int>& out_cells) const;

  double computePolylineLengthM(const std::vector<int>& cells) const;
  double computePolylineCost(const std::vector<int>& cells) const;

  static uint64_t packDirectedEdge(int from_id, int to_id);

private:
  Params params_;
  double circumscribed_radius_{ 0.0 };
  double resolution_{ 0.0 };

  std::vector<CellInfo> voronoi_;

  // vertex graph (on Voronoi skeleton)
  std::unordered_map<int, Vertex> vertices_;                   // vertex id -> vertex
  std::vector<Edge> edges_;                                    // undirected edges
  std::unordered_map<int, std::vector<int>> adjacency_edges_;  // vertex id -> edge ids

  std::mt19937 rng_;

  // Debug cache (world coordinates) for RViz visualization.
  std::vector<Points3d> last_candidate_paths_world_;
  std::vector<double> last_candidate_costs_;
  int last_best_candidate_idx_{ -1 };

  // 4-neighborhood offsets
  const std::array<std::pair<int, int>, 4> n4_{ { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } } };
  const std::array<Node, 8> motions8_{ { Node(0, 1, 1.0), Node(1, 0, 1.0), Node(0, -1, 1.0), Node(-1, 0, 1.0),
                                        Node(1, 1, std::sqrt(2.0)), Node(1, -1, std::sqrt(2.0)),
                                        Node(-1, 1, std::sqrt(2.0)), Node(-1, -1, std::sqrt(2.0)) } };
};

}  // namespace path_planner
}  // namespace rpp

#endif  // RPP_RHCF_GLOBAL_PLANNER_RHCF_PLANNER_H_
