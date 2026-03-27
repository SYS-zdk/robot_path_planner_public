/**
 * @file: raystar_planner.h
 * @brief: RayStar-style k-path global planner (CGAL-free, grid-based approximation)
 *
 * This planner follows the *interface contract* of the upstream Ray* (RayStar) package:
 * - computes multiple alternative routes (K)
 * - returns the best route to move_base
 * - publishes candidate paths and obstacle debug overlays via PathPlannerRosIO
 *
 * NOTE:
 * Upstream Ray* relies on CGAL to build polygonal obstacles + visibility regions.
 * This implementation is a grid-based approximation that stays buildable in this repo.
 */
#ifndef RPP_RAYSTAR_GLOBAL_PLANNER_RAYSTAR_PLANNER_H_
#define RPP_RAYSTAR_GLOBAL_PLANNER_RAYSTAR_PLANNER_H_

#include <string>
#include <unordered_set>
#include <vector>

#include "path_planner/path_planner.h"

#ifndef RPP_RAYSTAR_WITH_CGAL
#define RPP_RAYSTAR_WITH_CGAL 0
#endif

namespace rpp
{
namespace path_planner
{

class RaystarPathPlanner : public PathPlanner
{
public:
  struct Params
  {
    // Implementation switch:
    // - "grid": current CGAL-free grid approximation (Theta* + penalty replanning)
    // - "upstream_like": polygonal obstacles + visibility-region + gap-tree search (no CGAL)
    // - "upstream_cgal": upstream polymap + visibility-region via CGAL (optional build)
    std::string implementation{ "grid" };

    int k_paths{ 5 };                // K in Ray*
    int max_tries{ 24 };             // attempts to obtain K distinct candidates

    bool traverse_unknown{ true };   // upstream: allow_unknown
    bool allow_diagonal{ true };
    bool use_theta_star{ true };     // any-angle via line-of-sight relaxation

    // Planning bounds
    int max_expansions{ 250000 };
    int max_planning_time_ms{ 1500 };

    // Path cost
    bool use_costmap_weight{ true };
    double cost_penalty_weight{ 1.0 };

    // Diversity via re-planning penalty map
    double penalty_weight{ 8.0 };
    double penalty_increment{ 1.0 };
    int penalty_radius_cells{ 2 };

    // Approx homotopy signature reference obstacles (connected components)
    int component_min_cells{ 12 };
    int max_components{ 32 };
    int component_search_margin_cells{ 60 };

    // Debug data (consumed by PathPlannerEngine/PathPlannerRosIO)
    bool store_debug_data{ true };
    bool publish_poly_obstacles{ true };

    // Upstream-like visibility / polygonal map options
    // (kept lightweight; can be expanded later as we iterate)
    double visibility_ray_eps{ 1e-6 };          // radians, used for +/-epsilon rays at vertices
    int visibility_max_rays{ 8000 };            // safety cap
    int visibility_max_vertices{ 4096 };        // safety cap
    int polymap_min_loop_vertices{ 6 };         // discard tiny loops
  };

public:
  RaystarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, const Params& params, double obstacle_factor = 0.5);

  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) override;

  void appendDebugData(DebugData& data) const override;

private:
  bool upstreamLikePlan(const Point3d& start, const Point3d& goal, Points3d& out_path_map, Points3d& out_expand_map);

#if RPP_RAYSTAR_WITH_CGAL
  bool upstreamCgalPlan(const Point3d& start, const Point3d& goal, Points3d& out_path_map, Points3d& out_expand_map);
#endif

  struct ObstacleComponent
  {
    int min_x{ 0 };
    int max_x{ 0 };
    int min_y{ 0 };
    int max_y{ 0 };
    int cells{ 0 };
    double cx_map{ 0.0 };
    double cy_map{ 0.0 };
  };

  inline bool inBounds(int x, int y) const
  {
    return (x >= 0 && y >= 0 && x < nx_ && y < ny_);
  }

  bool isOccupiedCell(int x, int y) const;
  bool isTraversableCell(int x, int y) const;

  bool lineOfSight(int x0, int y0, int x1, int y1) const;

  double traversalCost(int x, int y, const std::vector<float>& penalty_grid) const;

  bool thetaStarPlan(const Point3d& start, const Point3d& goal, const std::vector<float>& penalty_grid,
                     Points3d& out_path_map, Points3d& out_expand_map, double& out_path_cost) const;

  void updatePenaltyGrid(const Points3d& path_map, std::vector<float>& penalty_grid) const;

  void computeObstacleComponentsInBox(int min_x, int min_y, int max_x, int max_y,
                                     std::vector<ObstacleComponent>& out_components) const;

  void selectSignatureReferencePoints(const std::vector<ObstacleComponent>& comps,
                                     std::vector<Point2d>& out_ref_world,
                                     std::vector<Points3d>& out_boxes_world) const;

  std::string computeHSignature(const Points3d& path_world, const std::vector<Point2d>& ref_world) const;

  static std::unordered_set<int> pathToCellSet(const Points3d& path_map, int nx);
  static double jaccardSimilarity(const std::unordered_set<int>& a, const std::unordered_set<int>& b);

  void clearDebugCache();

private:
  Params params_;
  double resolution_{ 0.0 };

  // Debug cache
  std::vector<Points3d> last_candidate_paths_world_;
  std::vector<double> last_candidate_costs_;
  int last_best_candidate_idx_{ -1 };
  std::vector<Points3d> last_poly_obstacles_world_;

  // Best run expand (map units)
  Points3d last_best_expand_map_;
};

}  // namespace path_planner
}  // namespace rpp

#endif  // RPP_RAYSTAR_GLOBAL_PLANNER_RAYSTAR_PLANNER_H_
