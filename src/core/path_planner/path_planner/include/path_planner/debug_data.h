/**
 * @file: debug_data.h
 * @brief: Planner debug snapshot shared between pipeline and plugins/visualization.
 *
 * NOTE: This header must NOT depend on any concrete planner implementation.
 */
#ifndef RPP_PATH_PLANNER_DEBUG_DATA_H_
#define RPP_PATH_PLANNER_DEBUG_DATA_H_

#include <vector>

#include "common/geometry/point.h"

namespace rpp
{
namespace path_planner
{

struct DebugData
{
  using Points3d = rpp::common::geometry::Points3d;

  // Timing (milliseconds)
  double planning_time_ms{ 0.0 };  // time spent in planner->plan()
  double total_time_ms{ 0.0 };     // total time including optional optimization
  bool has_total_time{ false };

  bool path_found{ false };

  // Planner outputs
  Points3d origin_path_map;       // map units (cells)
  Points3d expand_map;            // map units (cells)
  Points3d origin_plan_world;     // world units (meters)
  Points3d prune_plan_world;      // world units (meters)
  Points3d optimized_plan_world;  // world units (meters)

  // RHCF debug
  std::vector<Points3d> rhcf_candidate_paths_world;
  int rhcf_best_candidate_index{ -1 };

  // NAG debug
  std::vector<Points3d> nag_candidate_paths_world;
  int nag_best_candidate_index{ -1 };
  Points3d nag_cut_points_world;
  Points3d nag_cpr_cells_world;

  // RayStar debug (k-path visualization + polygonal obstacle overlay)
  std::vector<Points3d> raystar_candidate_paths_world;
  int raystar_best_candidate_index{ -1 };
  std::vector<Points3d> raystar_poly_obstacles_world;

  // GKVM debug (informative landmarks + greedy route)
  Points3d gkvm_landmarks_world;  // informative landmarks (excluding start/goal)
  Points3d gkvm_route_world;      // ordered route including start/goal

  // Parallel Curves debug (concentric curves visualization)
  // Points in world frame; theta() is interpreted as radius in meters.
  Points3d parallel_curves_circles_world;
};

}  // namespace path_planner
}  // namespace rpp

#endif  // RPP_PATH_PLANNER_DEBUG_DATA_H_
