/**
 * @file raystar_cgal_polymap.h
 *
 * CGAL-backed polygonal obstacle + visibility-region builder for Ray* (RayStar).
 *
 * This is an adaptation of the upstream Ray* implementation:
 *   https://github.com/ZJUTongYang/raystar
 *
 * Upstream license: MIT
 * Copyright (c) Tong Yang
 *
 * Notes for this repository:
 * - This file is only built/used when CGAL is available and enabled at build time.
 * - Interface is intentionally aligned with the existing CGAL-free "upstream_like" Polymap,
 *   so the same gap-tree search code can be reused.
 */

#ifndef RPP_RAYSTAR_GLOBAL_PLANNER_RAYSTAR_CGAL_POLYMAP_H_
#define RPP_RAYSTAR_GLOBAL_PLANNER_RAYSTAR_CGAL_POLYMAP_H_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <costmap_2d/costmap_2d.h>

#ifndef RPP_RAYSTAR_WITH_CGAL
#define RPP_RAYSTAR_WITH_CGAL 0
#endif

namespace rpp
{
namespace path_planner
{
namespace raystar_cgal
{

struct TopoIndex
{
  int obs{ -1 };
  int ver{ -1 };
};

inline bool topoValid(const TopoIndex& t)
{
  return t.obs >= 0 && t.ver >= 0;
}

class Polymap
{
public:
  Polymap(costmap_2d::Costmap2D* costmap, int nx, int ny, float obstacle_factor, bool traverse_unknown);

  bool build(int start_x, int start_y, int goal_x, int goal_y);

  bool solutionExist() const;

  int nxCells() const;
  int nyCells() const;

  const std::vector<std::vector<std::pair<int, int>>>& obstacles() const;

  TopoIndex locateVertex(int x, int y) const;

  // Directed adjacency along obstacle contour (matches upstream raystar::Polymap::areConsecutive).
  bool areConsecutive(const TopoIndex& prev, const TopoIndex& next) const;

  std::pair<int, int> getPrevObs(const TopoIndex& curr) const;
  std::pair<int, int> getNextObs(const TopoIndex& curr) const;

  void getVisibilityRegion(int sx, int sy, std::vector<std::pair<double, double>>& visibility_region,
                           std::vector<TopoIndex>& topo_V);

private:
  bool getPolyObstacles(int start_x, int start_y, int goal_x, int goal_y);
  void simplifyPolyObstacles(int start_x, int start_y, int goal_x, int goal_y);
  void registerVertices();
  void constructCGALRelated();
  bool calculateVisibilityRegion(int x, int y, std::vector<std::pair<double, double>>& result_V,
                                 std::vector<TopoIndex>& topo_V);

  bool isInTri(int x1, int y1, int x2, int y2, int x3, int y3, double x, double y) const;
  bool isAnObstacleEdge(const std::pair<int, int>& prev_pos, const std::pair<int, int>& next_pos) const;

  int locateAdjacentFacet(const std::pair<int, int>& prev, const std::pair<int, int>& next) const;

private:
  costmap_2d::Costmap2D* costmap_{ nullptr };
  int xsize_{ 0 };
  int ysize_{ 0 };
  float obstacle_factor_{ 1.0f };
  bool traverse_unknown_{ true };

  // Binary occupancy grid used by the upstream polymap logic: 0 = free, 1 = occupied.
  std::vector<uint8_t> occ_;

  // Polygonal obstacles (each is an ordered loop of integer vertices)
  std::vector<std::vector<std::pair<int, int>>> obs_;

  bool solution_exist_{ false };

  // Vertex topo lookup: for lattice point (x,y) -> (obs_index, ver_index), or (-1,-1) if not an obstacle vertex.
  std::vector<int> vertex_obs_index_;
  std::vector<int> vertex_ver_index_;

  // Cached visibility regions per integer source.
  std::unordered_map<int, std::vector<std::pair<double, double>>> V_storage_;
  std::unordered_map<int, std::vector<TopoIndex>> topoV_storage_;

#if RPP_RAYSTAR_WITH_CGAL
  // CGAL-related
  // Note: concrete types live in the .cpp to avoid including CGAL headers here.
  struct CgalState;
  std::unique_ptr<CgalState> cgal_;
#endif
};

}  // namespace raystar_cgal
}  // namespace path_planner
}  // namespace rpp

#endif  // RPP_RAYSTAR_GLOBAL_PLANNER_RAYSTAR_CGAL_POLYMAP_H_
