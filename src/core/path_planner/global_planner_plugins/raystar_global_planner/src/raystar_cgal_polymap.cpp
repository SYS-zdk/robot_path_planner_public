#include "raystar_global_planner/raystar_cgal_polymap.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <stack>
#include <unordered_map>
#include <utility>
#include <vector>

#include <costmap_2d/cost_values.h>

#if RPP_RAYSTAR_WITH_CGAL
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#endif

namespace rpp
{
namespace path_planner
{
namespace raystar_cgal
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

inline double normalizeAngle(double a)
{
  while (a > M_PI)
    a -= kTwoPi;
  while (a < -M_PI)
    a += kTwoPi;
  return a;
}

inline double normalizeAnglePositive(double a)
{
  a = std::fmod(a, kTwoPi);
  if (a < 0.0)
    a += kTwoPi;
  return a;
}

// Upstream pnpoly: return {in, on}. Uses the "two rays" trick.
inline std::pair<bool, bool> pnpoly(int nvert, const double* vertx, const double* verty, double testx, double testy)
{
  bool rightray = false;
  bool leftray = false;

  for (int i = 0, j = nvert - 1; i < nvert; j = i++)
  {
    const double yi = verty[i];
    const double yj = verty[j];
    if ((yi > testy) == (yj > testy))
      continue;

    const double temp = (vertx[j] - vertx[i]) * (testy - yi) / (yj - yi) + vertx[i];
    if (testx < temp)
      rightray = !rightray;
    if (testx > temp)
      leftray = !leftray;
  }

  if (rightray == leftray)
    return { rightray, false };
  return { false, true };
}

inline std::pair<double, double> findIntersection(const std::pair<int, int>& s, const std::pair<int, int>& g,
                                                 const std::pair<int, int>& p, const std::pair<int, int>& limit)
{
  // Intersection of line sg with line p-limit.
  const int a1 = g.second - s.second;
  const int b1 = -(g.first - s.first);
  const int c1 = -(g.second - s.second) * s.first + (g.first - s.first) * s.second;

  const int a2 = limit.second - p.second;
  const int b2 = -(limit.first - p.first);
  const int c2 = -(limit.second - p.second) * p.first + (limit.first - p.first) * p.second;

  std::pair<double, double> result;
  result.first = -1.0 * (static_cast<double>(c1) * b2 - static_cast<double>(c2) * b1) /
                 (static_cast<double>(a1) * b2 - static_cast<double>(a2) * b1);
  result.second = -1.0 * (static_cast<double>(c1) * a2 - static_cast<double>(c2) * a1) /
                  (static_cast<double>(b1) * a2 - static_cast<double>(b2) * a1);
  return result;
}

inline std::pair<double, double> findIntersection(const std::pair<int, int>& s, const std::pair<int, int>& g,
                                                 const std::pair<double, double>& p, double theta)
{
  const double a1 = std::sin(theta);
  const double b1 = -std::cos(theta);
  const double c1 = -std::sin(theta) * p.first + std::cos(theta) * p.second;

  const double a2 = static_cast<double>(g.second - s.second);
  const double b2 = -static_cast<double>(g.first - s.first);
  const double c2 = -static_cast<double>(g.second - s.second) * s.first + static_cast<double>(g.first - s.first) * s.second;

  std::pair<double, double> result;
  result.first = -(c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
  result.second = -(c1 * a2 - c2 * a1) / (b1 * a2 - b2 * a1);
  return result;
}

inline int roundToInt(double v)
{
  return static_cast<int>(std::llround(v));
}

}  // namespace

#if RPP_RAYSTAR_WITH_CGAL
struct Polymap::CgalState
{
  using K = CGAL::Exact_predicates_inexact_constructions_kernel;
  using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, CGAL::Default, CGAL::No_constraint_intersection_tag>;
  using Point = CDT::Point;

  CDT cdt;
  std::unordered_map<long long, int> cdt_table;
  int cdt_ver_num{ 0 };
  std::vector<std::array<std::pair<int, int>, 3>> facets;
};
#endif

Polymap::Polymap(costmap_2d::Costmap2D* costmap, int nx, int ny, float obstacle_factor, bool traverse_unknown)
  : costmap_(costmap)
  , xsize_(nx)
  , ysize_(ny)
  , obstacle_factor_(obstacle_factor)
  , traverse_unknown_(traverse_unknown)
{
}

bool Polymap::build(int start_x, int start_y, int goal_x, int goal_y)
{
  solution_exist_ = false;
  obs_.clear();
  V_storage_.clear();
  topoV_storage_.clear();

#if RPP_RAYSTAR_WITH_CGAL
  cgal_.reset(new CgalState());
#else
  return false;
#endif

  if (!costmap_ || xsize_ <= 0 || ysize_ <= 0)
    return false;

  const int n = xsize_ * ysize_;
  occ_.assign(static_cast<size_t>(n), 1);

  // Clamp to <= 1 so obstacle_factor_ > 1 cannot push the threshold above 255.
  const float eff = (std::isfinite(obstacle_factor_) && obstacle_factor_ > 0.0f) ? std::min(1.0f, obstacle_factor_) : 1.0f;
  const float occ_thresh = static_cast<float>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE) * eff;

  const unsigned char* raw = costmap_->getCharMap();
  for (int y = 0; y < ysize_; ++y)
  {
    for (int x = 0; x < xsize_; ++x)
    {
      const int id = x + y * xsize_;
      bool occ = true;
      if (x == 0 || y == 0 || x == xsize_ - 1 || y == ysize_ - 1)
      {
        occ = true;
      }
      else
      {
        const unsigned char c = raw[id];
        if (c == costmap_2d::NO_INFORMATION)
          occ = !traverse_unknown_;
        else
          occ = (static_cast<float>(c) >= occ_thresh);
      }
      occ_[static_cast<size_t>(id)] = occ ? 1 : 0;
    }
  }

  solution_exist_ = getPolyObstacles(start_x, start_y, goal_x, goal_y);
  if (!solution_exist_)
    return false;

  simplifyPolyObstacles(start_x, start_y, goal_x, goal_y);
  registerVertices();
  constructCGALRelated();

  return true;
}

bool Polymap::solutionExist() const
{
  return solution_exist_;
}

int Polymap::nxCells() const
{
  return xsize_;
}

int Polymap::nyCells() const
{
  return ysize_;
}

const std::vector<std::vector<std::pair<int, int>>>& Polymap::obstacles() const
{
  return obs_;
}

TopoIndex Polymap::locateVertex(int x, int y) const
{
  if (x < 0 || y < 0 || x >= xsize_ || y >= ysize_)
    return TopoIndex{};
  const int id = x + y * xsize_;
  return TopoIndex{ vertex_obs_index_[static_cast<size_t>(id)], vertex_ver_index_[static_cast<size_t>(id)] };
}

bool Polymap::areConsecutive(const TopoIndex& prev, const TopoIndex& next) const
{
  if (!topoValid(prev) || !topoValid(next))
    return false;
  if (prev.obs != next.obs)
    return false;
  const auto& loop = obs_.at(static_cast<size_t>(prev.obs));
  const int n = static_cast<int>(loop.size());
  if (n <= 1)
    return false;
  return ((next.ver - prev.ver + n) % n) == 1;
}

std::pair<int, int> Polymap::getPrevObs(const TopoIndex& curr) const
{
  const auto& loop = obs_.at(static_cast<size_t>(curr.obs));
  const int n = static_cast<int>(loop.size());
  const int j = (curr.ver - 1 + n) % n;
  return loop[static_cast<size_t>(j)];
}

std::pair<int, int> Polymap::getNextObs(const TopoIndex& curr) const
{
  const auto& loop = obs_.at(static_cast<size_t>(curr.obs));
  const int n = static_cast<int>(loop.size());
  const int j = (curr.ver + 1) % n;
  return loop[static_cast<size_t>(j)];
}

void Polymap::getVisibilityRegion(int sx, int sy, std::vector<std::pair<double, double>>& visibility_region,
                                 std::vector<TopoIndex>& topo_V)
{
  visibility_region.clear();
  topo_V.clear();

  if (sx < 0 || sy < 0 || sx >= xsize_ || sy >= ysize_)
    return;

  const int index = sx + sy * xsize_;
  const auto it = V_storage_.find(index);
  if (it != V_storage_.end())
  {
    visibility_region = it->second;
    topo_V = topoV_storage_[index];
    return;
  }

  calculateVisibilityRegion(sx, sy, visibility_region, topo_V);
  V_storage_[index] = visibility_region;
  topoV_storage_[index] = topo_V;
}

bool Polymap::getPolyObstacles(int start_x, int start_y, int goal_x, int goal_y)
{
  obs_.clear();

  const unsigned int nx = static_cast<unsigned int>(xsize_);
  const unsigned int ny = static_cast<unsigned int>(ysize_);

  if (start_x < 0 || start_y < 0 || start_x >= xsize_ || start_y >= ysize_)
    return false;
  if (goal_x < 0 || goal_y < 0 || goal_x >= xsize_ || goal_y >= ysize_)
    return false;

  // (1) Flood fill reachable free space and collect boundary edges.
  // NOTE: This follows the upstream raystar code, including its edge keying scheme.
  std::vector<uint8_t> mask(static_cast<size_t>(nx * ny), 0);

  std::unordered_map<int, int> edges;  // <source+target, source>
  std::stack<int> Q;

  Q.emplace(start_x + start_y * static_cast<int>(nx));
  while (!Q.empty())
  {
    const int cur = Q.top();
    Q.pop();

    const int x = cur % static_cast<int>(nx);
    const int y = (cur - x) / static_cast<int>(nx);

    if (occ_[static_cast<size_t>(cur)] != 0 || mask[static_cast<size_t>(cur)] != 0)
      continue;

    // Boundary edge insertion. These indices are safe because we treat the border as occupied.
    if (occ_[static_cast<size_t>(cur - 1)] != 0)
      edges[cur + (cur + static_cast<int>(nx))] = cur;  // +nx
    if (occ_[static_cast<size_t>(cur + 1)] != 0)
      edges[(cur + 1) + (cur + static_cast<int>(nx) + 1)] = cur + static_cast<int>(nx) + 1;  // -nx
    if (occ_[static_cast<size_t>(cur - static_cast<int>(nx))] != 0)
      edges[cur + (cur + 1)] = cur + 1;  // -1
    if (occ_[static_cast<size_t>(cur + static_cast<int>(nx))] != 0)
      edges[(cur + static_cast<int>(nx)) + (cur + static_cast<int>(nx) + 1)] = cur + static_cast<int>(nx);  // +1

    mask[static_cast<size_t>(cur)] = 1;

    if (occ_[static_cast<size_t>(cur - 1)] == 0)
      Q.push(cur - 1);
    if (occ_[static_cast<size_t>(cur + 1)] == 0)
      Q.push(cur + 1);
    if (occ_[static_cast<size_t>(cur - static_cast<int>(nx))] == 0)
      Q.push(cur - static_cast<int>(nx));
    if (occ_[static_cast<size_t>(cur + static_cast<int>(nx))] == 0)
      Q.push(cur + static_cast<int>(nx));
  }

  // (2) If the goal is not reachable, stop.
  const int goal_id = goal_x + goal_y * static_cast<int>(nx);
  if (mask[static_cast<size_t>(goal_id)] == 0)
    return false;

  // (3) Bug-style contour tracing on the edge set.
  std::list<std::pair<int, int>> boundary_points;

  while (!edges.empty())
  {
    boundary_points.clear();

    const auto first_iter = edges.begin();
    const int key = first_iter->first;
    const int value = first_iter->second;

    const int prev = value;
    const int next = key - value;

    const int prev_x = prev % static_cast<int>(nx);
    const int prev_y = (prev - prev_x) / static_cast<int>(nx);
    const int next_x = next % static_cast<int>(nx);
    const int next_y = (next - next_x) / static_cast<int>(nx);

    boundary_points.emplace_back(prev_x, prev_y);
    boundary_points.emplace_back(next_x, next_y);

    while (true)
    {
      const int x = boundary_points.back().first;
      const int y = boundary_points.back().second;
      const int cur = x + y * static_cast<int>(nx);

      int lb_free = 0, lt_free = 0, rb_free = 0, rt_free = 0;

      if (x > 0 && y > 0 && occ_[static_cast<size_t>(cur - static_cast<int>(nx) - 1)] == 0)
        lb_free = 1;
      if (x > 0 && occ_[static_cast<size_t>(cur - 1)] == 0)
        lt_free = 1;
      if (y > 0 && occ_[static_cast<size_t>(cur - static_cast<int>(nx))] == 0)
        rb_free = 1;
      if (occ_[static_cast<size_t>(cur)] == 0)
        rt_free = 1;

      const int num = lb_free * 8 + lt_free * 4 + rb_free * 2 + rt_free;

      // position map
      //  2 | 4
      // ------
      //  1 | 3

      switch (num)
      {
        case 1:
          boundary_points.emplace_back(x, y + 1);
          break;
        case 2:
          boundary_points.emplace_back(x + 1, y);
          break;
        case 3:
          boundary_points.emplace_back(x, y + 1);
          break;
        case 4:
          boundary_points.emplace_back(x - 1, y);
          break;
        case 5:
          boundary_points.emplace_back(x - 1, y);
          break;
        case 6:
        {
          auto iter = std::prev(boundary_points.end(), 2);
          if (iter->first == x && iter->second == y - 1)
            boundary_points.emplace_back(x + 1, y);
          else if (iter->first == x && iter->second == y + 1)
            boundary_points.emplace_back(x - 1, y);
          break;
        }
        case 7:
          boundary_points.emplace_back(x - 1, y);
          break;
        case 8:
          boundary_points.emplace_back(x, y - 1);
          break;
        case 9:
        {
          auto iter = std::prev(boundary_points.end(), 2);
          if (iter->first == x + 1 && iter->second == y)
            boundary_points.emplace_back(x, y + 1);
          else if (iter->first == x - 1 && iter->second == y)
            boundary_points.emplace_back(x, y - 1);
          break;
        }
        case 10:
          boundary_points.emplace_back(x + 1, y);
          break;
        case 11:
          boundary_points.emplace_back(x, y + 1);
          break;
        case 12:
          boundary_points.emplace_back(x, y - 1);
          break;
        case 13:
          boundary_points.emplace_back(x, y - 1);
          break;
        case 14:
          boundary_points.emplace_back(x + 1, y);
          break;
        default:
          // 0 or 15 should not happen on a proper boundary.
          boundary_points.emplace_back(x, y);
          break;
      }

      if (boundary_points.back().first == boundary_points.front().first &&
          boundary_points.back().second == boundary_points.front().second)
      {
        boundary_points.pop_back();
        break;
      }
    }

    std::vector<std::pair<int, int>> loop;
    loop.reserve(boundary_points.size());
    for (const auto& p : boundary_points)
      loop.emplace_back(p);

    obs_.push_back(loop);

    // Remove used edges (upstream uses curr+next as key).
    for (auto it = loop.begin(); it != loop.end(); ++it)
    {
      const int curr = it->first + it->second * static_cast<int>(nx);
      int next;
      if (std::next(it) == loop.end())
        next = loop.front().first + loop.front().second * static_cast<int>(nx);
      else
        next = std::next(it)->first + std::next(it)->second * static_cast<int>(nx);

      edges.erase(curr + next);
    }
  }

  return true;
}

void Polymap::registerVertices()
{
  vertex_obs_index_.assign(static_cast<size_t>(xsize_ * ysize_), -1);
  vertex_ver_index_.assign(static_cast<size_t>(xsize_ * ysize_), -1);

  for (size_t i = 0; i < obs_.size(); ++i)
  {
    for (size_t j = 0; j < obs_[i].size(); ++j)
    {
      const int x = obs_[i][j].first;
      const int y = obs_[i][j].second;
      if (x < 0 || y < 0 || x >= xsize_ || y >= ysize_)
        continue;
      const int id = x + y * xsize_;
      vertex_obs_index_[static_cast<size_t>(id)] = static_cast<int>(i);
      vertex_ver_index_[static_cast<size_t>(id)] = static_cast<int>(j);
    }
  }
}

bool Polymap::isInTri(int x1, int y1, int x2, int y2, int x3, int y3, double x, double y) const
{
  const double vertx[3] = { static_cast<double>(x1), static_cast<double>(x2), static_cast<double>(x3) };
  const double verty[3] = { static_cast<double>(y1), static_cast<double>(y2), static_cast<double>(y3) };
  const auto b = pnpoly(3, vertx, verty, x, y);
  return b.first || b.second;
}

void Polymap::simplifyPolyObstacles(int start_x, int start_y, int goal_x, int goal_y)
{
  for (auto& poly : obs_)
  {
    if (poly.size() < 3)
      continue;

    bool stable = false;
    size_t curr = 0;

    while (true)
    {
      if (poly.size() < 3)
        break;

      const size_t n = poly.size();
      const size_t prev = (curr + n - 1) % n;
      const size_t next = (curr + 1) % n;

      const int x1 = poly[prev].first;
      const int y1 = poly[prev].second;
      const int x2 = poly[curr].first;
      const int y2 = poly[curr].second;
      const int x3 = poly[next].first;
      const int y3 = poly[next].second;

      const double prev_dir = std::atan2(static_cast<double>(y2 - y1), static_cast<double>(x2 - x1));
      const double next_dir = std::atan2(static_cast<double>(y3 - y2), static_cast<double>(x3 - x2));

      bool simplifiable = false;

      if ((x3 - x2) * (y2 - y1) == (x2 - x1) * (y3 - y2))
      {
        // Collinear.
        simplifiable = true;
      }
      else
      {
        const double diff_dir = normalizeAngle(next_dir - prev_dir);
        simplifiable = true;

        if (diff_dir <= 0.0 || diff_dir > 0.999 * M_PI)
        {
          if (isInTri(x1, y1, x2, y2, x3, y3, start_x, start_y) || isInTri(x1, y1, x2, y2, x3, y3, goal_x, goal_y))
            simplifiable = false;

          if (simplifiable)
          {
            // Ensure no other obstacle vertex lies inside the inflated triangle.
            for (size_t oi = 0; oi < obs_.size() && simplifiable; ++oi)
            {
              for (size_t vi = 0; vi < obs_[oi].size(); ++vi)
              {
                const double testx = static_cast<double>(obs_[oi][vi].first);
                const double testy = static_cast<double>(obs_[oi][vi].second);

                if (!isInTri(x1, y1, x2, y2, x3, y3, testx, testy))
                  continue;

                // Triangle vertices do not count.
                if (&obs_[oi] != &poly)
                {
                  simplifiable = false;
                  break;
                }

                if (vi != prev && vi != curr && vi != next)
                {
                  simplifiable = false;
                  break;
                }
              }
            }
          }
        }
        else
        {
          simplifiable = false;
        }
      }

      if (simplifiable)
      {
        poly.erase(poly.begin() + static_cast<std::ptrdiff_t>(curr));
        if (curr >= poly.size())
          curr = poly.size() - 1;
        stable = false;
        continue;
      }

      if (curr == 0)
      {
        if (!stable)
          stable = true;
        else
          break;
      }

      curr = (curr + 1) % poly.size();
    }
  }
}

void Polymap::constructCGALRelated()
{
#if !RPP_RAYSTAR_WITH_CGAL
  return;
#else
  if (!cgal_)
    cgal_.reset(new CgalState());

  cgal_->cdt.clear();
  cgal_->cdt_table.clear();
  cgal_->facets.clear();

  // Insert obstacle boundaries as constraints.
  for (const auto& poly : obs_)
  {
    for (auto it = poly.begin(); it != poly.end(); ++it)
    {
      auto nxt = std::next(it);
      if (nxt == poly.end())
        nxt = poly.begin();
      cgal_->cdt.insert_constraint(CgalState::Point(it->first, it->second), CgalState::Point(nxt->first, nxt->second));
    }
  }

  (void)cgal_->cdt.is_valid();

  const long long size = static_cast<long long>(xsize_) * static_cast<long long>(ysize_);
  auto vid = [this](int x, int y) { return static_cast<long long>(x + y * xsize_); };

  int count = 0;
  for (auto fit = cgal_->cdt.finite_faces_begin(); fit != cgal_->cdt.finite_faces_end(); ++fit)
  {
    std::array<std::pair<int, int>, 3> tri;

    const int x0 = static_cast<int>(std::llround(CGAL::to_double(fit->vertex(0)->point().x())));
    const int y0 = static_cast<int>(std::llround(CGAL::to_double(fit->vertex(0)->point().y())));
    const int x1 = static_cast<int>(std::llround(CGAL::to_double(fit->vertex(1)->point().x())));
    const int y1 = static_cast<int>(std::llround(CGAL::to_double(fit->vertex(1)->point().y())));
    const int x2 = static_cast<int>(std::llround(CGAL::to_double(fit->vertex(2)->point().x())));
    const int y2 = static_cast<int>(std::llround(CGAL::to_double(fit->vertex(2)->point().y())));

    tri[0] = { x0, y0 };
    tri[1] = { x1, y1 };
    tri[2] = { x2, y2 };

    cgal_->facets.push_back(tri);

    const long long v0 = vid(x0, y0);
    const long long v1 = vid(x1, y1);
    const long long v2 = vid(x2, y2);

    cgal_->cdt_table[v0 + v1 * size] = count;
    cgal_->cdt_table[v1 + v2 * size] = count;
    cgal_->cdt_table[v2 + v0 * size] = count;

    ++count;
  }

  cgal_->cdt_ver_num = cgal_->cdt.number_of_vertices();
#endif
}

int Polymap::locateAdjacentFacet(const std::pair<int, int>& prev, const std::pair<int, int>& next) const
{
#if !RPP_RAYSTAR_WITH_CGAL
  return -1;
#else
  if (!cgal_)
    return -1;

  const long long size = static_cast<long long>(xsize_) * static_cast<long long>(ysize_);
  const long long pid = static_cast<long long>(prev.first + prev.second * xsize_);
  const long long nid = static_cast<long long>(next.first + next.second * xsize_);

  const long long key = pid + nid * size;
  const auto it = cgal_->cdt_table.find(key);
  if (it == cgal_->cdt_table.end())
    return -1;
  return it->second;
#endif
}

bool Polymap::isAnObstacleEdge(const std::pair<int, int>& prev_pos, const std::pair<int, int>& next_pos) const
{
  const TopoIndex topo_prev = locateVertex(prev_pos.first, prev_pos.second);
  const TopoIndex topo_next = locateVertex(next_pos.first, next_pos.second);
  return areConsecutive(topo_prev, topo_next);
}

bool Polymap::calculateVisibilityRegion(int round_x, int round_y, std::vector<std::pair<double, double>>& result_V,
                                       std::vector<TopoIndex>& topo_V)
{
#if !RPP_RAYSTAR_WITH_CGAL
  (void)round_x;
  (void)round_y;
  result_V.clear();
  topo_V.clear();
  return false;
#else
  result_V.clear();
  topo_V.clear();

  struct BungiuEdge
  {
    std::pair<double, double> prev_pos;
    std::pair<double, double> next_pos;
    TopoIndex topo_prev;
    TopoIndex topo_next;
    double limit_prev{ 0.0 };
    double limit_next{ 0.0 };
    std::pair<int, int> limit_prev_pos;
    std::pair<int, int> limit_next_pos;
    bool is_bd{ false };
  };

  if (!cgal_)
    return false;

  std::list<BungiuEdge> bd;
  bool open_visibility_region = false;

  // Check whether source point is an obstacle vertex.
  const TopoIndex src_topo = locateVertex(round_x, round_y);
  if (topoValid(src_topo))
  {
    open_visibility_region = true;

    const auto curr = src_topo;
    const auto prev_v = getPrevObs(curr);
    const auto next_v = getNextObs(curr);

    std::pair<int, int> the_vertex = prev_v;
    while (!(the_vertex.first == next_v.first && the_vertex.second == next_v.second))
    {
      const int loc = locateAdjacentFacet({ round_x, round_y }, the_vertex);
      if (loc < 0 || loc >= static_cast<int>(cgal_->facets.size()))
        break;

      int i = 0;
      for (; i < 3; ++i)
      {
        const auto& p = cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>(i)];
        if (!((p.first == round_x && p.second == round_y) || (p.first == the_vertex.first && p.second == the_vertex.second)))
          break;
      }
      if (i >= 3)
        break;

      bd.emplace_back();
      bd.back().prev_pos = { static_cast<double>(the_vertex.first), static_cast<double>(the_vertex.second) };
      bd.back().next_pos = { static_cast<double>(cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>(i)].first),
                             static_cast<double>(cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>(i)].second) };

      bd.back().topo_prev = locateVertex(the_vertex.first, the_vertex.second);
      bd.back().topo_next = locateVertex(static_cast<int>(bd.back().next_pos.first), static_cast<int>(bd.back().next_pos.second));

      bd.back().limit_prev = std::atan2(bd.back().prev_pos.second - round_y, bd.back().prev_pos.first - round_x);
      bd.back().limit_next = std::atan2(bd.back().next_pos.second - round_y, bd.back().next_pos.first - round_x);
      bd.back().limit_next = bd.back().limit_prev + normalizeAnglePositive(bd.back().limit_next - bd.back().limit_prev);

      bd.back().limit_prev_pos = { the_vertex.first, the_vertex.second };
      bd.back().limit_next_pos = { static_cast<int>(bd.back().next_pos.first), static_cast<int>(bd.back().next_pos.second) };

      bd.back().is_bd = isAnObstacleEdge(bd.back().limit_next_pos, bd.back().limit_prev_pos);

      the_vertex = bd.back().limit_next_pos;
    }
  }
  else
  {
    // Source point is within a triangle facet.
    int loc = -1;
    for (size_t fi = 0; fi < cgal_->facets.size(); ++fi)
    {
      const auto& tri = cgal_->facets[fi];
      if (isInTri(tri[0].first, tri[0].second, tri[1].first, tri[1].second, tri[2].first, tri[2].second, round_x, round_y))
      {
        loc = static_cast<int>(fi);
        break;
      }
    }
    if (loc < 0)
      return false;

    for (int i = 0; i < 3; ++i)
    {
      const auto& a = cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>(i)];
      const auto& b = cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>((i + 1) % 3)];

      bd.emplace_back();
      bd.back().prev_pos = { static_cast<double>(a.first), static_cast<double>(a.second) };
      bd.back().next_pos = { static_cast<double>(b.first), static_cast<double>(b.second) };
      bd.back().topo_prev = locateVertex(a.first, a.second);
      bd.back().topo_next = locateVertex(b.first, b.second);

      bd.back().limit_prev = std::atan2(bd.back().prev_pos.second - round_y, bd.back().prev_pos.first - round_x);
      bd.back().limit_next = std::atan2(bd.back().next_pos.second - round_y, bd.back().next_pos.first - round_x);
      bd.back().limit_next = bd.back().limit_prev + normalizeAnglePositive(bd.back().limit_next - bd.back().limit_prev);

      bd.back().limit_prev_pos = { a.first, a.second };
      bd.back().limit_next_pos = { b.first, b.second };

      bd.back().is_bd = isAnObstacleEdge(bd.back().limit_next_pos, bd.back().limit_prev_pos);
    }
  }

  // Iteratively expand the visibility boundary link.
  auto iter = bd.begin();
  while (true)
  {
    if (iter == bd.end())
      break;

    if (iter->is_bd)
    {
      ++iter;
      continue;
    }

    const int loc = locateAdjacentFacet({ static_cast<int>(iter->next_pos.first), static_cast<int>(iter->next_pos.second) },
                                        { static_cast<int>(iter->prev_pos.first), static_cast<int>(iter->prev_pos.second) });
    if (loc < 0 || loc >= static_cast<int>(cgal_->facets.size()))
    {
      ++iter;
      continue;
    }

    int k = 0;
    for (; k < 3; ++k)
    {
      const auto& p = cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>(k)];
      if (!((p.first == roundToInt(iter->prev_pos.first) && p.second == roundToInt(iter->prev_pos.second)) ||
            (p.first == roundToInt(iter->next_pos.first) && p.second == roundToInt(iter->next_pos.second))))
        break;
    }
    if (k >= 3)
    {
      ++iter;
      continue;
    }

    const auto& w = cgal_->facets[static_cast<size_t>(loc)][static_cast<size_t>(k)];

    const double theta = iter->limit_prev +
                         normalizeAngle(std::atan2(static_cast<double>(w.second - round_y), static_cast<double>(w.first - round_x)) -
                                        iter->limit_prev);

    if (theta < iter->limit_prev)
    {
      if (isAnObstacleEdge({ roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) }, w))
      {
        const auto endpoint_prev = findIntersection(w, { roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) },
                                                   { round_x, round_y }, iter->limit_prev_pos);
        const auto endpoint_next = findIntersection(w, { roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) },
                                                   { round_x, round_y }, iter->limit_next_pos);

        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = { static_cast<double>(iter->limit_prev_pos.first), static_cast<double>(iter->limit_prev_pos.second) };
        new_iter->next_pos = endpoint_prev;
        new_iter->topo_prev = locateVertex(iter->limit_prev_pos.first, iter->limit_prev_pos.second);
        new_iter->topo_next = locateVertex(roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second));
        new_iter->is_bd = true;

        new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = endpoint_prev;
        new_iter->next_pos = endpoint_next;
        new_iter->topo_prev = locateVertex(roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second));
        new_iter->topo_next = locateVertex(roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second));
        new_iter->is_bd = true;

        if (!(roundToInt(iter->next_pos.first) == roundToInt(endpoint_next.first) &&
              roundToInt(iter->next_pos.second) == roundToInt(endpoint_next.second)))
        {
          iter->prev_pos = endpoint_next;
          iter->next_pos = { static_cast<double>(iter->limit_next_pos.first), static_cast<double>(iter->limit_next_pos.second) };
          iter->topo_prev = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
          iter->topo_next = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
          iter->is_bd = true;
          ++iter;
        }
        else
        {
          iter = bd.erase(iter);
        }
      }
      else
      {
        iter->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        iter->topo_prev = locateVertex(w.first, w.second);
      }
    }
    else if (theta == iter->limit_prev)
    {
      auto new_iter = bd.insert(iter, BungiuEdge());
      new_iter->prev_pos = { static_cast<double>(iter->limit_prev_pos.first), static_cast<double>(iter->limit_prev_pos.second) };
      new_iter->next_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
      new_iter->topo_prev = locateVertex(iter->limit_prev_pos.first, iter->limit_prev_pos.second);
      new_iter->topo_next = locateVertex(w.first, w.second);
      new_iter->is_bd = true;

      if (isAnObstacleEdge({ roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) }, w))
      {
        const auto endpoint_next = findIntersection(w, { roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) },
                                                   { round_x, round_y }, iter->limit_next_pos);
        auto new_iter2 = bd.insert(iter, BungiuEdge());
        new_iter2->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        new_iter2->next_pos = endpoint_next;
        new_iter2->topo_prev = locateVertex(w.first, w.second);
        new_iter2->topo_next = locateVertex(roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second));
        new_iter2->is_bd = true;

        if (!(roundToInt(iter->next_pos.first) == roundToInt(endpoint_next.first) &&
              roundToInt(iter->next_pos.second) == roundToInt(endpoint_next.second)))
        {
          iter->prev_pos = endpoint_next;
          iter->next_pos = { static_cast<double>(iter->limit_next_pos.first), static_cast<double>(iter->limit_next_pos.second) };
          iter->topo_prev = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
          iter->topo_next = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
          iter->is_bd = true;
          ++iter;
        }
        else
        {
          iter = bd.erase(iter);
        }
      }
      else
      {
        iter->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        iter->topo_prev = locateVertex(w.first, w.second);
        iter->limit_prev_pos = w;
      }
    }
    else if (theta > iter->limit_prev && theta < iter->limit_next)
    {
      bool to_minus = false;

      if (isAnObstacleEdge(w, { roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) }))
      {
        const auto endpoint_prev = findIntersection({ roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) }, w,
                                                   { round_x, round_y }, iter->limit_prev_pos);

        if (!(roundToInt(endpoint_prev.first) == iter->limit_prev_pos.first &&
              roundToInt(endpoint_prev.second) == iter->limit_prev_pos.second))
        {
          auto new_iter = bd.insert(iter, BungiuEdge());
          new_iter->prev_pos = { static_cast<double>(iter->limit_prev_pos.first), static_cast<double>(iter->limit_prev_pos.second) };
          new_iter->next_pos = endpoint_prev;
          new_iter->topo_prev = locateVertex(iter->limit_prev_pos.first, iter->limit_prev_pos.second);
          new_iter->topo_next = locateVertex(w.first, w.second);
          new_iter->is_bd = true;
        }

        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = endpoint_prev;
        new_iter->next_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        new_iter->topo_prev = locateVertex(w.first, w.second);
        new_iter->topo_next = locateVertex(w.first, w.second);
        new_iter->is_bd = true;
      }
      else
      {
        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = iter->prev_pos;
        new_iter->next_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        new_iter->topo_prev = locateVertex(w.first, w.second);
        new_iter->topo_next = locateVertex(w.first, w.second);
        new_iter->limit_prev = iter->limit_prev;
        new_iter->limit_next = theta;
        new_iter->limit_prev_pos = iter->limit_prev_pos;
        new_iter->limit_next_pos = w;
        new_iter->is_bd = false;
        to_minus = true;
      }

      if (isAnObstacleEdge({ roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) }, w))
      {
        const auto endpoint_next = findIntersection(w, { roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second) },
                                                   { round_x, round_y }, iter->limit_next_pos);
        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        new_iter->next_pos = endpoint_next;
        new_iter->topo_prev = locateVertex(w.first, w.second);
        new_iter->topo_next = locateVertex(roundToInt(iter->next_pos.first), roundToInt(iter->next_pos.second));
        new_iter->is_bd = true;

        if (!(iter->limit_next_pos.first == roundToInt(endpoint_next.first) &&
              iter->limit_next_pos.second == roundToInt(endpoint_next.second)))
        {
          iter->prev_pos = endpoint_next;
          iter->next_pos = { static_cast<double>(iter->limit_next_pos.first), static_cast<double>(iter->limit_next_pos.second) };
          iter->topo_prev = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
          iter->topo_next = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
          iter->is_bd = true;
          if (to_minus)
            iter = std::prev(new_iter);
          else
            ++iter;
        }
        else
        {
          iter = bd.erase(iter);
          if (to_minus)
            iter = std::prev(new_iter);
        }
      }
      else
      {
        iter->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        iter->topo_prev = locateVertex(w.first, w.second);
        iter->limit_prev = theta;
        iter->limit_prev_pos = w;
        if (to_minus)
          --iter;
      }
    }
    else if (theta == iter->limit_next)
    {
      if (isAnObstacleEdge(w, { roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) }))
      {
        const auto endpoint_prev = findIntersection(w, { roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) },
                                                   { round_x, round_y }, iter->limit_prev_pos);

        if (!(roundToInt(endpoint_prev.first) == iter->limit_prev_pos.first &&
              roundToInt(endpoint_prev.second) == iter->limit_prev_pos.second))
        {
          auto new_iter = bd.insert(iter, BungiuEdge());
          new_iter->prev_pos = { static_cast<double>(iter->limit_prev_pos.first), static_cast<double>(iter->limit_prev_pos.second) };
          new_iter->next_pos = endpoint_prev;
          new_iter->topo_prev = locateVertex(iter->limit_prev_pos.first, iter->limit_prev_pos.second);
          new_iter->topo_next = locateVertex(w.first, w.second);
          new_iter->is_bd = true;
        }

        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = endpoint_prev;
        new_iter->next_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        new_iter->topo_prev = locateVertex(w.first, w.second);
        new_iter->topo_next = locateVertex(w.first, w.second);
        new_iter->is_bd = true;

        iter->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        iter->topo_prev = locateVertex(w.first, w.second);
        iter->is_bd = true;
        ++iter;
      }
      else
      {
        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = iter->prev_pos;
        new_iter->next_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        new_iter->topo_prev = locateVertex(roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second));
        new_iter->topo_next = locateVertex(w.first, w.second);
        new_iter->limit_prev = iter->limit_prev;
        new_iter->limit_next = theta;
        new_iter->limit_prev_pos = iter->limit_prev_pos;
        new_iter->limit_next_pos = w;
        new_iter->is_bd = false;

        iter->prev_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        iter->topo_prev = locateVertex(w.first, w.second);
        iter->is_bd = true;
        ++iter;
      }
    }
    else if (theta > iter->limit_next)
    {
      if (isAnObstacleEdge(w, { roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) }))
      {
        const auto endpoint_prev = findIntersection(w, { roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) },
                                                   { round_x, round_y }, iter->limit_prev_pos);
        const auto endpoint_next = findIntersection(w, { roundToInt(iter->prev_pos.first), roundToInt(iter->prev_pos.second) },
                                                   { round_x, round_y }, iter->limit_next_pos);

        if (!(roundToInt(endpoint_prev.first) == iter->limit_prev_pos.first &&
              roundToInt(endpoint_prev.second) == iter->limit_prev_pos.second))
        {
          auto new_iter = bd.insert(iter, BungiuEdge());
          new_iter->prev_pos = { static_cast<double>(iter->limit_prev_pos.first), static_cast<double>(iter->limit_prev_pos.second) };
          new_iter->next_pos = endpoint_prev;
          new_iter->topo_prev = locateVertex(iter->limit_prev_pos.first, iter->limit_prev_pos.second);
          new_iter->topo_next = locateVertex(w.first, w.second);
          new_iter->is_bd = true;
        }

        auto new_iter = bd.insert(iter, BungiuEdge());
        new_iter->prev_pos = endpoint_prev;
        new_iter->next_pos = endpoint_next;
        new_iter->topo_prev = locateVertex(w.first, w.second);
        new_iter->topo_next = locateVertex(w.first, w.second);
        new_iter->is_bd = true;

        iter->prev_pos = endpoint_next;
        iter->next_pos = { static_cast<double>(iter->limit_next_pos.first), static_cast<double>(iter->limit_next_pos.second) };
        iter->topo_prev = locateVertex(w.first, w.second);
        iter->topo_next = locateVertex(iter->limit_next_pos.first, iter->limit_next_pos.second);
        iter->is_bd = true;
        ++iter;
      }
      else
      {
        iter->next_pos = { static_cast<double>(w.first), static_cast<double>(w.second) };
        iter->topo_next = locateVertex(w.first, w.second);
      }
    }
    else
    {
      ++iter;
    }
  }

  for (const auto& e : bd)
  {
    result_V.emplace_back(e.prev_pos);
    topo_V.emplace_back(e.topo_prev);
  }

  if (open_visibility_region && !bd.empty())
  {
    result_V.emplace_back(bd.back().next_pos);
    topo_V.emplace_back(bd.back().topo_next);
  }

  return !bd.empty();
#endif
}

}  // namespace raystar_cgal
}  // namespace path_planner
}  // namespace rpp
