#include "raystar_global_planner/raystar_planner.h"

#if RPP_RAYSTAR_WITH_CGAL
#include "raystar_global_planner/raystar_cgal_polymap.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stack>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <costmap_2d/cost_values.h>

#include "common/geometry/point.h"
#include "common/geometry/collision_checker.h"
#include "common/structure/node.h"

namespace rpp
{
namespace path_planner
{
namespace
{

struct QueueNode
{
  int id{ -1 };
  int x{ 0 };
  int y{ 0 };
  double f{ 0.0 };
  double g{ 0.0 };

  bool operator<(const QueueNode& other) const
  {
    if (f == other.f)
    {
      return g > other.g;
    }
    return f > other.f;
  }
};

inline double hypot2(int x0, int y0, int x1, int y1)
{
  return std::hypot(static_cast<double>(x0 - x1), static_cast<double>(y0 - y1));
}

namespace upstream_like
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

inline double wrapToStart(double theta, double start)
{
  return start + normalizeAnglePositive(theta - start);
}

struct TopoIndex
{
  int obs{ -1 };
  int ver{ -1 };
};

inline bool topoValid(const TopoIndex& t)
{
  return t.obs >= 0 && t.ver >= 0;
}

inline uint64_t packXY(int x, int y)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) | static_cast<uint32_t>(y);
}

inline uint64_t packEdge(int u, int v)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(u)) << 32) | static_cast<uint32_t>(v);
}

inline double cross2(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

struct RayHit
{
  double ang{ 0.0 };
  double x{ 0.0 };
  double y{ 0.0 };
  TopoIndex topo;
  double dist2{ 0.0 };
};

// Ray p + t*r, t>=0 intersect segment [a,b]
inline bool raySegmentIntersection(double px, double py, double rx, double ry, double ax, double ay, double bx, double by,
                                  double& out_t, double& out_ix, double& out_iy)
{
  const double sx = bx - ax;
  const double sy = by - ay;

  const double rxs = cross2(rx, ry, sx, sy);
  const double qpx = ax - px;
  const double qpy = ay - py;
  const double qpxr = cross2(qpx, qpy, rx, ry);

  const double eps = 1e-12;
  if (std::abs(rxs) < eps)
  {
    // Parallel or collinear: ignore for now.
    return false;
  }

  const double t = cross2(qpx, qpy, sx, sy) / rxs;
  const double u = qpxr / rxs;

  if (t < 1e-9)
    return false;
  if (u < -1e-9 || u > 1.0 + 1e-9)
    return false;

  out_t = t;
  out_ix = px + t * rx;
  out_iy = py + t * ry;
  return true;
}

inline bool pointOnSegment(double px, double py, double ax, double ay, double bx, double by, double tol)
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double c = std::abs(cross2(vx, vy, wx, wy));
  if (c > tol)
    return false;
  const double dot = wx * vx + wy * vy;
  if (dot < -tol)
    return false;
  const double len2 = vx * vx + vy * vy;
  if (dot - len2 > tol)
    return false;
  return true;
}

inline std::pair<bool, bool> pnpoly(const std::vector<std::pair<double, double>>& poly, double x, double y)
{
  if (poly.size() < 3)
    return { false, false };

  bool inside = false;
  bool on = false;

  const double tol = 1e-8;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    const double xi = poly[i].first;
    const double yi = poly[i].second;
    const double xj = poly[j].first;
    const double yj = poly[j].second;

    if (pointOnSegment(x, y, xj, yj, xi, yi, tol))
    {
      on = true;
      // keep going to keep behavior stable
    }

    const bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 1e-18) + xi);
    if (intersect)
      inside = !inside;
  }

  return { inside, on };
}

class Polymap
{
public:
  Polymap(costmap_2d::Costmap2D* costmap, int nx, int ny, float obstacle_factor, bool traverse_unknown,
         int min_loop_vertices, double ray_eps, int max_rays, int max_vertices)
    : costmap_(costmap)
    , nx_(nx)
    , ny_(ny)
    , stride_(nx + 1)
    , obstacle_factor_(obstacle_factor)
    , traverse_unknown_(traverse_unknown)
    , min_loop_vertices_(std::max(3, min_loop_vertices))
    , ray_eps_(std::max(0.0, ray_eps))
    , max_rays_(std::max(64, max_rays))
    , max_vertices_(std::max(64, max_vertices))
  {
  }

  bool build(int start_x, int start_y, int goal_x, int goal_y)
  {
    solution_exist_ = false;
    obstacles_.clear();
    topo_.clear();
    outgoing_.clear();
    segments_.clear();
    all_vertices_.clear();
    V_cache_.clear();
    topoV_cache_.clear();

    if (!costmap_ || nx_ <= 0 || ny_ <= 0)
      return false;
    if (start_x < 0 || start_y < 0 || start_x >= nx_ || start_y >= ny_)
      return false;
    if (goal_x < 0 || goal_y < 0 || goal_x >= nx_ || goal_y >= ny_)
      return false;

    // (1) Reachability on free cells
    std::vector<uint8_t> reach(static_cast<size_t>(nx_ * ny_), 0);
    auto cellId = [this](int x, int y) { return x + nx_ * y; };

    if (isOccupiedCell(start_x, start_y) || isOccupiedCell(goal_x, goal_y))
      return false;

    std::stack<std::pair<int, int>> st;
    st.push({ start_x, start_y });
    reach[cellId(start_x, start_y)] = 1;

    const int dx4[4] = { 1, -1, 0, 0 };
    const int dy4[4] = { 0, 0, 1, -1 };

    while (!st.empty())
    {
      const auto cur = st.top();
      st.pop();

      for (int k = 0; k < 4; ++k)
      {
        const int nx = cur.first + dx4[k];
        const int ny = cur.second + dy4[k];
        if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_)
          continue;
        const int id = cellId(nx, ny);
        if (reach[id])
          continue;
        if (isOccupiedCell(nx, ny))
          continue;
        reach[id] = 1;
        st.push({ nx, ny });
      }
    }

    if (!reach[cellId(goal_x, goal_y)])
    {
      solution_exist_ = false;
      return false;
    }
    solution_exist_ = true;

    auto isSolidNeighbor = [&](int cx, int cy) {
      if (cx < 0 || cy < 0 || cx >= nx_ || cy >= ny_)
        return true;
      if (isOccupiedCell(cx, cy))
        return true;
      return reach[cellId(cx, cy)] == 0;
    };

    auto vid = [this](int vx, int vy) { return vx + stride_ * vy; };
    auto addDirectedEdge = [&](int ux, int uy, int vx, int vy) {
      const int u = vid(ux, uy);
      const int v = vid(vx, vy);
      outgoing_[u].push_back(v);
    };

    // (2) Boundary edges: directed so that reachable free space stays on the left.
    for (int y = 0; y < ny_; ++y)
    {
      for (int x = 0; x < nx_; ++x)
      {
        if (!reach[cellId(x, y)])
          continue;
        if (isOccupiedCell(x, y))
          continue;

        // right neighbor solid => upward edge on x+1
        if (isSolidNeighbor(x + 1, y))
          addDirectedEdge(x + 1, y, x + 1, y + 1);
        // left neighbor solid => downward edge on x
        if (isSolidNeighbor(x - 1, y))
          addDirectedEdge(x, y + 1, x, y);
        // up neighbor solid => leftward edge on y+1
        if (isSolidNeighbor(x, y + 1))
          addDirectedEdge(x + 1, y + 1, x, y + 1);
        // down neighbor solid => rightward edge on y
        if (isSolidNeighbor(x, y - 1))
          addDirectedEdge(x, y, x + 1, y);
      }
    }

    buildLoopsFromEdges();
    registerVertices();
    buildSegments();
    return true;
  }

  bool solutionExist() const
  {
    return solution_exist_;
  }

  int nxCells() const
  {
    return nx_;
  }

  int nyCells() const
  {
    return ny_;
  }

  const std::vector<std::vector<std::pair<int, int>>>& obstacles() const
  {
    return obstacles_;
  }

  TopoIndex locateVertex(int x, int y) const
  {
    auto it = topo_.find(packXY(x, y));
    if (it == topo_.end())
      return TopoIndex{};
    return it->second;
  }

  bool areConsecutive(const TopoIndex& prev, const TopoIndex& next) const
  {
    if (!topoValid(prev) || !topoValid(next))
      return false;
    if (prev.obs != next.obs)
      return false;
    const auto& loop = obstacles_[static_cast<size_t>(prev.obs)];
    const int n = static_cast<int>(loop.size());
    if (n <= 1)
      return false;
    // Match upstream raystar/polymap.h:
    // areConsecutive(prev, next) == true iff next is the immediate successor of prev on the obstacle loop.
    return next.ver == (prev.ver + 1) % n;
  }

  std::pair<int, int> getPrevObs(const TopoIndex& curr) const
  {
    const auto& loop = obstacles_.at(static_cast<size_t>(curr.obs));
    const int n = static_cast<int>(loop.size());
    const int j = (curr.ver - 1 + n) % n;
    return loop[static_cast<size_t>(j)];
  }

  std::pair<int, int> getNextObs(const TopoIndex& curr) const
  {
    const auto& loop = obstacles_.at(static_cast<size_t>(curr.obs));
    const int n = static_cast<int>(loop.size());
    const int j = (curr.ver + 1) % n;
    return loop[static_cast<size_t>(j)];
  }

  void getVisibilityRegion(int sx, int sy, std::vector<std::pair<double, double>>& visibility_region,
                           std::vector<TopoIndex>& topo_V)
  {
    visibility_region.clear();
    topo_V.clear();

    if (sx < 0 || sy < 0 || sx >= nx_ || sy >= ny_)
      return;

    const int cache_key = sx + nx_ * sy;
    auto it = V_cache_.find(cache_key);
    if (it != V_cache_.end())
    {
      visibility_region = it->second;
      topo_V = topoV_cache_[cache_key];
      return;
    }

    calculateVisibilityRegion(sx, sy, visibility_region, topo_V);
    if (!visibility_region.empty())
    {
      V_cache_[cache_key] = visibility_region;
      topoV_cache_[cache_key] = topo_V;
    }
  }

  // Conservative visibility test in the polygonal obstacle map:
  // returns true iff the segment (x0,y0)->(x1,y1) does NOT intersect any
  // obstacle boundary segment (except those incident to the start point).
  bool segmentVisible(double x0, double y0, double x1, double y1) const
  {
    if (segments_.empty())
      return true;

    auto sgn = [](double v, double eps) {
      if (v > eps)
        return 1;
      if (v < -eps)
        return -1;
      return 0;
    };

    auto orient = [](double ax, double ay, double bx, double by, double cx, double cy) {
      return cross2(bx - ax, by - ay, cx - ax, cy - ay);
    };

    auto segIntersect = [&](double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
      const double eps = 1e-10;
      const double o1 = orient(ax, ay, bx, by, cx, cy);
      const double o2 = orient(ax, ay, bx, by, dx, dy);
      const double o3 = orient(cx, cy, dx, dy, ax, ay);
      const double o4 = orient(cx, cy, dx, dy, bx, by);

      const int s1 = sgn(o1, eps);
      const int s2 = sgn(o2, eps);
      const int s3 = sgn(o3, eps);
      const int s4 = sgn(o4, eps);

      if (s1 * s2 < 0 && s3 * s4 < 0)
        return true;

      // Collinear / touching cases
      if (s1 == 0 && pointOnSegment(cx, cy, ax, ay, bx, by, 1e-9))
        return true;
      if (s2 == 0 && pointOnSegment(dx, dy, ax, ay, bx, by, 1e-9))
        return true;
      if (s3 == 0 && pointOnSegment(ax, ay, cx, cy, dx, dy, 1e-9))
        return true;
      if (s4 == 0 && pointOnSegment(bx, by, cx, cy, dx, dy, 1e-9))
        return true;

      return false;
    };

    const double tol = 1e-9;
    for (const auto& s : segments_)
    {
      // Allow starting exactly from an obstacle vertex without treating the
      // incident edges as blocking.
      if ((std::abs(x0 - s.ax) < tol && std::abs(y0 - s.ay) < tol) || (std::abs(x0 - s.bx) < tol && std::abs(y0 - s.by) < tol))
        continue;

      if (segIntersect(x0, y0, x1, y1, s.ax, s.ay, s.bx, s.by))
        return false;
    }
    return true;
  }

  // Ray visibility test: return true iff the closest obstacle intersection along
  // the ray from (x0,y0) toward (x1,y1) is NOT closer than the goal point.
  // This matches the intuition behind "goal is in visibility region" and is
  // robust to starting exactly on an obstacle vertex (t ~ 0 intersections are
  // ignored by raySegmentIntersection).
  bool rayVisibleTo(double x0, double y0, double x1, double y1) const
  {
    if (segments_.empty())
      return true;

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double L = std::hypot(dx, dy);
    if (L < 1e-9)
      return true;

    const double rx = dx / L;
    const double ry = dy / L;

    const double goal_t = L;
    const double tol = 1e-7;

    for (const auto& s : segments_)
    {
      double t, ix, iy;
      if (raySegmentIntersection(x0, y0, rx, ry, s.ax, s.ay, s.bx, s.by, t, ix, iy))
      {
        if (t + tol < goal_t)
          return false;
      }
    }
    return true;
  }

private:
  bool isOccupiedCell(int x, int y) const
  {
    if (!costmap_ || x < 0 || y < 0 || x >= nx_ || y >= ny_)
      return true;
    const int id = x + nx_ * y;
    const unsigned char c = costmap_->getCharMap()[id];
    if (c == costmap_2d::NO_INFORMATION)
      return !traverse_unknown_;
    // Clamp factor to <= 1 so a misconfigured factor>1 does not push the
    // threshold above 255 (making all obstacles traversable).
    const float eff = (std::isfinite(obstacle_factor_) && obstacle_factor_ > 0.0f) ? std::min(1.0f, obstacle_factor_) : 1.0f;
    return static_cast<float>(c) >= static_cast<float>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE) * eff;
  }

  static void removeCollinear(std::vector<std::pair<int, int>>& pts)
  {
    if (pts.size() < 3)
      return;

    std::vector<std::pair<int, int>> out;
    out.reserve(pts.size());

    auto isCollinear = [](const std::pair<int, int>& a, const std::pair<int, int>& b, const std::pair<int, int>& c) {
      const int abx = b.first - a.first;
      const int aby = b.second - a.second;
      const int bcx = c.first - b.first;
      const int bcy = c.second - b.second;
      return abx * bcy - aby * bcx == 0;
    };

    for (size_t i = 0; i < pts.size(); ++i)
    {
      const auto& prev = pts[(i + pts.size() - 1) % pts.size()];
      const auto& cur = pts[i];
      const auto& next = pts[(i + 1) % pts.size()];
      if (!isCollinear(prev, cur, next))
        out.push_back(cur);
    }

    if (out.size() >= 3)
      pts.swap(out);
  }

  static int dirIndex(int dx, int dy)
  {
    if (dx == 1 && dy == 0)
      return 0;  // E
    if (dx == 0 && dy == 1)
      return 1;  // N
    if (dx == -1 && dy == 0)
      return 2;  // W
    if (dx == 0 && dy == -1)
      return 3;  // S
    return -1;
  }

  static std::pair<int, int> dirFromIndex(int idx)
  {
    switch (idx & 3)
    {
      case 0:
        return { 1, 0 };
      case 1:
        return { 0, 1 };
      case 2:
        return { -1, 0 };
      default:
        return { 0, -1 };
    }
  }

  void buildLoopsFromEdges()
  {
    std::unordered_set<uint64_t> used;
    used.reserve(1024);

    auto id2xy = [this](int id) {
      const int x = id % stride_;
      const int y = id / stride_;
      return std::make_pair(x, y);
    };

    for (const auto& kv : outgoing_)
    {
      const int u0 = kv.first;
      const auto& outs = kv.second;
      for (int v0 : outs)
      {
        const uint64_t e0 = packEdge(u0, v0);
        if (used.find(e0) != used.end())
          continue;

        std::vector<std::pair<int, int>> loop;
        loop.reserve(256);

        int u = u0;
        int v = v0;
        const int start = u0;

        auto prev_xy = id2xy(u);
        auto cur_xy = id2xy(v);
        int dx = cur_xy.first - prev_xy.first;
        int dy = cur_xy.second - prev_xy.second;
        int dir = dirIndex(dx, dy);

        // seed
        loop.push_back(prev_xy);

        int guard = 0;
        while (guard++ < 200000)
        {
          used.insert(packEdge(u, v));
          loop.push_back(id2xy(v));

          if (v == start)
            break;

          const auto it = outgoing_.find(v);
          if (it == outgoing_.end() || it->second.empty())
            break;

          int next = -1;
          // Prefer right turn, then straight, then left, then back.
          const int pref[4] = { (dir + 3) & 3, dir & 3, (dir + 1) & 3, (dir + 2) & 3 };
          for (int pi = 0; pi < 4 && next < 0; ++pi)
          {
            const auto want = dirFromIndex(pref[pi]);
            for (int cand : it->second)
            {
              if (used.find(packEdge(v, cand)) != used.end())
                continue;
              auto vxy = id2xy(v);
              auto cxy = id2xy(cand);
              const int cdx = cxy.first - vxy.first;
              const int cdy = cxy.second - vxy.second;
              if (cdx == want.first && cdy == want.second)
              {
                next = cand;
                dir = pref[pi];
                break;
              }
            }
          }

          if (next < 0)
          {
            // fallback: pick any unused
            for (int cand : it->second)
            {
              if (used.find(packEdge(v, cand)) != used.end())
                continue;
              next = cand;
              auto vxy = id2xy(v);
              auto cxy = id2xy(cand);
              dir = dirIndex(cxy.first - vxy.first, cxy.second - vxy.second);
              break;
            }
          }

          if (next < 0)
            break;

          u = v;
          v = next;
        }

        if (!loop.empty() && loop.front() == loop.back())
          loop.pop_back();

        // NOTE:
        // The raw traced loop can contain many collinear points (grid boundary walking).
        // After simplification (removeCollinear), even a large rectangular obstacle can
        // become a 4-vertex polygon. The intention of `min_loop_vertices_` is to discard
        // *tiny* loops (noise), so we must apply it on the pre-simplified loop length.
        const int raw_loop_vertices = static_cast<int>(loop.size());

        // clean duplicates
        loop.erase(std::unique(loop.begin(), loop.end()), loop.end());
        removeCollinear(loop);

        if (raw_loop_vertices >= min_loop_vertices_ && static_cast<int>(loop.size()) >= 3)
          obstacles_.push_back(std::move(loop));
      }
    }
  }

  void registerVertices()
  {
    topo_.reserve(4096);
    all_vertices_.clear();

    for (size_t oi = 0; oi < obstacles_.size(); ++oi)
    {
      auto& loop = obstacles_[oi];
      for (size_t vi = 0; vi < loop.size(); ++vi)
      {
        const int x = loop[vi].first;
        const int y = loop[vi].second;
        const uint64_t k = packXY(x, y);
        if (topo_.find(k) == topo_.end())
        {
          topo_[k] = TopoIndex{ static_cast<int>(oi), static_cast<int>(vi) };
          all_vertices_.emplace_back(static_cast<double>(x), static_cast<double>(y));
        }
      }
    }
  }

  void buildSegments()
  {
    segments_.clear();
    segments_.reserve(8192);

    for (size_t oi = 0; oi < obstacles_.size(); ++oi)
    {
      const auto& loop = obstacles_[oi];
      if (loop.size() < 2)
        continue;
      for (size_t i = 0; i < loop.size(); ++i)
      {
        const auto& a = loop[i];
        const auto& b = loop[(i + 1) % loop.size()];
        segments_.push_back({ static_cast<double>(a.first), static_cast<double>(a.second), static_cast<double>(b.first),
                              static_cast<double>(b.second) });
      }
    }

    seg_bvh_.build(segments_);
  }

  TopoIndex snapTopo(double x, double y) const
  {
    const int rx = static_cast<int>(std::llround(x));
    const int ry = static_cast<int>(std::llround(y));
    if (std::abs(x - static_cast<double>(rx)) < 1e-6 && std::abs(y - static_cast<double>(ry)) < 1e-6)
      return locateVertex(rx, ry);
    return TopoIndex{};
  }

  void calculateVisibilityRegion(int sx, int sy, std::vector<std::pair<double, double>>& result_V,
                                std::vector<TopoIndex>& topo_V)
  {
    result_V.clear();
    topo_V.clear();

    if (segments_.empty() || all_vertices_.empty())
      return;

    // Visibility polygon via angular sweep on endpoint events.
    //
    // Classic visibility polygon algorithms evaluate the nearest obstacle
    // intersection for rays cast at each obstacle vertex angle (and a small
    // +/-epsilon offset) to disambiguate reflex vertices.
    //
    // Compared to the earlier "ray sampling" approach, this is an endpoint-event
    // scan: the set of angles is derived from obstacle endpoints, and ray queries
    // are accelerated by a BVH over obstacle boundary segments.
    const double eps = (ray_eps_ > 0.0) ? ray_eps_ : 1e-7;

    std::vector<double> angles;
    angles.reserve(std::min<size_t>(static_cast<size_t>(max_rays_), all_vertices_.size() * 2));

    const double px = static_cast<double>(sx);
    const double py = static_cast<double>(sy);

    for (const auto& v : all_vertices_)
    {
      const double base = std::atan2(v.second - py, v.first - px);
      angles.push_back(normalizeAnglePositive(base - eps));
      angles.push_back(normalizeAnglePositive(base + eps));

      if (static_cast<int>(angles.size()) >= max_rays_)
        break;
    }

    if (angles.size() < 8)
      return;

    std::sort(angles.begin(), angles.end());
    angles.erase(std::unique(angles.begin(), angles.end(), [](double a, double b) { return std::abs(a - b) < 1e-12; }), angles.end());

    std::vector<RayHit> hits;
    hits.reserve(angles.size());

    for (double ang : angles)
    {
      const double rx = std::cos(ang);
      const double ry = std::sin(ang);

      double best_t = std::numeric_limits<double>::infinity();
      double best_x = 0.0;
      double best_y = 0.0;
      int best_seg_idx = -1;

      if (!seg_bvh_.queryNearest(px, py, rx, ry, best_t, best_x, best_y, best_seg_idx))
        continue;

      RayHit h;
      h.ang = ang;
      h.x = best_x;
      h.y = best_y;
      {
        const double dx = best_x - px;
        const double dy = best_y - py;
        h.dist2 = dx * dx + dy * dy;
      }
      h.topo = snapTopo(best_x, best_y);

      // Fallback topo assignment: when the ray hits the interior of a boundary segment,
      // snapTopo() fails (non-integer intersection). Upstream Ray* associates visibility
      // vertices with obstacle topology, so we approximate by assigning the topo index
      // of the *closer* segment endpoint (eps rays usually hit very close to the true vertex).
      if (!topoValid(h.topo) && best_seg_idx >= 0 && best_seg_idx < static_cast<int>(segments_.size()))
      {
        const auto& best_seg = segments_[static_cast<size_t>(best_seg_idx)];

        const int ax_i = static_cast<int>(std::llround(best_seg.ax));
        const int ay_i = static_cast<int>(std::llround(best_seg.ay));
        const int bx_i = static_cast<int>(std::llround(best_seg.bx));
        const int by_i = static_cast<int>(std::llround(best_seg.by));

        const TopoIndex topo_a = locateVertex(ax_i, ay_i);
        const TopoIndex topo_b = locateVertex(bx_i, by_i);

        if (topoValid(topo_a) || topoValid(topo_b))
        {
          const double da2 = (best_x - best_seg.ax) * (best_x - best_seg.ax) + (best_y - best_seg.ay) * (best_y - best_seg.ay);
          const double db2 = (best_x - best_seg.bx) * (best_x - best_seg.bx) + (best_y - best_seg.by) * (best_y - best_seg.by);

          if (topoValid(topo_a) && topoValid(topo_b))
          {
            // Pick the endpoint closer to the intersection point.
            h.topo = (da2 <= db2) ? topo_a : topo_b;
          }
          else
          {
            h.topo = topoValid(topo_a) ? topo_a : topo_b;
          }
        }
      }

      // Ensure geometric position is consistent with topology.
      // Our ray casting may hit the interior of a segment; we approximate the
      // topology by snapping to a nearby obstacle vertex. In that case, we also
      // snap the reported visibility vertex position to the same obstacle vertex
      // coordinate; otherwise downstream scoped-visibility (which relies on exact
      // vertex matching) and child seed selection can become inconsistent.
      if (topoValid(h.topo))
      {
        const auto& loop = obstacles_[static_cast<size_t>(h.topo.obs)];
        if (h.topo.ver >= 0 && h.topo.ver < static_cast<int>(loop.size()))
        {
          h.x = static_cast<double>(loop[static_cast<size_t>(h.topo.ver)].first);
          h.y = static_cast<double>(loop[static_cast<size_t>(h.topo.ver)].second);
          const double dx = h.x - px;
          const double dy = h.y - py;
          h.dist2 = dx * dx + dy * dy;
        }
      }
      hits.push_back(h);

      if (static_cast<int>(hits.size()) >= max_vertices_)
        break;
    }

    if (hits.size() < 3)
      return;

    std::sort(hits.begin(), hits.end(), [](const RayHit& a, const RayHit& b) {
      if (a.ang == b.ang)
        return a.dist2 < b.dist2;
      return a.ang < b.ang;
    });

    // Reduce ray-sampling artifacts.
    //
    // In the upstream CGAL implementation, the visibility region is a polygon with a
    // relatively small number of vertices. Our CGAL-free version approximates the
    // visibility region by casting rays toward obstacle vertices with a small
    // +/- ray_eps perturbation. This can generate many near-duplicate rays that hit
    // the *same* obstacle vertex (same topo) at almost the same angle, and those
    // consecutive duplicates can be mistakenly treated as "gaps" by the downstream
    // gap-tree expansion (because areConsecutive() is false for identical vertices).
    //
    // To keep the downstream logic stable and closer to the upstream behavior, we
    // cluster hits by a small angular window and de-duplicate by topo within each
    // cluster. If multiple distinct topo vertices appear in the same angular cluster,
    // we keep the closest and the farthest hits to preserve a potential true gap.
    const double merge_ang_eps = std::max(1e-5, std::abs(ray_eps_) * 10.0);
    std::vector<RayHit> reduced;
    reduced.reserve(hits.size());

    std::vector<RayHit> uniq;
    uniq.reserve(16);

    for (size_t i = 0; i < hits.size();)
    {
      size_t j = i + 1;
      while (j < hits.size() && (hits[j].ang - hits[i].ang) <= merge_ang_eps)
        ++j;

      uniq.clear();
      uniq.reserve(j - i);
      for (size_t k = i; k < j; ++k)
      {
        const auto& h = hits[k];

        bool merged = false;
        if (topoValid(h.topo))
        {
          for (auto& u : uniq)
          {
            if (topoValid(u.topo) && u.topo.obs == h.topo.obs && u.topo.ver == h.topo.ver)
            {
              // Keep the closest hit for the same topo within this angle cluster.
              if (h.dist2 < u.dist2)
                u = h;
              merged = true;
              break;
            }
          }
        }

        if (!merged)
          uniq.push_back(h);
      }

      if (!uniq.empty())
      {
        if (uniq.size() == 1)
        {
          reduced.push_back(uniq.front());
        }
        else
        {
          std::sort(uniq.begin(), uniq.end(), [](const RayHit& a, const RayHit& b) {
            if (a.dist2 == b.dist2)
              return a.ang < b.ang;
            return a.dist2 < b.dist2;
          });

          const RayHit near_h = uniq.front();
          const RayHit far_h = uniq.back();

          reduced.push_back(near_h);

          // Only keep the far hit if it represents a distinct topology; otherwise it is
          // almost certainly an eps-ray duplicate that would create a spurious gap.
          const bool same_topo = topoValid(near_h.topo) && topoValid(far_h.topo) && near_h.topo.obs == far_h.topo.obs &&
                                 near_h.topo.ver == far_h.topo.ver;
          if (!same_topo)
            reduced.push_back(far_h);
        }
      }

      i = j;
      if (static_cast<int>(reduced.size()) >= max_vertices_)
        break;
    }

    if (reduced.size() >= 3)
      hits.swap(reduced);

    // Build vertex list; keep near-duplicate angles to allow gap detection downstream.
    result_V.reserve(hits.size());
    topo_V.reserve(hits.size());

    for (size_t i = 0; i < hits.size(); ++i)
    {
      const auto& h = hits[i];
      if (!result_V.empty())
      {
        const auto& p = result_V.back();
        if (std::abs(h.x - p.first) < 1e-9 && std::abs(h.y - p.second) < 1e-9)
        {
          // exact same point -> skip
          continue;
        }
      }

      result_V.emplace_back(h.x, h.y);
      topo_V.emplace_back(h.topo);
    }
  }

private:
  struct Segment2
  {
    double ax{ 0.0 }, ay{ 0.0 }, bx{ 0.0 }, by{ 0.0 };
  };

  struct Aabb2
  {
    double minx{ 0.0 };
    double maxx{ 0.0 };
    double miny{ 0.0 };
    double maxy{ 0.0 };
  };

  static Aabb2 segmentAabb(const Segment2& s)
  {
    Aabb2 b;
    b.minx = std::min(s.ax, s.bx);
    b.maxx = std::max(s.ax, s.bx);
    b.miny = std::min(s.ay, s.by);
    b.maxy = std::max(s.ay, s.by);
    return b;
  }

  static Aabb2 mergeAabb(const Aabb2& a, const Aabb2& b)
  {
    Aabb2 m;
    m.minx = std::min(a.minx, b.minx);
    m.maxx = std::max(a.maxx, b.maxx);
    m.miny = std::min(a.miny, b.miny);
    m.maxy = std::max(a.maxy, b.maxy);
    return m;
  }

  // 2D ray-AABB intersection (slab method). Returns true iff intersects for t>=0.
  static bool rayIntersectsAabb(double px, double py, double rx, double ry, const Aabb2& b, double& out_tmin)
  {
    const double INF = std::numeric_limits<double>::infinity();

    double tmin = 0.0;
    double tmax = INF;

    const double eps = 1e-15;

    // X slabs
    if (std::abs(rx) < eps)
    {
      if (px < b.minx || px > b.maxx)
        return false;
    }
    else
    {
      const double inv = 1.0 / rx;
      double t0 = (b.minx - px) * inv;
      double t1 = (b.maxx - px) * inv;
      if (t0 > t1)
        std::swap(t0, t1);
      tmin = std::max(tmin, t0);
      tmax = std::min(tmax, t1);
      if (tmax < tmin)
        return false;
    }

    // Y slabs
    if (std::abs(ry) < eps)
    {
      if (py < b.miny || py > b.maxy)
        return false;
    }
    else
    {
      const double inv = 1.0 / ry;
      double t0 = (b.miny - py) * inv;
      double t1 = (b.maxy - py) * inv;
      if (t0 > t1)
        std::swap(t0, t1);
      tmin = std::max(tmin, t0);
      tmax = std::min(tmax, t1);
      if (tmax < tmin)
        return false;
    }

    if (tmax < 0.0)
      return false;

    out_tmin = tmin;
    return true;
  }

  class SegmentBvh
  {
  public:
    void build(const std::vector<Segment2>& segs)
    {
      segs_ = &segs;
      nodes_.clear();
      root_ = -1;
      if (!segs_ || segs_->empty())
        return;

      std::vector<int> ids(segs_->size());
      for (size_t i = 0; i < ids.size(); ++i)
        ids[i] = static_cast<int>(i);

      root_ = buildRecursive(ids, 0, static_cast<int>(ids.size()));
    }

    bool queryNearest(double px, double py, double rx, double ry, double& out_t, double& out_x, double& out_y,
                      int& out_seg_idx) const
    {
      out_t = std::numeric_limits<double>::infinity();
      out_x = 0.0;
      out_y = 0.0;
      out_seg_idx = -1;

      if (root_ < 0 || !segs_ || segs_->empty())
        return false;

      queryNode(root_, px, py, rx, ry, out_t, out_x, out_y, out_seg_idx);
      return std::isfinite(out_t);
    }

  private:
    struct Node
    {
      Aabb2 box;
      int left{ -1 };
      int right{ -1 };
      int seg{ -1 };
    };

    int buildRecursive(std::vector<int>& ids, int begin, int end)
    {
      Node node;
      node.box = segmentAabb((*segs_)[static_cast<size_t>(ids[begin])]);
      for (int i = begin + 1; i < end; ++i)
      {
        node.box = mergeAabb(node.box, segmentAabb((*segs_)[static_cast<size_t>(ids[i])]));
      }

      const int count = end - begin;
      const int idx = static_cast<int>(nodes_.size());
      nodes_.push_back(node);

      if (count <= 1)
      {
        nodes_[static_cast<size_t>(idx)].seg = ids[begin];
        return idx;
      }

      const double ex = node.box.maxx - node.box.minx;
      const double ey = node.box.maxy - node.box.miny;
      const int axis = (ex >= ey) ? 0 : 1;

      const int mid = begin + count / 2;
      std::nth_element(ids.begin() + begin, ids.begin() + mid, ids.begin() + end,
                       [this, axis](int a, int b) {
                         const auto ba = segmentAabb((*segs_)[static_cast<size_t>(a)]);
                         const auto bb = segmentAabb((*segs_)[static_cast<size_t>(b)]);
                         const double ca = (axis == 0) ? 0.5 * (ba.minx + ba.maxx) : 0.5 * (ba.miny + ba.maxy);
                         const double cb = (axis == 0) ? 0.5 * (bb.minx + bb.maxx) : 0.5 * (bb.miny + bb.maxy);
                         return ca < cb;
                       });

      const int left = buildRecursive(ids, begin, mid);
      const int right = buildRecursive(ids, mid, end);
      nodes_[static_cast<size_t>(idx)].left = left;
      nodes_[static_cast<size_t>(idx)].right = right;
      return idx;
    }

    void queryNode(int node_idx, double px, double py, double rx, double ry, double& best_t, double& best_x,
                   double& best_y, int& best_seg_idx) const
    {
      if (node_idx < 0)
        return;

      const Node& n = nodes_[static_cast<size_t>(node_idx)];
      double box_tmin = 0.0;
      if (!rayIntersectsAabb(px, py, rx, ry, n.box, box_tmin))
        return;
      if (box_tmin > best_t)
        return;

      if (n.seg >= 0)
      {
        const auto& s = (*segs_)[static_cast<size_t>(n.seg)];
        double t, ix, iy;
        if (raySegmentIntersection(px, py, rx, ry, s.ax, s.ay, s.bx, s.by, t, ix, iy))
        {
          if (t < best_t)
          {
            best_t = t;
            best_x = ix;
            best_y = iy;
            best_seg_idx = n.seg;
          }
        }
        return;
      }

      // Visit child with smaller box_tmin first.
      int c1 = n.left;
      int c2 = n.right;
      double t1 = 0.0;
      double t2 = 0.0;
      const bool h1 = (c1 >= 0) ? rayIntersectsAabb(px, py, rx, ry, nodes_[static_cast<size_t>(c1)].box, t1) : false;
      const bool h2 = (c2 >= 0) ? rayIntersectsAabb(px, py, rx, ry, nodes_[static_cast<size_t>(c2)].box, t2) : false;

      if (h1 && h2)
      {
        if (t2 < t1)
        {
          std::swap(c1, c2);
          std::swap(t1, t2);
        }
        if (t1 <= best_t)
          queryNode(c1, px, py, rx, ry, best_t, best_x, best_y, best_seg_idx);
        if (t2 <= best_t)
          queryNode(c2, px, py, rx, ry, best_t, best_x, best_y, best_seg_idx);
      }
      else if (h1)
      {
        if (t1 <= best_t)
          queryNode(c1, px, py, rx, ry, best_t, best_x, best_y, best_seg_idx);
      }
      else if (h2)
      {
        if (t2 <= best_t)
          queryNode(c2, px, py, rx, ry, best_t, best_x, best_y, best_seg_idx);
      }
    }

  private:
    const std::vector<Segment2>* segs_{ nullptr };
    std::vector<Node> nodes_;
    int root_{ -1 };
  };

  costmap_2d::Costmap2D* costmap_{ nullptr };
  int nx_{ 0 };
  int ny_{ 0 };
  int stride_{ 0 };
  float obstacle_factor_{ 0.5f };
  bool traverse_unknown_{ true };
  int min_loop_vertices_{ 6 };
  double ray_eps_{ 1e-6 };
  int max_rays_{ 8000 };
  int max_vertices_{ 4096 };

  bool solution_exist_{ false };

  // obstacles: each loop is an ordered list of integer vertices (grid coordinates)
  std::vector<std::vector<std::pair<int, int>>> obstacles_;

  // directed boundary edges: u -> v
  std::unordered_map<int, std::vector<int>> outgoing_;

  // topo lookup for integer vertices
  std::unordered_map<uint64_t, TopoIndex> topo_;
  std::vector<std::pair<double, double>> all_vertices_;

  // visibility segments
  std::vector<Segment2> segments_;

  SegmentBvh seg_bvh_;

  // visibility cache
  std::unordered_map<int, std::vector<std::pair<double, double>>> V_cache_;
  std::unordered_map<int, std::vector<TopoIndex>> topoV_cache_;
};

}  // namespace upstream_like

}  // namespace

RaystarPathPlanner::RaystarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, const Params& params, double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor)
  , params_(params)
{
  resolution_ = costmap_ros_ && costmap_ros_->getCostmap() ? costmap_ros_->getCostmap()->getResolution() : 0.0;
}

void RaystarPathPlanner::clearDebugCache()
{
  last_candidate_paths_world_.clear();
  last_candidate_costs_.clear();
  last_best_candidate_idx_ = -1;
  last_poly_obstacles_world_.clear();
  last_best_expand_map_.clear();
}

bool RaystarPathPlanner::isOccupiedCell(int x, int y) const
{
  if (!inBounds(x, y) || !costmap_)
    return true;

  const int id = grid2Index(x, y);
  const unsigned char c = costmap_->getCharMap()[id];

  if (c == costmap_2d::NO_INFORMATION)
  {
    // If traverse_unknown is allowed, unknown is NOT occupied.
    return !params_.traverse_unknown;
  }

  // NOTE: In this codebase, `factor_` is commonly configured in (0, 1] to be
  // more conservative (smaller -> more obstacles). If factor_ > 1, multiplying
  // would push the threshold above 255 and effectively make all obstacles
  // traversable. Clamp to <= 1 to avoid invalid plans.
  const float eff = (std::isfinite(factor_) && factor_ > 0.0f) ? std::min(1.0f, factor_) : 1.0f;
  return static_cast<float>(c) >= static_cast<float>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE) * eff;
}

bool RaystarPathPlanner::isTraversableCell(int x, int y) const
{
  return !isOccupiedCell(x, y);
}

bool RaystarPathPlanner::lineOfSight(int x0, int y0, int x1, int y1) const
{
  using rpp::common::geometry::CollisionChecker;
  using rpp::common::geometry::Point2i;

  // NOTE:
  // BresenhamCollisionDetection() checks the end point (pt1) as well.
  // In upstream-like Ray* expansion, a node seed can lie on an obstacle boundary
  // vertex (or be rounded onto an occupied cell). Treating the seed cell as an
  // obstacle would make line-of-sight checks fail systematically. We therefore
  // skip occupancy checking at the LOS start cell (x0,y0).
  auto is_obs = [this, x0, y0](const Point2i& p) {
    if (p.x() == x0 && p.y() == y0)
      return false;
    if (!inBounds(p.x(), p.y()))
      return true;
    return isOccupiedCell(p.x(), p.y());
  };

  return !CollisionChecker::BresenhamCollisionDetection(Point2i(x0, y0), Point2i(x1, y1), is_obs);
}

double RaystarPathPlanner::traversalCost(int x, int y, const std::vector<float>& penalty_grid) const
{
  if (!inBounds(x, y) || !costmap_)
    return std::numeric_limits<double>::infinity();

  const int id = grid2Index(x, y);

  double cost = 0.0;

  if (params_.use_costmap_weight)
  {
    const unsigned char c = costmap_->getCharMap()[id];
    const double norm = (c == costmap_2d::NO_INFORMATION) ? 0.0 : (static_cast<double>(c) / 255.0);
    cost += params_.cost_penalty_weight * norm;
  }

  if (!penalty_grid.empty())
  {
    cost += params_.penalty_weight * static_cast<double>(penalty_grid[id]);
  }

  return cost;
}

bool RaystarPathPlanner::thetaStarPlan(const Point3d& start, const Point3d& goal, const std::vector<float>& penalty_grid,
                                      Points3d& out_path_map, Points3d& out_expand_map, double& out_path_cost) const
{
  out_path_map.clear();
  out_expand_map.clear();
  out_path_cost = 0.0;

  if (!costmap_)
    return false;

  const int sx = static_cast<int>(std::round(start.x()));
  const int sy = static_cast<int>(std::round(start.y()));
  const int gx = static_cast<int>(std::round(goal.x()));
  const int gy = static_cast<int>(std::round(goal.y()));

  if (!inBounds(sx, sy) || !inBounds(gx, gy))
    return false;
  if (!isTraversableCell(sx, sy) || !isTraversableCell(gx, gy))
    return false;

  const int start_id = grid2Index(sx, sy);
  const int goal_id = grid2Index(gx, gy);

  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(map_size_, INF);
  std::vector<int> parent(map_size_, -1);
  std::vector<uint8_t> closed(map_size_, 0);

  std::priority_queue<QueueNode> open;

  g_score[start_id] = 0.0;
  parent[start_id] = start_id;
  open.push(QueueNode{ start_id, sx, sy, hypot2(sx, sy, gx, gy), 0.0 });

  using Motion = rpp::common::structure::Node<int>;
  static const std::array<Motion, 8> motions8{ {
      Motion(0, 1, 1.0),
      Motion(1, 0, 1.0),
      Motion(0, -1, 1.0),
      Motion(-1, 0, 1.0),
      Motion(1, 1, std::sqrt(2.0)),
      Motion(1, -1, std::sqrt(2.0)),
      Motion(-1, 1, std::sqrt(2.0)),
      Motion(-1, -1, std::sqrt(2.0)),
  } };

  static const std::array<Motion, 4> motions4{ {
      Motion(0, 1, 1.0),
      Motion(1, 0, 1.0),
      Motion(0, -1, 1.0),
      Motion(-1, 0, 1.0),
  } };

  const Motion* motions = params_.allow_diagonal ? motions8.data() : motions4.data();
  const size_t motions_n = params_.allow_diagonal ? motions8.size() : motions4.size();

  const auto start_time = std::chrono::steady_clock::now();

  int expansions = 0;

  while (!open.empty())
  {
    const QueueNode cur = open.top();
    open.pop();

    if (cur.id < 0 || cur.id >= map_size_)
      continue;

    if (closed[cur.id])
      continue;

    // Skip stale queue entries.
    if (cur.g > g_score[cur.id])
      continue;

    closed[cur.id] = 1;
    out_expand_map.emplace_back(cur.x, cur.y);

    ++expansions;
    if (params_.max_expansions > 0 && expansions >= params_.max_expansions)
      break;

    if (params_.max_planning_time_ms > 0)
    {
      const auto now = std::chrono::steady_clock::now();
      const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
      if (dt_ms >= params_.max_planning_time_ms)
        break;
    }

    if (cur.id == goal_id)
    {
      out_path_cost = g_score[goal_id];

      // Reconstruct (goal -> start)
      int pid = goal_id;
      std::vector<int> rev_ids;
      rev_ids.reserve(512);
      rev_ids.push_back(goal_id);

      while (pid != start_id)
      {
        pid = parent[pid];
        if (pid < 0)
          return false;
        rev_ids.push_back(pid);
        if (static_cast<int>(rev_ids.size()) > map_size_)
          return false;
      }

      // Output in forward order
      out_path_map.reserve(rev_ids.size());
      for (auto it = rev_ids.rbegin(); it != rev_ids.rend(); ++it)
      {
        int x, y;
        index2Grid(*it, x, y);
        out_path_map.emplace_back(x, y);
      }

      return !out_path_map.empty();
    }

    // Determine current parent coordinates (for Theta*)
    int cur_parent_id = parent[cur.id];
    int px = cur.x;
    int py = cur.y;
    if (cur_parent_id >= 0)
    {
      index2Grid(cur_parent_id, px, py);
    }

    for (size_t mi = 0; mi < motions_n; ++mi)
    {
      const auto& m = motions[mi];
      const int nx = cur.x + m.x();
      const int ny = cur.y + m.y();

      if (!inBounds(nx, ny))
        continue;
      if (!isTraversableCell(nx, ny))
        continue;

      const int nid = grid2Index(nx, ny);
      if (closed[nid])
        continue;

      int new_parent = cur.id;
      double new_g = INF;

      if (params_.use_theta_star && cur.id != start_id && cur_parent_id >= 0 && cur_parent_id != cur.id &&
          lineOfSight(px, py, nx, ny))
      {
        // try shortcut through current's parent
        const double edge = hypot2(px, py, nx, ny);
        new_parent = cur_parent_id;
        new_g = g_score[cur_parent_id] + edge + traversalCost(nx, ny, penalty_grid);
      }
      else
      {
        const double edge = m.g();
        new_parent = cur.id;
        new_g = g_score[cur.id] + edge + traversalCost(nx, ny, penalty_grid);
      }

      if (new_g < g_score[nid])
      {
        g_score[nid] = new_g;
        parent[nid] = new_parent;

        const double h = hypot2(nx, ny, gx, gy);
        open.push(QueueNode{ nid, nx, ny, new_g + h, new_g });
      }
    }
  }

  return false;
}

void RaystarPathPlanner::updatePenaltyGrid(const Points3d& path_map, std::vector<float>& penalty_grid) const
{
  if (!costmap_ || map_size_ <= 0)
    return;

  if (penalty_grid.size() != static_cast<size_t>(map_size_))
  {
    penalty_grid.assign(map_size_, 0.0f);
  }

  const int r = std::max(0, params_.penalty_radius_cells);
  const float inc = static_cast<float>(params_.penalty_increment);

  for (const auto& p : path_map)
  {
    const int cx = static_cast<int>(std::round(p.x()));
    const int cy = static_cast<int>(std::round(p.y()));

    for (int dy = -r; dy <= r; ++dy)
    {
      for (int dx = -r; dx <= r; ++dx)
      {
        const int x = cx + dx;
        const int y = cy + dy;
        if (!inBounds(x, y))
          continue;

        const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
        const double w = (r <= 0) ? 1.0 : std::max(0.0, 1.0 - dist / static_cast<double>(r));

        penalty_grid[grid2Index(x, y)] += inc * static_cast<float>(w);
      }
    }
  }
}

void RaystarPathPlanner::computeObstacleComponentsInBox(int min_x, int min_y, int max_x, int max_y,
                                                       std::vector<ObstacleComponent>& out_components) const
{
  out_components.clear();
  if (!costmap_)
    return;

  min_x = std::max(0, min_x);
  min_y = std::max(0, min_y);
  max_x = std::min(nx_ - 1, max_x);
  max_y = std::min(ny_ - 1, max_y);

  if (min_x > max_x || min_y > max_y)
    return;

  const int w = max_x - min_x + 1;
  const int h = max_y - min_y + 1;

  std::vector<uint8_t> visited(static_cast<size_t>(w * h), 0);

  auto localIndex = [w](int lx, int ly) { return ly * w + lx; };

  const std::array<std::pair<int, int>, 8> n8{ { { 1, 0 },
                                                { -1, 0 },
                                                { 0, 1 },
                                                { 0, -1 },
                                                { 1, 1 },
                                                { 1, -1 },
                                                { -1, 1 },
                                                { -1, -1 } } };

  std::queue<std::pair<int, int>> q;

  for (int y = min_y; y <= max_y; ++y)
  {
    for (int x = min_x; x <= max_x; ++x)
    {
      const int lx = x - min_x;
      const int ly = y - min_y;
      const int li = localIndex(lx, ly);

      if (visited[li])
        continue;

      if (!isOccupiedCell(x, y))
      {
        visited[li] = 1;
        continue;
      }

      // BFS this component
      ObstacleComponent comp;
      comp.min_x = comp.max_x = x;
      comp.min_y = comp.max_y = y;
      double sx = 0.0, sy = 0.0;
      int cells = 0;

      visited[li] = 1;
      q.push({ x, y });

      while (!q.empty())
      {
        const auto cell = q.front();
        q.pop();

        const int cx = cell.first;
        const int cy = cell.second;

        ++cells;
        sx += static_cast<double>(cx);
        sy += static_cast<double>(cy);

        comp.min_x = std::min(comp.min_x, cx);
        comp.max_x = std::max(comp.max_x, cx);
        comp.min_y = std::min(comp.min_y, cy);
        comp.max_y = std::max(comp.max_y, cy);

        for (const auto& d : n8)
        {
          const int dx = d.first;
          const int dy = d.second;
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < min_x || nx > max_x || ny < min_y || ny > max_y)
            continue;

          const int nlx = nx - min_x;
          const int nly = ny - min_y;
          const int nli = localIndex(nlx, nly);
          if (visited[nli])
            continue;

          visited[nli] = 1;
          if (isOccupiedCell(nx, ny))
          {
            q.push({ nx, ny });
          }
        }
      }

      comp.cells = cells;
      if (cells > 0)
      {
        comp.cx_map = sx / static_cast<double>(cells);
        comp.cy_map = sy / static_cast<double>(cells);
      }

      out_components.push_back(comp);
    }
  }
}

void RaystarPathPlanner::selectSignatureReferencePoints(const std::vector<ObstacleComponent>& comps,
                                                      std::vector<Point2d>& out_ref_world,
                                                      std::vector<Points3d>& out_boxes_world) const
{
  out_ref_world.clear();
  out_boxes_world.clear();

  std::vector<ObstacleComponent> filtered;
  filtered.reserve(comps.size());

  for (const auto& c : comps)
  {
    if (c.cells >= params_.component_min_cells)
      filtered.push_back(c);
  }

  std::sort(filtered.begin(), filtered.end(), [](const ObstacleComponent& a, const ObstacleComponent& b) {
    return a.cells > b.cells;
  });

  if (params_.max_components > 0 && static_cast<int>(filtered.size()) > params_.max_components)
  {
    filtered.resize(static_cast<size_t>(params_.max_components));
  }

  out_ref_world.reserve(filtered.size());
  out_boxes_world.reserve(filtered.size());

  for (const auto& c : filtered)
  {
    double wx, wy;
    map2World(c.cx_map, c.cy_map, wx, wy);
    out_ref_world.emplace_back(wx, wy);

    // Bounding box poly (closed)
    if (!params_.publish_poly_obstacles)
      continue;

    double x0, y0, x1, y1;
    map2World(c.min_x, c.min_y, x0, y0);
    map2World(c.max_x, c.max_y, x1, y1);

    Points3d box;
    box.emplace_back(x0, y0);
    box.emplace_back(x1, y0);
    box.emplace_back(x1, y1);
    box.emplace_back(x0, y1);
    box.emplace_back(x0, y0);
    out_boxes_world.push_back(std::move(box));
  }
}

std::string RaystarPathPlanner::computeHSignature(const Points3d& path_world, const std::vector<Point2d>& ref_world) const
{
  if (path_world.size() < 2 || ref_world.empty())
    return {};

  std::string sig;
  sig.reserve(ref_world.size() * 4);

  for (size_t k = 0; k < ref_world.size(); ++k)
  {
    const double cx = ref_world[k].x();
    const double cy = ref_world[k].y();

    int s = 0;

    for (size_t i = 1; i < path_world.size(); ++i)
    {
      const double x1 = path_world[i - 1].x();
      const double y1 = path_world[i - 1].y();
      const double x2 = path_world[i].x();
      const double y2 = path_world[i].y();

      // segment crosses horizontal line at cy?
      const bool cond = ((y1 <= cy && y2 > cy) || (y2 <= cy && y1 > cy));
      if (!cond)
        continue;

      const double dy = (y2 - y1);
      if (std::abs(dy) < 1e-9)
        continue;

      const double t = (cy - y1) / dy;
      const double x_int = x1 + t * (x2 - x1);

      if (x_int > cx)
      {
        s += (y2 > y1) ? 1 : -1;
      }
    }

    sig += std::to_string(s);
    if (k + 1 < ref_world.size())
      sig.push_back(',');
  }

  return sig;
}

std::unordered_set<int> RaystarPathPlanner::pathToCellSet(const Points3d& path_map, int nx)
{
  std::unordered_set<int> s;
  s.reserve(path_map.size() * 2);
  for (const auto& p : path_map)
  {
    const int x = static_cast<int>(std::round(p.x()));
    const int y = static_cast<int>(std::round(p.y()));
    s.insert(y * nx + x);
  }
  return s;
}

double RaystarPathPlanner::jaccardSimilarity(const std::unordered_set<int>& a, const std::unordered_set<int>& b)
{
  if (a.empty() && b.empty())
    return 1.0;
  if (a.empty() || b.empty())
    return 0.0;

  const std::unordered_set<int>* small = &a;
  const std::unordered_set<int>* large = &b;
  if (a.size() > b.size())
    std::swap(small, large);

  size_t inter = 0;
  for (const auto& v : *small)
  {
    if (large->find(v) != large->end())
      ++inter;
  }

  const size_t uni = a.size() + b.size() - inter;
  return uni == 0 ? 1.0 : (static_cast<double>(inter) / static_cast<double>(uni));
}

void RaystarPathPlanner::appendDebugData(DebugData& data) const
{
  if (!params_.store_debug_data)
    return;

  data.raystar_candidate_paths_world = last_candidate_paths_world_;
  data.raystar_best_candidate_index = last_best_candidate_idx_;
  data.raystar_poly_obstacles_world = last_poly_obstacles_world_;
}

bool RaystarPathPlanner::upstreamLikePlan(const Point3d& start, const Point3d& goal, Points3d& out_path_map,
                                         Points3d& out_expand_map)
{
  using upstream_like::Polymap;
  using upstream_like::TopoIndex;
  using upstream_like::kTwoPi;

  out_path_map.clear();
  out_expand_map.clear();

  if (!costmap_)
    return false;

  const int sx = static_cast<int>(std::llround(start.x()));
  const int sy = static_cast<int>(std::llround(start.y()));
  const int gx = static_cast<int>(std::llround(goal.x()));
  const int gy = static_cast<int>(std::llround(goal.y()));

  if (!inBounds(sx, sy) || !inBounds(gx, gy))
  {
    if (params_.store_debug_data)
      R_WARN << "[raystar upstream_like] out of bounds: start_map=(" << start.x() << "," << start.y() << ") -> (" << sx
             << "," << sy << "), goal_map=(" << goal.x() << "," << goal.y() << ") -> (" << gx << "," << gy
             << ") map_size=(" << nx_ << "," << ny_ << ")";
    return false;
  }
  if (!isTraversableCell(sx, sy) || !isTraversableCell(gx, gy))
  {
    if (params_.store_debug_data && costmap_)
    {
      const unsigned char cs = costmap_->getCharMap()[grid2Index(sx, sy)];
      const unsigned char cg = costmap_->getCharMap()[grid2Index(gx, gy)];
      R_WARN << "[raystar upstream_like] start/goal not traversable: start_cell=(" << sx << "," << sy << ") cost="
             << static_cast<int>(cs) << ", goal_cell=(" << gx << "," << gy << ") cost=" << static_cast<int>(cg)
             << ", allow_unknown=" << (params_.traverse_unknown ? "true" : "false") << ", factor=" << factor_;
    }
    return false;
  }

  Polymap the_map(costmap_, nx_, ny_, factor_, params_.traverse_unknown, params_.polymap_min_loop_vertices,
                 params_.visibility_ray_eps, params_.visibility_max_rays, params_.visibility_max_vertices);

  if (!the_map.build(sx, sy, gx, gy) || !the_map.solutionExist())
  {
    if (params_.store_debug_data)
      R_WARN << "[raystar upstream_like] polymap build failed or start->goal unreachable (4-neighbor floodfill).";
    return false;
  }

  // Debug polygonal obstacles
  if (params_.store_debug_data && params_.publish_poly_obstacles)
  {
    last_poly_obstacles_world_.clear();
    last_poly_obstacles_world_.reserve(the_map.obstacles().size());
    for (const auto& loop : the_map.obstacles())
    {
      if (loop.size() < 2)
        continue;
      Points3d poly;
      poly.reserve(loop.size() + 1);
      for (const auto& v : loop)
      {
        double wx, wy;
        map2World(v.first, v.second, wx, wy);
        poly.emplace_back(wx, wy);
      }
      // close
      poly.push_back(poly.front());
      last_poly_obstacles_world_.push_back(std::move(poly));
    }
  }

  struct Candidate
  {
    int N{ -1 };
    int C{ -1 };
    double F{ 0.0 };
  };

  struct Child
  {
    int Nindex{ -1 };
    int Cindex{ -1 };

    double start_angle{ 0.0 };
    double end_angle{ 0.0 };

    std::pair<int, int> c{ 0, 0 };               // integer seed of child node
    std::pair<double, double> o{ 0.0, 0.0 };     // opposite obstacle vertex on the gap

    TopoIndex c_topo;
    TopoIndex o_topo;

    bool is_left_gap{ false };
    double g{ 0.0 };
    double h{ 0.0 };
  };

  struct Node
  {
    int idx{ 0 };
    std::pair<int, int> seed{ 0, 0 };
    double start_angle{ 0.0 };
    double end_angle{ kTwoPi };
    int parent{ -1 };

    double G{ 0.0 };
    double H{ 0.0 };

    std::vector<Child> children;
    std::vector<std::pair<double, double>> V;
    std::vector<TopoIndex> topoV;

    std::vector<std::pair<int, int>> local_path;
    std::vector<int> path_node_idx;

    void generateChild(Polymap* pMap)
    {
      children.clear();
      if (V.size() < 2 || topoV.size() != V.size() || !pMap)
        return;

      std::vector<double> theta_list(V.size(), 0.0);
      for (size_t i = 0; i < V.size(); ++i)
      {
        const double raw = std::atan2(V[i].second - static_cast<double>(seed.second),
                                      V[i].first - static_cast<double>(seed.first));
        theta_list[i] = upstream_like::wrapToStart(raw, start_angle);
      }

      std::vector<int> valid_gap_indices;
      valid_gap_indices.reserve(V.size());
      std::vector<bool> is_left(V.size(), false);

      constexpr double threshold2 = 0.0001 * 0.0001;

      for (size_t i = 0; i + 1 < V.size(); ++i)
      {
        const size_t next = i + 1;
        const double diff = upstream_like::normalizeAngle(theta_list[next] - theta_list[i]);
        if (diff * diff >= threshold2)
          continue;

        const auto topo_i = topoV[i];
        const auto topo_next = topoV[next];
        if (!upstream_like::topoValid(topo_i) || !upstream_like::topoValid(topo_next))
          continue;

        if (pMap->areConsecutive(topo_next, topo_i))
          continue;

        const double dis_i = (V[i].first - seed.first) * (V[i].first - seed.first) +
                             (V[i].second - seed.second) * (V[i].second - seed.second);
        const double dis_next = (V[next].first - seed.first) * (V[next].first - seed.first) +
                                (V[next].second - seed.second) * (V[next].second - seed.second);
        is_left[i] = dis_i > dis_next;
        valid_gap_indices.push_back(static_cast<int>(i));
      }

      for (int gi : valid_gap_indices)
      {
        const size_t i = static_cast<size_t>(gi);
        const size_t next = (i + 1) % V.size();
        const auto topo_i = topoV[i];
        const auto topo_next = topoV[next];
        if (!upstream_like::topoValid(topo_i) || !upstream_like::topoValid(topo_next))
          continue;

        Child ch;
        ch.Nindex = idx;
        ch.Cindex = -1;
        ch.is_left_gap = is_left[i];

        if (ch.is_left_gap)
        {
          // Match upstream raystar.cpp: left-gap child point is V[next]
          ch.c = { static_cast<int>(std::llround(V[next].first)), static_cast<int>(std::llround(V[next].second)) };
          ch.start_angle = upstream_like::normalizeAngle(theta_list[next]);
          const auto next_obs = pMap->getNextObs(topo_next);
          const double contour_from_next = std::atan2(static_cast<double>(next_obs.second) - V[next].second,
                                                     static_cast<double>(next_obs.first) - V[next].first);
          ch.end_angle = ch.start_angle + upstream_like::normalizeAnglePositive(contour_from_next - ch.start_angle);
          ch.c_topo = topo_next;
          ch.o_topo = topo_i;
          ch.o = { V[i].first, V[i].second };
        }
        else
        {
          // Match upstream raystar.cpp: non-left-gap child point is V[i]
          ch.c = { static_cast<int>(std::llround(V[i].first)), static_cast<int>(std::llround(V[i].second)) };
          const auto prev_obs = pMap->getPrevObs(topo_i);
          const double contour_from_prev = std::atan2(static_cast<double>(prev_obs.second) - V[i].second,
                                                     static_cast<double>(prev_obs.first) - V[i].first);
          ch.start_angle = contour_from_prev;
          ch.end_angle = contour_from_prev + upstream_like::normalizeAnglePositive(theta_list[i] - contour_from_prev);
          ch.c_topo = topo_i;
          ch.o_topo = topo_next;
          ch.o = { V[next].first, V[next].second };
        }

        // Discard out-of-bounds child seeds (can happen on map border loops)
        if (ch.c.first < 0 || ch.c.second < 0 || ch.c.first >= pMap->nxCells() || ch.c.second >= pMap->nyCells())
          continue;

        ch.g = G + std::hypot(static_cast<double>(seed.first - ch.c.first), static_cast<double>(seed.second - ch.c.second));
        children.push_back(ch);
      }

      for (size_t i = 0; i < children.size(); ++i)
      {
        children[i].Cindex = static_cast<int>(i);
      }
    }
  };

  struct PathSolution
  {
    std::vector<std::pair<int, int>> path;
    double cost{ 0.0 };
    std::vector<int> node_ids;
  };

  auto popBest = [](std::vector<Candidate>& Q) {
    auto comp = [](const Candidate& a, const Candidate& b) { return a.F > b.F; };
    std::pop_heap(Q.begin(), Q.end(), comp);
    Candidate best = Q.back();
    Q.pop_back();
    return best;
  };

  auto pushCand = [](std::vector<Candidate>& Q, const Candidate& c) {
    auto comp = [](const Candidate& a, const Candidate& b) { return a.F > b.F; };
    Q.push_back(c);
    std::push_heap(Q.begin(), Q.end(), comp);
  };

  auto scopedVisibility = [&](const std::vector<Node>& N, const Candidate& cand, std::vector<std::pair<double, double>>& Vout,
                              std::vector<TopoIndex>& topoOut) {
    Vout.clear();
    topoOut.clear();

    const int parent_idx = cand.N;
    const int child_idx = cand.C;
    const auto& ch = N[parent_idx].children[static_cast<size_t>(child_idx)];
    const auto new_seed = ch.c;

    std::vector<std::pair<double, double>> fullV;
    std::vector<TopoIndex> fullTopo;
    the_map.getVisibilityRegion(new_seed.first, new_seed.second, fullV, fullTopo);

    if (fullV.empty() || fullTopo.size() != fullV.size())
      return;

    std::pair<double, double> start_obs;
    std::pair<double, double> end_obs;
    TopoIndex start_obs_topo;
    TopoIndex end_obs_topo;

    double start_angle;
    double end_angle;

    if (ch.is_left_gap)
    {
      start_obs = ch.o;
      const auto end_obs_i = the_map.getNextObs(ch.c_topo);
      end_obs = { static_cast<double>(end_obs_i.first), static_cast<double>(end_obs_i.second) };
      start_obs_topo = ch.o_topo;
      end_obs_topo = the_map.locateVertex(end_obs_i.first, end_obs_i.second);
      start_angle = std::atan2(start_obs.second - new_seed.second, start_obs.first - new_seed.first);
      end_angle = std::atan2(end_obs.second - new_seed.second, end_obs.first - new_seed.first);
    }
    else
    {
      const auto start_obs_i = the_map.getPrevObs(ch.c_topo);
      start_obs = { static_cast<double>(start_obs_i.first), static_cast<double>(start_obs_i.second) };
      end_obs = ch.o;
      start_obs_topo = the_map.locateVertex(start_obs_i.first, start_obs_i.second);
      end_obs_topo = ch.o_topo;
      start_angle = std::atan2(start_obs.second - new_seed.second, start_obs.first - new_seed.first);
      end_angle = std::atan2(end_obs.second - new_seed.second, end_obs.first - new_seed.first);
    }

    end_angle = start_angle + upstream_like::normalizeAnglePositive(end_angle - start_angle);

    // Filter by angle
    for (size_t i = 0; i < fullV.size(); ++i)
    {
      const double raw = std::atan2(fullV[i].second - new_seed.second, fullV[i].first - new_seed.first);
      const double theta = upstream_like::wrapToStart(raw, start_angle);
      if (theta >= start_angle - 1e-7 && theta <= end_angle + 1e-7)
      {
        Vout.push_back(fullV[i]);
        topoOut.push_back(fullTopo[i]);
      }
    }

    auto findIdx = [](const std::vector<std::pair<double, double>>& vv, const std::pair<double, double>& p) {
      for (size_t i = 0; i < vv.size(); ++i)
      {
        if (std::abs(vv[i].first - p.first) < 1e-6 && std::abs(vv[i].second - p.second) < 1e-6)
          return i;
      }
      return vv.size();
    };

    // Ensure start limit
    size_t loc = findIdx(Vout, start_obs);
    if (loc == Vout.size())
    {
      Vout.insert(Vout.begin(), start_obs);
      topoOut.insert(topoOut.begin(), start_obs_topo);
    }
    else
    {
      Vout.erase(Vout.begin(), Vout.begin() + static_cast<std::ptrdiff_t>(loc));
      topoOut.erase(topoOut.begin(), topoOut.begin() + static_cast<std::ptrdiff_t>(loc));
    }

    // Ensure end limit
    loc = findIdx(Vout, end_obs);
    if (loc == Vout.size())
    {
      Vout.push_back(end_obs);
      topoOut.push_back(end_obs_topo);
    }
    else
    {
      Vout.erase(Vout.begin() + static_cast<std::ptrdiff_t>(loc + 1), Vout.end());
      topoOut.erase(topoOut.begin() + static_cast<std::ptrdiff_t>(loc + 1), topoOut.end());
    }
  };

  const int want_k = std::max(1, params_.k_paths);
  std::vector<PathSolution> solutions;
  solutions.reserve(static_cast<size_t>(want_k));

  std::vector<Candidate> Q;
  Q.reserve(4096);
  {
    const double h0 = std::hypot(static_cast<double>(sx - gx), static_cast<double>(sy - gy));
    Q.push_back(Candidate{ -1, -1, h0 });
    std::make_heap(Q.begin(), Q.end(), [](const Candidate& a, const Candidate& b) { return a.F > b.F; });
  }

  std::vector<Node> N;
  N.reserve(4096);

  const auto start_time = std::chrono::steady_clock::now();
  int expansions = 0;
  double min_dist_to_goal = std::numeric_limits<double>::infinity();
  int pnpoly_solution_hits = 0;
  int los_solution_hits = 0;

  while (!Q.empty())
  {
    if (params_.max_expansions > 0 && expansions >= params_.max_expansions)
      break;

    if (params_.max_planning_time_ms > 0)
    {
      const auto now = std::chrono::steady_clock::now();
      const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
      if (dt_ms >= params_.max_planning_time_ms)
        break;
    }

    if (static_cast<int>(solutions.size()) >= want_k)
      break;

    const Candidate best = popBest(Q);
    const int parent_idx = best.N;
    const int child_idx = best.C;

    Node new_node;
    std::vector<std::pair<double, double>> Vtemp;
    std::vector<TopoIndex> topoVtemp;

    if (parent_idx == -1)
    {
      the_map.getVisibilityRegion(sx, sy, Vtemp, topoVtemp);
      if (Vtemp.empty() || topoVtemp.size() != Vtemp.size())
      {
        if (params_.store_debug_data)
          R_WARN << "[raystar upstream_like] empty visibility region at start seed (" << sx << "," << sy
                 << "), obstacles=" << the_map.obstacles().size() << ", V=" << Vtemp.size() << ", topoV="
                 << topoVtemp.size();
        return false;
      }

      new_node.idx = 0;
      new_node.seed = { sx, sy };
      new_node.parent = -1;
      new_node.G = 0.0;
      new_node.H = best.F;
      new_node.start_angle = 0.0;
      new_node.end_angle = kTwoPi;
      new_node.V = std::move(Vtemp);
      new_node.topoV = std::move(topoVtemp);
      new_node.local_path = { { sx, sy } };
      new_node.path_node_idx = { 0 };

      new_node.generateChild(&the_map);
      N.push_back(std::move(new_node));
    }
    else
    {
      if (parent_idx < 0 || parent_idx >= static_cast<int>(N.size()))
        continue;
      const auto& parent = N[static_cast<size_t>(parent_idx)];
      if (child_idx < 0 || child_idx >= static_cast<int>(parent.children.size()))
        continue;

      const auto& chosen = parent.children[static_cast<size_t>(child_idx)];
      const auto new_seed = chosen.c;
      if (new_seed.first < 0 || new_seed.second < 0 || new_seed.first >= nx_ || new_seed.second >= ny_)
        continue;

      scopedVisibility(N, best, Vtemp, topoVtemp);
      if (Vtemp.empty() || topoVtemp.size() != Vtemp.size())
        continue;

      new_node.idx = static_cast<int>(N.size());
      new_node.seed = new_seed;
      new_node.parent = parent_idx;
      new_node.G = chosen.g;
      new_node.H = chosen.h;
      new_node.start_angle = chosen.start_angle;
      new_node.end_angle = chosen.end_angle;
      new_node.V = std::move(Vtemp);
      new_node.topoV = std::move(topoVtemp);

      new_node.local_path = parent.local_path;
      new_node.local_path.push_back(new_seed);

      new_node.path_node_idx = parent.path_node_idx;
      new_node.path_node_idx.push_back(new_node.idx);

      new_node.generateChild(&the_map);
      N.push_back(std::move(new_node));
    }

    ++expansions;
    out_expand_map.emplace_back(N.back().seed.first, N.back().seed.second);

    min_dist_to_goal = std::min(min_dist_to_goal,
                  std::hypot(static_cast<double>(N.back().seed.first - gx),
                         static_cast<double>(N.back().seed.second - gy)));

    // push children candidates
    for (auto& ch : N.back().children)
    {
      ch.h = std::hypot(static_cast<double>(ch.c.first - gx), static_cast<double>(ch.c.second - gy));
      pushCand(Q, Candidate{ ch.Nindex, ch.Cindex, ch.g + ch.h });
    }

    // Solution check:
    // - Upstream Ray* uses "goal is inside visibility region".
    // - Our CGAL-free visibility polygon is an approximation, so to avoid false negatives
    //   (and the resulting 'no solutions found'), also accept a discrete line-of-sight
    //   connection in the costmap grid.
    const auto in_on = upstream_like::pnpoly(N.back().V, static_cast<double>(gx), static_cast<double>(gy));
    const bool goal_in_visibility = (in_on.first || in_on.second);
    const bool goal_in_los = lineOfSight(N.back().seed.first, N.back().seed.second, gx, gy);
    if (goal_in_visibility || goal_in_los)
    {
      if (goal_in_visibility)
        ++pnpoly_solution_hits;
      if (goal_in_los)
        ++los_solution_hits;

      PathSolution sol;
      sol.path = N.back().local_path;
      sol.path.push_back({ gx, gy });
      sol.cost = N.back().G + std::hypot(static_cast<double>(N.back().seed.first - gx),
                                         static_cast<double>(N.back().seed.second - gy));
      sol.node_ids = N.back().path_node_idx;
      solutions.push_back(std::move(sol));
    }
  }

  if (solutions.empty())
  {
    if (params_.store_debug_data)
      R_WARN << "[raystar upstream_like] no solutions found. expansions=" << expansions << ", nodes=" << N.size()
             << ", open_queue=" << Q.size() << ", max_expansions=" << params_.max_expansions
             << ", max_planning_time_ms=" << params_.max_planning_time_ms
             << ", min_dist_to_goal=" << min_dist_to_goal
             << ", pnpoly_hits=" << pnpoly_solution_hits << ", los_hits=" << los_solution_hits;
    return false;
  }

  std::sort(solutions.begin(), solutions.end(), [](const PathSolution& a, const PathSolution& b) { return a.cost < b.cost; });

  auto rasterize = [](int x0, int y0, int x1, int y1, std::vector<std::pair<int, int>>& out) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    while (true)
    {
      out.push_back({ x, y });
      if (x == x1 && y == y1)
        break;
      const int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y += sy;
      }
    }
  };

  auto segmentTraversable = [&](const std::vector<std::pair<int, int>>& seg) {
    for (const auto& p : seg)
    {
      if (!isTraversableCell(p.first, p.second))
        return false;
    }
    return true;
  };

  auto snapToTraversable = [&](int& x, int& y) {
    if (!inBounds(x, y))
      return false;
    if (isTraversableCell(x, y))
      return true;

    // Upstream Ray* allows vertices on obstacle boundaries; our grid output must
    // be collision-free, so we snap a boundary vertex to a nearby traversable cell.
    const int ox = x;
    const int oy = y;
    constexpr int kMaxR = 3;
    for (int r = 1; r <= kMaxR; ++r)
    {
      for (int dy = -r; dy <= r; ++dy)
      {
        for (int dx = -r; dx <= r; ++dx)
        {
          // scan the ring only
          if (std::abs(dx) != r && std::abs(dy) != r)
            continue;
          const int nx = ox + dx;
          const int ny = oy + dy;
          if (!inBounds(nx, ny))
            continue;
          if (isTraversableCell(nx, ny))
          {
            x = nx;
            y = ny;
            return true;
          }
        }
      }
    }
    return false;
  };

  auto connectVertsOnGrid = [&](const std::vector<std::pair<int, int>>& verts, std::vector<std::pair<int, int>>& out) {
    out.clear();
    if (verts.size() < 2)
      return false;

    int ax = verts.front().first;
    int ay = verts.front().second;
    if (!snapToTraversable(ax, ay))
      return false;

    out.push_back({ ax, ay });

    std::vector<float> empty_penalty;
    empty_penalty.clear();

    std::vector<std::pair<int, int>> seg;
    seg.reserve(256);

    for (size_t i = 1; i < verts.size(); ++i)
    {
      int bx = verts[i].first;
      int by = verts[i].second;
      if (!snapToTraversable(bx, by))
        return false;

      if (ax == bx && ay == by)
        continue;

      // 1) Try straight rasterized segment first (fast path).
      seg.clear();
      rasterize(ax, ay, bx, by, seg);
      if (!seg.empty() && segmentTraversable(seg))
      {
        // avoid duplicate join point
        seg.erase(seg.begin());
        out.insert(out.end(), seg.begin(), seg.end());
        ax = bx;
        ay = by;
        continue;
      }

      // 2) Otherwise, connect via grid planner (Theta* / A* depending on params).
      Points3d sub_path_map;
      Points3d sub_expand;
      double sub_cost = 0.0;
      if (!thetaStarPlan(Point3d(ax, ay, 0.0), Point3d(bx, by, 0.0), empty_penalty, sub_path_map, sub_expand, sub_cost))
        return false;
      if (sub_path_map.size() < 2)
        return false;

      // append, skipping the first point (already in out)
      for (size_t k = 1; k < sub_path_map.size(); ++k)
      {
        out.push_back({ static_cast<int>(std::lround(sub_path_map[k].x())), static_cast<int>(std::lround(sub_path_map[k].y())) });
      }

      ax = bx;
      ay = by;
    }

    return out.size() >= 2;
  };

  // output best *valid* path (map coords)
  size_t best_sol_idx = solutions.size();
  std::vector<std::pair<int, int>> best_dense;

  for (size_t si = 0; si < solutions.size(); ++si)
  {
    std::vector<std::pair<int, int>> dense;
    if (!connectVertsOnGrid(solutions[si].path, dense))
      continue;
    if (dense.empty())
      continue;
    best_sol_idx = si;
    best_dense = std::move(dense);
    break;
  }

  if (best_sol_idx == solutions.size())
  {
    if (params_.store_debug_data)
      R_WARN << "[raystar upstream_like] solutions found but all are in-collision in grid; rejecting.";
    return false;
  }

  out_path_map.reserve(best_dense.size());
  for (const auto& p : best_dense)
    out_path_map.emplace_back(p.first, p.second);

  // Debug: store the final selected path (world)
  if (params_.store_debug_data)
  {
    last_candidate_paths_world_.clear();
    last_candidate_costs_.clear();

    Points3d world;
    world.reserve(best_dense.size());
    for (const auto& p : best_dense)
    {
      double wx, wy;
      map2World(p.first, p.second, wx, wy);
      world.emplace_back(wx, wy);
    }
    last_candidate_paths_world_.push_back(std::move(world));
    if (best_sol_idx < solutions.size())
      last_candidate_costs_.push_back(solutions[best_sol_idx].cost);
    last_best_candidate_idx_ = 0;
  }

  return !out_path_map.empty();
}

#if RPP_RAYSTAR_WITH_CGAL
bool RaystarPathPlanner::upstreamCgalPlan(const Point3d& start, const Point3d& goal, Points3d& out_path_map,
                                         Points3d& out_expand_map)
{
  using raystar_cgal::Polymap;
  using raystar_cgal::TopoIndex;
  using upstream_like::kTwoPi;

  out_path_map.clear();
  out_expand_map.clear();

  if (!costmap_)
    return false;

  const int sx = static_cast<int>(std::llround(start.x()));
  const int sy = static_cast<int>(std::llround(start.y()));
  const int gx = static_cast<int>(std::llround(goal.x()));
  const int gy = static_cast<int>(std::llround(goal.y()));

  if (!inBounds(sx, sy) || !inBounds(gx, gy))
    return false;
  if (!isTraversableCell(sx, sy) || !isTraversableCell(gx, gy))
    return false;

  Polymap the_map(costmap_, nx_, ny_, factor_, params_.traverse_unknown);
  if (!the_map.build(sx, sy, gx, gy) || !the_map.solutionExist())
    return false;

  // Debug polygonal obstacles
  if (params_.store_debug_data && params_.publish_poly_obstacles)
  {
    last_poly_obstacles_world_.clear();
    last_poly_obstacles_world_.reserve(the_map.obstacles().size());
    for (const auto& loop : the_map.obstacles())
    {
      if (loop.size() < 2)
        continue;
      Points3d poly;
      poly.reserve(loop.size() + 1);
      for (const auto& v : loop)
      {
        double wx, wy;
        map2World(v.first, v.second, wx, wy);
        poly.emplace_back(wx, wy);
      }
      poly.push_back(poly.front());
      last_poly_obstacles_world_.push_back(std::move(poly));
    }
  }

  struct Candidate
  {
    int N{ -1 };
    int C{ -1 };
    double F{ 0.0 };
  };

  struct Child
  {
    int Nindex{ -1 };
    int Cindex{ -1 };

    double start_angle{ 0.0 };
    double end_angle{ 0.0 };

    std::pair<int, int> c{ 0, 0 };
    std::pair<double, double> o{ 0.0, 0.0 };

    TopoIndex c_topo;
    TopoIndex o_topo;

    bool is_left_gap{ false };
    double g{ 0.0 };
    double h{ 0.0 };
  };

  struct Node
  {
    int idx{ 0 };
    std::pair<int, int> seed{ 0, 0 };
    double start_angle{ 0.0 };
    double end_angle{ kTwoPi };
    int parent{ -1 };

    double G{ 0.0 };
    double H{ 0.0 };

    std::vector<Child> children;
    std::vector<std::pair<double, double>> V;
    std::vector<TopoIndex> topoV;

    std::vector<std::pair<int, int>> local_path;
    std::vector<int> path_node_idx;

    void generateChild(Polymap* pMap)
    {
      children.clear();
      if (V.size() < 2 || topoV.size() != V.size() || !pMap)
        return;

      std::vector<double> theta_list(V.size(), 0.0);
      for (size_t i = 0; i < V.size(); ++i)
      {
        const double raw = std::atan2(V[i].second - static_cast<double>(seed.second),
                                      V[i].first - static_cast<double>(seed.first));
        theta_list[i] = upstream_like::wrapToStart(raw, start_angle);
      }

      std::vector<int> valid_gap_indices;
      valid_gap_indices.reserve(V.size());
      std::vector<bool> is_left(V.size(), false);

      constexpr double threshold2 = 0.0001 * 0.0001;

      for (size_t i = 0; i + 1 < V.size(); ++i)
      {
        const size_t next = i + 1;
        const double diff = upstream_like::normalizeAngle(theta_list[next] - theta_list[i]);
        if (diff * diff >= threshold2)
          continue;

        const auto topo_i = topoV[i];
        const auto topo_next = topoV[next];
        if (!raystar_cgal::topoValid(topo_i) || !raystar_cgal::topoValid(topo_next))
          continue;

        if (pMap->areConsecutive(topo_next, topo_i))
          continue;

        const double dis_i = (V[i].first - seed.first) * (V[i].first - seed.first) +
                             (V[i].second - seed.second) * (V[i].second - seed.second);
        const double dis_next = (V[next].first - seed.first) * (V[next].first - seed.first) +
                                (V[next].second - seed.second) * (V[next].second - seed.second);
        is_left[i] = dis_i > dis_next;
        valid_gap_indices.push_back(static_cast<int>(i));
      }

      for (int gi : valid_gap_indices)
      {
        const size_t i = static_cast<size_t>(gi);
        const size_t next = (i + 1) % V.size();
        const auto topo_i = topoV[i];
        const auto topo_next = topoV[next];
        if (!raystar_cgal::topoValid(topo_i) || !raystar_cgal::topoValid(topo_next))
          continue;

        Child ch;
        ch.Nindex = idx;
        ch.Cindex = -1;
        ch.is_left_gap = is_left[i];

        if (ch.is_left_gap)
        {
          ch.c = { static_cast<int>(V[next].first), static_cast<int>(V[next].second) };
          ch.start_angle = upstream_like::normalizeAngle(theta_list[next]);
          const auto next_obs = pMap->getNextObs(topo_next);
          const double contour_from_next = std::atan2(static_cast<double>(next_obs.second) - V[next].second,
                                                     static_cast<double>(next_obs.first) - V[next].first);
          ch.end_angle = ch.start_angle + upstream_like::normalizeAnglePositive(contour_from_next - ch.start_angle);
          ch.c_topo = topo_next;
          ch.o_topo = topo_i;
          ch.o = { V[i].first, V[i].second };
        }
        else
        {
          ch.c = { static_cast<int>(V[i].first), static_cast<int>(V[i].second) };
          const auto prev_obs = pMap->getPrevObs(topo_i);
          const double contour_from_prev = std::atan2(static_cast<double>(prev_obs.second) - V[i].second,
                                                     static_cast<double>(prev_obs.first) - V[i].first);
          ch.start_angle = contour_from_prev;
          ch.end_angle = contour_from_prev + upstream_like::normalizeAnglePositive(theta_list[i] - contour_from_prev);
          ch.c_topo = topo_i;
          ch.o_topo = topo_next;
          ch.o = { V[next].first, V[next].second };
        }

        if (ch.c.first < 0 || ch.c.second < 0 || ch.c.first >= pMap->nxCells() || ch.c.second >= pMap->nyCells())
          continue;

        ch.g = G + std::hypot(static_cast<double>(seed.first - ch.c.first), static_cast<double>(seed.second - ch.c.second));
        children.push_back(ch);
      }

      for (size_t i = 0; i < children.size(); ++i)
      {
        children[i].Cindex = static_cast<int>(i);
      }
    }
  };

  struct PathSolution
  {
    std::vector<std::pair<int, int>> path;
    double cost{ 0.0 };
    std::vector<int> node_ids;
  };

  auto popBest = [](std::vector<Candidate>& Q) {
    auto comp = [](const Candidate& a, const Candidate& b) { return a.F > b.F; };
    std::pop_heap(Q.begin(), Q.end(), comp);
    Candidate best = Q.back();
    Q.pop_back();
    return best;
  };

  auto pushCand = [](std::vector<Candidate>& Q, const Candidate& c) {
    auto comp = [](const Candidate& a, const Candidate& b) { return a.F > b.F; };
    Q.push_back(c);
    std::push_heap(Q.begin(), Q.end(), comp);
  };

  auto scopedVisibility = [&](const std::vector<Node>& N, const Candidate& cand, std::vector<std::pair<double, double>>& Vout,
                              std::vector<TopoIndex>& topoOut) {
    Vout.clear();
    topoOut.clear();

    const int parent_idx = cand.N;
    const int child_idx = cand.C;
    const auto& ch = N[parent_idx].children[static_cast<size_t>(child_idx)];
    const auto new_seed = ch.c;

    std::vector<std::pair<double, double>> fullV;
    std::vector<TopoIndex> fullTopo;
    the_map.getVisibilityRegion(new_seed.first, new_seed.second, fullV, fullTopo);

    if (fullV.empty() || fullTopo.size() != fullV.size())
      return;

    std::pair<double, double> start_obs;
    std::pair<double, double> end_obs;
    TopoIndex start_obs_topo;
    TopoIndex end_obs_topo;

    double start_angle;
    double end_angle;

    if (ch.is_left_gap)
    {
      start_obs = ch.o;
      const auto end_obs_i = the_map.getNextObs(ch.c_topo);
      end_obs = { static_cast<double>(end_obs_i.first), static_cast<double>(end_obs_i.second) };
      start_obs_topo = ch.o_topo;
      end_obs_topo = the_map.locateVertex(end_obs_i.first, end_obs_i.second);
      start_angle = std::atan2(start_obs.second - new_seed.second, start_obs.first - new_seed.first);
      end_angle = std::atan2(end_obs.second - new_seed.second, end_obs.first - new_seed.first);
    }
    else
    {
      const auto start_obs_i = the_map.getPrevObs(ch.c_topo);
      start_obs = { static_cast<double>(start_obs_i.first), static_cast<double>(start_obs_i.second) };
      end_obs = ch.o;
      start_obs_topo = the_map.locateVertex(start_obs_i.first, start_obs_i.second);
      end_obs_topo = ch.o_topo;
      start_angle = std::atan2(start_obs.second - new_seed.second, start_obs.first - new_seed.first);
      end_angle = std::atan2(end_obs.second - new_seed.second, end_obs.first - new_seed.first);
    }

    end_angle = start_angle + upstream_like::normalizeAnglePositive(end_angle - start_angle);

    for (size_t i = 0; i < fullV.size(); ++i)
    {
      const double raw = std::atan2(fullV[i].second - new_seed.second, fullV[i].first - new_seed.first);
      const double theta = upstream_like::wrapToStart(raw, start_angle);
      if (theta >= start_angle - 1e-7 && theta <= end_angle + 1e-7)
      {
        Vout.push_back(fullV[i]);
        topoOut.push_back(fullTopo[i]);
      }
    }

    auto findIdx = [](const std::vector<std::pair<double, double>>& vv, const std::pair<double, double>& p) {
      for (size_t i = 0; i < vv.size(); ++i)
      {
        if (std::abs(vv[i].first - p.first) < 1e-6 && std::abs(vv[i].second - p.second) < 1e-6)
          return i;
      }
      return vv.size();
    };

    size_t loc = findIdx(Vout, start_obs);
    if (loc == Vout.size())
    {
      Vout.insert(Vout.begin(), start_obs);
      topoOut.insert(topoOut.begin(), start_obs_topo);
    }
    else
    {
      Vout.erase(Vout.begin(), Vout.begin() + static_cast<std::ptrdiff_t>(loc));
      topoOut.erase(topoOut.begin(), topoOut.begin() + static_cast<std::ptrdiff_t>(loc));
    }

    loc = findIdx(Vout, end_obs);
    if (loc == Vout.size())
    {
      Vout.push_back(end_obs);
      topoOut.push_back(end_obs_topo);
    }
    else
    {
      Vout.erase(Vout.begin() + static_cast<std::ptrdiff_t>(loc + 1), Vout.end());
      topoOut.erase(topoOut.begin() + static_cast<std::ptrdiff_t>(loc + 1), topoOut.end());
    }
  };

  const int want_k = std::max(1, params_.k_paths);
  std::vector<PathSolution> solutions;
  solutions.reserve(static_cast<size_t>(want_k));

  std::vector<Candidate> Q;
  Q.reserve(4096);
  {
    const double h0 = std::hypot(static_cast<double>(sx - gx), static_cast<double>(sy - gy));
    Q.push_back(Candidate{ -1, -1, h0 });
    std::make_heap(Q.begin(), Q.end(), [](const Candidate& a, const Candidate& b) { return a.F > b.F; });
  }

  std::vector<Node> N;
  N.reserve(4096);

  const auto start_time = std::chrono::steady_clock::now();
  int expansions = 0;

  while (!Q.empty())
  {
    if (params_.max_expansions > 0 && expansions >= params_.max_expansions)
      break;

    if (params_.max_planning_time_ms > 0)
    {
      const auto now = std::chrono::steady_clock::now();
      const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
      if (dt_ms >= params_.max_planning_time_ms)
        break;
    }

    if (static_cast<int>(solutions.size()) >= want_k)
      break;

    const Candidate best = popBest(Q);
    const int parent_idx = best.N;
    const int child_idx = best.C;

    Node new_node;
    std::vector<std::pair<double, double>> Vtemp;
    std::vector<TopoIndex> topoVtemp;

    if (parent_idx == -1)
    {
      the_map.getVisibilityRegion(sx, sy, Vtemp, topoVtemp);
      if (Vtemp.empty() || topoVtemp.size() != Vtemp.size())
        return false;

      new_node.idx = 0;
      new_node.seed = { sx, sy };
      new_node.parent = -1;
      new_node.G = 0.0;
      new_node.H = best.F;
      new_node.start_angle = 0.0;
      new_node.end_angle = kTwoPi;
      new_node.V = std::move(Vtemp);
      new_node.topoV = std::move(topoVtemp);
      new_node.local_path = { { sx, sy } };
      new_node.path_node_idx = { 0 };

      new_node.generateChild(&the_map);
      N.push_back(std::move(new_node));
    }
    else
    {
      if (parent_idx < 0 || parent_idx >= static_cast<int>(N.size()))
        continue;
      const auto& parent = N[static_cast<size_t>(parent_idx)];
      if (child_idx < 0 || child_idx >= static_cast<int>(parent.children.size()))
        continue;

      const auto& chosen = parent.children[static_cast<size_t>(child_idx)];
      const auto new_seed = chosen.c;
      if (new_seed.first < 0 || new_seed.second < 0 || new_seed.first >= nx_ || new_seed.second >= ny_)
        continue;

      scopedVisibility(N, best, Vtemp, topoVtemp);
      if (Vtemp.empty() || topoVtemp.size() != Vtemp.size())
        continue;

      new_node.idx = static_cast<int>(N.size());
      new_node.seed = new_seed;
      new_node.parent = parent_idx;
      new_node.G = chosen.g;
      new_node.H = chosen.h;
      new_node.start_angle = chosen.start_angle;
      new_node.end_angle = chosen.end_angle;
      new_node.V = std::move(Vtemp);
      new_node.topoV = std::move(topoVtemp);

      new_node.local_path = parent.local_path;
      new_node.local_path.push_back(new_seed);

      new_node.path_node_idx = parent.path_node_idx;
      new_node.path_node_idx.push_back(new_node.idx);

      new_node.generateChild(&the_map);
      N.push_back(std::move(new_node));
    }

    ++expansions;
    out_expand_map.emplace_back(N.back().seed.first, N.back().seed.second);

    for (auto& ch : N.back().children)
    {
      ch.h = std::hypot(static_cast<double>(ch.c.first - gx), static_cast<double>(ch.c.second - gy));
      pushCand(Q, Candidate{ ch.Nindex, ch.Cindex, ch.g + ch.h });
    }

    const auto in_on = upstream_like::pnpoly(N.back().V, static_cast<double>(gx), static_cast<double>(gy));
    if (in_on.first || in_on.second)
    {
      PathSolution sol;
      sol.path = N.back().local_path;
      sol.path.push_back({ gx, gy });
      sol.cost = N.back().G + std::hypot(static_cast<double>(N.back().seed.first - gx),
                                         static_cast<double>(N.back().seed.second - gy));
      sol.node_ids = N.back().path_node_idx;
      solutions.push_back(std::move(sol));
    }
  }

  if (solutions.empty())
    return false;

  std::sort(solutions.begin(), solutions.end(), [](const PathSolution& a, const PathSolution& b) { return a.cost < b.cost; });

  auto rasterize = [](int x0, int y0, int x1, int y1, std::vector<std::pair<int, int>>& out) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    while (true)
    {
      out.push_back({ x, y });
      if (x == x1 && y == y1)
        break;
      const int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y += sy;
      }
    }
  };

  auto densifyPath = [&](const std::vector<std::pair<int, int>>& verts) {
    std::vector<std::pair<int, int>> dense;
    if (verts.empty())
      return dense;
    dense.reserve(verts.size() * 8);
    for (size_t i = 1; i < verts.size(); ++i)
    {
      std::vector<std::pair<int, int>> seg;
      seg.reserve(64);
      rasterize(verts[i - 1].first, verts[i - 1].second, verts[i].first, verts[i].second, seg);
      if (i > 1 && !dense.empty() && !seg.empty())
        seg.erase(seg.begin());
      dense.insert(dense.end(), seg.begin(), seg.end());
    }
    return dense;
  };

  const auto best_dense = densifyPath(solutions.front().path);
  out_path_map.reserve(best_dense.size());
  for (const auto& p : best_dense)
    out_path_map.emplace_back(p.first, p.second);

  if (params_.store_debug_data)
  {
    last_candidate_paths_world_.clear();
    last_candidate_costs_.clear();
    last_candidate_paths_world_.reserve(solutions.size());
    last_candidate_costs_.reserve(solutions.size());

    for (const auto& sol : solutions)
    {
      const auto dense = densifyPath(sol.path);
      Points3d world;
      world.reserve(dense.size());
      for (const auto& p : dense)
      {
        double wx, wy;
        map2World(p.first, p.second, wx, wy);
        world.emplace_back(wx, wy);
      }
      last_candidate_paths_world_.push_back(std::move(world));
      last_candidate_costs_.push_back(sol.cost);
    }
    last_best_candidate_idx_ = 0;
  }

  return !out_path_map.empty();
}
#endif

bool RaystarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  clearDebugCache();

  path.clear();
  expand.clear();

  if (!costmap_)
    return false;

  if (params_.implementation == "upstream_cgal")
  {
#if RPP_RAYSTAR_WITH_CGAL
    const bool ok = upstreamCgalPlan(start, goal, path, expand);
    if (ok)
    {
      last_best_expand_map_ = expand;
    }
    return ok;
#else
    return false;
#endif
  }

  if (params_.implementation == "upstream_like")
  {
    const bool ok = upstreamLikePlan(start, goal, path, expand);
    if (ok)
    {
      last_best_expand_map_ = expand;
    }
    return ok;
  }

  // Compute reference obstacle components (for signature + poly debug)
  const int sx = static_cast<int>(std::round(start.x()));
  const int sy = static_cast<int>(std::round(start.y()));
  const int gx = static_cast<int>(std::round(goal.x()));
  const int gy = static_cast<int>(std::round(goal.y()));

  const int margin = std::max(0, params_.component_search_margin_cells);
  const int min_x = std::min(sx, gx) - margin;
  const int max_x = std::max(sx, gx) + margin;
  const int min_y = std::min(sy, gy) - margin;
  const int max_y = std::max(sy, gy) + margin;

  std::vector<ObstacleComponent> comps;
  computeObstacleComponentsInBox(min_x, min_y, max_x, max_y, comps);

  std::vector<Point2d> ref_world;
  selectSignatureReferencePoints(comps, ref_world, last_poly_obstacles_world_);

  // Candidate generation: iterative re-planning with penalty map
  std::vector<float> penalty_grid(map_size_, 0.0f);

  std::vector<Points3d> cand_paths_map;
  std::vector<Points3d> cand_paths_world;
  std::vector<Points3d> cand_expands_map;
  std::vector<double> cand_costs;
  std::vector<std::string> cand_sigs;
  std::vector<std::unordered_set<int>> cand_cell_sets;

  cand_paths_map.reserve(static_cast<size_t>(std::max(1, params_.k_paths)));

  const int tries = std::max(1, params_.max_tries);
  const int want_k = std::max(1, params_.k_paths);

  for (int t = 0; t < tries; ++t)
  {
    Points3d run_path_map;
    Points3d run_expand_map;
    double run_cost = 0.0;

    if (!thetaStarPlan(start, goal, penalty_grid, run_path_map, run_expand_map, run_cost))
    {
      // If we already have some candidates, stop; otherwise planning failed.
      if (!cand_paths_map.empty())
        break;
      return false;
    }

    // Convert to world
    Points3d run_path_world;
    run_path_world.reserve(run_path_map.size());
    for (const auto& p : run_path_map)
    {
      double wx, wy;
      map2World(p.x(), p.y(), wx, wy);
      run_path_world.emplace_back(wx, wy);
    }

    const std::string sig = computeHSignature(run_path_world, ref_world);
    const auto cell_set = pathToCellSet(run_path_map, nx_);

    bool duplicate = false;
    for (size_t i = 0; i < cand_paths_map.size(); ++i)
    {
      if (!sig.empty() && sig == cand_sigs[i])
      {
        const double j = jaccardSimilarity(cell_set, cand_cell_sets[i]);
        if (j > 0.9)
        {
          duplicate = true;
          break;
        }
      }
    }

    if (!duplicate)
    {
      cand_paths_map.push_back(run_path_map);
      cand_paths_world.push_back(run_path_world);
      cand_expands_map.push_back(run_expand_map);
      cand_costs.push_back(run_cost);
      cand_sigs.push_back(sig);
      cand_cell_sets.push_back(cell_set);

      if (static_cast<int>(cand_paths_map.size()) >= want_k)
      {
        updatePenaltyGrid(run_path_map, penalty_grid);
        break;
      }
    }

    // Always update penalty to push for diversity in the next try.
    updatePenaltyGrid(run_path_map, penalty_grid);
  }

  if (cand_paths_map.empty())
    return false;

  // Choose best by cost
  int best_i = 0;
  double best_cost = cand_costs[0];
  for (size_t i = 1; i < cand_costs.size(); ++i)
  {
    if (cand_costs[i] < best_cost)
    {
      best_cost = cand_costs[i];
      best_i = static_cast<int>(i);
    }
  }

  path = cand_paths_map[best_i];
  expand = cand_expands_map[best_i];

  // Debug cache
  last_candidate_paths_world_ = cand_paths_world;
  last_candidate_costs_ = cand_costs;
  last_best_candidate_idx_ = best_i;
  last_best_expand_map_ = cand_expands_map[best_i];

  return true;
}

}  // namespace path_planner
}  // namespace rpp
