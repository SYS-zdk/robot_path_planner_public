/**
 * @file: rhcf_planner.cpp
 * @brief: RHCF global planner implementation (Voronoi biased random walk)
 * @date: 2026.03.10
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include <costmap_2d/cost_values.h>
#include <ros/console.h>

#include "rhcf_global_planner/rhcf_planner.h"

namespace rpp
{
namespace path_planner
{
namespace
{
inline double hypotCells(int ax, int ay, int bx, int by)
{
  return std::hypot(static_cast<double>(ax - bx), static_cast<double>(ay - by));
}

}  // namespace

RhcfPathPlanner::RhcfPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double circumscribed_radius, const Params& params,
                                 double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor)
  , params_(params)
  , circumscribed_radius_(circumscribed_radius)
  , resolution_(costmap_ ? costmap_->getResolution() : 0.0)
  , voronoi_(static_cast<size_t>(map_size_))
{
  if (params_.use_fixed_seed)
  {
    rng_.seed(static_cast<uint32_t>(params_.fixed_seed));
  }
  else
  {
    std::random_device rd;
    rng_.seed(rd());
  }

  // Clamp parameters to sensible ranges (avoid undefined behavior)
  params_.K_homotopy_classes = std::max(1, params_.K_homotopy_classes);
  params_.max_random_walks = std::max(params_.K_homotopy_classes, params_.max_random_walks);
  params_.max_vertex_steps_per_walk = std::max(10, params_.max_vertex_steps_per_walk);
  params_.discounting_factor = std::min(1.0, std::max(0.5, params_.discounting_factor));
  params_.edge_similarity_threshold = std::min(0.999, std::max(0.0, params_.edge_similarity_threshold));
  params_.visited_vertex_weight = std::min(1.0, std::max(0.0, params_.visited_vertex_weight));
}

bool RhcfPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  path.clear();
  expand.clear();

  last_candidate_paths_world_.clear();
  last_candidate_costs_.clear();
  last_best_candidate_idx_ = -1;

  if (!costmap_ros_ || !costmap_)
  {
    ROS_ERROR("[RHCF] costmap is null");
    return false;
  }

  if (!updateVoronoi())
  {
    ROS_ERROR("[RHCF] Voronoi layer not available. Make sure a VoronoiLayer is loaded in the global costmap.");
    return false;
  }

  Node start_node(static_cast<int>(start.x()), static_cast<int>(start.y()));
  Node goal_node(static_cast<int>(goal.x()), static_cast<int>(goal.y()));
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

  if (start_node.id() < 0 || start_node.id() >= map_size_ || goal_node.id() < 0 || goal_node.id() >= map_size_)
  {
    ROS_WARN("[RHCF] start/goal out of costmap bounds");
    return false;
  }

  // 1) Connect start/goal to Voronoi skeleton
  std::vector<int> path_start_to_v, path_goal_to_v;
  int v_start_id = -1;
  int v_goal_id = -1;

  if (!searchToVoronoi(start_node, path_start_to_v, v_start_id))
  {
    ROS_WARN("[RHCF] failed to connect start to Voronoi skeleton");
    return false;
  }
  if (!searchToVoronoi(goal_node, path_goal_to_v, v_goal_id))
  {
    ROS_WARN("[RHCF] failed to connect goal to Voronoi skeleton");
    return false;
  }

  // 2) Build a reduced vertex graph on the Voronoi skeleton
  buildVoronoiVertexGraph(v_start_id, v_goal_id);

  if (vertices_.find(v_start_id) == vertices_.end() || vertices_.find(v_goal_id) == vertices_.end())
  {
    ROS_WARN("[RHCF] Voronoi vertex graph does not contain start/goal skeleton nodes");
    return false;
  }

  // Connectivity sanity check (avoid wasting time on random walks when graph is disconnected)
  {
    std::queue<int> q;
    std::unordered_set<int> vis;
    vis.reserve(vertices_.size());
    q.push(v_start_id);
    vis.insert(v_start_id);

    while (!q.empty())
    {
      const int v = q.front();
      q.pop();
      if (v == v_goal_id)
        break;

      const auto it = adjacency_edges_.find(v);
      if (it == adjacency_edges_.end())
        continue;
      const auto& adj = it->second;
      for (const int eid : adj)
      {
        const Edge& e = edges_[static_cast<size_t>(eid)];
        const int nb = (e.a == v) ? e.b : e.a;
        if (vis.insert(nb).second)
          q.push(nb);
      }
    }

    if (vis.find(v_goal_id) == vis.end())
    {
      ROS_WARN("[RHCF] Voronoi vertex graph disconnected (start->goal unreachable). vertices=%zu edges=%zu", vertices_.size(),
               edges_.size());
      return false;
    }
  }

  // 3) Run RHCF: biased random walks to sample K diverse paths
  struct Candidate
  {
    std::vector<int> vertex_seq;
    std::vector<int> cells;
    double cost{ 0.0 };
  };

  std::vector<Candidate> accepted;
  accepted.reserve(static_cast<size_t>(params_.K_homotopy_classes));

  std::unordered_set<int> expand_cells;

  // Directed edge weights used by the random walk (updated across attempts).
  // This mirrors the upstream idea: chosen edge gets multiplied by df, the other edges
  // from the same node get divided by df, biasing subsequent walks towards new paths.
  std::unordered_map<uint64_t, double> directed_weights;
  directed_weights.reserve(edges_.size() * 2);

  for (const auto& e : edges_)
  {
    const double eps = 1e-9;

    // Optional goal bias via a heuristic term (set w_heuristic=0 to mimic upstream closer).
    const double h_ab = (params_.w_heuristic > 0.0) ? (params_.w_heuristic * heuristicM(e.b, v_goal_id)) : 0.0;
    const double h_ba = (params_.w_heuristic > 0.0) ? (params_.w_heuristic * heuristicM(e.a, v_goal_id)) : 0.0;

    const double w_ab = 1.0 / (eps + e.cost + h_ab);
    const double w_ba = 1.0 / (eps + e.cost + h_ba);
    directed_weights.emplace(packDirectedEdge(e.a, e.b), w_ab);
    directed_weights.emplace(packDirectedEdge(e.b, e.a), w_ba);
  }

  int walks_reached_goal = 0;

  for (int attempt = 0;
       attempt < params_.max_random_walks && static_cast<int>(accepted.size()) < params_.K_homotopy_classes; ++attempt)
  {
    Candidate cand;
    if (!runOneRandomWalk(v_start_id, v_goal_id, directed_weights, cand.vertex_seq, cand.cost, expand_cells))
      continue;

    ++walks_reached_goal;

    // Upstream-style uniqueness check: reject only if the vertex sequence is identical.
    bool distinct = true;
    for (const auto& prev : accepted)
    {
      if (cand.vertex_seq.size() != prev.vertex_seq.size())
        continue;

      bool same = true;
      for (size_t i = 0; i < cand.vertex_seq.size(); ++i)
      {
        if (cand.vertex_seq[i] != prev.vertex_seq[i])
        {
          same = false;
          break;
        }
      }
      if (same)
      {
        distinct = false;
        break;
      }
    }

    if (!distinct)
      continue;

    if (!reconstructCellsFromVertexSeq(cand.vertex_seq, cand.cells))
      continue;

    accepted.push_back(std::move(cand));
  }

  if (accepted.empty())
  {
    ROS_WARN("[RHCF] no RHCF candidate path found (walks_reached_goal=%d/%d, vertices=%zu edges=%zu)",
             walks_reached_goal, params_.max_random_walks, vertices_.size(), edges_.size());
    return false;
  }

  // pick best geometric path among accepted candidates
  size_t best_idx = 0;
  for (size_t i = 1; i < accepted.size(); ++i)
  {
    if (accepted[i].cost < accepted[best_idx].cost)
      best_idx = i;
  }
  const Candidate* best = &accepted[best_idx];

  // Cache candidates in world coordinates for optional RViz visualization.
  last_candidate_paths_world_.reserve(accepted.size());
  last_candidate_costs_.reserve(accepted.size());
  for (size_t i = 0; i < accepted.size(); ++i)
  {
    last_candidate_costs_.push_back(accepted[i].cost);

    Points3d pts_world;
    pts_world.reserve(accepted[i].cells.size());
    for (const int cell_id : accepted[i].cells)
    {
      int x, y;
      index2Grid(cell_id, x, y);
      double wx, wy;
      map2World(static_cast<double>(x), static_cast<double>(y), wx, wy);
      pts_world.emplace_back(wx, wy, 0.0);
    }
    last_candidate_paths_world_.push_back(std::move(pts_world));
  }
  last_best_candidate_idx_ = static_cast<int>(best_idx);

  // 4) Stitch: start -> v_start, (best Voronoi walk) v_start -> v_goal, v_goal -> goal
  std::vector<int> final_cells;
  final_cells.reserve(path_start_to_v.size() + best->cells.size() + path_goal_to_v.size());

  // start->v_start: already in correct order
  for (int idx : path_start_to_v)
    final_cells.push_back(idx);

  // v_start->v_goal: avoid duplicating v_start
  for (size_t i = 0; i < best->cells.size(); ++i)
  {
    if (!final_cells.empty() && best->cells[i] == final_cells.back())
      continue;
    final_cells.push_back(best->cells[i]);
  }

  // v_goal->goal: reverse (goal->v_goal) and avoid duplicating v_goal
  for (auto rit = path_goal_to_v.rbegin(); rit != path_goal_to_v.rend(); ++rit)
  {
    if (!final_cells.empty() && *rit == final_cells.back())
      continue;
    final_cells.push_back(*rit);
  }

  // convert to planner path (map cells)
  path.reserve(final_cells.size());
  int last_x = 0, last_y = 0;
  for (size_t i = 0; i < final_cells.size(); ++i)
  {
    int x, y;
    index2Grid(final_cells[i], x, y);
    if (i > 0 && x == last_x && y == last_y)
      continue;
    last_x = x;
    last_y = y;
    path.emplace_back(static_cast<double>(x), static_cast<double>(y), 0.0);
  }

  // expand visualization: mark visited skeleton vertices across attempts
  expand.reserve(expand_cells.size());
  for (const int id : expand_cells)
  {
    int x, y;
    index2Grid(id, x, y);
    expand.emplace_back(static_cast<double>(x), static_cast<double>(y), 0.0);
  }

  return !path.empty();
}

bool RhcfPathPlanner::updateVoronoi()
{
  // Find VoronoiLayer from layered costmap plugins
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
        boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
    if (!voronoi_layer)
      continue;

    boost::unique_lock<boost::mutex> lock(voronoi_layer->getMutex());
    const DynamicVoronoi& vor = voronoi_layer->getVoronoi();
    if (vor.getSizeX() == 0u || vor.getSizeY() == 0u)
    {
      ROS_ERROR("[RHCF] VoronoiLayer map is empty");
      return false;
    }

    const double res = costmap_->getResolution();
    for (int y = 0; y < ny_; ++y)
    {
      for (int x = 0; x < nx_; ++x)
      {
        CellInfo& c = voronoi_[static_cast<size_t>(grid2Index(x, y))];
        c.dist_m = static_cast<float>(vor.getDistance(x, y) * res);
        c.is_voronoi = vor.isVoronoi(x, y);
      }
    }

    return true;
  }

  return false;
}

bool RhcfPathPlanner::isInBounds(int x, int y) const
{
  return x >= 0 && y >= 0 && x < nx_ && y < ny_;
}

bool RhcfPathPlanner::isTraversableCell(int x, int y, int cur_id) const
{
  if (!isInBounds(x, y))
    return false;

  const int id = grid2Index(x, y);
  if (id < 0 || id >= map_size_)
    return false;

  const unsigned char c = costmap_->getCharMap()[id];
  if (!params_.traverse_unknown && c == costmap_2d::NO_INFORMATION)
    return false;

  // similar to AStar planner: allow escaping from inflation if current is worse
  if (c >= costmap_2d::LETHAL_OBSTACLE * factor_ && (cur_id < 0 || c >= costmap_->getCharMap()[cur_id]))
    return false;

  return true;
}

bool RhcfPathPlanner::isSkeletonCell(int x, int y) const
{
  if (!isInBounds(x, y))
    return false;

  const int id = grid2Index(x, y);
  const CellInfo& c = voronoi_[static_cast<size_t>(id)];
  if (!c.is_voronoi)
    return false;

  // ensure enough clearance to obstacles
  if (c.dist_m < static_cast<float>(circumscribed_radius_))
    return false;

  // ensure traversable
  return isTraversableCell(x, y, -1);
}

int RhcfPathPlanner::skeletonDegree4(int x, int y) const
{
  int deg = 0;
  for (const auto& d : n4_)
  {
    const int nx = x + d.first;
    const int ny = y + d.second;
    if (isSkeletonCell(nx, ny))
      ++deg;
  }
  return deg;
}

bool RhcfPathPlanner::searchToVoronoi(const Node& start, std::vector<int>& out_path_cells, int& out_voronoi_id)
{
  out_path_cells.clear();
  out_voronoi_id = -1;

  if (isSkeletonCell(start.x(), start.y()))
  {
    out_voronoi_id = start.id();
    out_path_cells.push_back(start.id());
    return true;
  }

  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open;
  std::unordered_map<int, Node> closed;

  Node s = start;
  open.push(s);

  while (!open.empty())
  {
    Node cur = open.top();
    open.pop();

    if (closed.find(cur.id()) != closed.end())
      continue;

    closed.emplace(cur.id(), cur);

    if (isSkeletonCell(cur.x(), cur.y()))
    {
      // backtrace to get start->cur
      auto backtrace = _convertClosedListToPath<Node>(closed, s, cur);
      out_path_cells.reserve(backtrace.size());
      for (auto it = backtrace.rbegin(); it != backtrace.rend(); ++it)
      {
        const int idx = grid2Index(it->x(), it->y());
        if (!out_path_cells.empty() && idx == out_path_cells.back())
          continue;
        out_path_cells.push_back(idx);
      }

      out_voronoi_id = cur.id();
      return true;
    }

    for (const auto& m : motions8_)
    {
      Node nxt = cur + m;
      const int id = grid2Index(nxt.x(), nxt.y());
      nxt.set_id(id);

      if (closed.find(id) != closed.end())
        continue;

      if (id < 0 || id >= map_size_)
        continue;

      if (!isTraversableCell(nxt.x(), nxt.y(), cur.id()))
        continue;

      // prefer cells with a bit of clearance even before reaching skeleton
      const CellInfo& ci = voronoi_[static_cast<size_t>(id)];
      if (ci.dist_m < static_cast<float>(circumscribed_radius_ * 0.5))
        continue;

      nxt.set_g(cur.g() + m.g());
      nxt.set_h(0.0);
      nxt.set_pid(cur.id());

      open.push(nxt);
    }
  }

  return false;
}

void RhcfPathPlanner::buildVoronoiVertexGraph(int v_start_id, int v_goal_id)
{
  vertices_.clear();
  edges_.clear();
  adjacency_edges_.clear();

  // collect vertex set
  std::unordered_set<int> vertex_ids;
  vertex_ids.reserve(static_cast<size_t>(map_size_ / 20));

  for (int y = 0; y < ny_; ++y)
  {
    for (int x = 0; x < nx_; ++x)
    {
      if (!isSkeletonCell(x, y))
        continue;

      const int id = grid2Index(x, y);
      const int deg = skeletonDegree4(x, y);
      if (deg != 2 || id == v_start_id || id == v_goal_id)
      {
        vertex_ids.insert(id);
      }
    }
  }

  auto addVertexIfNeeded = [&](int id) {
    if (vertices_.find(id) != vertices_.end())
      return;
    int x, y;
    index2Grid(id, x, y);
    vertices_[id] = Vertex{ id, x, y };
  };

  for (const int id : vertex_ids)
    addVertexIfNeeded(id);

  // trace edges from each vertex
  std::unordered_set<uint64_t> visited_half_edges;
  visited_half_edges.reserve(vertices_.size() * 4);

  auto packHalf = [](int from, int to) -> uint64_t {
    return (static_cast<uint64_t>(static_cast<uint32_t>(from)) << 32) | static_cast<uint32_t>(to);
  };

  for (auto itv = vertices_.begin(); itv != vertices_.end(); ++itv)
  {
    const int vid = itv->first;
    int vx, vy;
    index2Grid(vid, vx, vy);

    for (const auto& d : n4_)
    {
      const int nx = vx + d.first;
      const int ny = vy + d.second;
      if (!isSkeletonCell(nx, ny))
        continue;

      const int n_id = grid2Index(nx, ny);
      const uint64_t half_key = packHalf(vid, n_id);
      if (visited_half_edges.find(half_key) != visited_half_edges.end())
        continue;

      // trace corridor until next vertex
      std::vector<int> poly;
      poly.reserve(128);
      poly.push_back(vid);
      poly.push_back(n_id);

      int prev_id = vid;
      int cur_id = n_id;

      while (vertex_ids.find(cur_id) == vertex_ids.end())
      {
        int cx, cy;
        index2Grid(cur_id, cx, cy);

        // find next skeleton neighbor excluding prev
        int next_id = -1;
        int next_count = 0;
        for (const auto& dd : n4_)
        {
          const int tx = cx + dd.first;
          const int ty = cy + dd.second;
          if (!isSkeletonCell(tx, ty))
            continue;
          const int tid = grid2Index(tx, ty);
          if (tid == prev_id)
            continue;
          next_id = tid;
          ++next_count;
        }

        if (next_count != 1 || next_id < 0)
          break;

        poly.push_back(next_id);
        prev_id = cur_id;
        cur_id = next_id;

        // safety stop
        if (static_cast<int>(poly.size()) > map_size_)
          break;
      }

      const int end_id = cur_id;
      if (end_id == vid)
        continue;

      addVertexIfNeeded(end_id);

      // mark half-edges visited along the polyline
      for (size_t i = 1; i < poly.size(); ++i)
      {
        visited_half_edges.insert(packHalf(poly[i - 1], poly[i]));
      }

      // create undirected edge
      Edge e;
      e.id = static_cast<int>(edges_.size());
      e.a = vid;
      e.b = end_id;
      e.cells = std::move(poly);
      e.length_m = computePolylineLengthM(e.cells);
      e.cost = computePolylineCost(e.cells);

      edges_.push_back(std::move(e));
      adjacency_edges_[vid].push_back(edges_.back().id);
      adjacency_edges_[end_id].push_back(edges_.back().id);
    }
  }

  // ensure start/goal have adjacency lists (even if isolated)
  (void)adjacency_edges_[v_start_id];
  (void)adjacency_edges_[v_goal_id];
}

bool RhcfPathPlanner::runOneRandomWalk(int v_start_id, int v_goal_id,
                                      std::unordered_map<uint64_t, double>& inout_directed_weights,
                                      std::vector<int>& out_vertex_seq, double& out_cost,
                                      std::unordered_set<int>& inout_expand_cells)
{
  out_vertex_seq.clear();
  out_cost = 0.0;

  // must have graph connectivity
  if (adjacency_edges_.find(v_start_id) == adjacency_edges_.end() ||
      adjacency_edges_.find(v_goal_id) == adjacency_edges_.end())
    return false;

  std::unordered_set<int> visited_vertices;
  visited_vertices.reserve(256);

  int cur = v_start_id;
  out_vertex_seq.push_back(cur);
  visited_vertices.insert(cur);

  for (int step = 0; step < params_.max_vertex_steps_per_walk; ++step)
  {
    inout_expand_cells.insert(cur);

    if (cur == v_goal_id)
      break;

    const auto& adj = adjacency_edges_[cur];
    if (adj.empty())
      return false;

    std::vector<double> weights;
    weights.reserve(adj.size());

    std::vector<int> next_vertices;
    next_vertices.reserve(adj.size());

    std::vector<int> next_edge_ids;
    next_edge_ids.reserve(adj.size());

    for (const int eid : adj)
    {
      const Edge& e = edges_[static_cast<size_t>(eid)];
      const int nxt = (e.a == cur) ? e.b : e.a;

      const bool visited = (visited_vertices.find(nxt) != visited_vertices.end());

      const auto itw = inout_directed_weights.find(packDirectedEdge(cur, nxt));
      if (itw == inout_directed_weights.end())
        continue;

      const double w = itw->second;
      if (!(w > 0.0))
        continue;

      // Allow revisits for implicit backtracking, but down-weight them.
      const double w_eff = visited ? (w * params_.visited_vertex_weight) : w;
      if (!(w_eff > 0.0))
        continue;

      weights.push_back(w_eff);
      next_vertices.push_back(nxt);
      next_edge_ids.push_back(eid);
    }

    // all zero weights -> dead end
    if (weights.empty())
      return false;

    std::discrete_distribution<size_t> dist(weights.begin(), weights.end());
    const size_t pick = dist(rng_);

    const int chosen_edge_id = next_edge_ids[pick];
    const Edge& chosen_edge = edges_[static_cast<size_t>(chosen_edge_id)];
    const int nxt = next_vertices[pick];

    out_cost += chosen_edge.cost;

    // Update edge weights for subsequent random walks (upstream df_ mechanism).
    // Chosen edge: weight *= df, other edges from current: weight /= df.
    const double df = params_.discounting_factor;
    if (df > 0.0)
    {
      for (const int eid : adj)
      {
        const Edge& e = edges_[static_cast<size_t>(eid)];
        const int nb = (e.a == cur) ? e.b : e.a;

        const uint64_t k_uv = packDirectedEdge(cur, nb);
        const uint64_t k_vu = packDirectedEdge(nb, cur);

        auto itu = inout_directed_weights.find(k_uv);
        auto itv = inout_directed_weights.find(k_vu);
        if (itu == inout_directed_weights.end() || itv == inout_directed_weights.end())
          continue;

        if (nb == nxt)
        {
          itu->second *= df;
          itv->second *= df;
        }
        else
        {
          itu->second /= df;
          itv->second /= df;
        }
      }
    }

    cur = nxt;
    out_vertex_seq.push_back(cur);
    visited_vertices.insert(cur);

    if (cur == v_goal_id)
      break;
  }

  if (out_vertex_seq.empty() || out_vertex_seq.back() != v_goal_id)
    return false;

  return true;
}

double RhcfPathPlanner::heuristicM(int v_id, int goal_id) const
{
  const auto itv = vertices_.find(v_id);
  const auto itg = vertices_.find(goal_id);
  if (itv == vertices_.end() || itg == vertices_.end())
    return 0.0;

  const Vertex& v = itv->second;
  const Vertex& g = itg->second;
  return hypotCells(v.x, v.y, g.x, g.y) * resolution_;
}

void RhcfPathPlanner::appendEdgeCellsDirected(int from_v, int to_v, const Edge& e, std::vector<int>& io_cells) const
{
  if (e.a == from_v && e.b == to_v)
  {
    for (size_t i = 0; i < e.cells.size(); ++i)
    {
      const int idx = e.cells[i];
      if (!io_cells.empty() && io_cells.back() == idx)
        continue;
      io_cells.push_back(idx);
    }
  }
  else if (e.a == to_v && e.b == from_v)
  {
    for (auto rit = e.cells.rbegin(); rit != e.cells.rend(); ++rit)
    {
      const int idx = *rit;
      if (!io_cells.empty() && io_cells.back() == idx)
        continue;
      io_cells.push_back(idx);
    }
  }
}

bool RhcfPathPlanner::reconstructCellsFromVertexSeq(const std::vector<int>& vertex_seq, std::vector<int>& out_cells) const
{
  out_cells.clear();
  if (vertex_seq.size() < 1)
    return false;

  out_cells.push_back(vertex_seq.front());

  for (size_t i = 1; i < vertex_seq.size(); ++i)
  {
    const int a = vertex_seq[i - 1];
    const int b = vertex_seq[i];

    // find the shared edge
    const auto it = adjacency_edges_.find(a);
    if (it == adjacency_edges_.end())
      return false;

    const auto& adj = it->second;
    const Edge* edge_ptr = nullptr;
    for (const int eid : adj)
    {
      const Edge& e = edges_[static_cast<size_t>(eid)];
      if ((e.a == a && e.b == b) || (e.a == b && e.b == a))
      {
        edge_ptr = &e;
        break;
      }
    }
    if (!edge_ptr)
      return false;

    appendEdgeCellsDirected(a, b, *edge_ptr, out_cells);
  }

  return !out_cells.empty();
}

double RhcfPathPlanner::computePolylineLengthM(const std::vector<int>& cells) const
{
  if (cells.size() < 2)
    return 0.0;

  double len_cells = 0.0;
  int px, py;
  index2Grid(cells.front(), px, py);
  for (size_t i = 1; i < cells.size(); ++i)
  {
    int x, y;
    index2Grid(cells[i], x, y);
    len_cells += std::hypot(static_cast<double>(x - px), static_cast<double>(y - py));
    px = x;
    py = y;
  }
  return len_cells * resolution_;
}

double RhcfPathPlanner::computePolylineCost(const std::vector<int>& cells) const
{
  if (cells.empty())
    return 0.0;

  const double len_m = computePolylineLengthM(cells);

  // clearance penalty: integral of 1/(d+eps) along polyline
  double clearance_pen = 0.0;
  double costmap_pen = 0.0;

  for (size_t i = 0; i < cells.size(); ++i)
  {
    const int id = cells[i];
    const CellInfo& ci = voronoi_[static_cast<size_t>(id)];
    const double d = std::max(0.0, static_cast<double>(ci.dist_m));
    clearance_pen += 1.0 / (1e-3 + d);

    const unsigned char c = costmap_->getCharMap()[id];
    if (c == costmap_2d::NO_INFORMATION)
    {
      costmap_pen += params_.traverse_unknown ? 0.5 : 1.0;
    }
    else
    {
      costmap_pen += static_cast<double>(c) / 255.0;
    }
  }

  // normalize penalties by number of samples
  clearance_pen /= static_cast<double>(cells.size());
  costmap_pen /= static_cast<double>(cells.size());

  // geometric cost (lower is better)
  const double cost = params_.w_length * len_m + params_.w_clearance * clearance_pen + params_.w_costmap * costmap_pen;
  return cost;
}

uint64_t RhcfPathPlanner::packDirectedEdge(int from_id, int to_id)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(from_id)) << 32) | static_cast<uint32_t>(to_id);
}

}  // namespace path_planner
}  // namespace rpp
