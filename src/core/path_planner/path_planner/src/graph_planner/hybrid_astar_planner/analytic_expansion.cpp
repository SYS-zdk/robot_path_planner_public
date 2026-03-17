
/**
 * *********************************************************
 *
 * @file: analytic_expansion.h
 * @brief: Analytic expansion using geometry methods
 * @author: Yang Haodong
 * @date: 2025-01-17
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/geometry/point.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "path_planner/graph_planner/hybrid_astar_planner/analytic_expansion.h"

using namespace rpp::common::geometry;

namespace rpp
{
namespace path_planner
{
template <typename NodeT>
AnalyticExpansion<NodeT>::AnalyticExpansion(const MotionModel& motion_model, const HybridSearchInfo& search_info)
  : motion_model_(motion_model), search_info_(search_info), collision_checker_(nullptr)
{
}

template <typename NodeT>
AnalyticExpansion<NodeT>::~AnalyticExpansion()
{
  for (auto node : expansions_node_)
  {
    delete node;
  }
  expansions_node_.clear();
}

template <typename NodeT>
void AnalyticExpansion<NodeT>::setCollisionChecker(const std::shared_ptr<CollisionChecker>& collision_checker)
{
  collision_checker_ = collision_checker;
}

template <typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::tryAnalyticExpansion(const NodePtr& curr_node,
                                                                                          const NodePtr& goal_node)
{
  if (!collision_checker_ || !curr_node || !goal_node)
  {
    return NodePtr(nullptr);
  }

  // This must be a valid motion model for analytic expansion to be attempted
  if (motion_model_ == MotionModel::DUBIN || motion_model_ == MotionModel::REEDS_SHEPP ||
      motion_model_ == MotionModel::STATE_LATTICE)
  {
    const double resolution = collision_checker_->getCostmapROS()->getCostmap()->getResolution();
    if (!(resolution > 0.0))
    {
      return NodePtr(nullptr);
    }

    const double max_len_cells = search_info_.analytic_expansion_max_length / resolution;
    // Cheap lower-bound gate: if even Euclidean is too large, don't attempt.
    const auto& curr_pose = NodeT::getCoords(curr_node->index());
    const double dx = curr_pose.x() - goal_node->pose().x();
    const double dy = curr_pose.y() - goal_node->pose().y();
    if (std::hypot(dx, dy) > max_len_cells)
    {
      return NodePtr(nullptr);
    }

    Points3d motion_path;
    Point3d from(curr_node->pose().x(), curr_node->pose().y(),
                 curr_node->motion_table.getAngleFromBin(curr_node->pose().theta()));
    Point3d to(goal_node->pose().x(), goal_node->pose().y(),
               goal_node->motion_table.getAngleFromBin(goal_node->pose().theta()));
    if (curr_node->motion_table.curve_gen->generation(from, to, motion_path))
    {
      // Enforce max-length using the actual analytic curve length.
      const double curve_len_cells = curr_node->motion_table.curve_gen->distance(motion_path);
      if (!(curve_len_cells <= max_len_cells))
      {
        return NodePtr(nullptr);
      }

      for (auto node : expansions_node_)
      {
        delete node;
      }
      expansions_node_.clear();

      NodePtr prev = curr_node;
      for (size_t i = 1; i < motion_path.size() - 1; i++)
      {
        auto& pose = motion_path[i];
        pose.setTheta(curr_node->motion_table.getOrientationBin(pose.theta()));
        const unsigned int index_2d =
            static_cast<uint64_t>(pose.x()) +
            static_cast<uint64_t>(pose.y()) * static_cast<uint64_t>(curr_node->motion_table.map_width);
        if (collision_checker_->inCollision(index_2d))
        {
          return NodePtr(nullptr);
        }
        NodePtr n = new NodeT(NodeT::getIndex(pose));
        n->parent = prev;
        n->visited();
        expansions_node_.push_back(n);
        prev = n;
      }
      goal_node->parent = prev;
      goal_node->visited();
      return goal_node;
    }
  }

  // No valid motion model - return nullptr
  return NodePtr(nullptr);
}

template class AnalyticExpansion<NodeHybrid>;
}  // namespace path_planner
}  // namespace rpp