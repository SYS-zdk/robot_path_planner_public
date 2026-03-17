
/**
 * *********************************************************
 *
 * @file: ramer_douglas_peucker.cpp
 * @brief: ramer douglas peucker for path downsampling
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/geometry/line_segment2d.h"
#include "path_planner/path_prune/ramer_douglas_peucker.h"

namespace rpp
{
namespace path_planner
{
/**
 * @brief Empty constructor
 */
RDPPathProcessor::RDPPathProcessor() : delta_(0.0){};

/**
 * @brief Constructor
 * @param delta The error threshold
 */
RDPPathProcessor::RDPPathProcessor(double delta) : delta_(delta){};

/**
 * @brief Process the path according to a certain expectation
 * @param path_in The path to process
 * @param path_out The processed path
 */
void RDPPathProcessor::process(const Points3d& path_in, Points3d& path_out)
{
  path_out.clear();
  int max_idx = -1;
  double max_dist = -1.0;
  int path_size = static_cast<int>(path_in.size());
  rpp::common::geometry::LineSegment2d line({ path_in[0].x(), path_in[0].y() },
                                            { path_in[path_size - 1].x(), path_in[path_size - 1].y() });
  for (int i = 1; i < path_size - 1; i++)
  {
    double d = line.distanceTo({ path_in[i].x(), path_in[i].y() });
    if (d > max_dist)
    {
      max_dist = d;
      max_idx = i;
    }
  }

  if (max_dist > delta_)
  {
    Points3d left_pts, right_pts;
    left_pts.reserve(max_idx + 1);
    right_pts.reserve(path_size - max_idx);
    for (int i = 0; i <= max_idx; i++)
    {
      left_pts.emplace_back(path_in[i].x(), path_in[i].y());
    }
    for (int i = max_idx; i < path_size; i++)
    {
      right_pts.emplace_back(path_in[i].x(), path_in[i].y());
    }

    Points3d left_result, right_result;
    process(left_pts, left_result);
    process(right_pts, right_result);
    path_out.insert(path_out.end(), left_result.begin(), left_result.end() - 1);
    path_out.insert(path_out.end(), right_result.begin(), right_result.end());
  }
  else
  {
    path_out.emplace_back(path_in[0].x(), path_in[0].y());
    path_out.emplace_back(path_in[path_size - 1].x(), path_in[path_size - 1].y());
  }
}
}  // namespace path_planner
}  // namespace rpp