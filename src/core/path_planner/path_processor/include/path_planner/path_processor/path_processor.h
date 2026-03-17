
/**
 * *********************************************************
 *
 * @file: path_processor.h
 * @brief: Path pre-processing or post-processing
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_PATH_PROCESSOR_PATH_PROCESSOR_H_
#define RMP_PATH_PLANNER_PATH_PROCESSOR_PATH_PROCESSOR_H_

#include "common/geometry/point.h"

namespace rpp
{
namespace path_planner
{
class PathProcessor
{
protected:
  using Point3d = rpp::common::geometry::Point3d;
  using Points3d = rpp::common::geometry::Points3d;

public:
  /**
   * @brief Empty constructor
   */
  PathProcessor() = default;

  /**
   * @brief  Destructor
   */
  virtual ~PathProcessor() = default;

  /**
   * @brief Process the path according to a certain expectation
   * @param path_in The path to process
   * @param path_out The processed path
   */
  virtual void process(const Points3d& path_in, Points3d& path_out) = 0;

  void setObstacles(const Points3d& obstacles);
  void setObstacles(Points3d&& obstacles);

protected:
  Points3d obstacles_;
};
}  // namespace path_planner
}  // namespace rpp

#endif