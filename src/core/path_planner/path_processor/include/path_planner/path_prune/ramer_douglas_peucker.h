
/**
 * *********************************************************
 *
 * @file: ramer_douglas_peucker.h
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
#ifndef RMP_PATH_PLANNER_PATH_PROCESSOR_PATH_PRUNE_RDP_H_
#define RMP_PATH_PLANNER_PATH_PROCESSOR_PATH_PRUNE_RDP_H_

#include "path_planner/path_processor/path_processor.h"

namespace rpp
{
namespace path_planner
{
class RDPPathProcessor : public PathProcessor
{
public:
  /**
   * @brief Empty constructor
   */
  RDPPathProcessor();

  /**
   * @brief Constructor
   * @param delta The error threshold
   */
  RDPPathProcessor(double delta);

  /**
   * @brief  Destructor
   */
  virtual ~RDPPathProcessor() = default;

  /**
   * @brief Process the path according to a certain expectation
   * @param path_in The path to process
   * @param path_out The processed path
   */
  void process(const Points3d& path_in, Points3d& path_out);

private:
  double delta_;  // the error threshold
};
}  // namespace path_planner
}  // namespace rpp

#endif