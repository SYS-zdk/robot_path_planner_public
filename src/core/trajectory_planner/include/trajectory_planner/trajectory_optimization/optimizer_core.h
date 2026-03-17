
/***********************************************************
 *
 * @file: optimizer_core.h
 * @brief: Trajectory optimization core
 * @author: Yang Haodong
 * @date: 2025-01-17
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_OPTIMIZER_CORE_H_
#define RMP_TRAJECTORY_OPTIMIZATION_OPTIMIZER_CORE_H_

#include "trajectory_planner/trajectory_optimization/minimumsnap_optimizer/minimumsnap_optimizer.h"
#include "common/safety_corridor/ackermann_config.h"
#include "trajectory_planner/trajectory_optimization/conjugate_optimizer/conjugate_optimizer.h"
#include "trajectory_planner/trajectory_optimization/lbfgs_optimizer/lbfgs_optimizer.h"
// NOTE: Some optimizers (SplineTrajectoryOptimizer / MincoSplineOptimizer) are
// intentionally not shipped in this public repository. They are not included
// here to keep the public build self-contained.

#endif