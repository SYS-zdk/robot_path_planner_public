
/***********************************************************
 *
 * @file: optimizer.h
 * @brief: Trajectory optimization //Optimizer 的抽象基类，用于轨迹优化。它提供了一些基本的接口和方法，用于实现具体的轨迹优化算法
 * @author: Yang Haodong
 * @date: 2023-12-29
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_OPTIMIZER_H_

#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/point.h"
#include "common/structure/trajectory.h"

namespace rpp
{
namespace trajectory_optimization
{
class Optimizer
{
protected:
  using Point2d = rpp::common::geometry::Point2d;
  using Points2d = rpp::common::geometry::Points2d;
  using Point3d = rpp::common::geometry::Point3d;
  using Points3d = rpp::common::geometry::Points3d;
  using Trajectory2d = rpp::common::structure::Trajectory2d;
  using Trajectory3d = rpp::common::structure::Trajectory3d;

public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param costmap_ros costmap ROS wrapper
   */
  Optimizer(); //默认构造函数，不初始化成本地图
  Optimizer(costmap_2d::Costmap2DROS* costmap_ros); //参数 costmap_ros 是成本地图的 ROS 包装器，用于初始化 costmap_ros_ 成员变量

  /**
   * @brief Destroy the trajectory optimizer object
   */
  virtual ~Optimizer() = default;

  /**
   * @brief Running trajectory optimization
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  virtual bool run(const Points3d& waypoints) = 0; //运行轨迹优化，接收路径点集合，生成优化后的轨迹
  virtual bool run(const Trajectory3d& traj) = 0; //运行轨迹优化，接收初始轨迹，生成优化后的轨迹

  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  virtual bool getTrajectory(Trajectory3d& traj) = 0; //获取优化后的轨迹

protected:
  /**
   * @brief Judge whether the grid(x, y) is inside the map
   * @param x grid coordinate x
   * @param y grid coordinate y
   * @return true if inside the map else false
   */
  bool _insideMap(unsigned int x, unsigned int y); //判断某个网格坐标是否在地图范围内，参数 x 和 y 是网格坐标；返回 true 表示在地图范围内，false 表示不在地图范围内
  /**
   * @brief Judge whether the grid(x, y) is inside the map
   * @param x world coordinate x
   * @param y world coordinate y
   * @return true if inside the map else false
   */
  bool _insideMap(double x, double y); //判断某个世界坐标是否在地图范围内 //参数 x 和 y 是世界坐标

protected:
  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap ROS wrapper //成本地图的 ROS 包装器，用于获取障碍物信息
  unsigned int nx_, ny_, map_size_;        // map size //地图的尺寸信息
};

}  // namespace trajectory_optimization
}  // namespace rpp
#endif