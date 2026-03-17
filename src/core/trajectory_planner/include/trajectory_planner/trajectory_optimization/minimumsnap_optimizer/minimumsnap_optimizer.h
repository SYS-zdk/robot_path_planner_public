
/***********************************************************
 *
 * @file: minimumsnap_optimizer.h
 * @brief: Trajectory optimization using minimumsnap methods // MinimumsnapOptimizer 的类，它实现了基于最小快照（minimum snap）方法的轨迹优化
 * @author: Yang Haodong
 * @date: 2024-09-20
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_MINIMUMSNAP_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_MINIMUMSNAP_OPTIMIZER_H_

#include <Eigen/Dense>

#include "common/geometry/polygon2d.h"
#include "common/geometry/curve/quintic_polynomial.h"

#include "trajectory_planner/trajectory_optimization/optimizer.h"

#include "common/safety_corridor/ackermann_config.h"

namespace rpp
{
namespace common
{
namespace safety_corridor
{
class ConvexSafetyCorridor;
}
}  // namespace common
}  // namespace rpp

namespace rpp
{
namespace trajectory_optimization
{
class MinimumsnapOptimizer : public Optimizer //继承自一个基类 Optimizer，这可能包含一些通用的优化方法和属性
{
private:
  using Polygon2d = rpp::common::geometry::Polygon2d;
  using PolyCurve = rpp::common::geometry::QuinticPolynomial;
  using State2d = std::array<double, 6>;  // x, vx, ax, py, vy, ay；std::array 是一个一维数组，存储方式是连续的内存块，没有特定的“行”或“列”概念，只是一个简单的线性数组；
//Eigen::Matrix 是 Eigen 库中的一个矩阵类，用于表示矩阵和向量。默认情况下，Eigen::Matrix<double, 6> 是一个 6x1 的列向量，即 6 行 1 列
  rpp::AckermannConfig ackermann_config_;
public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param costmap_ros costmap ROS wrapper
   * @param max_iter the maximum iterations for optimization
   * @param vel_max the maximum velocity (m/s)
   * @param acc_max the maximum acceleration (m/s2)
   * @param jerk_max the maximum jerk (m/s3)
   * @param safety_range The safety range to limit polygon space [m]
   */
  MinimumsnapOptimizer(int max_iter, double vel_max, double acc_max, double jerk_max); //含参构造，初始化优化的最大迭代次数、最大速度、最大加速度和最大急动度
  MinimumsnapOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int max_iter, double vel_max, double acc_max,
                       double jerk_max, double safety_range, const rpp::AckermannConfig& ackermann_cfg); //含参构造，带成本地图和安全范围
  MinimumsnapOptimizer(Points3d obstacles, int max_iter, double vel_max, double acc_max, double jerk_max,
                       double safety_range = 0.5); //含参构造，带障碍物和安全范围
  ~MinimumsnapOptimizer() = default;

  /**
   * @brief Running trajectory optimization
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool run(const Points3d& waypoints); //运行轨迹优化，接收路径点集合，生成优化后的轨迹
  bool run(const Trajectory3d& traj); //运行轨迹优化，接收初始轨迹，生成优化后的轨迹

  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  bool getTrajectory(Trajectory3d& traj); //获取优化后的轨迹

  /**
   * @brief Get the convex safety corridor
   * @param polygons the convex safety corridor
   */
  const std::vector<Polygon2d>& getPolygons() const; //获取凸安全走廊的多边形集合

private:
  /**
   * @brief trajectory optimization executor
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failr_ed
   */
  bool optimize(const Points3d& waypoints); //执行轨迹优化

  /**
   * @brief solve the minimumsnap qp problem using osqp
   * @param start_state start point with <px, vx, ax, py, vy, ay>
   * @param target_state target point with <px, vx, ax, py, vy, ay>
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool solve(const State2d& start_state, const State2d& target_state, const Points3d& waypoints); //求解优化snap的QP问题
  /**
   * @brief auxiliary matrix
   * @param dt_i time allocation of segment i
   * @return A^+_i
   */
  Eigen::MatrixXd A_i_plus(double dt_i); //计算辅助矩阵A_i~+

  /**
   * @brief auxiliary matrix
   * @param dt_i time allocation of segment i
   * @return A^-_i
   */
  Eigen::MatrixXd A_i_minus(double dt_i); ////计算辅助矩阵A_i~-

private:
  int max_iter_;      // the maximum iterations for optimization 优化的最大迭代次数
  int k_;             // segment number for piecewise curve 分段曲线的段数
  int dim_;           // dimension 维度
  double vel_max_;    // the maximum velocity (m/s) 最大速度（m/s）
  double acc_max_;    // the maximum acceleration (m/s2) 最大加速度（m/s²）
  double jerk_max_;   // the maximum jerk (m/s3) jerk_max_：最大急动度（m/s³）
  const int n_{ 5 };  // polynomial order 多项式阶数（固定为 5） 初始化列表语法（也称为统一初始化或列表初始化）。这种语法允许在定义变量时直接初始化变量，而不需要使用赋值操作符。
  const int r_{ 4 };  // smooth order (4 for minimum snap, 3 for minimum jerk) r_：平滑阶数（4 表示最小快照，3 表示最小急动）

  std::vector<Polygon2d> polygons_; //凸安全走廊的多边形集合
  std::vector<double> time_allocations_; //每段的时间分配
  std::vector<std::vector<PolyCurve>> trajectories_; //优化后的轨迹集合
  std::unique_ptr<rpp::common::safety_corridor::ConvexSafetyCorridor> safety_corridor_; //凸安全走廊对象
};
}  // namespace trajectory_optimization
}  // namespace rpp

#endif