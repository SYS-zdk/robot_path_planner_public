
/***********************************************************
 *
 * @file: minimumsnap_optimizer.cpp
 * @brief: Trajectory optimization using minimumsnap methods
 * @author: Yang Haodong
 * @date: 2024-09-20
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <chrono>
#include <osqp/osqp.h>

#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "common/geometry/curve/bezier_curve.h"
#include "common/safety_corridor/convex_safety_corridor.h"

#include "path_planner/path_prune/ramer_douglas_peucker.h"
#include "trajectory_planner/trajectory_optimization/minimumsnap_optimizer/time_allocation.h"
#include "trajectory_planner/trajectory_optimization/minimumsnap_optimizer/minimumsnap_optimizer.h"
#include "common/safety_corridor/ackermann_config.h"

using namespace rpp::path_planner;

namespace
{
constexpr double kPruneDelta = 0.25;
constexpr double kPruneMaxInterval = 5.0;
constexpr int kTimeSamples = 100;
}  // namespace

namespace rpp
{
namespace trajectory_optimization
{
/**
 * @brief Construct a new trajectory optimizer object
 * @param costmap_ros costmap ROS wrapper
 * @param max_iter the maximum iterations for optimization
 * @param vel_max the maximum velocity (m/s)
 * @param acc_max the maximum acceleration (m/s2)
 * @param jerk_max the maximum jerk (m/s3)
 * @param safety_range The safety range to limit polygon space [m]
 */
MinimumsnapOptimizer::MinimumsnapOptimizer(int max_iter, double vel_max, double acc_max, double jerk_max)
  : Optimizer(), max_iter_(max_iter), vel_max_(vel_max), acc_max_(acc_max), jerk_max_(jerk_max)
{
}

MinimumsnapOptimizer::MinimumsnapOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int max_iter, double vel_max,
                                           double acc_max, double jerk_max, double safety_range,
                                           const rpp::AckermannConfig& ackermann_cfg)
  : Optimizer(costmap_ros)
  , max_iter_(max_iter)
  , vel_max_(vel_max)
  , acc_max_(acc_max)
  , jerk_max_(jerk_max)
  , ackermann_config_(ackermann_cfg)
{
  // safety corridor
  safety_corridor_ = std::make_unique<rpp::common::safety_corridor::ConvexSafetyCorridor>(costmap_ros, safety_range, ackermann_cfg);
}

MinimumsnapOptimizer::MinimumsnapOptimizer(Points3d obstacles, int max_iter, double vel_max, double acc_max,
                                           double jerk_max, double safety_range)
  : Optimizer(), max_iter_(max_iter), vel_max_(vel_max), acc_max_(acc_max), jerk_max_(jerk_max)
{
  // safety corridor
  safety_corridor_ = std::make_unique<rpp::common::safety_corridor::ConvexSafetyCorridor>(obstacles, safety_range);
}

/**
 * @brief Running trajectory optimization //接收一系列路径点（waypoints），并生成优化后的轨迹
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool MinimumsnapOptimizer::run(const Points3d& waypoints)
{
  // dimenstion 设置维度
  dim_ = 2;

  // key-points
  Points3d key_points;
  auto pruner = std::make_shared<RDPPathProcessor>(kPruneDelta); //使用 RDP（Ramer-Douglas-Peucker）算法对路径点进行简化，提取关键点；kPruneDelta 是 RDP 算法的阈值，用于控制路径点的简化程度
  pruner->process(waypoints, key_points);
  k_ = static_cast<int>(key_points.size()) - 1; //计算路径段数k,路径段数等于关键点数量减一
  trajectories_ = std::vector<std::vector<PolyCurve>>(k_, std::vector<PolyCurve>(dim_));  //初始化轨迹集合 trajectories_，每个路径段包含两个维度x，y的五次多项式
  //std::vector<std::vector<PolyCurve>>这是一个二维向量，外层向量的每个元素是一个内层向量，内层向量的每个元素是一个 PolyCurve 对象；k_ 是一个整数，表示轨迹的段数（即外层向量的大小）；dim_ 是一个整数，表示每个轨迹段的维度数（即内层向量的大小）

  // time allocations
  TimeAllocator::trapezoidalAllocation<Point3d>(key_points, vel_max_, acc_max_, time_allocations_); //使用梯形速度分配方法为每个路径段分配时间,vel_max_ 和 acc_max_ 分别是最大速度和最大加速度

  // convex safety corridor
  safety_corridor_->decompose(key_points, polygons_); //使用 safety_corridor_ 对象生成凸安全走廊 //polygons_ 存储生成的多边形集合

  return optimize(key_points); //调用 optimize 方法，对提取的关键点进行优化,返回优化结果
}

bool MinimumsnapOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

/**
 * @brief Get the optimized trajectory
 * @param traj the trajectory buffer
 * @return true if optimizes successfully, else failed
 */
bool MinimumsnapOptimizer::getTrajectory(Trajectory3d& traj)
{
  traj.reset(kTimeSamples);
  std::vector<double> time_segments;

  double time_sum = 0.0;
  for (const double time : time_allocations_)
  {
    time_sum += time;
    time_segments.push_back(time_sum);
  }

  int i = 0;
  double t = 0.0;
  while (t < time_sum)
  {
    auto iter = std::lower_bound(time_segments.begin(), time_segments.end(), t);
    int index = iter - time_segments.begin();
    double time_point = index > 0 ? t - time_segments[index - 1] : t;
    double time_horizon = time_allocations_[index];
    double u = time_point / time_horizon;

    traj.time.push_back(t);
    traj.position.emplace_back(trajectories_[index][0].x(u), trajectories_[index][1].x(u));
    traj.velocity.emplace_back(trajectories_[index][0].dx(u) / time_horizon,
                               trajectories_[index][1].dx(u) / time_horizon);
    traj.acceletation.emplace_back(trajectories_[index][0].ddx(u) / (time_horizon * time_horizon),
                                   trajectories_[index][1].ddx(u) / (time_horizon * time_horizon));
    i++;
    t += time_sum / kTimeSamples;
  }

  return true;
}

/**
 * @brief Get the convex safety corridor
 * @param polygons the convex safety corridor
 */
const std::vector<MinimumsnapOptimizer::Polygon2d>& MinimumsnapOptimizer::getPolygons() const
{
  return polygons_;
}

/**
 * @brief trajectory optimization executor
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool MinimumsnapOptimizer::optimize(const Points3d& waypoints)
{
  State2d start_state = { waypoints[0].x(), 0.0, 0.0, waypoints[0].y(), 0.0, 0.0 }; //初始化起始状态和目标状态，包括位置、速度和加速度
  State2d target_state = { waypoints.back().x(), 0.0, 0.0, waypoints.back().y(), 0.0, 0.0 }; //back() 返回路径点集合中的最后一个点，即目标点

  int iter = 0;
  auto start = std::chrono::high_resolution_clock::now(); //获取当前时间点 //std::chrono::high_resolution_clock这是一个高分辨率时钟，通常用于测量时间间隔（例如程序的运行时间）；now()获取当前时间点，返回一个 std::chrono::time_point 对象，表示当前时刻
  while (iter < max_iter_)
  {
    iter += 1;

    // optimization
    solve(start_state, target_state, waypoints);
 
    // update time allocations 检查轨迹的每个段是否满足速度、加速度和抖动的约束条件，如果某个段不满足约束条件，它会调整更新该段的时间分配，以确保轨迹的可行性和平滑性
    bool condition_fit = true; //被初始化为 true，表示所有轨迹段都满足约束条件
    for (int seg = 0; seg < k_; seg++) //循环遍历每个轨迹段 seg
    {
      const double dt = time_allocations_[seg]; //dt 表示当前段的时间分配
      const double dt_2 = dt * dt; //表示 dt 的平方
      const double dt_3 = dt * dt * dt; //表示 dt 的立方
      double v_max = vel_max_, a_max = acc_max_, j_max = jerk_max_; //v_max、a_max 和 j_max 分别表示当前段的最大速度、加速度和抖动，初始值为约束值
      double t = 0.0;

      double curvature_max_seg =0.0;//---------------

      while (t < dt) //使用 while 循环在当前段的时间范围内进行采样，步长为 dt / kTimeSamples
      {
        double u = t / dt; //u 是当前时间 t 的归一化值，范围为 [0, 1)，通过归一化，u 可以用于在轨迹段内进行插值计算
        
        v_max = std::max(
            v_max, std::max(std::fabs(trajectories_[seg][0].dx(u)) / dt, std::fabs(trajectories_[seg][1].dx(u)) / dt)); //dx(u) 表示轨迹在归一化时间u处的X和Y的分速度，为了将速度从归一化时间单位转换为实际的时间单位，需要除以时间间隔
        a_max = std::max(a_max, std::max(std::fabs(trajectories_[seg][0].ddx(u)) / dt_2,
                                         std::fabs(trajectories_[seg][1].ddx(u)) / dt_2));
        j_max = std::max(j_max, std::max(std::fabs(trajectories_[seg][0].dddx(u)) / dt_3,
                                         std::fabs(trajectories_[seg][1].dddx(u)) / dt_3)); //解释：1.x(u)=x(t/dt) 2.x'(u)=dx/du=(dx/dt)*(dt/du)=x'(t)*dt
        t += dt / kTimeSamples; //用于更新时间变量 t，以便在下一个循环迭代中继续采样
      }

      // feasibility check 可行性检查 检查当前段的最大速度、加速度和加加速度是否超过约束值
      if (std::fabs(v_max - vel_max_) > 1e-2 || std::fabs(a_max - acc_max_) > 1e-2 ||
          std::fabs(j_max - jerk_max_) > 1e-2)
      {
        double ratio = std::max(1.0, std::max(v_max / vel_max_, std::max(std::pow(a_max / acc_max_, 1.0 / 2.0), //确保 ratio 至少为 1.0，避免时间分配减少
                                                                         std::pow(j_max / jerk_max_, 1.0 / 3.0)))); //如果某段轨迹不满足约束条件，则计算一个调整比例 ratio；v_max / vel_max_这个比值会大于 1，表示速度超出了约束；这里的平方根操作是因为加速度与时间的平方成正比，调整时间分配时需要考虑这种关系，值会大于 1，表示加速度超出了约束
        time_allocations_[seg] *= ratio; //ratio 是基于最大速度、加速度和加加速度的调整比例，确保调整后的时间分配能够满足约束条件。
        condition_fit = false; //设置为 false，表示需要重新检查所有轨迹段
      }//ratio 的计算结果是一个大于或等于 1.0 的值，表示需要将当前轨迹段的时间分配增加多少倍，以确保速度、加速度和加加速度都满足约束条件
    }

    if (condition_fit) //表示轨迹优化是否成功完成。
    {
      auto finish = std::chrono::high_resolution_clock::now(); //使用 std::chrono::high_resolution_clock 获取当前时间点
      std::chrono::duration<double> elapsed = finish - start; //elapsed 是一个 std::chrono::duration 类型的对象，表示时间间隔;std::chrono::duration<double> 表示时间间隔以秒为单位，返回一个 double 类型的值
      R_DEBUG << "Minimumsnap optimization time: " << elapsed.count(); //将elapsed结果输出到日志
      return true;
    }
  }
// while (iter < max_iter_)
//   {
//     iter += 1;

//     // 核心优化步骤（保持不变）
//     solve(start_state, target_state, waypoints);

//     bool condition_fit = true;
//     const double curvature_max_ = 0.83;    // 最大允许曲率（根据机器人物理限制设定）
//     const double curvature_tol_ = 0.2;   // 曲率容忍度（10%）
//     const double w_curvature = 0.8;        // 曲率权重系数（可调参数）

//     // 分段约束检查
//     for (int seg = 0; seg < k_; seg++)
//     {
//         const double dt = time_allocations_[seg];
//         const double dt_2 = dt * dt;
//         const double dt_3 = dt_2 * dt;
        
//         // 初始化约束值
//         double v_max = 0.0, a_max = 0.0, j_max = 0.0;
//         double curvature_max_seg = 0.0;

//         // 时间采样检查（新增曲率跟踪）
//         const int kTimeSamples = 20;  // 每段采样点数
//         for (int i = 0; i <= kTimeSamples; ++i) {
//             const double u = static_cast<double>(i) / kTimeSamples;
            
//             // 速度约束检查
//             const double vx = std::fabs(trajectories_[seg][0].dx(u)) / dt;
//             const double vy = std::fabs(trajectories_[seg][1].dx(u)) / dt;
//             v_max = std::max(v_max, std::max(vx, vy));

//             // 加速度约束检查
//             const double ax = std::fabs(trajectories_[seg][0].ddx(u)) / dt_2;
//             const double ay = std::fabs(trajectories_[seg][1].ddx(u)) / dt_2;
//             a_max = std::max(a_max, std::max(ax, ay));

//             // Jerk约束检查
//             const double jx = std::fabs(trajectories_[seg][0].dddx(u)) / dt_3;
//             const double jy = std::fabs(trajectories_[seg][1].dddx(u)) / dt_3;
//             j_max = std::max(j_max, std::max(jx, jy));

//             // 曲率计算（新增核心逻辑）-------------------------------
//             // 计算实际导数（考虑时间缩放）
//             const double dx = trajectories_[seg][0].dx(u) / dt;    // 速度x分量
//             const double dy = trajectories_[seg][1].dx(u) / dt;    // 速度y分量
//             const double ddx = trajectories_[seg][0].ddx(u) / dt_2; // 加速度x分量
//             const double ddy = trajectories_[seg][1].ddx(u) / dt_2; // 加速度y分量

//             // 曲率公式：κ = |dx*ddy - dy*ddx| / (dx² + dy²)^1.5
//             const double numerator = std::abs(dx*ddy - dy*ddx);
//             const double denominator = std::pow(dx*dx + dy*dy, 1.5);
            
//             if (denominator > 1e-6) {  // 数值稳定性保护
//                 const double curvature = numerator / denominator;
//                 curvature_max_seg = std::max(curvature_max_seg, curvature);
//             }
//         }

//         // 约束检查与自适应调整 --------------------------------------
//         bool need_adjust = false;
//         double ratio = 1.0;

//         // 约束违反判断（改进容差机制）
//         const bool velocity_violation = (v_max > vel_max_ * (1.0 + 0.05));     // 5%速度容差
//         const bool acceleration_violation = (a_max > acc_max_ * (1.0 + 0.1));  // 10%加速度容差
//         const bool jerk_violation = (j_max > jerk_max_ * (1.0 + 0.15));        // 15%jerk容差
//         const bool curvature_violation = (curvature_max_seg > curvature_max_ * (1.0 + curvature_tol_));

//         if (velocity_violation || acceleration_violation || jerk_violation || curvature_violation)
//         {
//             // 动态调整比例计算（带曲率权重融合）
//             const double ratio_vel = v_max / vel_max_;
//             const double ratio_acc = std::sqrt(a_max / acc_max_);
//             const double ratio_jerk = std::cbrt(j_max / jerk_max_);
//             double ratio_curvature = 1.0;
            
//             if (curvature_max_ > 1e-6) {
//                 ratio_curvature = std::cbrt(curvature_max_seg / curvature_max_);
//             }

//             // 混合权重计算：ratio = max(纯约束比例, 曲率加权比例)
//             ratio = std::max(
//                 std::max({ratio_vel, ratio_acc, ratio_jerk}),
//                 w_curvature * ratio_curvature + (1.0 - w_curvature)
//             );
//             ratio = std::max(1.0, ratio);  // 保证时间不缩短

//             // 应用时间调整
//             time_allocations_[seg] *= ratio;
//             R_DEBUG << "Segment " << seg << " time scaled by " << ratio 
//                     << " (v=" << v_max << "/a=" << a_max 
//                     << "/j=" << j_max << "/κ=" << curvature_max_seg << ")";
            
//             condition_fit = false;
//         }
//     }

//     // 收敛检查与性能监控 --------------------------------------------
//     if (condition_fit) {
//         auto finish = std::chrono::high_resolution_clock::now();
//         std::chrono::duration<double> elapsed = finish - start;
//         R_INFO << "Optimization converged in " << iter << " iterations. Total time: " 
//                << elapsed.count() << "s";
//         return true;
//     } else {
//         R_DEBUG << "Re-optimizing with new time allocation...";
//     }
//   }

  return false;
}

/**
 * @brief solve the minimumsnap qp problem using osqp
 * @param start_state start point with <px, vx, ax, py, vy, ay>
 * @param target_state target point with <px, vx, ax, py, vy, ay>
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool MinimumsnapOptimizer::solve(const State2d& start_state, const State2d& target_state, const Points3d& waypoints)
{
  // Bernstein transform
  auto B = rpp::common::geometry::Bernstein::matrix(5); //生成一个 5 阶 Bernstein 矩阵。Bernstein 多项式用于将多项式系数从控制点表示转换为标准多项式表示

  // objective function
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_ * k_ * (n_ + 1), dim_ * k_ * (n_ + 1)); //总系数矩阵Q  ，zero用于创建一个指定大小的零矩阵 /solve()是MinimumsnapOptimizer类的成员函数，dim_这些是同类里的成员变量
  Eigen::MatrixXd Q_sub = Eigen::MatrixXd::Zero(n_ + 1, n_ + 1); //子系数矩阵Q_sub 6*6 类的成员函数可以直接访问类的私有成员变量，这是类封装机制的一部分，私有成员变量虽然不能被类外部的代码直接访问，但类内部的成员函数可以自由地访问和修改这些私有成员变量

  for (int i = r_; i < n_ + 1; i++) //填充一个子矩阵Q_sub，子矩阵用于构建优化问题中的目标函数矩阵Q，r_已经设置为4即优化snap，n_=5即5阶多项式
  {
    for (int j = r_; j < n_ + 1; j++)
    {
      Q_sub(i, j) = rpp::common::math::factorial(i) / rpp::common::math::factorial(i - r_) *
            rpp::common::math::factorial(j) / rpp::common::math::factorial(j - r_) / (i + j - 2 * r_ + 1);
    } //factorial(i)即i的阶乘
  }

  for (int seg = 0; seg < k_; seg++) //将每个轨迹段的目标函数子矩阵Q_sub填充到总系数矩阵Q
  {
    for (int miu = 0; miu < dim_; miu++)
    {
      int s = miu * k_ * (n_ + 1) + seg * (n_ + 1); //计算起始索引，miu：当前维度的索引，seg：当前轨迹段的索引，s：当前子块在系数矩阵Q中的起始索引
      Q.block(s, s, n_ + 1, n_ + 1) = 1.0 / std::pow(time_allocations_[seg], 2 * r_ - 3) * B.transpose() * Q_sub * B; //将Q_sub填充到Q中特定位置
    } //Q.block(s, s, n_ + 1, n_ + 1)：具体来说，它从索引 s 开始，提取 n_ + 1 行和 n_ + 1 列的子矩阵，并将其赋值为计算得到的矩阵。；1.0 / std::pow(time_allocations_[seg], 2 * r_ - 3)：时间缩放因子，用于调整目标函数的权重，确保时间分配的影响
  } //B.transpose() * Q_sub * B，这个表达式的作用是将目标函数矩阵Q_sub从多项式系数空间转换到控制点空间

  // waypoints constatints
  Eigen::VectorXd l_b_p = -OSQP_INFTY * Eigen::VectorXd::Ones(dim_ * 3 * (k_ + 1)); //OSQP_INFTY：一个表示无穷大的常量，通常用于表示无界约束
  Eigen::VectorXd u_b_p = OSQP_INFTY * Eigen::VectorXd::Ones(dim_ * 3 * (k_ + 1)); //Eigen::VectorXd::Ones(size)：创建一个大小为 size 的列向量，所有元素初始化为 1
  Eigen::MatrixXd A_p = Eigen::MatrixXd::Zero(dim_ * 3 * (k_ + 1), dim_ * k_ * (n_ + 1)); //Eigen::MatrixXd::Zero(rows, cols)：创建一个大小为 rows 行和 cols 列的零矩阵

  // start / target pva
  Eigen::MatrixXd A_start = A_i_minus(time_allocations_[0]); //初始化无界约束向量
  Eigen::MatrixXd A_target = A_i_plus(time_allocations_.back());
  for (int miu = 0; miu < dim_; miu++) //设置线性约束，l_b_p 和 u_b_p 是下界和上界向量，用于定义优化变量的取值范围
  {
    l_b_p(miu * 3 * (k_ + 1)) = start_state[miu * 3]; //start_state[0]\start_state[3]都是起始点的位置分量，即px、py，分别存入列向量l_b_p(0),l_b_p(3)的位置
    l_b_p(miu * 3 * (k_ + 1) + 1) = start_state[miu * 3 + 1]; //start_state[0]\start_state[3]，vx\vy;
    l_b_p(miu * 3 * (k_ + 1) + 2) = start_state[miu * 3 + 2]; //ax,ay
    u_b_p(miu * 3 * (k_ + 1)) = start_state[miu * 3];
    u_b_p(miu * 3 * (k_ + 1) + 1) = start_state[miu * 3 + 1];
    u_b_p(miu * 3 * (k_ + 1) + 2) = start_state[miu * 3 + 2];
    A_p.block(miu * 3 * (k_ + 1), miu * k_ * (n_ + 1), 3, n_ + 1) = A_start; //A_p 是约束矩阵，用于定义线性约束；block将矩阵 A_start 赋值给矩阵 A_p 的一个特定子块

    l_b_p(miu * 3 * (k_ + 1) + 3 * k_) = target_state[miu * 3];
    l_b_p(miu * 3 * (k_ + 1) + 3 * k_ + 1) = target_state[miu * 3 + 1];
    l_b_p(miu * 3 * (k_ + 1) + 3 * k_ + 2) = target_state[miu * 3 + 2];
    u_b_p(miu * 3 * (k_ + 1) + 3 * k_) = target_state[miu * 3];
    u_b_p(miu * 3 * (k_ + 1) + 3 * k_ + 1) = target_state[miu * 3 + 1];
    u_b_p(miu * 3 * (k_ + 1) + 3 * k_ + 2) = target_state[miu * 3 + 2];
    A_p.block(miu * 3 * (k_ + 1) + 3 * k_, (miu * k_ + k_ - 1) * (n_ + 1), 3, n_ + 1) = A_target; //起始索引(miu * 3 * (k_ + 1) + 3 * k_, (miu * k_ + k_ - 1) * (n_ + 1)),存放的子块大小(3, n_ + 1)即从起始索引位置开始的（3*6）大小的子块

    // inter - position 处理的是中间路点（waypoints）的约束条件，等式约束中的连续性约束
    for (int seg = 0; seg < k_ - 1; seg++)
    {
      l_b_p(miu * 3 * (k_ + 1) + 3 * seg + 3) = waypoints[seg + 1][miu];
      u_b_p(miu * 3 * (k_ + 1) + 3 * seg + 3) = waypoints[seg + 1][miu];
      A_p(miu * 3 * (k_ + 1) + 3 * seg + 3, miu * k_ * (n_ + 1) + seg * (n_ + 1) + n_) = 1.0; //设置中间路点的连续性约束，即必须要经过的路径点位置
    }
  }

  // smooth constraints //构建平滑性约束,连续性约束，位置、速度、加速度
  Eigen::MatrixXd A_s = Eigen::MatrixXd::Zero(dim_ * 3 * (k_ - 1), dim_ * k_ * (n_ + 1)); //初始化A_s：平滑性约束矩阵
  Eigen::VectorXd b_s = Eigen::VectorXd::Zero(dim_ * 3 * (k_ - 1)); //初始化b_s：平滑性约束的右侧向量
  for (int i = 0; i < k_ - 1; i++) //遍历所有连接点（k_ - 1 个连接点）
  {
    Eigen::MatrixXd A_prev = A_i_plus(time_allocations_[i]); //A_prev 是前一个路径段的终点约束矩阵，通过调用 A_i_plus 方法生成
    Eigen::MatrixXd A_next = -A_i_minus(time_allocations_[i + 1]); //A_next 是下一个路径段的起点约束矩阵，通过调用 A_i_minus 方法生成，并取负号
    for (int miu = 0; miu < dim_; miu++) //遍历每个维度（dim_）
    {
      int row_start = miu * 3 * (k_ - 1); //row_start 是当前维度在 A_s 矩阵中的起始行索引
      int col_start = miu * k_ * (n_ + 1); //col_start 是当前维度在 A_s 矩阵中的起始列索引
      A_s.block(row_start + 3 * i, col_start + i * (n_ + 1), 3, n_ + 1) = A_prev; //使用 Eigen::MatrixXd::block 方法将 A_prev 和 A_next 填充到 A_s 矩阵的相应位置
      A_s.block(row_start + 3 * i, col_start + (i + 1) * (n_ + 1), 3, n_ + 1) = A_next;
    }
  }

  // safety corridors //构建安全走廊线性不等式约束
  int ieq_idx = 0, ieq_nums = 0; //ieq_idx 是当前约束的索引;ieq_nums 是总的约束数量，等于所有多边形的顶点数乘以多项式的阶数n+1
  for (const auto& polygon : polygons_) //polygons_ 是一个多边形集合，每个多边形对应一个路径段的安全区域;每个多边形由一系列顶点组成，这些顶点定义了安全区域的边界
  {
    ieq_nums += polygon.num_points(); //遍历每个路径段的安全走廊
  }
  ieq_nums *= (n_ + 1);
  Eigen::MatrixXd A_safe = Eigen::MatrixXd::Zero(ieq_nums, dim_ * k_ * (n_ + 1)); //A_safe 是约束矩阵
  Eigen::VectorXd b_safe = Eigen::VectorXd::Zero(ieq_nums); //b_safe 是约束右侧的向量
  for (int seg = 0; seg < k_; seg++)
  {
    const auto& polygon = polygons_[seg];
    for (int h = 0; h < polygon.num_points(); h++)
    {
      const auto& pt = polygon.points()[h];
      const auto& next_pt = polygon.points()[polygon.next(h)];
      rpp::common::geometry::Vec2d n(next_pt.y() - pt.y(), -(next_pt.x() - pt.x())); //n 是这条边的法向量，表示从当前边指向安全区域内部的方向。法向量通过向量叉积计算得到，并归一化为单位向量。
      n.normalize();
      for (int i = 0; i < n_ + 1; i++)
      {
        b_safe(ieq_idx) = n.innerProd(pt); //计算法向量 n 与顶点 pt 的点积，作为约束右侧的值
        for (int miu = 0; miu < dim_; miu++)
        {
          A_safe(ieq_idx, (miu * k_ + seg) * (n_ + 1) + i) = miu == 0 ? n.x() : n.y(); //填充约束矩阵 A_safe,对于每个路径段的每个多项式系数（n_ + 1 个），在约束矩阵中填入法向量的分量。
        }
        ieq_idx += 1;
      } //这段代码通过构建安全走廊的线性不等式约束，确保生成的轨迹在优化过程中始终位于安全区域内
    } //利用多边形的法向量和顶点，将几何约束转换为优化问题中的线性不等式约束，从而在轨迹优化中实现碰撞检测和避障功能。
  }

  // Calculate kernel //是将目标函数矩阵Q转换为 OSQP 求解器所需的稀疏矩阵格式,OSQP 使用稀疏矩阵来表示二次规划问题的目标函数和约束条件，以提高求解效率
  std::vector<c_float> P_data; //初始化变量,P_data：存储稀疏矩阵的非零元素值
  std::vector<c_int> P_indices; //存储稀疏矩阵的非零元素对应的行索引
  std::vector<c_int> P_indptr; //存储稀疏矩阵的列指针，表示每一列的非零元素的起始位置
  int ind_P = 0; //当前非零元素的索引，用于填充 P_data 和 P_indices
  for (int col = 0; col < dim_ * k_ * (n_ + 1); ++col) //遍历目标函数矩阵Q,外层循环：遍历列,优化变量的总数，即矩阵Q大小
  {
    P_indptr.push_back(ind_P);
    for (int row = 0; row <= col; ++row) //内层循环：遍历行，Q是对称矩阵，只需要遍历上三角部分，row <= col 确保只处理上三角部分，将对称部分的值复制到下三角部分
    {
      P_data.push_back(Q(row, col) * 2.0); //填充稀疏矩阵数据
      P_indices.push_back(row); //记录当前非零元素的行索引，Q(row, col) * 2.0：OSQP 的目标函数形式为(1/2 * X_T*P*X)
      ind_P++; //更新非零元素的索引
    }
  }
  P_indptr.push_back(ind_P); //在处理每一列的非零元素后，将当前的 ind_P 值推入 P_indptr，表示下一列的起始位置

  // Calculate offset
  std::vector<c_float> q_data(dim_ * k_ * (n_ + 1), 0.0); //初始化，q_data 是目标函数的线性项系数向量，初始化为零向量，大小为 dim_ * k_ * (n_ + 1)，即优化变量的总数

  // Calculate affine constraints
  std::vector<c_float> A_data; //A_data：存储稀疏矩阵的非零元素值
  std::vector<c_int> A_indices; //A_indices：存储稀疏矩阵的非零元素对应的行索引
  std::vector<c_int> A_indptr; //A_indptr：存储稀疏矩阵的列指针，表示每一列的非零元素的起始位置
  int ind_A = 0; //当前非零元素的索引，用于填充 A_data 和 A_indices
  const int c_nums = dim_ * 3 * (k_ + 1) + dim_ * 3 * (k_ - 1) + ieq_nums; //c_nums 是总的约束数量，包括路径点约束、平滑性约束和安全走廊约束，dim_ * 3 * (k_ + 1)：路径点约束的数量，dim_ * 3 * (k_ - 1)：平滑性约束的数量，ieq_nums：安全走廊约束的数量
  for (int col = 0; col < dim_ * k_ * (n_ + 1); ++col) //遍历优化变量，遍历每一列，即优化变量的每一个维度
  {
    A_indptr.push_back(ind_A);
    for (int row = 0; row < c_nums; ++row) //遍历每一行，即每一个约束
    {
      double data; //存储当前列和行的约束值
      if (row < dim_ * 3 * (k_ + 1)) //根据 row 的范围，确定当前约束属于哪一类，row < dim_ * 3 * (k_ + 1)：路径点约束，从矩阵 A_p 中获取值
      {
        data = A_p(row, col);
      }
      else if (row < dim_ * 3 * (k_ + 1) + dim_ * 3 * (k_ - 1)) //row < dim_ * 3 * (k_ + 1) + dim_ * 3 * (k_ - 1)：平滑性约束，从矩阵 A_s 中获取值
      {
        data = A_s(row - dim_ * 3 * (k_ + 1), col);
      }
      else //其他：安全走廊约束，从矩阵 A_safe 中获取值
      {
        data = A_safe(row - dim_ * 3 * (k_ + 1) - dim_ * 3 * (k_ - 1), col);
      }

      if (std::fabs(data) > rpp::common::math::kMathEpsilon) //检查当前约束值 data 是否为非零值
      {
        A_data.push_back(data); //将 data 存储到 A_data 中
        A_indices.push_back(row); //将当前行索引 row 存储到 A_indices 中
        ind_A++; //更新非零元素的索引 ind_A
      }
    }
  }
  A_indptr.push_back(ind_A); //在处理完当前列的所有非零元素后，将当前的 ind_A 值推入 A_indptr，表示下一列的起始位置

  // Calculate constraints 计算二次规划问题中的线性约束的上下界，在二次规划问题中，线性约束通常表示为：l<=Ax<=u，A是约束矩阵，x是优化变量
  std::vector<c_float> lower_bounds; //存储线性约束的下界
  std::vector<c_float> upper_bounds; //存储线性约束的上界
  for (int row = 0; row < c_nums; row++) //遍历所有约束
  {
    if (row < dim_ * 3 * (k_ + 1)) //row < dim_ * 3 * (k_ + 1)：判断当前约束是否属于路径点约束
    {
      lower_bounds.push_back(l_b_p(row)); //l_b_p(row) 和 u_b_p(row)：分别是路径点约束的下界和上界
      upper_bounds.push_back(u_b_p(row)); //对于路径点约束，下界和上界是相等的，确保轨迹通过指定的路径点
    }
    else if (row < dim_ * 3 * (k_ + 1) + dim_ * 3 * (k_ - 1)) //判断当前约束是否属于平滑性约束
    {
      lower_bounds.push_back(0.0); //平滑性约束确保轨迹在连接点处的位置、速度和加速度连续
      upper_bounds.push_back(0.0); //对于平滑性约束，下界和上界都设置为 0.0，表示等式约束
    }
    else //当前约束属于安全走廊约束
    {
      lower_bounds.push_back(-OSQP_INFTY); //下界设置为 负无穷，表示没有下界限制，只有上界限制
      upper_bounds.push_back(b_safe(row - dim_ * 3 * (k_ + 1) - dim_ * 3 * (k_ - 1))); //安全走廊约束的上界
    }
  }

  // qp solution //二次规划问题的求解，并解析了求解器的输出结果 使用OSQP求解器
  OSQPWorkspace* work = nullptr; //OSQP 工作空间指针，用于存储求解过程中的中间结果
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData))); //OSQP 数据结构，存储二次规划问题的输入数据
  OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings))); //OSQP 设置结构，存储求解器的配置参数
  osqp_set_default_settings(settings); //将 settings 设置为默认值
  settings->verbose = false; //关闭求解器的详细输出，减少日志信息
  settings->warm_start = true; //启用热启动功能，利用上一次求解的结果作为初始值，提高求解效率

  data->n = dim_ * k_ * (n_ + 1); //配置二次规划问题，优化变量的数量，即x的维度
  data->m = c_nums; //线性约束的数量
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(), P_indices.data(), P_indptr.data()); //目标函数的Hessian 矩阵，以稀疏矩阵格式存储，csc_matrix 是 OSQP 提供的稀疏矩阵构造函数，用于创建列压缩存储（CSC）格式的稀疏矩阵
  data->q = q_data.data(); //目标函数的线性项系数向量q
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data()); //线性约束矩阵A，以稀疏矩阵格式存储
  data->l = lower_bounds.data(); //线性约束的下界和上界向量
  data->u = upper_bounds.data();

  osqp_setup(&work, data, settings); //根据提供的数据和设置，初始化 OSQP 工作空间
  osqp_solve(work); //调用 OSQP 求解器，求解二次规划问题
  auto status = work->info->status_val; //获取求解状态码，用于判断求解是否成功

  if ((status < 0) || (status != 1 && status != 2)) //检查求解状态码，status < 0：表示求解失败，== 1：表示求解成功，找到了最优解，== 2：表示求解成功，但可能未收敛到最优解
  {
    R_WARN << "failed optimization status: " << work->info->status;
    return false; //如果求解失败，输出警告信息并返回 false
  }

  // parse solution //解析求解结果
  for (int seg = 0; seg < k_; seg++) //外层循环，遍历每个路径段
  {
    for (int miu = 0; miu < dim_; miu++) //中层循环，遍历每个维度
    {
      Eigen::VectorXd res = Eigen::VectorXd::Zero(n_ + 1);
      for (int i = 0; i <= n_; i++) //内层循环，遍历每个多项式系数
      {
        res[i] = work->solution->x[(miu * k_ + seg) * (n_ + 1) + i]; //work->solution->x：存储优化变量的解
      }
      auto params = B * res; //将优化变量转换为多项式参数，B 是 Bernstein 矩阵
      trajectories_[seg][miu].update({ params[0], params[1], params[2], params[3], params[4], params[5] }); //更新轨迹段的参数
    }
  }

  // Cleanup //清理 OSQP 求解器在求解过程中分配的资源，以避免内存泄漏
  osqp_cleanup(work); //释放 OSQP 工作空间 work 中分配的所有资源
  c_free(data->A); //释放线性约束矩阵A的内存
  c_free(data->P); //释放目标函数矩阵P的内存
  c_free(data); //释放 OSQPData 结构的内存
  c_free(settings); //释放 OSQPSettings 结构的内存

  return true;
}

/**
 * @brief auxiliary matrix 辅助矩阵，用于表示轨迹段的终点的约束条件
 * @param dt_i time allocation of segment i 当前轨迹段的时间分配
 * @return A^+_i
 */
Eigen::MatrixXd MinimumsnapOptimizer::A_i_plus(double dt_i)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, n_ + 1);
  const double dt_i_2 = dt_i * dt_i; //计算时间平方
  A(0, n_) = 1; //填充位置
  A(1, n_ - 1) = -n_ / dt_i; //填充速度
  A(1, n_) = n_ / dt_i;
  A(2, n_ - 2) = n_ * (n_ - 1) / dt_i_2; //填充加速度
  A(2, n_ - 1) = -2 * n_ * (n_ - 1) / dt_i_2;
  A(2, n_) = n_ * (n_ - 1) / dt_i_2;
  return A;
}

/**
 * @brief auxiliary matrix 辅助矩阵，用于表示轨迹段的起点的约束条件
 * @param dt_i time allocation of segment i
 * @return A^-_i
 */
Eigen::MatrixXd MinimumsnapOptimizer::A_i_minus(double dt_i)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, n_ + 1);
  const double dt_i_2 = dt_i * dt_i;
  A(0, 0) = 1;
  A(1, 0) = -n_ / dt_i;
  A(1, 1) = n_ / dt_i;
  A(2, 0) = n_ * (n_ - 1) / dt_i_2;
  A(2, 1) = -2 * n_ * (n_ - 1) / dt_i_2;
  A(2, 2) = n_ * (n_ - 1) / dt_i_2;
  return A;
}
}  // namespace trajectory_optimization

}  // namespace rpp