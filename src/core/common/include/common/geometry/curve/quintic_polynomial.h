
/**
 * *********************************************************
 *
 * @file: quintic_polynomial.h
 * @brief: Quintic_polynomial curve generation 
 * @author: Zhang Dingkun
 * @date: 2025-03-07
 * @version: 1.0
 *
 * Copyright (c) 2025, Zhang Dingkun.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_GEOMETRY_QUINTIC_POLYNOMIAL_H_
#define RMP_COMMON_GEOMETRY_QUINTIC_POLYNOMIAL_H_

#include <array>

namespace rpp
{
namespace common
{
namespace geometry
{
class QuinticPolynomial //用于生成和操作五次多项式；五次多项式通常用于轨迹规划，因为它可以生成平滑的轨迹，同时满足初始和目标位置、速度和加速度的约束
{
public:
  QuinticPolynomial(); // 默认构造函数
  QuinticPolynomial(const std::array<double, 6>& params); //含参构造，参数 params 是一个包含 6 个系数的数组，用于初始化五次多项式的系数
  ~QuinticPolynomial() = default;

  void update(const std::array<double, 6>& params); //更新五次多项式的系数
  void solve(const std::array<double, 3>& start_pva, const std::array<double, 3>& end_pva, double T); //求解五次多项式的系数，满足初始和目标位置、速度和加速度的约束；//start_pva 是初始位置、速度和加速度；//end_pva 是目标位置、速度和加速度；//T是时间间隔

  double x(double t) const; //计算五次多项式在时间t时的位置
  double dx(double t) const; ////计算五次多项式在时间t时的速度
  double ddx(double t) const; //计算五次多项式在时间t时的加速度
  double dddx(double t) const; //计算五次多项式在时间t时的急动度jerk
  double ddddx(double t) const; //计算五次多项式在时间t时的跳跃度snap

private:
  double a0_, a1_, a2_, a3_, a4_, a5_; //五次多项式的系数
};
}  // namespace geometry
}  // namespace common
}  // namespace rpp

#endif