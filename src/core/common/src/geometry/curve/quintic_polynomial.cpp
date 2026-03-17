
/**
 * *********************************************************
 *
 * @file: quintic_polynomial.h
 * @brief: quintic polynomial generation 实现了一个五次多项式（Quintic Polynomial）的生成和求解
 * @author: Yang Haodong
 * @date: 2025-1-17
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cmath>
#include <Eigen/Dense>

#include "common/geometry/curve/quintic_polynomial.h"

namespace rpp
{
namespace common
{
namespace geometry
{
QuinticPolynomial::QuinticPolynomial() : a0_(0.0), a1_(0.0), a2_(0.0), a3_(0.0), a4_(0.0), a5_(0.0)
{
} //默认构造，初始化所有系数为 0

QuinticPolynomial::QuinticPolynomial(const std::array<double, 6>& params) //params ，表示对一个包含 6 个 double 类型元素的数组的引用
  : a0_(params[0]), a1_(params[1]), a2_(params[2]), a3_(params[3]), a4_(params[4]), a5_(params[5]){}; //含参构造，使用传入的参数数组初始化系数

void QuinticPolynomial::update(const std::array<double, 6>& params) //更新多项式的系数
{
  a0_ = params[0];
  a1_ = params[1];
  a2_ = params[2];
  a3_ = params[3];
  a4_ = params[4];
  a5_ = params[5];
}

void QuinticPolynomial::solve(const std::array<double, 3>& start_pva, const std::array<double, 3>& end_pva, double T) //根据起始点和终点的位置、速度和加速度，以及总时间 T，求解五次多项式的系数
{
  double x0 = start_pva[0];
  double v0 = start_pva[1];
  double a0 = start_pva[2];
  double xt = end_pva[0];
  double vt = end_pva[1];
  double at = end_pva[2];

  Eigen::Matrix3d A; //使用 Eigen 库求解线性方程组，目的是求解一个线性方程组Ax=b；Eigen::Matrix3d 是一个 3x3 的双精度浮点矩阵
  A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
      6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3); //是 Eigen 提供的一种便捷语法，用于初始化矩阵的元素，顺序是a00,a01,a02,a10,a11,a12,a20,a21,a22

  Eigen::Vector3d b(xt - x0 - v0 * T - a0 * T * T / 2, vt - v0 - a0 * T, at - a0); //Eigen::Vector3d 是一个 3 维的双精度浮点向量b,3*1的列向量；每个逗号为一项
  Eigen::Vector3d x = A.lu().solve(b); //A.lu().solve(b) 是 Eigen 提供的求解线性方程组的方法 //A.lu() 表示对矩阵A进行LU分解，solve(b) 表示求解方程组，返回x

  // Quintic polynomial coefficient //更新系数
  a0_ = x0;
  a1_ = v0;
  a2_ = a0 / 2.0;
  a3_ = x(0);
  a4_ = x(1);
  a5_ = x(2);
}

double QuinticPolynomial::x(double t) const //计算位置
{
  return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) + a4_ * std::pow(t, 4) + a5_ * std::pow(t, 5);
}

double QuinticPolynomial::dx(double t) const //计算速度
{
  return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2) + 4 * a4_ * std::pow(t, 3) + 5 * a5_ * std::pow(t, 4);
}

double QuinticPolynomial::ddx(double t) const //计算加速度
{
  return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t, 2) + 20 * a5_ * std::pow(t, 3);
}

double QuinticPolynomial::dddx(double t) const //计算加加速度
{
  return 6 * a3_ + 24 * a4_ * t + 60 * a5_ * std::pow(t, 2);
}

double QuinticPolynomial::ddddx(double t) const //计算四阶导数（snap）
{
  return 24 * a4_ + 120 * a5_ * t;
}
}  // namespace geometry
}  // namespace common
}  // namespace rpp