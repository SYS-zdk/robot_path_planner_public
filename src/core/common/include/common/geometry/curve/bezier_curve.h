
/**
 * *********************************************************
 *
 * @file: bezier_curve.h
 * @brief: Bezier curve generation 定义了一个贝塞尔曲线（Bezier Curve）生成器
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
#ifndef RMP_COMMON_GEOMETRY_BEZIER_CURVE_H_
#define RMP_COMMON_GEOMETRY_BEZIER_CURVE_H_

#include <Eigen/Dense>

#include "common/geometry/curve/curve.h"

namespace rpp
{
namespace common
{
namespace geometry
{
class BezierCurve : public Curve
{
public:
  /**
   * @brief Construct a new Bezier generation object
   * @param step        Simulation or interpolation size (default: 0.1) step：模拟或插值的步长，默认值为 0.1
   * @param offset      The offset of control points (default: 3.0) 控制点的偏移量，默认值为 3.0
   */
  BezierCurve(); //使用默认参数初始化贝塞尔曲线生成器
  BezierCurve(double step, double offset);

  /**
   * @brief Destroy the Bezier generation object
   */
  ~BezierCurve();

  /**
   * @brief Running trajectory generation 输入二维路径点 points，生成平滑的三维轨迹 path
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points2d& points, Points3d& path);

  /**
   * @brief Running trajectory generation 输入三维路径点 points，生成平滑的三维轨迹 path
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points3d& points, Points3d& path);

  /**
   * @brief Calculate the Bezier curve point. 计算贝塞尔曲线在参数t时的点
   * @param t scale factor
   * @param control_pts control points
   * @return point point in Bezier curve with t
   */
  Point2d bezier(double t, const Points2d& control_pts); 

  /**
   * @brief Calculate control points heuristically. 根据起点和终点的姿态（位置和朝向），启发式地计算控制点
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return control_pts control points
   */
  Points2d getControlPoints(const Point3d& start, const Point3d& goal); 

  /**
   * @brief Generate the path. //根据起点和终点的姿态，生成平滑的轨迹
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @param path  The smoothed trajectory points
   * @return true if generate successfully, else failed
   */
  bool generation(const Point3d& start, const Point3d& goal, Points3d& path);

  /**
   * @brief Configure the offset of control points.
   * @param offset  The offset of control points
   */
  void setOffset(double offset); //设置控制点的偏移量

private:
  // Calculate the number of combinations 计算组合数n_r，用于贝塞尔曲线的计算
  int _comb(int n, int r);

protected:
  double offset_;  // The offset of control points 控制点的偏移量
};

class Bernstein //提供贝塞尔曲线的 Bernstein 矩阵。
{
public:
  Bernstein() = default;
  ~Bernstein() = default;

  static Eigen::MatrixXd matrix(int n) //提供贝塞尔曲线的 Bernstein 矩阵，如果n=5，返回预定义的5阶Bernstein 矩阵，否则，返回一个零矩阵
  {
    if (n == 5U) //在某些编译器设置下，将有符号整数与无符号整数进行比较可能会引发警告。使用 5U 可以避免这种情况，5U 是一个整数值，表示无符号整数（unsigned int）类型的字面量
    {
      return matrix5_;
    }
    else
    {
      return Eigen::MatrixXd::Zero(n, n);
    }
  }

private:
  static Eigen::Matrix<double, 6, 6> matrix5_; //预定义的 5 阶 Bernstein 矩阵；Eigen::Matrix<double, 6, 6>：表示一个 6x6 的矩阵，元素类型为 double；static：表示这是一个静态成员变量，属于类本身，而不是类的某个实例。所有实例共享同一个 matrix5_
};
}  // namespace geometry
}  // namespace common
}  // namespace rpp
#endif