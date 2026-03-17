
/**
 * *********************************************************
 *
 * @file: bezier_curve.cpp
 * @brief: Bezier curve generation //贝塞尔曲线（Bezier Curve）生成器，用于生成平滑的轨迹
 * @author: Yang Haodong
 * @date: 2023-12-22
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cassert>

#include "common/geometry/curve/bezier_curve.h"

namespace rpp
{
namespace common
{
namespace geometry
{
/**
 * @brief Construct a new Bezier generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param offset      The offset of control points (default: 3.0)
 */
BezierCurve::BezierCurve(double step, double offset) : Curve(step), offset_(offset) //BezierCurve 类继承自一个基类 Curve
{
}
BezierCurve::BezierCurve() : Curve(0.1), offset_(3) //step_插值步长，默认值为 0.1；offset_控制点的偏移量，默认值为 3.0
{
}

/**
 * @brief Destroy the Bezier generation object
 */
BezierCurve::~BezierCurve()
{
}

/**
 * @brief Calculate the Bezier curve point.
 * @param t scale factor
 * @param control_pts control points
 * @return point point in Bezier curve with t
 */
Point2d BezierCurve::bezier(double t, const Points2d& control_pts) //计算给定参数 t和控制点集合 control_pts 的贝塞尔曲线上的点

{
  size_t n = control_pts.size() - 1; //计算控制点数量，n 是控制点的数量减一
  double pt_x = 0, pt_y = 0; //初始化
  for (size_t i = 0; i < n + 1; i++)
  {
    pt_x += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].x(); //这是贝塞尔曲线的参数方程，遍历所有控制点，计算每个控制点对最终点的贡献
    pt_y += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].y(); //使用组合数 _comb(n, i) 计算权重
  } //将每个控制点的贡献累加到结果点的x和y坐标
  return { pt_x, pt_y }; //返回计算得到的贝塞尔曲线上的点
}

/**
 * @brief Calculate control points heuristically. 启发式方法来计算贝塞尔曲线的控制点
 * @param start Initial pose (x, y, yaw) 给定起始点和目标点的位置和方向，该方法生成一组控制点，用于定义贝塞尔曲线
 * @param goal  Target pose (x, y, yaw)
 * @return control_pts control points
 */
Points2d BezierCurve::getControlPoints(const Point3d& start, const Point3d& goal) //点的位置和方向
{
  double sx = start.x(), sy = start.y(), syaw = start.theta(); //提取起始点和目标点的坐标和方向
  double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

  double d = std::hypot(sx - gx, sy - gy) * offset_; //计算起始点和目标点之间的欧几里得距离 ；//使用偏移量 offset_ 调整控制点的位置。offset_ 是一个类成员变量，用于控制控制点的偏移量。

  Points2d control_pts; //生成控制点，对于一个三次贝塞尔曲线，需要四个控制点。
  control_pts.emplace_back(sx, sy); //起始点，位置sx,sy方向syaw
  control_pts.emplace_back(sx + d * cos(syaw), sy + d * sin(syaw)); // 起始点的切线方向上的点，从起始点出发沿着syaw偏移d的位置
  control_pts.emplace_back(gx - d * cos(gyaw), gy - d * sin(gyaw)); // 目标点的切线方向上的点，从目标点出发沿着gyaw偏移-d的位置
  control_pts.emplace_back(gx, gy); //目标点

  return control_pts;
}

/**
 * @brief Generate the path. 从一个起始点到一个目标点的贝塞尔曲线轨迹生成
 * @param start Initial pose (x, y, yaw) 接收起始点和目标点的位置和方向，生成平滑的贝塞尔曲线路径
 * @param goal  Target pose (x, y, yaw)
 * @param path  The smoothed trajectory points
 * @return true if generate successfully, else failed
 */
bool BezierCurve::generation(const Point3d& start, const Point3d& goal, Points3d& path)  //path生成的轨迹
{
  double sx = start.x(), sy = start.y(), syaw = start.theta(); //提取起始点和目标点的坐标和方向
  double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

  size_t n_points = static_cast<size_t>(std::hypot(sx - gx, sy - gy) / step_); //计算路径点的数量，基于起始点和目标点之间的欧几里得距离和插值步长 step_
  const Points2d& control_pts = getControlPoints(start, goal); //调用 getControlPoints 方法，生成贝塞尔曲线的控制点

  path.clear(); //清空目标路径 path
  path.reserve(n_points); //预分配路径点的内存，提高性能
  for (size_t i = 0; i < n_points; i++) //遍历路径点，计算每个插值点t
  {
    double t = static_cast<double>(i) / static_cast<double>(n_points - 1);
    const auto& waypoint = bezier(t, control_pts); //调用 bezier 方法，计算贝塞尔曲线上的点
    path.emplace_back(waypoint.x(), waypoint.y()); //将计算得到的点添加到路径 path 中
  }

  path[0].setTheta(start.theta()); //设置起始点和目标点的方向
  path.back().setTheta(goal.theta());
  for (size_t i = 1; i < n_points - 1; i++) //遍历中间路径点，计算每个点的方向
  {
    auto prev_interp_vec = Vec2d(path[i - 1].x(), path[i - 1].y());
    auto curr_interp_vec = Vec2d(path[i].x(), path[i].y());
    auto next_interp_vec = Vec2d(path[i + 1].x(), path[i + 1].y());
    auto tangent_dir = rpp::common::math::tangentDir(prev_interp_vec, curr_interp_vec, next_interp_vec, false); //使用 tangentDir 方法计算切线方向
    auto dir = tangent_dir.innerProd(curr_interp_vec - prev_interp_vec) >= 0 ? tangent_dir : -tangent_dir; //根据切线方向设置路径点的方向
    path[i].setTheta(dir.angle());
  }

  return true;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool BezierCurve::run(const Points2d& points, Points3d& path)
{
  if (points.size() < 4)
  {
    return false;
  }
  else
  {
    Points3d poses;
    poses.emplace_back(points.begin()->x(), points.begin()->y(), 0);
    for (size_t i = 1; i < points.size() - 1; i++)
    {
      double theta1 = std::atan2(points[i].y() - points[i - 1].y(), points[i].x() - points[i - 1].x());
      double theta2 = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
      poses.emplace_back(points[i].x(), points[i].y(), (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().x(), points.back().y(), 0);

    return run(poses, path);
  }
}

/**
 * @brief Running trajectory generation //实现了贝塞尔曲线的轨迹生成功能。它接收一系列路径点（包含位置和方向），并生成平滑的贝塞尔曲线轨迹
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool BezierCurve::run(const Points3d& points, Points3d& path)
{
  if (points.size() < 4)
  {
    return false;
  }
  else
  {
    path.clear();
    Points3d path_i;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      if (!generation(points[i], points[i + 1], path_i))
      {
        return false;
      }
      path.insert(path.end(), path_i.begin(), path_i.end());
    }

    return !path.empty();
  }
}

/**
 * @brief Configure the offset of control points.
 * @param offset  The offset of control points
 */
void BezierCurve::setOffset(double offset)
{
  assert(offset > 0);
  offset_ = offset;
}

// Calculate the number of combinations //一个递归函数 _comb，用于计算组合数 //组合数表示从 n个不同元素中选择  r 个元素的方法数。
int BezierCurve::_comb(int n, int r) //n：总元素数 ; r：选择的元素数
{
  if ((r == 0) || (r == n))
    return 1;
  else
    return _comb(n - 1, r - 1) + _comb(n - 1, r);
}

Eigen::Matrix<double, 6, 6> Bernstein::matrix5_ =
    (Eigen::Matrix<double, 6, 6>() << 1, 0, 0, 0, 0, 0, -5, 5, 0, 0, 0, 0, 10, -20, 10, 0, 0, 0, -10, 30, -30, 10, 0, 0,
     5, -20, 30, -20, 5, 0, -1, 5, -10, 10, -5, 1)
        .finished();   //这个矩阵matrix5_是一个五次（5阶）Bernstein多项式的系数矩阵。每一行对应一个Bernstein多项式，每一列对应多项式中的不同幂次项t的0次到t的5次

}  // namespace geometry
}  // namespace common
}  // namespace rpp