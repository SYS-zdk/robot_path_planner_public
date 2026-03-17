
/**
 * *********************************************************
 *
 * @file: curve.h
 * @brief: Curve generation //通用的曲线生成基类，提供了计算路径长度和设置插值步长的功能
 * @author: Yang Haodong
 * @date: 2023-12-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/util/log.h"
#include "common/geometry/curve/curve.h"

namespace rpp
{
namespace common
{
namespace geometry
{
/**
 * @brief Construct a new Curve object
 * @param step  Simulation or interpolation size //参数 step 用于初始化插值步长
 */
Curve::Curve(double step) : step_(step)
{
}

/**
 * @brief Calculate the distance of given path.
 * @param path    the trajectory
 * @return length the length of path
 */
double Curve::distance(const Points3d& path) //计算给定路径的总长度
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    length += std::hypot(path[i - 1].x() - path[i].x(), path[i - 1].y() - path[i].y()); //欧几里德距离：sqrt((x_2-x_1)^2+(Y_2-Y_1)^2)
  }
  return length;
}

/**
 * @brief Configure the simulation step. 设置插值步长
 * @param step    Simulation or interpolation size //插值步长，用于控制路径生成的精度
 */
void Curve::setStep(double step)
{
  CHECK_GT(step, 0.0); //使用 CHECK_GT 宏检查步长是否大于 0。CHECK_GT 是一个断言宏，用于确保某个值大于另一个值。如果条件不满足，程序会抛出异常并终止。
  step_ = step;
}
}  // namespace geometry
}  // namespace common
}  // namespace rpp