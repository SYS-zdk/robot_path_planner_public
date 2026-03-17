
/***********************************************************
 *
 * @file: time_allocation.h
 * @brief: time allocation for trajectory generation 用于为轨迹生成分配时间，提供了两种时间分配方法
 * @author: Yang Haodong
 * @date: 2024-09-30
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_TIME_ALLOCATION_H_
#define RMP_TRAJECTORY_OPTIMIZATION_TIME_ALLOCATION_H_

#include <vector>

namespace rpp
{
namespace trajectory_optimization
{
class TimeAllocator
{
public:
  TimeAllocator() = default;
  ~TimeAllocator() = default;

  template <typename Point> //用于定义模板函数或模板类
  static void normalAllocation(const std::vector<Point>& waypoints, double v_max, std::vector<double>& time_allocations) //为每个路径段分配时间，基于最大速度
  {
    time_allocations.clear(); //初始化
    for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; i++) //循环遍历路径段数
    {
      const auto& wp_1 = waypoints[i]; //获取当前点和下一个点
      const auto& wp_2 = waypoints[i + 1];
      time_allocations.push_back(std::hypot(wp_1.x() - wp_2.x(), wp_1.y() - wp_2.y()) / v_max); //先求出路径段的欧式距离，然后求出该路径段最短需要的时间
    }
  }

  template <typename Point>
  static void trapezoidalAllocation(const std::vector<Point>& waypoints, double v_max, double a_max,
                                    std::vector<double>& time_allocations) //梯形速度分配方法，用于为轨迹生成分配时间
  {
    time_allocations.clear();
    for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; i++)
    {
      const auto& wp_1 = waypoints[i];
      const auto& wp_2 = waypoints[i + 1];
      const double dist = std::hypot(wp_1.x() - wp_2.x(), wp_1.y() - wp_2.y());
      double dt = dist > v_max * v_max / a_max ? v_max / a_max + dist / v_max : 2 * std::sqrt(dist / a_max);
      time_allocations.push_back(dt);
    }
  }
};
}  // namespace trajectory_optimization
}  // namespace rpp
#endif