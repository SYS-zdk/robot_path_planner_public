
/**
 * *********************************************************
 *
 * @file: trajectory.h
 * @brief: Trajectory with time, position, velocity and acceletation //Trajectory 的模板类，用于表示带有时间、位置、速度和加速度的轨迹
 * @author: Yang Haodong
 * @date: 2024-10-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_STRUCTURE_TRAJECTORY_H_
#define RMP_COMMON_STRUCTURE_TRAJECTORY_H_

#include "common/geometry/point.h"

namespace rpp
{
namespace common
{
namespace structure
{
template <typename Point>
class Trajectory
{
public:
  Trajectory(int size = 0) : size_(size) //参数 size 是轨迹的初始大小，默认值为 0  ;初始化成员变量 size_，并为每个向量预留空间以提高性能
  {
    time.reserve(size);
    position.reserve(size);
    velocity.reserve(size);
    acceletation.reserve(size);
  }

  ~Trajectory() 
  {
    reset(); //在析构函数中调用 reset 方法，释放所有资源
  }

  void clear() //清空轨迹中的所有数据
  {
    time.clear();
    position.clear();
    velocity.clear();
    acceletation.clear();
  }
  void reset(int size = 0) //重置轨迹的大小，并释放所有资源
  {
    size_ = size; //参数 size 是新的轨迹大小，默认值为 0

    if (!time.empty())
    {
      std::vector<double>().swap(time); //swap 函数来交换两个对象的内容;std::vector<double>()创建一个临时的空向量,调用 swap 方法;交换后，time 向量的内容被移动到临时的空向量中，而 time 向量变成空的
    }

    if (!position.empty())
    {
      std::vector<Point>().swap(position); //临时的空向量在作用域结束时被销毁,释放内存
    }

    if (!velocity.empty())
    {
      std::vector<Point>().swap(velocity); //clear 方法只会清空向量的内容，但不会释放内存。如果向量很大，这可能会导致不必要的内存占用
    }

    if (!acceletation.empty())
    {
      std::vector<Point>().swap(acceletation);
    }

    if (size > 0)
    {
      time.reserve(size);
      position.reserve(size);
      velocity.reserve(size);
      acceletation.reserve(size);
    }
  }

public:
  std::vector<double> time;
  std::vector<Point> position;
  std::vector<Point> velocity;
  std::vector<Point> acceletation;

private:
  int size_;
};

using Trajectory2d = Trajectory<rpp::common::geometry::Point2d>;
using Trajectory3d = Trajectory<rpp::common::geometry::Point3d>; //Trajectory3d 被定义为一个具体的类，用于表示三维空间中的轨迹。这个类继承了 Trajectory 模板类的所有功能，但将点的类型固定为Point3d

}  // namespace structure
}  // namespace common
}  // namespace rpp

#endif