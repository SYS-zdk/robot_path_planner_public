/**
 * @file Helper.h
 * @brief Utility helpers for angles, hashing and kinematics bounds in the hybrid planner.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_HELPER_H_
#define HYBRID_HELPER_H_

#include <Node.h>
#include <string>

namespace hlpmpccorridor_local_planner {
  class Helper {
    public:
      static double getScaledDist(double x1, double x2, double y1, double y2); //计算两个点或节点之间的缩放距离
      static double getScaledDist(Node &n1, Node &n2); 
      static double getInRangeAngle(double angle); //处理角度数据
      static std::string getHash(int n1, int n2, int n3); //生成唯一的哈希字符串
      static void getLeftRightTh(double time, double orientation, double cur_vel, double max_velocity, double max_acc, double &left_th, double &right_th); //计算左右轮的速度 
      static double getAngleDist(double a1, double a2); //计算两个角度之间的最小差值
  }; //time：时间；orientation：朝向角度；cur_vel：当前速度；max_velocity：最大速度；max_acc：最大加速度；left_th, right_th：输出参数，表示左右轮的速度
};
#endif
