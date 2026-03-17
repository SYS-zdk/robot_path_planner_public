
/*
 * Copyright 2024 Fahim Shahriar
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file Helper.cpp
 * @brief Implementation of helper utilities for the hybrid planner.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#include <Helper.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace hlpmpccorridor_local_planner {

// Given two node objects, return the distance between the nodes
double Helper::getScaledDist(hlpmpccorridor_local_planner::Node &n1, hlpmpccorridor_local_planner::Node &n2) {
    double dist = std::hypot(n1.pos_x - n2.pos_x, n1.pos_y - n2.pos_y);
    return dist;
}//计算两个节点 (n1 和 n2) 之间的欧几里得距离

// Given the x and y coordinates of two points, return the distance between the two points
double Helper::getScaledDist(double x1, double x2, double y1, double y2) {
    double dist = std::hypot(x1 - x2, y1 - y2);
    return dist;
} //直接通过坐标值计算两点 (x1, y1) 和 (x2, y2) 之间的距离

// Find the (smallest) angular distance between two points 
double Helper::getAngleDist(double a1, double a2) {
    return std::min(std::abs(a1 - a2),
                    std::min(std::abs(a1 - a2 - (M_PI * 2)), std::abs(a1 - a2 + (M_PI * 2))));
} //计算两个角度 a1 和 a2 之间的最小差值（考虑圆周循环性）。当 a1=350°，a2=10° 时，实际最小差值为 20°，而非 340° 

// Given an angle in radian, return the corresponding angle value in the range 0 to 2 * PI
double Helper::getInRangeAngle(double angle) {
    double m_pi_2 = M_PI * 2;
    if (angle < 0) return angle + m_pi_2;
    if (angle > m_pi_2) return angle - m_pi_2;
    return angle;
} //将任意角度调整到 0 到 2π 范围内，避免超出圆周范围

// Given three integers, return their hash 
std::string Helper::getHash(int n1, int n2, int n3) {
    std::stringstream ss;
    // Use delimiters to avoid collisions (e.g., 1,23,4 vs 12,3,4).
    ss << n1 << "," << n2 << "," << n3;
    return ss.str(); 
} //将三个整数拼接成字符串哈希值，用于快速索引或唯一标识 输入 1, 2, 3 输出 "123"

// Given the state of the robot, return the left most and right most angle (theta) it can achieve in the given time period 根据当前速度、加速度和时间，计算机器人在给定时间内能达到的左右最大转向角度
void Helper::getLeftRightTh(double time, double orientation, double cur_vel, double max_velocity, double acc, double &left_th, double &right_th) {
    double time_to_max_vel_th_left = std::abs((max_velocity - cur_vel) / acc); // 计算左转时间阈值 加速到最大正向角速度（max_velocity）
    double time_to_max_vel_th_right = std::abs((-max_velocity - cur_vel) / acc); // 计算右转时间阈值 加速到最大反向角速度（-max_velocity）

    if (time_to_max_vel_th_left < time) { //若时间足够加速到最大速度：先加速到最大速度，再匀速
        left_th = orientation + ((cur_vel * time_to_max_vel_th_left) + (0.5 * acc * time_to_max_vel_th_left * time_to_max_vel_th_left));
        left_th = left_th + (max_velocity * (time - time_to_max_vel_th_left));
    } else { //若时间不足：仅计算加速阶段的位移
        left_th = orientation + ((cur_vel * time) + (0.5 * acc * time * time));
    }

    if (time_to_max_vel_th_right < time) { 
        right_th = orientation + ((cur_vel * time_to_max_vel_th_right) - (0.5 * acc * time_to_max_vel_th_right * time_to_max_vel_th_right));
        right_th = right_th - (max_velocity * (time - time_to_max_vel_th_right));
    } else {
        right_th = orientation + ((cur_vel * time) - (0.5 * acc * time * time));
    }
  // 归一化角度
    left_th = Helper::getInRangeAngle(left_th);
    right_th = Helper::getInRangeAngle(right_th);
}
};
