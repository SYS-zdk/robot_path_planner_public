/**
 * @file AStar_Node.h
 * @brief Node data structure used by Hybrid A* / graph search routines.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_ASTR_NODE_H_
#define HYBRID_ASTR_NODE_H_

#include <string>

namespace hlpmpccorridor_local_planner {
  class AStar_Node{
    public:
      AStar_Node(int p_number, std::string p_hash, double p_pos_x, double p_pos_y, double p_heading_angle, double p_sim_time, double p_cost, int p_parent, double p_hr_cost, double p_vel_th); //定义一个构造函数
      AStar_Node(); //默认构造
      int number; //存储节点的编号
      std::string hash; //定义一个字符串成员变量
      double pos_x; //存储节点的 x 坐标
      double pos_y; //存储节点的 y 坐标
      double heading_angle; //存储节点的朝向角度
      double sim_time; //存储节点的模拟时间
      double cost; //存储节点的代价
      int parent; //存储父节点的编号
      double hr_cost; //存储节点的启发式代价
      double vel_th; //存储节点的速度和转向角度

      bool operator<(const AStar_Node &e) const; //重载小于操作符  定义如何比较两个 AStar_Node 对象
  };
};
#endif
