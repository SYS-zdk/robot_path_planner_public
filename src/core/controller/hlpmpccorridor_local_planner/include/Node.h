/**
 * @file Node.h
 * @brief Node representation used in layered planning/search.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_NODE_H_
#define HYBRID_NODE_H_

#include <string>

namespace hlpmpccorridor_local_planner { //用于存储机器人路径规划中的节点信息
  class Node {
    public:
      Node(int p_index, int p_layer_no, int p_node_no, double p_pos_x, double p_pos_y, double p_angle, double p_obstacle_cost, bool p_dead); //含参构造

      int index; //节点的全局唯一索引
      int layer_no; //节点所在的层编号
      int node_no; //节点在该层中的编号
      double pos_x; //节点在二维平面上的坐标
      double pos_y;
      double angle; //节点的方向角度
      double obstacle_cost; //节点的障碍物成本值
      bool dead; //节点是否为不可用状态
      bool is_backward; // 新增反向标记

  };
};
#endif
