/**
 * @file Edge.h
 * @brief Edge data structure for layered graph search.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_EDGE_H_
#define HYBRID_EDGE_H_

namespace hlpmpccorridor_local_planner {
  class Edge {
    public:
      Edge(int p_node_index, double p_cost); //构造函数声明

      int get_node_index() const; //成员函数 获取边连接的节点索引
      double get_cost() const; //成员函数 获取边的代价

      bool operator<(const hlpmpccorridor_local_planner::Edge &e) const; //声明小于操作符 < 的重载，定义如何比较两个 Edge 对象

    private:
      int node_index; //存储边连接的节点索引
      double cost; //存储边的代价


  };
};
#endif
