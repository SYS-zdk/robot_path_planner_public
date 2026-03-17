
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
 * @file Edge.cpp
 * @brief Implementation of graph edge helpers.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#include <Edge.h>

namespace hlpmpccorridor_local_planner {
    Edge::Edge(int p_node_index, double p_cost){
        node_index = p_node_index;
        cost = p_cost;
    } //定义带参数的构造函数的实现，初始化 Edge 类的成员变量

    int Edge::get_node_index() const{
        return node_index; //返回边连接的节点索引
    }
    
    double Edge::get_cost()const{
        return cost; //返回边的代价
    }

    bool Edge::operator<(const hlpmpccorridor_local_planner::Edge &e)const{
        return cost > e.cost; //比较两个 Edge 对象的代价
    }
};