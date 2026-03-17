
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
 * @file Node.cpp
 * @brief Implementation of the layered node structure.
 * @author Fahim Shahriar (original); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 */

#include <Node.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace hlpmpccorridor_local_planner { //含参构造的具体实现，将外部的参数设置传入内部
Node::Node(int p_index, int p_layer_no, int p_node_no, double p_pos_x, double p_pos_y, double p_angle, double p_obstacle_cost, bool p_dead) {
    index = p_index;
    layer_no = p_layer_no;
    node_no = p_node_no;
    pos_x = p_pos_x;
    pos_y = p_pos_y;
    angle = p_angle;
    obstacle_cost = p_obstacle_cost;
    dead = p_dead;
}
};  // namespace hlpmpccorridor_local_planner