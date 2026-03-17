/**
 * @file hybrid_planner_ros.cpp
 * @brief Implementation of the ROS wrapper for the Hybrid MPC corridor local planner.
 * @author Eitan Marder-Eppstein (DWA/original); Fahim Shahriar (Apache 2.0 additions); Zhang Dingkun (integration)
 * @date 2026-03-17
 * @version 1.0
 *
 * @note Upstream license texts are retained below.
 */

// The setup code for hybrid planner is based on the DWA planner,
// the default local planner for Turtlebot3 gazebo.
// As such, the Software License Agreement of DWA planner is included in this file.

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/


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

#include <base_local_planner/goal_functions.h>
#include <hybrid_planner_ros.h>
#include <nav_core/parameter_magic.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <tf2/utils.h>

#include <Eigen/Core>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <memory>
#include <tf2/exceptions.h>
#include <tf2/exceptions.h>
#include <osqp/osqp.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hlpmpccorridor_local_planner::HybridMpcCorridorPlannerROS, nav_core::BaseLocalPlanner)

namespace hlpmpccorridor_local_planner {

void HybridMpcCorridorPlannerROS::reconfigureCB(dwa_local_planner::DWAPlannerConfig& config, uint32_t level) { //动态重配置回调函数
    if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
    }
    if (!setup_) {
        default_config_ = config;
        setup_ = true;
    }

    // update generic local planner params 更新通用本地规划器参数
    base_local_planner::LocalPlannerLimits limits;
    limits.max_vel_trans = config.max_vel_trans;
    limits.min_vel_trans = config.min_vel_trans;
    limits.max_vel_x = config.max_vel_x;
    limits.min_vel_x = config.min_vel_x;
    limits.max_vel_y = config.max_vel_y;
    limits.min_vel_y = config.min_vel_y;
    limits.max_vel_theta = config.max_vel_theta;
    limits.min_vel_theta = config.min_vel_theta;
    limits.acc_lim_x = config.acc_lim_x;
    limits.acc_lim_y = config.acc_lim_y;
    limits.acc_lim_theta = config.acc_lim_theta;
    limits.acc_lim_trans = config.acc_lim_trans;
    limits.xy_goal_tolerance = config.xy_goal_tolerance;
    limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
    limits.prune_plan = config.prune_plan;
    limits.trans_stopped_vel = config.trans_stopped_vel;
    limits.theta_stopped_vel = config.theta_stopped_vel;
    planner_util_.reconfigureCB(limits, config.restore_defaults);

    // update dwa specific configuration 更新 DWA 特定配置
    dp_->reconfigure(config);
}

HybridMpcCorridorPlannerROS::HybridMpcCorridorPlannerROS() : initialized_(false),
                                       odom_helper_("odom"),
                                       setup_(false) {
    // initialize blending and smoothing state
    blend_tau_ = 0.15; // seconds
    v_switch_threshold_ = 0.12; // m/s threshold to prefer MPC
    alpha_ = 0.0;
    last_cmd_time_ = ros::Time::now();
    last_cmd_vel_.linear.x = 0.0;
    last_cmd_vel_.linear.y = 0.0;
    last_cmd_vel_.angular.z = 0.0;
    // OSQP persistent members
    osqp_work_ = nullptr;
    osqp_data_ = nullptr;
    osqp_settings_ = nullptr;
    osqp_n_ = 0;
    osqp_m_ = 0;
    osqp_initialized_ = false;
} //构造函数初始化 HybridMpcCorridorPlannerROS 对象

void HybridMpcCorridorPlannerROS::initialize(
    std::string name,
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
    if (!isInitialized()) { //确保规划器只初始化一次
        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1); //发布全局路径
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1); //发布局部路径
        tf_ = tf; //存储用于坐标变换的 tf2_ros::Buffer 对象
        costmap_ros_ = costmap_ros; //存储 ROS 封装的代价地图对象
        costmap_ros_->getRobotPose(current_pose_); //通过 costmap_ros_->getRobotPose(current_pose_) 更新 current_pose_

        // make sure to update the costmap we'll use for this cycle 更新当前周期使用的 costmap
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

        planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID()); //初始化底层路径规划工具，依赖代价地图和全局坐标系名称


        // create the actual planner that we'll use.. it'll configure itself from the parameter server 创建实际的规划器
        dp_ = boost::shared_ptr<HybridPlanner>(new HybridPlanner(name, &planner_util_)); //创建混合规划器 HybridPlanner 的实例，并将其托管在 boost::shared_ptr 智能指针中
        // pass costmap wrapper to planner to enable safety corridor construction
        dp_->setCostmapRos(costmap_ros_);

        // no external MPC controller will be used; the planner contains a simple internal tracker

        if (private_nh.getParam("odom_topic", odom_topic_)) { //从参数服务器读取 odom_topic 参数（默认值通常为 "odom")
            odom_helper_.setOdomTopic(odom_topic_); //若参数存在，设置 odom_helper_（里程计辅助工具类）的订阅话题
        } //确保规划器能正确接收里程计数据以进行运动控制

        initialized_ = true;

        // Warn about deprecated parameters -- remove this block in N-turtle 提示用户某些旧版参数名称已被弃用，并建议使用新名称（例如 max_vel_trans 改为 max_trans_vel）
        nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
        nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
        nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
        nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
        nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
        nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

        //设置动态重配置服务器，在不重启节点的情况下实时调整参数（如速度限制、加速度限制），便于调试和优化
        dsrv_ = new dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>(private_nh); //允许运行时通过 ROS 工具（如 rqt_reconfigure）修改规划器参数
        dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>::CallbackType cb = [this](auto& config, auto level) { reconfigureCB(config, level); }; //使用 Lambda 表达式将 reconfigureCB 方法绑定为参数更新的回调函数
        dsrv_->setCallback(cb);
        // read MPC and blending parameters from rosparam
        private_nh.param("mpc_horizon", mpc_horizon_, 4);
        private_nh.param("mpc_dt", mpc_dt_, dp_->getSimPeriod());
        private_nh.param("mpc_q_v", mpc_q_v_, 1.0);
        private_nh.param("mpc_q_w", mpc_q_w_, 1.0);
        private_nh.param("mpc_r_v", mpc_r_v_, 0.1);
        private_nh.param("mpc_r_w", mpc_r_w_, 0.1);
        private_nh.param("blend_tau", blend_tau_, 0.15);
        private_nh.param("v_switch_threshold", v_switch_threshold_, 0.12);
        private_nh.param("osqp_verbose", osqp_verbose_, false);
        private_nh.param("osqp_warm_start", osqp_warm_start_, true);
        // sigmoid alpha parameters (continuous mapping)
        private_nh.param("alpha_sigmoid_k", alpha_sigmoid_k_, 8.0);
        private_nh.param("alpha_sigmoid_thr", alpha_sigmoid_thr_, v_switch_threshold_);

        // try to initialize persistent OSQP workspace (may fail; we'll fallback later)
        if (!initOSQPWorkspace()) {
            ROS_WARN("Persistent OSQP workspace initialization failed; MPC will attempt on-demand setup per cycle.");
        } else {
            ROS_INFO("Persistent OSQP workspace initialized successfully.");
        }
    } else {
        ROS_WARN("This planner has already been initialized, doing nothing.");
    }
}

bool HybridMpcCorridorPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) { //设置全局路径规划，orig_global_plan 是一个包含多个 PoseStamped 消息的向量，表示全局路径
    if (!isInitialized()) { //检查规划器是否已初始化。如果没有初始化，输出错误信息并返回 false
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    // when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching(); //重置控制器的状态

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan); //将全局路径传递给 dp_（混合规划器实例）进行处理
}

bool HybridMpcCorridorPlannerROS::isGoalReached() { //检查目标点是否已到达
    if (!isInitialized()) { //检查规划器是否已初始化
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    if (!costmap_ros_->getRobotPose(current_pose_)) { //尝试获取机器人的当前位姿
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) { //检查目标点是否已到达
        ROS_INFO("Goal reached");
        return true;
    } else {
        return false;
    }
}

void HybridMpcCorridorPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) { //发布局部路径
    base_local_planner::publishPlan(path, l_plan_pub_); //用 base_local_planner::publishPlan，将局部路径发布到 ROS 话题 l_plan_pub_
}

void HybridMpcCorridorPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) { //发布全局路径
    base_local_planner::publishPlan(path, g_plan_pub_); //调用 base_local_planner::publishPlan，将全局路径发布到 ROS 话题 g_plan_pub_
}

HybridMpcCorridorPlannerROS::~HybridMpcCorridorPlannerROS() { //析构函数，用于清理资源
    // make sure to clean things up 调用 delete dsrv_，释放动态参数服务器 dsrv_ 的内存
    delete dsrv_; 
    // cleanup persistent osqp workspace and data if allocated
    if (osqp_work_) {
        osqp_cleanup(osqp_work_);
        osqp_work_ = nullptr;
    }
    if (osqp_data_) {
        // free csc internal arrays if created via csc_matrix
        if (osqp_data_->A) c_free(osqp_data_->A);
        if (osqp_data_->P) c_free(osqp_data_->P);
        c_free(osqp_data_);
        osqp_data_ = nullptr;
    }
    if (osqp_settings_) {
        c_free(osqp_settings_);
        osqp_settings_ = nullptr;
    }
}

bool HybridMpcCorridorPlannerROS::initOSQPWorkspace() {
    // build persistent OSQP problem data based on current MPC params
    int N = std::max(1, mpc_horizon_);
    int n = 2 * N;

    // construct dense P then convert to upper-triangular CSC
    std::vector<double> P_dense(n * n, 0.0);
    for (int k = 0; k < N; ++k) {
        int vi = k;
        int wi = N + k;
        P_dense[vi * n + vi] += 2.0 * mpc_q_v_;
        P_dense[wi * n + wi] += 2.0 * mpc_q_w_;
    }
    for (int k = 1; k < N; ++k) {
        int v_k = k;
        int v_km = k - 1;
        P_dense[v_k * n + v_k] += 2.0 * mpc_r_v_;
        P_dense[v_km * n + v_km] += 2.0 * mpc_r_v_;
        P_dense[v_k * n + v_km] += -2.0 * mpc_r_v_;
        P_dense[v_km * n + v_k] += -2.0 * mpc_r_v_;

        int w_k = N + k;
        int w_km = N + k - 1;
        P_dense[w_k * n + w_k] += 2.0 * mpc_r_w_;
        P_dense[w_km * n + w_km] += 2.0 * mpc_r_w_;
        P_dense[w_k * n + w_km] += -2.0 * mpc_r_w_;
        P_dense[w_km * n + w_k] += -2.0 * mpc_r_w_;
    }

    // build CSC for P (upper triangular)
    osqp_P_data_.clear(); osqp_P_indices_.clear(); osqp_P_indptr_.assign(n+1, 0);
    int nnzP = 0;
    const double p_tol = 1e-12;
    for (int col = 0; col < n; ++col) {
        osqp_P_indptr_[col] = nnzP;
        for (int row = 0; row <= col; ++row) {
            double val = P_dense[row * n + col];
            if (std::fabs(val) > p_tol) {
                osqp_P_data_.push_back((c_float)val);
                osqp_P_indices_.push_back((c_int)row);
                nnzP++;
            }
        }
    }
    osqp_P_indptr_[n] = nnzP;

    // build identity A as CSC
    osqp_A_data_.clear(); osqp_A_indices_.clear(); osqp_A_indptr_.assign(n+1, 0);
    int nnzA = 0;
    for (int col = 0; col < n; ++col) {
        osqp_A_indptr_[col] = nnzA;
        osqp_A_data_.push_back(1.0);
        osqp_A_indices_.push_back((c_int)col);
        nnzA++;
    }
    osqp_A_indptr_[n] = nnzA;

    // allocate q, lower, upper
    osqp_q_.assign(n, 0.0);
    osqp_lower_.assign(n, 0.0);
    osqp_upper_.assign(n, 0.0);

    // default bounds from limits (will be updated each cycle)
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
    for (int k = 0; k < N; ++k) {
        osqp_lower_[k] = (c_float)limits.min_vel_x;
        osqp_upper_[k] = (c_float)limits.max_vel_x;
        osqp_lower_[N + k] = (c_float)(-limits.max_vel_theta);
        osqp_upper_[N + k] = (c_float)(limits.max_vel_theta);
    }

    // allocate OSQPData and Settings
    osqp_data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    osqp_settings_ = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    if (!osqp_data_ || !osqp_settings_) {
        ROS_WARN("Failed to allocate OSQP data/settings");
        if (osqp_data_) c_free(osqp_data_);
        if (osqp_settings_) c_free(osqp_settings_);
        return false;
    }

    osqp_set_default_settings(osqp_settings_);
    osqp_settings_->verbose = osqp_verbose_;
    osqp_settings_->warm_start = osqp_warm_start_;

    osqp_n_ = n;
    osqp_m_ = n;
    osqp_data_->n = osqp_n_;
    osqp_data_->m = osqp_m_;

    // build csc_matrix wrappers pointing to vector data
    osqp_data_->P = csc_matrix(osqp_data_->n, osqp_data_->n, (c_int)osqp_P_data_.size(), osqp_P_data_.data(), osqp_P_indices_.data(), osqp_P_indptr_.data());
    osqp_data_->A = csc_matrix(osqp_data_->m, osqp_data_->n, (c_int)osqp_A_data_.size(), osqp_A_data_.data(), osqp_A_indices_.data(), osqp_A_indptr_.data());
    osqp_data_->q = osqp_q_.data();
    osqp_data_->l = osqp_lower_.data();
    osqp_data_->u = osqp_upper_.data();

    // setup workspace
    c_int ret = osqp_setup(&osqp_work_, osqp_data_, osqp_settings_);
    if (ret != 0 || osqp_work_ == nullptr) {
        ROS_WARN("osqp_setup failed when creating persistent workspace (ret=%d)", (int)ret);
        if (osqp_work_) osqp_cleanup(osqp_work_);
        if (osqp_data_) {
            if (osqp_data_->A) c_free(osqp_data_->A);
            if (osqp_data_->P) c_free(osqp_data_->P);
            c_free(osqp_data_);
            osqp_data_ = nullptr;
        }
        c_free(osqp_settings_);
        osqp_settings_ = nullptr;
        osqp_initialized_ = false;
        return false;
    }

    osqp_initialized_ = true;
    ROS_DEBUG("Initialized persistent OSQP workspace: n=%d m=%d nnzP=%d nnzA=%d", osqp_n_, osqp_m_, nnzP, nnzA);
    return true;
}

bool HybridMpcCorridorPlannerROS::hybridComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands 此函数是混合局部路径规划器的核心方法，通过动态窗口法（DWA）或其他混合策略计算机器人最优速度指令
    if (!isInitialized()) { //初始化检查
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel); // 从里程计获取机器人当前速度，robot_vel 包含线速度和角速度（存储为 PoseStamped 的位置和方向，需确保 odom_helper_ 正确订阅了里程计话题（如 /odom）

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    // compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID(); 

    // call with updated footprint 调用动态规划器寻找最优路径
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds); //global_pose：机器人全局位姿（来自 costmap），robot_vel：机器人当前速度，drive_cmds：输出参数，存储计算出的最优速度指令
    // ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    // pass along drive commands 速度指令生成
    cmd_vel.linear.x = drive_cmds.pose.position.x; //线速度：从 drive_cmds.pose.position 提取 x 和 y 分量
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation); //角速度：通过 tf2::getYaw 从方向四元数中提取偏航角（绕 Z 轴旋转速度）

    // if we cannot move... tell someone 无效路径处理
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if (path.cost_ < 0) { //表示所有轨迹均被成本函数拒绝（如障碍物过近）
        ROS_DEBUG_NAMED("dwa_local_planner",
                        "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
        local_plan.clear();
        publishLocalPlan(local_plan);
        return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan 局部路径生成与发布
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) { //遍历轨迹点，将每个点转换为 PoseStamped 消息
        double p_x, p_y, p_th;
        path.getPoint(i, p_x, p_y, p_th);

        geometry_msgs::PoseStamped p;
        p.header.frame_id = costmap_ros_->getGlobalFrameID(); //设置全局坐标系（如 map）和当前时间戳
        p.header.stamp = ros::Time::now();
        p.pose.position.x = p_x;
        p.pose.position.y = p_y;
        p.pose.position.z = 0.0;
        tf2::Quaternion q; //通过 tf2::Quaternion 设置偏航角
        q.setRPY(0, 0, p_th);
        tf2::convert(q, p.pose.orientation);
        local_plan.push_back(p);
    }

    // publish information to the visualizer

    publishLocalPlan(local_plan); //通过 publishLocalPlan 发布到 l_plan_pub_ 话题

    // Use internal simple MPC-like tracker and optional OSQP MPC, then blend with HLP output
    geometry_msgs::Twist simple_cmd;
    bool have_simple = computeSimpleMPC(local_plan, simple_cmd);

    geometry_msgs::Twist osqp_cmd;
    bool have_osqp = computeOSQPMPC(local_plan, osqp_cmd);

    // Diagnostic logging to observe MPC/fallback behavior
    if (have_osqp) {
        ROS_INFO("OSQP MPC succeeded: v=%.3f w=%.3f", osqp_cmd.linear.x, osqp_cmd.angular.z);
    } else {
        ROS_WARN("OSQP MPC failed this cycle, attempting simple MPC fallback");
        if (have_simple) {
            ROS_INFO("Simple MPC fallback succeeded: v=%.3f w=%.3f", simple_cmd.linear.x, simple_cmd.angular.z);
        } else {
            ROS_WARN("Simple MPC fallback also failed; using HLP command");
        }
    }

    // hlp command computed earlier from dp_->findBestPath
    geometry_msgs::Twist hlp_cmd;
    hlp_cmd.linear.x = drive_cmds.pose.position.x;
    hlp_cmd.linear.y = drive_cmds.pose.position.y;
    hlp_cmd.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    geometry_msgs::Twist out_cmd;

    if (have_osqp) {
        // pick continuous target alpha by sigmoid mapping based on velocity difference
        double dv_diff = osqp_cmd.linear.x - hlp_cmd.linear.x;
        ROS_INFO("v_osqp - v_hlp = %.6f (sigmoid_thr=%.6f k=%.6f)", dv_diff, alpha_sigmoid_thr_, alpha_sigmoid_k_);
        double abs_d = std::fabs(dv_diff);
        // sigmoid centered at alpha_sigmoid_thr_, with steepness alpha_sigmoid_k_
        double target_alpha = 1.0 / (1.0 + std::exp(-alpha_sigmoid_k_ * (abs_d - alpha_sigmoid_thr_)));
        ROS_DEBUG("alpha_target(sigmoid)=%.6f (abs_d=%.6f)", target_alpha, abs_d);
        // smooth alpha towards target
        ros::Time now = ros::Time::now();
        double dt = std::max(1e-6, (now - last_cmd_time_).toSec());
        double factor = std::min(1.0, dt / std::max(1e-6, blend_tau_));
        alpha_ = alpha_ + (target_alpha - alpha_) * factor;
        ROS_INFO("alpha_target=%.6f alpha_filtered=%.6f (factor=%.6f)", target_alpha, alpha_, factor);

        // blended command between hlp and osqp
        out_cmd.linear.x = alpha_ * osqp_cmd.linear.x + (1.0 - alpha_) * hlp_cmd.linear.x;
        out_cmd.linear.y = alpha_ * osqp_cmd.linear.y + (1.0 - alpha_) * hlp_cmd.linear.y;
        out_cmd.angular.z = alpha_ * osqp_cmd.angular.z + (1.0 - alpha_) * hlp_cmd.angular.z;
    } else if (have_simple) {
        // no OSQP result -> fallback to internal simple MPC
        out_cmd = simple_cmd;
        alpha_ = 0.0;
    } else {
        // last resort: use hlp cmd
        out_cmd = hlp_cmd;
        alpha_ = 0.0;
    }

    // enforce acceleration limits to ensure smooth acceleration
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
    ros::Time now = ros::Time::now();
    double dt = std::max(1e-6, (now - last_cmd_time_).toSec());
    double max_dv = limits.acc_lim_x * dt;
    double max_dw = limits.acc_lim_theta * dt;

    double dv = out_cmd.linear.x - last_cmd_vel_.linear.x;
    if (dv > max_dv) out_cmd.linear.x = last_cmd_vel_.linear.x + max_dv;
    if (dv < -max_dv) out_cmd.linear.x = last_cmd_vel_.linear.x - max_dv;

    double dw = out_cmd.angular.z - last_cmd_vel_.angular.z;
    if (dw > max_dw) out_cmd.angular.z = last_cmd_vel_.angular.z + max_dw;
    if (dw < -max_dw) out_cmd.angular.z = last_cmd_vel_.angular.z - max_dw;

    // update last command and time
    last_cmd_time_ = now;
    last_cmd_vel_ = out_cmd;

    cmd_vel = out_cmd;
    ROS_DEBUG("Using blended command alpha=%.3f (vx=%.3f, wz=%.3f)", alpha_, cmd_vel.linear.x, cmd_vel.angular.z);

    return true;
}

// A simple, robust tracking controller that mimics MPC behavior without external deps.
// It selects a lookahead point on the local_plan and computes a proportional velocity command.
bool HybridMpcCorridorPlannerROS::computeSimpleMPC(const std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::Twist& cmd_vel) {
    if (plan.empty()) return false;

    // get current robot pose in same frame
    geometry_msgs::PoseStamped robot_pose = current_pose_; // current_pose_ is maintained in class

    // ROS_INFO("computeSimpleMPC: robot_pose frame='%s' stamp=%.9f (x=%.3f,y=%.3f)", robot_pose.header.frame_id.c_str(), robot_pose.header.stamp.toSec(), robot_pose.pose.position.x, robot_pose.pose.position.y);

    // log first few plan points for debugging frame mismatches
    for (size_t i = 0; i < std::min<size_t>(3, plan.size()); ++i) {
        const auto &pp = plan[i];
        // ROS_INFO("computeSimpleMPC: plan[%zu] frame='%s' stamp=%.9f (x=%.3f,y=%.3f)", i, pp.header.frame_id.c_str(), pp.header.stamp.toSec(), pp.pose.position.x, pp.pose.position.y);
    }

    double rx = robot_pose.pose.position.x;
    double ry = robot_pose.pose.position.y;
    double rtheta = tf2::getYaw(robot_pose.pose.orientation);

    // parameters (could be moved to ROS params)
    double lookahead = 0.6; // meters
    double k_v = 0.8;       // linear gain
    double k_w = 1.5;       // angular gain

    // find a lookahead point using cumulative arc length along the plan (more robust than per-point Euclidean)
    size_t target_idx = plan.size() - 1;
    double cumulative = 0.0;
    double max_dist = 0.0;
    size_t max_idx = 0;

    // previous point starts at robot position
    double prev_x = rx;
    double prev_y = ry;
    for (size_t i = 0; i < plan.size(); ++i) {
        // transform to robot frame if needed
        geometry_msgs::PoseStamped p_local = plan[i];
        if (p_local.header.frame_id != robot_pose.header.frame_id) {
            try {
                p_local.header.stamp = plan[i].header.stamp;
                tf_->transform(plan[i], p_local, robot_pose.header.frame_id);
            } catch (tf2::TransformException &ex) {
                // ROS_WARN("computeSimpleMPC: failed to transform plan[%zu] from %s to %s: %s", i, plan[i].header.frame_id.c_str(), robot_pose.header.frame_id.c_str(), ex.what());
                continue;
            }
        }

        double px = p_local.pose.position.x;
        double py = p_local.pose.position.y;
        double seg = std::hypot(px - prev_x, py - prev_y);
        cumulative += seg;

        double eucl = std::hypot(px - rx, py - ry);
        if (eucl > max_dist) { max_dist = eucl; max_idx = i; }

        if (cumulative >= lookahead) { target_idx = i; break; }

        prev_x = px; prev_y = py;
    }

    // if none reached lookahead, pick the farthest point instead of blindly using last element
    geometry_msgs::PoseStamped target_pose = plan[target_idx];
    if (target_idx == plan.size() - 1 && max_dist > 1e-6) {
        target_idx = max_idx;
        target_pose = plan[target_idx];
    }
    if (target_pose.header.frame_id != robot_pose.header.frame_id) {
        try {
            tf_->transform(plan[target_idx], target_pose, robot_pose.header.frame_id);
            // ROS_INFO("computeSimpleMPC: transformed target index %zu into frame '%s' -> (%.3f, %.3f)", target_idx, target_pose.header.frame_id.c_str(), target_pose.pose.position.x, target_pose.pose.position.y);
        } catch (tf2::TransformException &ex) {
            // ROS_WARN("computeSimpleMPC: failed to transform target point %zu: %s", target_idx, ex.what());
            return false;
        }
    }

    double tx = target_pose.pose.position.x;
    double ty = target_pose.pose.position.y;
    double dx = tx - rx;
    double dy = ty - ry;
    double dist = std::hypot(dx, dy);
    if (!(std::isfinite(dist) && dist > 1e-6)) return false;

    double desired_yaw = std::atan2(dy, dx);
    double err_yaw = angles::shortest_angular_distance(rtheta, desired_yaw);

    // limit velocities by planner limits
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
    double max_v = limits.max_vel_x;
    double max_w = limits.max_vel_theta;

    double v = std::min(max_v, k_v * dist);
    double w = std::max(-max_w, std::min(max_w, k_w * err_yaw));

    // If target is very close, avoid aggressive rotation-in-place: reduce angular response and allow a small forward motion
    const double very_close_dist = 0.06; // 6 cm
    const double close_dist = 0.15; // 15 cm
    if (dist <= very_close_dist) {
        // make rotation gentler and keep a tiny forward velocity to escape oscillation
        w = std::max(-0.6, std::min(0.6, w * 0.5));
        if (v < 0.02) v = 0.02;
    } else if (dist <= close_dist) {
        w *= 0.6; // reduce turning when quite close
    }

    // ROS_INFO("computeSimpleMPC: target_idx=%zu tx=%.3f ty=%.3f dist=%.3f desired_yaw=%.3f err_yaw=%.3f max_v=%.3f max_w=%.3f -> v=%.3f w=%.3f",
    //          target_idx, tx, ty, dist, desired_yaw, err_yaw, max_v, max_w, v, w);

    // small slow-down when angle error big
    if (std::fabs(err_yaw) > 0.7) v *= 0.25;
    if (std::fabs(err_yaw) > 0.4) v *= 0.6;

    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = w;

    return true;
}

// Small OSQP-based QP MPC: solve for (v, w) to track a reference (v_ref, w_ref)
bool HybridMpcCorridorPlannerROS::computeOSQPMPC(const std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::Twist& cmd_vel) {
    if (plan.empty()) return false;

    // read limits and params
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
    double dt = (mpc_dt_ > 0.0) ? mpc_dt_ : dp_->getSimPeriod();

    // previous velocity from odom
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    double prev_v = robot_vel.pose.position.x;
    double prev_w = tf2::getYaw(robot_vel.pose.orientation);

    // reference from simple MPC (robust fallback)
    geometry_msgs::Twist ref_cmd;
    if (!computeSimpleMPC(plan, ref_cmd)) {
        ref_cmd.linear.x = prev_v;
        ref_cmd.angular.z = prev_w;
    }
    double v_ref = ref_cmd.linear.x;
    double w_ref = ref_cmd.angular.z;

    int N = std::max(1, mpc_horizon_);
    int n = 2 * N; // variables: [v0..vN-1, w0..wN-1]

    // ensure persistent workspace is initialized (try on-demand)
    if (!osqp_initialized_) {
        if (!initOSQPWorkspace()) {
            ROS_WARN("OSQP persistent workspace not available and on-demand init failed.");
            return false;
        }
    }

    // update linear cost q based on current reference
    osqp_q_.assign(n, 0.0);
    for (int k = 0; k < N; ++k) {
        osqp_q_[k] = (c_float)(-2.0 * mpc_q_v_ * v_ref);
        osqp_q_[N + k] = (c_float)(-2.0 * mpc_q_w_ * w_ref);
    }

    // update bounds
    for (int k = 0; k < N; ++k) {
        osqp_lower_[k] = (c_float)limits.min_vel_x;
        osqp_upper_[k] = (c_float)limits.max_vel_x;
        osqp_lower_[N + k] = (c_float)(-limits.max_vel_theta);
        osqp_upper_[N + k] = (c_float)(limits.max_vel_theta);
    }

    // update q and bounds in the workspace (warm-start)
    osqp_update_lin_cost(osqp_work_, osqp_q_.data());
    osqp_update_bounds(osqp_work_, osqp_lower_.data(), osqp_upper_.data());

    // solve
    osqp_solve(osqp_work_);
    c_int status = osqp_work_ ? osqp_work_->info->status_val : -1;
    if ((status < 0) || (status != 1 && status != 2)) {
        ROS_WARN("OSQP persistent solve failed (status=%d)", (int)status);
        return false;
    }

    double v0 = osqp_work_->solution->x[0];
    double w0 = osqp_work_->solution->x[N];
    ROS_INFO("OSQP MPC solution: v0=%.3f w0=%.3f", v0, w0);

    cmd_vel.linear.x = v0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = w0;
    return true;
}

bool HybridMpcCorridorPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal 此函数是混合局部规划器的核心方法，负责根据机器人当前位置和路径规划结果，选择动态窗口法（DWA）或停止旋转控制策略，生成速度指令。
    if (!costmap_ros_->getRobotPose(current_pose_)) { //获取机器人当前位姿
        ROS_ERROR("Could not get robot pose");
        return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) { //将全局路径（global_plan）映射到机器人当前的局部坐标系（如odom）中，确保路径在局部代价地图（costmap）的可操作范围内。通过TF（Transform Library）获取全局路径与机器人当前位姿的坐标变换关系，将全局路径的每个点转换到局部坐标系，截断超出局部地图范围的路径点
        ROS_ERROR("Could not get local plan"); //planner_util_ 是路径规划工具类，负责坐标变换和路径裁剪，转换后的路径仅包含机器人附近的关键点，减少计算量
        return false;
    }

    // if the global plan passed in is empty... we won't do anything 路径有效性检查
    if (transformed_plan.empty()) {  //若转换后路径为空，说明无有效路径可跟踪
        ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
        return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory 更新规划器路径与代价
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) { //目标位置到达判断，若是，执行（computeVelocityCommandsStopRotate）
        // publish an empty plan because we've reached our goal position 发布空路径：清除 RViz 中的路径显示，避免误导
        std::vector<geometry_msgs::PoseStamped> local_plan; // 发布空局部路径
        std::vector<geometry_msgs::PoseStamped> transformed_plan; // 发布空全局路径 片段
        publishGlobalPlan(transformed_plan);
        publishLocalPlan(local_plan);
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits(); //获取机器人当前的运动学限制参数，如最大加速度、规划周期等
        return latchedStopRotateController_.computeVelocityCommandsStopRotate( //用于机器人接近目标时的减速停止与朝向调整的核心函数，当机器人已接近导航目标时（如位置误差小于xy_goal_tolerance），根据加速度限制逐步降低线速度至零，避免急停，若机器人尚未达到目标朝向容忍度（yaw_goal_tolerance），则控制机器人原地旋转至目标方向
            cmd_vel,
            limits.getAccLimits(),
            dp_->getSimPeriod(),
            &planner_util_, //指向规划工具的指针，用于获取规划器的辅助功能
            odom_helper_, //里程计帮助对象，用于处理里程计信息
            current_pose_,
            [this](auto pos, auto vel, auto vel_samples) { return dp_->checkTrajectory(pos, vel, vel_samples); }); //通过 Lambda 表达式调用 dp_->checkTrajectory，确保旋转过程中无碰撞
    } else { //未到达目标位置：调用 hybridComputeVelocityCommands 生成速度指令
        bool isOk = hybridComputeVelocityCommands(current_pose_, cmd_vel);
        if (isOk) {
            publishGlobalPlan(transformed_plan); //成功时发布全局路径
        } else {
            ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path."); //失败时发布空路径并输出警告
            std::vector<geometry_msgs::PoseStamped> empty_plan;
            publishGlobalPlan(empty_plan);
        }
        return isOk;
    }
}

};  // namespace hlpmpccorridor_local_planner
