/**
 * @file hybrid_planner_ros.h
 * @brief ROS wrapper (nav_core::BaseLocalPlanner) for the Hybrid MPC corridor local planner.
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

#ifndef HYBRID_LOCAL_PLANNER_HYBRID_PLANNER_ROS_H_
#define HYBRID_LOCAL_PLANNER_HYBRID_PLANNER_ROS_H_ //防止头文件被重复包含

#include <angles/angles.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/DWAPlannerConfig.h>  // Hybrid planner uses the same config file as DWA planner
#include <dynamic_reconfigure/server.h>
#include <hybrid_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>

#include <osqp/osqp.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>

// forward-declare MPCController to avoid including its header in this header file
namespace rpp {
namespace controller {
class MPCController;
}
}

namespace hlpmpccorridor_local_planner {
/**
 * @class HybridMpcCorridorPlannerROS
 * @brief ROS Wrapper for the HybridPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class HybridMpcCorridorPlannerROS : public nav_core::BaseLocalPlanner { //ROS wrapper with corridor support
   public:
    /**
     * @brief  Constructor for DWAPlannerROS wrapper
     */
    HybridMpcCorridorPlannerROS();

    /**
     * @brief  Constructs the ros wrapper
     * @param name The name to give this instance of the trajectory planner
     * @param tf A pointer to a transform listener
     * @param costmap The cost map to use for assigning costs to trajectories
     */
    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief  Destructor for the wrapper
     */
    ~HybridMpcCorridorPlannerROS();

    /**
     * @brief  Given the current position, orientation, and velocity of the robot,
     * compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid trajectory was found, false otherwise
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Given the current position, orientation, and velocity of the robot,
     * compute velocity commands to send to the base, using dynamic window approach
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid trajectory was found, false otherwise
     */
    bool hybridComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Set the plan that the controller is following
     * @param orig_global_plan The plan to pass to the controller
     * @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
     * @brief  Check if the goal pose has been achieved
     * @return True if achieved, false otherwise
     */
    bool isGoalReached();

    bool isInitialized() {
        return initialized_;
    }

    // Simple internal MPC-like tracker used as a fallback when external MPC is unavailable
    bool computeSimpleMPC(const std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::Twist& cmd_vel);
    // Try to compute commands via OSQP-based MPC. Returns true on successful solve.
    bool computeOSQPMPC(const std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::Twist& cmd_vel);

    // initialize persistent OSQP workspace (called from initialize())
    bool initOSQPWorkspace();

   private:
    /**
     * @brief Callback to update the local planner's parameters based on dynamic reconfigure
     */
    void reconfigureCB(dwa_local_planner::DWAPlannerConfig& config, uint32_t level);

    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    tf2_ros::Buffer* tf_;  ///< @brief Used for transforming point clouds

    // for visualisation, publishers of global and local plan
    ros::Publisher g_plan_pub_, l_plan_pub_;

    base_local_planner::LocalPlannerUtil planner_util_;

    boost::shared_ptr<HybridPlanner> dp_;  ///< @brief The trajectory controller

    costmap_2d::Costmap2DROS* costmap_ros_;
    // internal simple MPC-like tracker (no external dependencies)

    dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>* dsrv_;
    dwa_local_planner::DWAPlannerConfig default_config_;
    bool setup_;
    geometry_msgs::PoseStamped current_pose_;

    base_local_planner::LatchedStopRotateController latchedStopRotateController_; //用于机器人导航的控制器，主要负责在机器人接近目标位置时的停止和旋转控制。它通过检测机器人是否到达目标位置，并在必要时发出停止和旋转命令，确保机器人能够以一种稳定的方式停止并调整姿态，最终精确对准目标方向

    bool initialized_;

    base_local_planner::OdometryHelperRos odom_helper_;
    std::string odom_topic_;
    // blending and smoothing state used for hybrid speed selection
    double blend_tau_;            // time constant for alpha smoothing (seconds)
    double v_switch_threshold_;   // threshold on velocity diff to prefer MPC
    double alpha_;                // current blend factor [0..1], 1 -> favor MPC
    ros::Time last_cmd_time_;
    geometry_msgs::Twist last_cmd_vel_; // last output command used for accel continuity
    // MPC tuning parameters
    int mpc_horizon_;
    double mpc_dt_;
    double mpc_q_v_, mpc_q_w_;
    double mpc_r_v_, mpc_r_w_;
    bool osqp_verbose_;
    bool osqp_warm_start_;
    // Sigmoid mapping params for alpha target (continuous switching)
    double alpha_sigmoid_k_;   // steepness
    double alpha_sigmoid_thr_; // threshold (center) in m/s
    // Persistent OSQP workspace and data for warm-start
    OSQPWorkspace* osqp_work_;
    OSQPData* osqp_data_;
    OSQPSettings* osqp_settings_;
    std::vector<c_float> osqp_P_data_;
    std::vector<c_int> osqp_P_indices_;
    std::vector<c_int> osqp_P_indptr_;
    std::vector<c_float> osqp_A_data_;
    std::vector<c_int> osqp_A_indices_;
    std::vector<c_int> osqp_A_indptr_;
    std::vector<c_float> osqp_q_;
    std::vector<c_float> osqp_lower_;
    std::vector<c_float> osqp_upper_;
    int osqp_n_;
    int osqp_m_;
    bool osqp_initialized_;
};
};  // namespace hlpmpccorridor_local_planner
#endif
