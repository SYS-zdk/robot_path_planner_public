/**
 * @file hybrid_planner.cpp
 * @brief Implementation of the HybridPlanner (trajectory generation, scoring and selection).
 * @author Zhang Dingkun (integration/adaptation)
 * @date 2026-03-17
 * @version 1.0
 */

#include <base_local_planner/goal_functions.h>
#include <hybrid_planner.h>

#include <cmath>

#include <Edge.h>
#include <Node.h>
#include <Helper.h>
#include <AStar_Node.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_obstacle_cost.h>

#include <pedsim_msgs/TrackedPersons.h>

// safety corridor
#include <common/safety_corridor/convex_safety_corridor.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <map>
#include <queue>
#include <string>

namespace hlpmpccorridor_local_planner {
void HybridPlanner::reconfigure(dwa_local_planner::DWAPlannerConfig &config) { //用于动态调整混合规划器的参数，通过 ROS 动态配置接口 (DWAPlannerConfig) 更新内部模块的配置。该函数在多线程环境中通过互斥锁保证线程安全
    boost::mutex::scoped_lock l(configuration_mutex_); //线程安全保护，通过互斥锁 configuration_mutex_ 确保配置更新过程的原子性，避免多线程竞争

    generator_.setParameters(  // 更新轨迹生成器参数，sim_time：轨迹模拟的总时间（秒），sim_granularity：轨迹模拟的时间步长（米/步），angular_sim_granularity：角度模拟的步长（弧度/步），use_dwa：是否启用动态窗口法（DWA）速度限制，sim_period_：规划周期（秒）
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    //路径对齐成本权重计算
    double resolution = planner_util_->getCostmap()->getResolution(); //代价地图的分辨率（米/像素），用于将配置参数转换为物理单位
    path_distance_bias_ = resolution * config.path_distance_bias; //路径对齐的权重系数，最终权重为 分辨率 × 配置值
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(path_distance_bias_); //path_costs_ 和 alignment_costs_ 共用同一缩放因子，确保路径跟踪和对齐的权重一致
    alignment_costs_.setScale(path_distance_bias_);

    //目标接近度成本权重
    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_); //当前位姿到目标的距离成本
    goal_front_costs_.setScale(goal_distance_bias_); //机器人前方某点（forward_point_distance_）到目标的距离成本

    //障碍物避让成本权重
    occdist_scale_ = config.occdist_scale; //occdist_scale 直接控制障碍物距离的惩罚强度。值越大，机器人越倾向于远离障碍物
    obstacle_costs_.setScale(occdist_scale_);

    //震荡检测与重置参数
    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_); //在机器人前方 forward_point_distance_ 米处设置虚拟目标点，引导机器人朝向目标方向移动
    alignment_costs_.setXShift(forward_point_distance_);

    // obstacle costs can vary due to scaling footprint feature 动态障碍物避让参数   max_scaling_factor：最大缩放比例，控制机器人靠近障碍物时的减速幅度；scaling_speed：缩放速度阈值，低于此速度时不启用避让缩放
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed); //max_vel_trans：最大平移速度，用于计算动态障碍物避让的缩放因子

    twirling_costs_.setScale(config.twirling_scale); //抑制机器人原地旋转（Twirling），twirling_scale 值越大，旋转惩罚越强

    // 速度采样数量验证与修正
    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;
    // 检查并修正无效的采样数量
    if (vx_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
        vx_samp = 1;
        config.vx_samples = vx_samp;
    }

    if (vy_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
        vy_samp = 1;
        config.vy_samples = vy_samp;
    }

    if (vth_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
        vth_samp = 1;
        config.vth_samples = vth_samp;
    }

    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
}

//构造函数负责初始化混合路径规划器的核心组件，包括代价地图、成本函数、可视化工具和 ROS 通信接口
HybridPlanner::HybridPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) : planner_util_(planner_util),
                                                                                                     obstacle_costs_(planner_util->getCostmap()),
                                                                                                     path_costs_(planner_util->getCostmap()),
                                                                                                     goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
                                                                                                     goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
                                                                                                     alignment_costs_(planner_util->getCostmap())
                                                                                                      {

    ros::NodeHandle private_nh("~/" + name); //ROS 节点与可视化初始化

    // 正确：构造函数体内赋值发布者
    current_pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    // current pose for local planner should be in local/global frame (default to odom here)
    current_pose_.header.frame_id = std::string("odom");

    step_count = 0;
    ros::Rate r(10); //3
    marker_pub = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10); //发布路径标记

    // 初始化四元数
    astar_path.pose.orientation.w = 1.0f;
    astar_path.pose.orientation.x = 0.0f;
    astar_path.pose.orientation.y = 0.0f;
    astar_path.pose.orientation.z = 0.0f;

    astar_path.header.frame_id = "map"; // 全局坐标系 for global visualization
    astar_path.type = visualization_msgs::Marker::LINE_LIST; // 线列表类型
    astar_path.id = 1;
    astar_path.color.g = 1.0f;
    astar_path.color.a = 1.0;
    astar_path.scale.x = 0.05; // 线宽
    astar_path.scale.y = 0.05;
    astar_path.ns = "points_and_lines";

    goal_front_costs_.setStopOnFailure(false);
    alignment_costs_.setStopOnFailure(false);

    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.

    std::string controller_frequency_param_name;
    if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) { //优先从 ROS 参数服务器获取 controller_frequency
        sim_period_ = 0.05; // 默认 20Hz
    } else {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if (controller_frequency > 0) {
            sim_period_ = 1.0 / controller_frequency;
        } else { //频率 ≤ 0 时使用默认值，避免除零错误
            ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            sim_period_ = 0.05;
        }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags(); //resetOscillationFlags：重置震荡检测状态（如位移累计值）

    bool sum_scores; //sum_scores：控制障碍物成本是否为累加模式（如 true 时多个障碍物成本叠加）
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);

    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, true);
    map_viz_.initialize(name,
                        planner_util->getGlobalFrame(),
                        [this](int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
                            return getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost); //通过 Lambda 表达式将 getCellCosts 绑定为代价网格的数据源
                        });

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));

    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1); //发布轨迹点云，用于候选轨迹的可视化分析
    private_nh.param("publish_traj_pc", publish_traj_pc_, true);

    // 非阻塞可视化参数与发布器（使用 private_nh 绑定到 planner 命名空间）
    private_nh.param("viz_rate", viz_rate_, 10.0);
    graph_pub_global_ = private_nh.advertise<visualization_msgs::MarkerArray>("graph_visualization", 1);
    astar_pub_global_ = private_nh.advertise<visualization_msgs::Marker>("visualization_marker_global", 1);
    // 额外创建根命名空间发布器，兼容原始实现发布到 /graph_visualization（非私有命名空间）的情况
    {
        ros::NodeHandle nh_root; // 全局命名空间
        graph_pub_root_ = nh_root.advertise<visualization_msgs::MarkerArray>("graph_visualization", 1);
        astar_pub_root_ = nh_root.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }
    have_viz_data_ = false;
    if (viz_rate_ > 0.001) {
        viz_timer_ = private_nh.createTimer(ros::Duration(1.0 / viz_rate_), &HybridPlanner::vizTimerCB, this);
    }

    // safety corridor publisher
    corridor_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("safety_corridor", 1);

    // read safety range parameter
    private_nh.param("safety_corridor_range", safety_range_, 0.6);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    // std::vector<base_local_planner::TrajectoryCostFunction *> critics;
    critics.push_back(&oscillation_costs_);  // discards oscillating motions (assisgns cost -1) // 震荡检测
    critics.push_back(&obstacle_costs_);     // discards trajectories that move into obstacles // 碰撞检测
    critics.push_back(&goal_front_costs_);   // prefers trajectories that make the nose go towards (local) nose goal // 目标前方对齐
    critics.push_back(&alignment_costs_);    // prefers trajectories that keep the robot nose on nose path// 路径朝向对齐
    critics.push_back(&path_costs_);         // prefers trajectories on global path // 全局路径跟随
    critics.push_back(&goal_costs_);         // prefers trajectories that go towards (local) goal, based on wave propagation 目标接近
    critics.push_back(&twirling_costs_);     // optionally prefer trajectories that don't spin 旋转抑制

    critics.push_back(&dynamic_obstacle_costs_);

    double social_scale;
    private_nh.param("social_scale", social_scale, 0.0);
    social_costs_.setScale(social_scale);
    critics.push_back(&social_costs_);

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator *> generator_list; //轨迹生成器列表构建
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics); //评分规划器初始化，按顺序调用 critics 列表中的每个评估器计算轨迹成本

    private_nh.param("cheat_factor", cheat_factor_, 1.0); //作弊因子参数加载

    double dyn_scale;
    private_nh.param("dynamic_obstacle_scale", dyn_scale, 0.0);
    dynamic_obstacle_costs_.setScale(dyn_scale);
    dynamic_obstacle_costs_.setSimPeriod(sim_period_);
    double robot_radius;
    private_nh.param("robot_radius", robot_radius, 0.3);
    dynamic_obstacle_costs_.setRobotRadius(robot_radius);

    // Extra safety buffer (m) beyond robot_radius when checking predicted pedestrian collisions.
    // If predicted dist < robot_radius + dynamic_safe_distance, the trajectory is rejected.
    double dynamic_safe_distance;
    private_nh.param("dynamic_safe_distance", dynamic_safe_distance, 0.6);
    dynamic_obstacle_costs_.setSafeDistance(dynamic_safe_distance);
    dynamic_obstacle_costs_.setDynamicPeds(&dynamic_peds_);

    ped_sub_ = private_nh.subscribe("/ped_visualization", 1, &HybridPlanner::pedCallback, this);
}

// used for visualization only, total_costs are not really total costs 计算指定地图单元格的各类成本，用于可视化（非实际总成本）
bool HybridPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) { //cx, cy：单元格坐标；path_cost：路径距离成本；goal_cost：目标距离成本；occ_cost：占据成本；total_cost：加权总和，仅用于显示
    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }

    total_cost =
        path_distance_bias_ * path_cost +
        goal_distance_bias_ * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
} //逻辑：获取各成本值-->检查可行性-->计算总成本

bool HybridPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
    oscillation_costs_.resetOscillationFlags(); //重置震荡检测标志
    return planner_util_->setPlan(orig_global_plan); //调用 planner_util_->setPlan 更新全局路径
}

void HybridPlanner::pedCallback(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(ped_mutex_);
    dynamic_peds_.clear();
    for (const auto& person : msg->tracks)
    {
        DynamicPed p;
        p.x = person.pose.pose.position.x;
        p.y = person.pose.pose.position.y;
        p.vx = person.twist.twist.linear.x;
        p.vy = person.twist.twist.linear.y;
        dynamic_peds_.push_back(p);
    }
}

void HybridPlanner::setCostmapRos(costmap_2d::Costmap2DROS* costmap_ros) {
    if (!costmap_ros) return;
    // construct safety corridor using costmap wrapper and configured safety range
    rpp::AckermannConfig ack_cfg;
    ack_cfg.wheelbase = 0.3;
    ack_cfg.max_steer_angle = 0.83;
    ack_cfg.track_width = 0.3;
    safety_corridor_ = std::make_unique<rpp::common::safety_corridor::ConvexSafetyCorridor>(costmap_ros, safety_range_, ack_cfg);

    // wire social layer access for trajectory scoring (does not write to master costmap)
    social_costs_.setLayeredCostmap(costmap_ros->getLayeredCostmap());
}

/**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused. 验证给定速度样本生成的轨迹是否合法
 */
bool HybridPlanner::checkTrajectory(
    Eigen::Vector3f pos, //当前位置（包括x, y, theta）
    Eigen::Vector3f vel, //当前速度（包括线速度vx, vy，角速度vtheta） vel_samples待检查的速度样本
    Eigen::Vector3f vel_samples) {
    oscillation_costs_.resetOscillationFlags(); //重置震荡检测的状态
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back(); //从 global_plan_ 中获取最后一个点作为目标点，并将其转换为 Eigen::Vector3f 格式
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
                          vel,
                          goal,
                          &limits,
                          vsamples_); //调用 generator_.initialise()，传入当前位置、速度、目标点、规划限制和速度采样参数
    generator_.generateTrajectory(pos, vel, vel_samples, traj); //生成一条基于当前状态和速度样本的轨迹
    // If safety corridor is available, verify every sampled point of the
    // generated trajectory lies within some corridor polygon. If any point
    // falls outside, reject the trajectory early.
    if (safety_corridor_ && !corridor_polygons_.empty()) {
        for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
            double px, py, pth;
            traj.getPoint(i, px, py, pth);
            rpp::common::geometry::Vec2d pt(px, py);
            bool in_any = false;
            for (const auto &poly : corridor_polygons_) {
                if (poly.isPointIn(pt) || poly.isPointOnBoundary(pt)) {
                    in_any = true;
                    break;
                }
            }
            if (!in_any) {
                // Trajectory leaves the corridor — reject.
                return false;
            }
        }
    }

    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1); //计算轨迹的成本
    // if the trajectory is a legal one... the check passes
    if (cost >= 0) {
        return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    // otherwise the check fails
    return false;
}

void HybridPlanner::updatePlanAndLocalCosts( //更新全局plan，并调整局部代价函数
    const geometry_msgs::PoseStamped &global_pose, //全局坐标系下的机器人姿态
    const std::vector<geometry_msgs::PoseStamped> &new_plan, //new_plan：新的全局路径
    const std::vector<geometry_msgs::Point> &footprint_spec) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
        global_plan_[i] = new_plan[i];
    }

    obstacle_costs_.setFootprint(footprint_spec); //设置机器人的足印，用于障碍物检测

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_); //设置路径代价函数的目标点为新的全局路径

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_); //设置目标代价函数的目标点为新的全局路径

    // alignment costs
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y); //计算当前机器人位置与目标点的位置距离 sq_dist

    // we want the robot nose to be drawn to its final position 当机器人接近路径终点时，若目标点位于机器人中心正后方，直接转向可能导致路径抖动或需要180度大角度旋转，尤其在狭窄空间或复杂路径中。
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end  解决方案：将目标点沿机器人到终点的方向前移一定距离（forward_point_distance_），生成一个虚拟的“前向目标点”，引导机器人提前调整方向。
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_; //复制全局路径：创建 front_global_plan 作为全局路径的副本，避免修改原始路径
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]); //计算机器人当前位置到原始目标点的方位角（全局坐标系下）
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
                                               forward_point_distance_ * cos(angle_to_goal); //将路径终点沿 angle_to_goal 方向前移 forward_point_distance_ 距离
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
                                                                                              sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan); //设置前端目标成本函数的目标点，使路径规划倾向于让机器人前端对齐此点

    // keeping the nose on the path
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) { //如果距离大于 forward_point_distance_ * cheat_factor_，则启用路径对齐代价函数
        alignment_costs_.setScale(path_distance_bias_);
        // costs for robot being aligned with path (nose on path, not ju
        alignment_costs_.setTargetPoses(global_plan_);
    } else {
        // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.否则，禁用路径对齐代价函数，以避免在接近目标时的不稳定行为
        alignment_costs_.setScale(0.0);
    }

    // Build safety corridor (if available) based on the new plan and publish markers
    if (safety_corridor_) {
        // convert plan to Points3d
        rpp::common::geometry::Points3d pts;
        pts.reserve(global_plan_.size());
        for (const auto &p : global_plan_) {
            rpp::common::geometry::Point3d pp;
            pp.setX(p.pose.position.x);
            pp.setY(p.pose.position.y);
            pp.setTheta(tf2::getYaw(p.pose.orientation));
            pts.push_back(pp);
        }
        corridor_polygons_.clear();
        if (safety_corridor_->decompose(pts, corridor_polygons_)) {
            visualization_msgs::MarkerArray ma;
            int id = 0;
            // compute lifetime based on viz_rate_
            double viz_period = (viz_rate_ > 0.001) ? (1.0 / viz_rate_) : 0.2;
            ros::Duration marker_life = ros::Duration(std::max(0.05, viz_period * 1.5));
            for (const auto &poly : corridor_polygons_) {
                // Outline marker (keep existing line visualization)
                visualization_msgs::Marker outline;
                outline.header.frame_id = planner_util_->getGlobalFrame();
                outline.header.stamp = ros::Time::now();
                outline.ns = "safety_corridor";
                outline.id = id++;
                outline.type = visualization_msgs::Marker::LINE_STRIP;
                outline.action = visualization_msgs::Marker::ADD;
                outline.scale.x = 0.03;
                outline.color.r = 0.2;
                outline.color.g = 0.6;
                outline.color.b = 0.2;
                outline.color.a = 0.8;
                outline.lifetime = marker_life;

                // Filled marker - use TRIANGLE_LIST and fan-triangulate from centroid
                visualization_msgs::Marker fill;
                fill.header.frame_id = planner_util_->getGlobalFrame();
                fill.header.stamp = ros::Time::now();
                fill.ns = "safety_corridor_filled";
                fill.id = id++;
                fill.type = visualization_msgs::Marker::TRIANGLE_LIST;
                fill.action = visualization_msgs::Marker::ADD;
                fill.color.r = 0.4;
                fill.color.g = 0.8;
                fill.color.b = 0.4;
                fill.color.a = 0.35; // make a bit more visible for debugging
                // ensure scale non-zero (some viewers ignore otherwise)
                fill.scale.x = 1.0;
                fill.scale.y = 1.0;
                fill.scale.z = 1.0;
                fill.pose.orientation.w = 1.0;
                fill.lifetime = marker_life;

                // collect points (use Vec2d)
                std::vector<rpp::common::geometry::Vec2d> pts;
                pts.reserve(poly.points().size());
                for (const auto &v : poly.points()) {
                    rpp::common::geometry::Vec2d p;
                    p.setX(v.x());
                    p.setY(v.y());
                    pts.push_back(p);
                }

                // compute centroid
                    if (pts.size() >= 3) {
                    double cx = 0.0, cy = 0.0;
                    for (const auto &p : pts) { cx += p.x(); cy += p.y(); }
                    cx /= (double)pts.size();
                    cy /= (double)pts.size();
                    geometry_msgs::Point cpt;
                    cpt.x = cx; cpt.y = cy; cpt.z = 0.0;

                    // add outline points
                    for (const auto &p : pts) {
                        geometry_msgs::Point gp;
                        gp.x = p.x(); gp.y = p.y(); gp.z = 0.0;
                        outline.points.push_back(gp);
                    }
                    if (!outline.points.empty()) outline.points.push_back(outline.points.front());

                    // triangulate fan from centroid
                    for (size_t i = 0; i < pts.size(); ++i) {
                        const auto &p1 = pts[i];
                        const auto &p2 = pts[(i + 1) % pts.size()];
                        geometry_msgs::Point gp1, gp2;
                        gp1.x = p1.x(); gp1.y = p1.y(); gp1.z = 0.0;
                        gp2.x = p2.x(); gp2.y = p2.y(); gp2.z = 0.0;
                        // triangle (centroid, p1, p2)
                        fill.points.push_back(cpt);
                        fill.points.push_back(gp1);
                        fill.points.push_back(gp2);
                        // push per-vertex color to ensure RViz respects it
                        std_msgs::ColorRGBA vcol;
                        vcol.r = fill.color.r; vcol.g = fill.color.g; vcol.b = fill.color.b; vcol.a = fill.color.a;
                        fill.colors.push_back(vcol);
                        fill.colors.push_back(vcol);
                        fill.colors.push_back(vcol);
                    }

                    ma.markers.push_back(fill);
                    ma.markers.push_back(outline);
                } else {
                    // fallback: just draw outline if not enough points
                    for (const auto &v : poly.points()) {
                        geometry_msgs::Point gp;
                        gp.x = v.x(); gp.y = v.y(); gp.z = 0.0;
                        outline.points.push_back(gp);
                    }
                    if (!outline.points.empty()) outline.points.push_back(outline.points.front());
                    outline.lifetime = marker_life;
                    ma.markers.push_back(outline);
                }
            }
            corridor_pub_.publish(ma);
        } else {
            // decomposition failed or no corridor -> clear previous markers to avoid stale visuals
            visualization_msgs::Marker clear_m;
            clear_m.action = visualization_msgs::Marker::DELETEALL;
            visualization_msgs::MarkerArray clear_ma;
            clear_ma.markers.push_back(clear_m);
            corridor_pub_.publish(clear_ma);
        }
    }
}

bool HybridPlanner::prepareCritics() { //初始化所有轨迹成本函数（critics），确保它们在参与轨迹评分前完成必要的准备工作
    if (critics.size() == 0) return false; //空向量检查
    for (std::vector<base_local_planner::TrajectoryCostFunction *>::iterator loop_critic = critics.begin(); loop_critic != critics.end(); ++loop_critic) { //遍历所有 TrajectoryCostFunction 对象，调用其 prepare() 方法
        base_local_planner::TrajectoryCostFunction *loop_critic_p = *loop_critic;
        if (loop_critic_p->prepare() == false) { //每个评估函数的初始化方法，用于完成必要的准备工作
            ROS_WARN("A scoring function failed to prepare");
            return false;
        }
    }
    return true;
}

std::vector<Node> HybridPlanner::getDijkstraPath(const double &sim_x,
                                                 const double &sim_y,
                                                 const double &sim_th,
                                                 const double &orientation_vel,
                                                 const geometry_msgs::PoseStamped &global_vel) {
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // 清除历史标记（新增）
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub.publish(clear_marker); // 清除所有旧标记

    // First the graph's minimum distance(min_dist) and maximum distance (max_dist) is calculated // 速度与加速度约束计算
    double time_to_min_vel = std::abs((limits.min_vel_x - global_vel.pose.position.x) / limits.acc_lim_x); // 计算减速到最小速度所需时间
    double min_dist_time = std::min(sim_period_, time_to_min_vel); // 确定实际减速时间（不超过模拟周期）
    double min_dist = (global_vel.pose.position.x * min_dist_time) - (0.5 * limits.acc_lim_x * min_dist_time * min_dist_time); // 匀减速公式计算最短移动距离

    double min_vel_in_sim_period = std::max(limits.min_vel_x, global_vel.pose.position.x - (limits.acc_lim_x * sim_period_)); // 模拟周期内速度的最小可能值（减速受限）
    double max_vel_in_sim_period = std::min(limits.max_vel_x, global_vel.pose.position.x + (limits.acc_lim_x * sim_period_)); // 模拟周期内速度的最大可能值（加速受限）
    // 最大距离计算（分段处理）
    double time_to_max_vel = std::abs((limits.max_vel_x - global_vel.pose.position.x) / limits.acc_lim_x);
    double max_dist = 0;

    if (time_to_max_vel < sim_period_) {  // 加速到最大速度后匀速运动
        max_dist = (CAP_TIME - time_to_max_vel) * limits.max_vel_x;
        max_dist += (global_vel.pose.position.x * time_to_max_vel) + (0.5 * limits.acc_lim_x * time_to_max_vel * time_to_max_vel);
    } else { // 整个周期内加速但未达到最大速度
        double max_vel_in_sim_period = global_vel.pose.position.x + (limits.acc_lim_x * sim_period_);
        max_dist = (CAP_TIME - sim_period_) * max_vel_in_sim_period;
        max_dist += (global_vel.pose.position.x * sim_period_) + (0.5 * limits.acc_lim_x * sim_period_ * sim_period_);
    }
    //图结构初始化
    int num_points = (DJK_LAYERS * DJK_LAYER_NODES) + 2;             // Total number of points in the graph // 总节点数（包含起点和终点）
    double delta_dist = (max_dist - min_dist) / (DJK_LAYERS - 1.0);  // Distance between each layer // 层间距
    double cur_dist = min_dist; // 当前层的基础距离

    int edge_count = 0;

    // The graph creation part starts

    std::vector<Node> all_nodes; // 存储所有节点 存储节点属性（位置、角度、成本等）
    std::map<std::pair<int, int>, int> node_map; // 层级与节点索引映射 用于快速查找节点索引
    std::vector<std::vector<Edge>> G(num_points); // 邻接表表示的图 

    // Single point trajectories are used to calcuate the cost of a point
    // Using existing ROS cost calculating methods 起点初始化
    base_local_planner::Trajectory tj_first;
    tj_first.addPoint(sim_x, sim_y, sim_th); // 当前位姿作为起点
    double obs_cost_first = obstacle_costs_.scoreTrajectory(tj_first) * obstacle_costs_.getScale(); //scoreTrajectory用于评估给定轨迹的代价，getScale函数返回一个权重值，用于调整评估函数在总代价计算中的相对重要性

    Node start_node(0, -1, 0, sim_x, sim_y, sim_th, obs_cost_first, false); // 初始节点
    all_nodes.push_back(start_node);

        // 可视化标记初始化（移到图构建循环外）
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker nodes_marker, edges_marker;

    // 节点标记配置（仅初始化一次）
    nodes_marker.header.frame_id = "map";
    nodes_marker.header.stamp = ros::Time::now();
    nodes_marker.ns = "graph_nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::Marker::POINTS;
    nodes_marker.action = visualization_msgs::Marker::ADD;
    nodes_marker.pose.orientation.w = 1.0;
    nodes_marker.scale.x = 0.002;
    nodes_marker.scale.y = 0.002;
    nodes_marker.color.r = 0.4; //颜色
    nodes_marker.color.g = 0.8;
    nodes_marker.color.b = 0.8;
    nodes_marker.color.a = 1.0;

    // 边标记配置（仅初始化一次）
    edges_marker.header = nodes_marker.header;
    edges_marker.ns = "graph_edges";
    edges_marker.id = 1;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.scale.x = 0.001; // 边更细
    edges_marker.color.r = 0.4;
    edges_marker.color.g = 0.23; //色
    edges_marker.color.b = 0.72;
    edges_marker.color.a = 1.0;

    // 已将发布器移到构造函数并使用定时器异步发布，避免在此函数内重复创建发布器或阻塞

    //构建分层图
    for (int layer = 0; layer < DJK_LAYERS; layer++) { //逐层构建图结构。每层代表机器人在某一时间步长内的可能位置和角度
        double left_th, right_th; //计算每层的左右极端角度
        double st = sim_period_ * (layer + 1); //当前层的时间步长

        // For each layer, we calculate the left most theta (left_th) and right most theta (right_th) the mobile robot can move to
        Helper::getLeftRightTh(st, sim_th, orientation_vel, limits.max_vel_theta, limits.acc_lim_theta, left_th, right_th); //计算机器人在当前层可能达到的左右极端角度（left_th 和 right_th），考虑了机器人的当前角度（sim_th）、方向速度（orientation_vel）、最大角速度（limits.max_vel_theta）和角加速度约束（limits.acc_lim_theta）。

        // left_th and right_th is capped to a allowed max value (allowed_max_th) 
        double allowed_max_th = (M_PI_2 / DJK_LAYERS) * layer; //计算当前层允许的最大角度变化范围
        if (Helper::getAngleDist(sim_th, left_th) > allowed_max_th) left_th = Helper::getInRangeAngle(sim_th + allowed_max_th); //超过允许范围，则将其限制在允许范围内
        if (Helper::getAngleDist(sim_th, right_th) > allowed_max_th) right_th = Helper::getInRangeAngle(sim_th - allowed_max_th);

        // delta_angle is the angular distance between each point in the graph, for a given layer 计算角度步长
        double delta_angle = Helper::getAngleDist(right_th, left_th) / (DJK_LAYER_NODES - 1.0); //计算每层中相邻节点之间的角度差

        double cur_angle = left_th; //初始化为 left_th，用于逐点计算节点的角度

        // 存储当前层新增的节点和边，用于可视化
        std::vector<geometry_msgs::Point> layer_nodes;
        std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point>> layer_edges;

        for (int point_n = 0; point_n < DJK_LAYER_NODES; point_n++) { //用于在当前层创建节点
            geometry_msgs::Point p;
            // Calculate the coordinate of the new point //根据当前距离（cur_dist）和角度（cur_angle）计算节点的坐标
            p.x = sim_x + (cur_dist * cos(cur_angle)); 
            p.y = sim_y + (cur_dist * sin(cur_angle));
            p.z = 0;

            base_local_planner::Trajectory tj;
            tj.addPoint(p.x, p.y, cur_angle);
            // Calculate the cost of the new point
            double obs_cost = obstacle_costs_.scoreTrajectory(tj) * obstacle_costs_.getScale(); //障碍物代价
            double pth_cost = path_costs_.scoreTrajectory(tj) * path_costs_.getScale(); //路径代价
            double goal_cost = goal_costs_.scoreTrajectory(tj) * goal_costs_.getScale(); //目标代价 getScale()是权重

            // Check if the node is dead (On an obstacle or such) 检查节点是否为“死节点”
            bool dead = false;
            if (obs_cost < 0 || pth_cost < 0 || goal_cost < 0) dead = true; //负值，则标记该节点为“死节点”

            int index = all_nodes.size(); //当前节点在 all_nodes 向量中的索引

            int dead_count = 0;

            if (layer == 0) {
                // For layer 0, the edges are connected to one node in the previous layer //根据当前层的不同，边的创建方式也不同；第0层的节点仅连接到起点（索引为0的节点）。

                // Calculate edge cost and add the edge to the graph 边的代价由障碍物代价、路径代价、目标代价和距离代价组成
                double edge_cost = obs_cost + pth_cost + goal_cost +
                                   (Helper::getScaledDist(sim_x, p.x, sim_y, p.y) * DIST_COST_SCALE); //距离代价，计算当前点与模拟位置之间的距离，并乘以一个缩放因子
                G[0].push_back(Edge(index, edge_cost)); //将边添加到图的第一层

                edge_count++;
            } else {
                // For other layers, one node is connected to 3 nodes in the previous layer (if possible) //每个节点连接到前一层的3个相邻节点（如果存在）
                int point_1 = point_n; //示当前节点在前一层的三个相邻节点（前一个、当前和后一个）
                int point_0 = point_n - 1;
                int point_2 = point_n + 1;

                std::vector<int> pts; //存储前一层的相邻节点索引
                pts.push_back(point_1);
                if (point_0 >= 0) pts.push_back(point_0);
                if (point_2 < DJK_LAYER_NODES) pts.push_back(point_2);

                for (int itr = 0; itr < pts.size(); itr++) { //遍历前一层的相邻节点
                    int point_prv = pts[itr]; //前一层的节点索引
                    int point_prv_index = node_map[{layer - 1, point_prv}]; //前一层节点在 all_nodes 中的索引
                    Node prv_node = all_nodes[point_prv_index]; //前一层的节点对象

                    // Calculate edge cost and add the edge to the graph
                    double dist = Helper::getScaledDist(p.x, prv_node.pos_x, p.y, prv_node.pos_y); //计算边的代价，并检查前一层的节点是否为“死节点”
                    double edge_cost = obs_cost + pth_cost + goal_cost + (dist * DIST_COST_SCALE);
                    if (prv_node.dead) { //如果是，则将边的代价设为一个很大的值（DEAD_EDGE_COST）
                        edge_cost = DEAD_EDGE_COST;
                        dead_count++;
                    }
                    G[point_prv_index].push_back(Edge(index, edge_cost)); //将边添加到图中

                    edge_count++;
                }
                if (layer == DJK_LAYERS - 1) { 
                    // The last layer nodes are connected to the destination node 最后一层的节点连接到终点（索引为 num_points - 1 的节点）
                    if (!dead) //如果当前节点不是“死节点”，则边的代价设为0；否则设为 DEAD_EDGE_COST
                        G[index].push_back(Edge(num_points - 1, 0));
                    else
                        G[index].push_back(Edge(num_points - 1, DEAD_EDGE_COST));
                }
            }

            if (dead_count == 3) dead = true;
            Node node(index, layer, point_n, p.x, p.y, cur_angle, obs_cost, dead);
            all_nodes.push_back(node);
            node_map.insert({{layer, point_n}, index}); //将当前层和节点编号映射到 index，方便后续查找

            cur_angle -= delta_angle; //在每次迭代中减去 delta_angle，并确保角度在合理范围内--->更新角度
            cur_angle = Helper::getInRangeAngle(cur_angle);

            // 添加当前节点到层节点列表（可视化用）
            geometry_msgs::Point p2;
            p2.x = node.pos_x;
            p2.y = node.pos_y;
            p2.z = 0;
            layer_nodes.push_back(p2);
        }
        cur_dist += delta_dist; //在每层结束后增加 delta_dist，以构建下一层

        // 收集当前层的边（根据边的类型分类处理）
        if (layer == 0) {
            // 层0的边：起点 -> 层0节点
            const Node& start_node = all_nodes[0];
            for (size_t i = 0; i < layer_nodes.size(); i++) {
                int index = 1 + i; // 层0节点在all_nodes中的起始索引为1（起点是0）
                const Node& to_node = all_nodes[index];

                geometry_msgs::Point p1;
                p1.x = start_node.pos_x;
                p1.y = start_node.pos_y;
                p1.z = 0.0;

                geometry_msgs::Point p2;
                p2.x = to_node.pos_x;
                p2.y = to_node.pos_y;
                p2.z = 0.0;

                layer_edges.emplace_back(p1, p2);
            }
        } else {
            // 其他层的边：前一层相邻节点 -> 当前层节点
            for (size_t i = 0; i < G.size(); i++) {
                for (const auto& edge : G[i]) {
                    int to_index = edge.get_node_index();
                    if (all_nodes[i].layer_no == layer - 1 && all_nodes[to_index].layer_no == layer) {
                        geometry_msgs::Point p1;
                        p1.x = all_nodes[i].pos_x;
                        p1.y = all_nodes[i].pos_y;
                        p1.z = 0.0;

                        geometry_msgs::Point p2;
                        p2.x = all_nodes[to_index].pos_x;
                        p2.y = all_nodes[to_index].pos_y;
                        p2.z = 0.0;

                        layer_edges.emplace_back(p1, p2);
                    }
                }
            }
        }

        // 将当前层节点和边添加到可视化标记中
        nodes_marker.points.insert(nodes_marker.points.end(), layer_nodes.begin(), layer_nodes.end());
        for (const auto& [p1, p2] : layer_edges) {
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        // 将每层结果累积到 nodes_marker/edges_marker 中，最终整体发布由定时器异步完成（非阻塞）
        // （保留 nodes_marker/edges_marker 的累积行为以在末尾构造最终标记数组）
    }

    // After creating the graph, Dijkstra's algorithm is used to find the best path -----------Dijkstra算法求解最短路径
    //初始化
    std::priority_queue<Edge> Q; //创建优先队列Q用于存储待处理的边，优先队列会根据边的代价自动排序，确保每次取出代价最小的边

    std::vector<double> dist(num_points, 100000000.0); //初始化距离向量dist，所有节点的初始距离设为一个大值
    std::vector<bool> visited(num_points, false); //初始化访问标记向量visited和父节点向量parent
    std::vector<int> parent(num_points, -1); //标记节点是否已被访问过，避免重复处理

    int start = 0, end = num_points - 1; //设置起点start为0，终点end为最后一个节点

    //Dijkstra算法执行
    dist[start] = 0; //将起点推入优先队列，初始距离为0。

    Q.push(Edge(start, 0)); //将起点添加到优先队列中，优先队列按距离排序，确保每次提取距离最小的节点

    while (!Q.empty()) { //循环处理队列中的每个节点，直到队列为空或找到终点
        Edge u = Q.top(); //取出队列顶部的节点
        Q.pop(); //从队列中移除该节点
        if (u.get_node_index() == end) break; //检查是否为终点：如果是终点，跳出循环
        if (visited[u.get_node_index()]) continue; //如果节点已被访问过，跳过

        for (int i = 0; i < G[u.get_node_index()].size(); i++) { //遍历当前节点的所有邻接边
            Edge v = G[u.get_node_index()][i]; //获取邻居节点
            if (visited[v.get_node_index()]) continue; //跳过已访问的邻接节点：如果邻接节点已被访问过，跳过

            if (u.get_cost() + v.get_cost() < dist[v.get_node_index()]) { //检查通过当前节点到达邻居节点的距离是否更小
                dist[v.get_node_index()] = u.get_cost() + v.get_cost(); //更新邻居节点的距离
                Q.push(Edge(v.get_node_index(), dist[v.get_node_index()])); //将邻居节点添加到优先队列中
                parent[v.get_node_index()] = u.get_node_index(); //记录父节点：更新邻接节点的父节点为当前节点
            }
        }
        visited[u.get_node_index()] = true; //标记当前节点为已访问
        
    }

    // Add all the nodes in the shortest path to the nodes_in_path vector ------------生成最优路径
    int current_node = parent[end]; //路径回溯 从终点开始，通过父节点信息回溯到起点，生成路径节点列表

    std::vector<Node> nodes_in_path; 

    while (current_node != -1) {
        Node u = all_nodes[current_node];
        nodes_in_path.push_back(u);//将路径节点存储到nodes_in_path向量中
        current_node = parent[current_node];
    }

    // Reverse the order of the nodes 将路径反转，以得到从起点到终点的正确顺序
    std::reverse(nodes_in_path.begin(), nodes_in_path.end());

        // ----------------------- 新增路径可视化逻辑 -----------------------
    // Dijkstra算法生成路径后，添加路径标记（带生命周期）
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "shortest_path";
    path_marker.id = 2;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.03;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0; //青色
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    path_marker.lifetime = ros::Duration(0.1); // 与图标记生命周期一致

    // 填充路径点
    for (const auto& node : nodes_in_path) {
        geometry_msgs::Point p;
        p.x = node.pos_x;
        p.y = node.pos_y;
        p.z = 0;
        path_marker.points.push_back(p);
    }

    // 合并并发布所有标记（图 + 路径）
    visualization_msgs::MarkerArray final_marker_array;
    final_marker_array.markers.push_back(nodes_marker);
    final_marker_array.markers.push_back(edges_marker);
    final_marker_array.markers.push_back(path_marker);
    // 将最终可视化数据复制到缓冲区，由定时器异步发布（避免阻塞主控制循环）
    {
        // 补充时间戳并调整 lifetime 以适配 viz_rate_
        ros::Time now = ros::Time::now();
        double viz_period = 1.0 / std::max(0.001, viz_rate_);
        ros::Duration path_life = ros::Duration(std::max(0.05, viz_period * 1.5));
        for (auto &m : final_marker_array.markers) {
            m.header.stamp = now;
            if (m.lifetime.toSec() <= 0.0) m.lifetime = ros::Duration(viz_period * 2.0);
        }
        path_marker.header.stamp = now;
        path_marker.lifetime = path_life;

        boost::mutex::scoped_lock l(viz_mutex_);
        viz_marker_array_buffer_ = final_marker_array;
        viz_path_marker_buffer_ = path_marker;
        have_viz_data_ = true;
    }
    // ----------------------- 路径可视化结束 -----------------------
    
    // 最后补充终点节点的可视化（如果需要）
    if (!all_nodes.empty()) {
        const Node& end_node = all_nodes.back();
        geometry_msgs::Point end_point;
        end_point.x = end_node.pos_x;
        end_point.y = end_node.pos_y;
        end_point.z = 0.0;
        nodes_marker.points.push_back(end_point);
        // 将终点信息合并到缓冲区，由定时器异步发布
        {
            ros::Time now = ros::Time::now();
            nodes_marker.header.stamp = now;
            edges_marker.header.stamp = now;
            boost::mutex::scoped_lock l(viz_mutex_);
            viz_marker_array_buffer_.markers.clear();
            viz_marker_array_buffer_.markers.push_back(nodes_marker);
            viz_marker_array_buffer_.markers.push_back(edges_marker);
            have_viz_data_ = true;
        }
    }
    return nodes_in_path; //返回路径节点列表
}

/*
* This function takes the path (vector of Node) from the Dijkstra's algorithm and calculates
* the allowed velocity for the mobile robot, based on the distance to obstacles 根据路径（由Dijkstra算法生成的节点序列）计算机器人的允许速度和目标节点索引
*/
void HybridPlanner::getAllowedVelocityAndGoalNodeIndex(std::vector<Node> &nodes_in_path,
                                                       const geometry_msgs::PoseStamped &global_vel,
                                                       const double sim_period_,
                                                       double &allowed_velocity,
                                                       int &goal_node_index) {
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits(); //获取当前本地规划器的限制条件，包括最大速度、加速度等
    int itr = 0; //迭代器，用于遍历路径节点
    double max_obs_cost = 0; //路径中最大的障碍物代价，初始化为0
    int last_node_index = 0; //有效路径的最后一个节点索引，初始化为0

    // First find the valid path length and also the max obstacle cost of the path
    for (itr = 0; itr < nodes_in_path.size(); itr++) { //遍历路径中的每个节点，直到遇到“死节点”
        Node u = nodes_in_path[itr];
        if (u.dead) {
            break;
        }
        last_node_index = itr;
        double obs_cost = u.obstacle_cost;
        if (obs_cost > max_obs_cost) max_obs_cost = obs_cost; //记录路径中最大的障碍物代价
    }

    double velocity_reduction_multiplyer = 0;
    double obs_cost_max_lim = 254.0 * obstacle_costs_.getScale();
    double scaled_max_obs_cost = max_obs_cost / obs_cost_max_lim;

    // Closer the robot is to an obstacle, higher the velocity_reduction_multiplyer will be
    // This is used to reduce the speed of the robot in close proximity to obstacles
    if (scaled_max_obs_cost >= 0.25 && scaled_max_obs_cost <= 0.5) {
        velocity_reduction_multiplyer = ((15 / 0.25) * scaled_max_obs_cost) - 15;
    } else if (scaled_max_obs_cost >= 0.5 && scaled_max_obs_cost <= 0.75) {
        velocity_reduction_multiplyer = ((25 / 0.25) * scaled_max_obs_cost) - 35;
    } else if (scaled_max_obs_cost >= 0.75) {
        velocity_reduction_multiplyer = ((35 / 0.25) * scaled_max_obs_cost) - 65;
    }
    velocity_reduction_multiplyer *= 0.01;

    allowed_velocity = limits.max_vel_x - (limits.max_vel_x * velocity_reduction_multiplyer);

    if (allowed_velocity < global_vel.pose.position.x) {
        double tm = (global_vel.pose.position.x - allowed_velocity) / limits.acc_lim_x;
        if (tm > sim_period_) {
            allowed_velocity = global_vel.pose.position.x - (limits.acc_lim_x * sim_period_);
        }
    } else {
        double tm = (allowed_velocity - global_vel.pose.position.x) / limits.acc_lim_x;
        if (tm > sim_period_) {
            allowed_velocity = global_vel.pose.position.x + (limits.acc_lim_x * sim_period_);
        }
    }

    // Using the allowed velocity, calculate the allowed distance and the index to the goal node
    double allowed_dist = allowed_velocity * CAP_TIME;


    for (itr = 1; itr <= last_node_index; itr++) {
        double m_dist = Helper::getScaledDist(nodes_in_path[itr], nodes_in_path[itr - 1]);
        allowed_dist -= m_dist;
        if (allowed_dist < 0) {
            break;
        }
    }
    goal_node_index = itr;
}

/*
 * Given the current state of the robot, find a good trajectory 
 * Also provide the velocities for robot to follow
 */
base_local_planner::Trajectory HybridPlanner::findBestPath(
    const geometry_msgs::PoseStamped &global_pose,
    const geometry_msgs::PoseStamped &global_vel,
    geometry_msgs::PoseStamped &drive_velocities) {

    // make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    // 1. 更新位姿时间戳和数据
    current_pose_.header.stamp = ros::Time::now();  // 强制更新时间戳（确保实时性）
    current_pose_.pose = global_pose.pose;  // 直接使用输入的位姿（位置+朝向）
    // 2. 发布位姿（建议在规划周期结束前发布，避免被后续逻辑覆盖）
    current_pose_pub_.publish(current_pose_);

    //变量初始化
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation)); //三维列向量
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // // prepare cost functions and generators for this run
    generator_.initialise(pos, vel, goal, &limits, vsamples_); //初始化路径生成器（generator_），为当前的路径规划运行准备必要的数据结构和参数；base_local_planner::SimpleTrajectoryGenerator 是一个路径生成器类，用于生成候选路径。

    bool invalid = false;

    if (!prepareCritics()) { //初始化评估函数（critics），这些评估函数将用于后续的路径规划中，以评估生成的路径是否合适
        invalid = true;
    } else {
        double v_x, v_th;

        astar_path.points.clear();

        // First, predict where the robot will be after the PROCESSING_TIME 预测机器人位置
        double orientation = Helper::getInRangeAngle(tf2::getYaw(global_pose.pose.orientation)); //机器人当前的朝向角度（弧度）
        double orientation_vel = tf2::getYaw(global_vel.pose.orientation); //机器人当前的旋转速度（弧度/秒）

        double sim_dist = global_vel.pose.position.x * PROCESSING_TIME; //机器人在 PROCESSING_TIME 内沿x轴移动的距离 速度 × 时间
        double sim_th_dist = orientation_vel * PROCESSING_TIME; //机器人在 PROCESSING_TIME 内旋转的角度，计算公式为 旋转速度 × 处理时间
        double sim_th = Helper::getInRangeAngle(orientation + sim_th_dist); //机器人在 PROCESSING_TIME 后的朝向角度，计算公式为 当前朝向 + 旋转角度
        double sim_x = global_pose.pose.position.x + (sim_dist * cos(sim_th)); //机器人在 PROCESSING_TIME 后的位置
        double sim_y = global_pose.pose.position.y + (sim_dist * sin(sim_th));

        // Find the best path for the robot to follow 寻找最佳路径
        std::vector<Node> nodes_in_path = getDijkstraPath(sim_x, sim_y, sim_th, orientation_vel, global_vel);

        double allowed_velocity = 0; //初始化为0，表示允许的最大速度
        int goal_node_index = 0; //初始化为0，表示目标节点的索引

        double goal_x = goal_pose.pose.position.x; //目标位置的坐标和朝向
        double goal_y = goal_pose.pose.position.y;
        double goal_th = Helper::getInRangeAngle(tf2::getYaw(goal_pose.pose.orientation));

        double goal_dist = std::hypot(sim_x - goal_x, sim_y - goal_y); //机器人当前位置与目标位置之间的欧几里得距离
        double dec_dist = limits.max_vel_x * GOAL_DEC_TIME; //减速距离，计算公式为 最大线速度 × 减速时间

        Node goal_node = nodes_in_path[0]; //路径中的第一个节点即目标点
        double goal_head_angle = 0; //目标朝向角度，初始化为0
        double allowed_dist; //允许的距离
        double delta_th = M_PI / (NUM_ANGLES * 0.5); //角度增量，用于在角度空间中采样

        // Find out the goal position and heading for the mobile robot, and the allowed velocity 

        // If the actual goal is within reach, use that goal as destination. The velocity is selected based on the distance to the goal 目标在减速距离内
        if (dec_dist > goal_dist) { //机器人可以直接以最大速度行驶到目标位置
            double dec_multiplyer = (goal_dist / dec_dist); //减速因子
            allowed_velocity = limits.max_vel_x * dec_multiplyer; //允许的最大速度
            allowed_dist = goal_dist; //允许的距离，设置为目标距离

            goal_node.pos_x = goal_x;
            goal_node.pos_y = goal_y;
            goal_node.angle = goal_th; //更新为目标位置和朝向
            goal_head_angle = goal_th; //设置为目标朝向
        } else {
            // If the actual goal is not within reach, select a velcity and a suitable point in the Dijkstra's path as goal 目标不在减速距离内，使用Dijkstra算法找到临时目标点
            getAllowedVelocityAndGoalNodeIndex(nodes_in_path, global_vel, sim_period_, allowed_velocity, goal_node_index); //调用函数计算允许的速度和目标节点的索引
            goal_node = nodes_in_path[goal_node_index]; //更新为路径中的目标节点
            allowed_dist = allowed_velocity * CAP_TIME; //允许的距离

            // After we have the goal point, calculate the best heading for that point

            double min_cost_head_angle = 20000000.0; //初始化为一个很大的值，用于存储最小代价的朝向角度

            double goal_head_th = goal_node.angle; //目标节点的朝向角度
            goal_head_th = Helper::getInRangeAngle(goal_head_th);

            double PI_3 = (M_PI / 3);

            double temp_left_angle = Helper::getInRangeAngle(goal_head_th + PI_3); //计算目标朝向的左右边界
            double temp_right_angle = Helper::getInRangeAngle(goal_head_th - PI_3);

            double goal_left_angle = std::floor(temp_left_angle / delta_th) * delta_th; //将左右边界调整为允许的角度增量
            double goal_right_angle = std::ceil(temp_right_angle / delta_th) * delta_th;

            int num_iters = Helper::getAngleDist(goal_left_angle, goal_right_angle) / delta_th; //计算需要采样的角度数量
            double head_angle = goal_left_angle; //初始化为左边界角度

            for (int itr = 0; itr <= num_iters; itr++, head_angle -= delta_th) { //在左右边界之间采样，逐步减少角度
                head_angle = Helper::getInRangeAngle(head_angle);
                base_local_planner::Trajectory tj; //创建一个轨迹对象，表示机器人在目标位置和当前角度下的状态
                tj.addPoint(goal_node.pos_x, goal_node.pos_y, head_angle);
                double alignment_cost = alignment_costs_.scoreTrajectory(tj) * alignment_costs_.getScale(); //计算轨迹的对齐代价，评估机器人朝向与目标方向的对齐程度
                double total_cost = alignment_cost;
                if (total_cost < min_cost_head_angle) { //如果当前角度的代价更小，更新最小代价和最佳朝向角度
                    min_cost_head_angle = total_cost;
                    goal_head_angle = head_angle;
                }
            }
            //速度限制
            double min_vel_in_sim_period = std::max(limits.min_vel_x, global_vel.pose.position.x - (limits.acc_lim_x * sim_period_)); //计算在模拟周期内允许的最小速度，确保速度不会低于最小速度限制
            double max_vel_in_sim_period = std::min(limits.max_vel_x, global_vel.pose.position.x + (limits.acc_lim_x * sim_period_)); //计算在模拟周期内允许的最大速度，确保速度不会超过最大速度限制

            if (allowed_velocity > max_vel_in_sim_period) allowed_velocity = max_vel_in_sim_period; //如果允许速度超过最大速度限制，将其限制在最大速度
            if (allowed_velocity < min_vel_in_sim_period) allowed_velocity = min_vel_in_sim_period; //如果允许速度低于最小速度限制，将其限制在最小速度
        }

        // Guard against near-stop conditions: Hybrid A* uses allowed_dist/allowed_velocity as a time period.
        // When approaching the goal, both can become ~0, causing 0/0 -> NaN and corrupting downstream math.
        if (allowed_velocity <= 1e-6 || allowed_dist <= 1e-6) {
            result_traj_.resetPoints();
            // Keep a valid trajectory object; command a stop.
            result_traj_.xv_ = 0.0;
            result_traj_.yv_ = 0.0;
            result_traj_.thetav_ = 0.0;
            result_traj_.cost_ = 100.0;
            // Skip Hybrid A* theta search
            invalid = false;
        } else {

        // At this point, we have the goal node, the desired heading for the goal node and the X velocity of the robot
        // Now, we will use the Hybrid A star algorithm to find the Theta velocity for the robot.    

        double min_x = std::min(sim_x, goal_node.pos_x) - 0.1; //确定搜索空间范围，min_x 和 min_y确定搜索空间的最小x和y坐标，扩展0.1米以确保包含目标位置，
        double min_y = std::min(sim_y, goal_node.pos_y) - 0.1;
        double max_x = std::max(sim_x, goal_node.pos_x) + 0.1; //max_x 和 max_y：确定搜索空间的最大x和y坐标，扩展0.1米以确保包含目标位置
        double max_y = std::max(sim_y, goal_node.pos_y) + 0.1;

        // The X, Y and Theta values are converted to discrete cells

        double delta_x = (max_x - min_x) / NUM_X_DIVS; // 离散化搜索空间，delta_x 和 delta_y：计算每个离散单元格的宽度和高度
        double delta_y = (max_y - min_y) / NUM_Y_DIVS;

        int begin_cell_x = (sim_x - min_x) / delta_x; //确定起始和目标单元格
        int begin_cell_y = (sim_y - min_y) / delta_y; //将起始位置和朝向转换为离散单元格坐标
        int begin_cell_th = sim_th / delta_th;
        // Each cell is represented using a hash value. This helps to keep track which cells are visited
        std::string begin_hash = Helper::getHash(begin_cell_x, begin_cell_y, begin_cell_th); //生成起始单元格的哈希值，用于跟踪访问状态

        double begin_x = min_x + (begin_cell_x * delta_x); //将离散单元格坐标转换回实际坐标
        double begin_y = min_y + (begin_cell_y * delta_y);
        double begin_th = begin_cell_th * delta_th;

        int end_cell_x = (goal_node.pos_x - min_x) / delta_x; //将目标位置和朝向转换为离散单元格坐标
        int end_cell_y = (goal_node.pos_y - min_y) / delta_y;
        int end_cell_th = goal_head_angle / delta_th;
        std::string end_hash = Helper::getHash(end_cell_x, end_cell_y, end_cell_th); //生成目标单元格的哈希值

        double end_x = min_x + (end_cell_x * delta_x); //将离散单元格坐标转换回实际坐标
        double end_y = min_y + (end_cell_y * delta_y);
        double end_th = end_cell_th * delta_th;

        AStar_Node last, lowest; //初始化HybridA*算法参数，用于存储算法中的节点信息
        double lowest_hr_cost = 20000000; //初始化为一个很大的值，用于存储最低的启发式代价

        double vel_th_min = -limits.max_vel_theta; //机器人旋转速度的最小和最大限制
        double vel_th_max = limits.max_vel_theta;
        double astar_sim_period = allowed_dist / allowed_velocity; //模拟周期，计算公式为允许的距离除以允许的速度
        
        // At the start, we consider time period of larger values. This results in the A Star algorithm having 
        // larger edges in the beginning
        double cur_sim_period = astar_sim_period / 2; //初始化为 astar_sim_period 的一半，表示当前模拟周期
        double min_sim_period = astar_sim_period / std::pow(2, 5); //最小模拟周期

        int th_dist_divs = 30; //设置为30，表示在角度空间中采样的点数

        std::clock_t hastar_start, hastar_ctime; // hastar_start记录算法开始运行的时间，hastar_ctime用于记录当前时间
        hastar_start = std::clock();

        // The Hybrid A star algorithm is allowed to run for a specific period of time (allowed_time).
        // We consider the closes node the Hybrid A star algorithm reaches in this time period
        double allowed_time = 0.03; //设置为0.03秒，表示Hybrid A*算法允许的最大运行时间
        int node_count = 0; //初始化为0，用于计数生成的节点数

        AStar_Node begin_node(0, begin_hash, begin_x, begin_y, begin_th, cur_sim_period, 0, -1, 0, orientation_vel); //定义一个A*算法的节点类，创建起始节点

        // The Hybrid A star algorithm begins to find the best path between the starting node and the goal node

        std::priority_queue<AStar_Node> Q2; //优先队列，用于存储待处理的节点，按优先级（通常是启发式成本）排序

        std::map<std::string, int> visited_map; //映射，用于跟踪已访问的节点，键为节点的哈希值，值为节点ID
        std::map<std::string, double> cost_map; //用于存储节点的成本，键为节点的哈希值，值为成本
        std::vector<AStar_Node> nodes; //向量，用于存储所有生成的节点

        nodes.push_back(begin_node); //将起始节点添加到节点向量中
        Q2.push(begin_node); //将起始节点添加到优先队列中

        bool first_node = true; //布尔值，表示是否是第一个节点，初始化为 true
        bool found = false; //布尔值，表示是否找到目标节点，初始化为 false

        while (!Q2.empty()) { //循环直到优先队列为空
            AStar_Node u = Q2.top(); //提取优先队列中优先级最高的节点
            Q2.pop(); //从队列中移除该节点

            node_count++; //增加节点计数
            if (node_count % 10 == 0) { //每处理10个节点，检查是否超过允许时间
                hastar_ctime = std::clock(); //记录当前时间
                double hastar_time = double(hastar_ctime - hastar_start) / double(CLOCKS_PER_SEC); //计算已用时间
                if (hastar_time > allowed_time) break; //如果超过允许时间，跳出循环
            }

            if (u.hash == end_hash) { //如果当前节点是目标节点，记录并跳出循环
                last = u;
                found = true;
                break;
            }

            if (visited_map.find(u.hash) != visited_map.end()) continue; //如果节点已被访问过，跳过
            visited_map[u.hash] = 1; //标记节点为已访问

            if (!first_node && u.hr_cost < lowest_hr_cost) { //更新最低启发式成本
                lowest_hr_cost = u.hr_cost; //更新当前模拟周期
                lowest = u;
            }

            cur_sim_period = u.sim_time;

            double left_th, right_th;

            double vel_th = u.number == 0 ? u.vel_th : 0;

            // For each node, find the left-most and the right-most nodes it can reach //计算当前节点可以达到的左右边界角度
            Helper::getLeftRightTh(cur_sim_period, u.heading_angle, vel_th, limits.max_vel_theta, limits.acc_lim_theta, left_th, right_th);

            double delta_dist = Helper::getAngleDist(left_th, right_th) / th_dist_divs; //计算角度增量

            int ni = -1; //初始化为-1，用于控制循环中的步长方向
            double cur_dist = cur_sim_period * allowed_velocity; //计算当前模拟周期内的行驶距离

            // Sample nodes which are in between the left-most and the right-most nodes.
            for (int i = th_dist_divs / 2; i <= th_dist_divs; i += ni) { //从 th_dist_divs / 2 开始，逐步增加或减少 i，生成新节点
                if (i < 0) {
                    ni = 1;
                    i = (th_dist_divs / 2);
                    continue;
                }

                double new_th = Helper::getInRangeAngle(left_th - (i * delta_dist)); //计算新节点的朝向角度，确保在合理范围内
                int new_cell_th = new_th / delta_th; //将角度转换为离散单元格坐标
                new_th = new_cell_th * delta_th;
                double new_x = u.pos_x + (cur_dist * cos(new_th)); //计算新节点的位置
                double new_y = u.pos_y + (cur_dist * sin(new_th));
                int new_cell_x = (new_x - min_x) / delta_x; //将位置转换为离散单元格坐标
                int new_cell_y = (new_y - min_y) / delta_y;
                new_x = min_x + (new_cell_x * delta_x);
                new_y = min_y + (new_cell_y * delta_y);

                // if(new_x < min_x || new_y < min_y || new_x > max_x || new_y > max_y) continue;

                // Calculate the hash of the new node
                std::string new_hash = Helper::getHash(new_cell_x, new_cell_y, new_cell_th); //生成新节点的哈希值，用于唯一标识节点

                base_local_planner::Trajectory tj2; //创建一个轨迹对象，表示新节点的状态
                tj2.addPoint(new_x, new_y, new_th);
                double obs_cost = obstacle_costs_.scoreTrajectory(tj2) * 0.001; //计算新节点的障碍物代价

                if (obs_cost < 0) continue; //如果代价无效，跳过当前节点

                // Calculate the cost of the new node
                double parent_th_dist = Helper::getAngleDist(new_th, u.heading_angle); //计算新节点与父节点的角度差
                double parent_dist_cost = std::hypot(new_x - u.pos_x, new_y - u.pos_y) + (parent_th_dist * 0.5); //计算新节点与父节点的距离和角度代价
                double hr_cost = std::hypot(new_x - end_x, new_y - end_y) + (Helper::getAngleDist(new_th, goal_head_angle) * 0.1); //计算启发式代价

                double vel_th = 0;

                double total_cost = u.cost - u.hr_cost + obs_cost + parent_dist_cost + hr_cost; //综合所有代价，计算新节点的总代价

                bool add_node = false;

                if (cost_map.find(new_hash) == cost_map.end()) { 
                    // Add the new node to the priority queue, if the node is not yet visited 如果新节点未被访问过，添加到成本映射中
                    cost_map[new_hash] = total_cost - hr_cost;
                    add_node = true;
                } else {
                    // Add the node to the priority queue if the new cost of the node is lower than the preivous cost 如果新节点已被访问过，检查新代价是否更低，如果是则更新
                    double prv_cost = cost_map[new_hash];
                    if (prv_cost < total_cost) {
                        continue;
                    }
                    add_node = true;
                    cost_map[new_hash] = total_cost - hr_cost;
                }

                if (add_node) { //如果决定添加新节点，创建并添加到节点列表和优先队列中
                    double tm = std::max(cur_sim_period / 2, min_sim_period); //计算新节点的模拟周期，确保它不小于最小模拟周期
                    AStar_Node new_node(nodes.size(), new_hash, new_x, new_y, new_th, tm, total_cost, u.number, hr_cost, vel_th); ///创建新节点
                    nodes.push_back(new_node); //将新节点添加到节点列表中
                    Q2.push(new_node); //将新节点添加到优先队列中，以便后续处理
                }
            }

            if (first_node) { //如果当前处理的是第一个节点，执行以下操作
                first_node = false; //标记第一个节点已处理
                th_dist_divs = 4; //调整角度采样点数为4，以优化搜索效率
            }
        }

        // Detect the best path found by the Hybrid A star algorithm and calculate the Theta velocity using the path

        AStar_Node final_node, node_itr;
        if (found) //布尔值，表示是否找到目标节点
            final_node = last; //last记录的目标节点
        else
            final_node = lowest; //lowest：记录的启发式成本最低的节点

        node_itr = final_node; //用于遍历路径的迭代器，初始化为 final_node

        result_traj_.resetPoints(); //result_traj_：存储最终路径的轨迹对象，resetPoints()：清除轨迹中的所有点，为新路径做准备

        while (true) { //循环直到回溯到起点
            geometry_msgs::Point px; //创建一个点对象，用于存储当前节点的位置
            px.x = node_itr.pos_x; 
            px.y = node_itr.pos_y;
            px.z = 0;

            result_traj_.addPoint(node_itr.pos_x, node_itr.pos_y, node_itr.heading_angle); //将当前节点的位置和朝向添加到结果轨迹中

            astar_path.points.push_back(px);  // For path visualization 将当前节点的位置添加到路径可视化列表中
            if (node_itr.number != final_node.number && node_itr.parent != -1) { //如果当前节点没有父节点，跳出循环
                astar_path.points.push_back(px); // For path visualization
            }
            if (node_itr.parent == -1) break; // 如果当前节点没有父节点，跳出循环
            if (nodes[node_itr.parent].parent == -1) { // 如果父节点没有祖父节点，计算角速度
                const double node_orientation = Helper::getInRangeAngle(node_itr.heading_angle); //当前节点的朝向角度
                const double dtheta = angles::shortest_angular_distance(sim_th, node_orientation); //带符号最短角差

                const double denom = (astar_sim_period / 2.0);
                if (denom > 1e-6) {
                    v_th = dtheta / denom;
                } else {
                    v_th = 0.0;
                }

                // 限幅，避免异常大角速度（尤其在 near-stop 或 denom 略小的情况下）
                v_th = std::max(-limits.max_vel_theta, std::min(limits.max_vel_theta, v_th));
                v_th = v_th * VEL_TH_MULTIPLYER;
            }
            node_itr = nodes[node_itr.parent]; //移动到父节点，继续回溯
        }


        v_x = allowed_velocity; //设置线速度为允许的最大速度

        astar_path.header.stamp = ros::Time::now();
        // 把 astar 路径写入缓冲，由定时器异步发布
        {
            boost::mutex::scoped_lock l(viz_mutex_);
            viz_path_marker_buffer_ = astar_path;
            have_viz_data_ = true;
        }

        // Before accepting the final trajectory, check safety corridor (if exists)
        bool corridor_ok = true;
        if (safety_corridor_ && !corridor_polygons_.empty()) {
            // check whether final node is inside any polygon
            rpp::common::geometry::Vec2d final_pt(final_node.pos_x, final_node.pos_y);
            corridor_ok = false;
            for (const auto &poly : corridor_polygons_) {
                if (poly.isPointIn(final_pt) || poly.isPointOnBoundary(final_pt)) {
                    corridor_ok = true;
                    break;
                }
            }
        }

        // Publish the calculated velocities to ROS (mark as invalid if corridor rejects)
        if (!corridor_ok) {
            result_traj_.cost_ = -1; // reject trajectory
        } else {
            result_traj_.xv_ = v_x; //设置线速度
            result_traj_.yv_ = 0; //设置y方向速度为0
            result_traj_.thetav_ = v_th; //设置角速度
            result_traj_.cost_ = 100; //设置路径代价为100
        }

        } // end guard else (allowed_velocity/dist valid)
    }

    // verbose publishing of point clouds 布尔值，表示是否需要发布点云可视化
    if (publish_cost_grid_pc_) {
        // we'll publish the visualization of the costs to rviz before returning our best trajectory
        map_viz_.publishCostCloud(planner_util_->getCostmap()); //map_viz_：一个对象，用于可视化成本地图
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans); //oscillation_costs_：一个对象，用于处理振荡成本

    // if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0 || invalid) { //如果路径代价为负或路径无效，发布零速度
        drive_velocities.pose.position.x = 0;
        drive_velocities.pose.position.y = 0;
        drive_velocities.pose.position.z = 0;
        drive_velocities.pose.orientation.w = 1;
        drive_velocities.pose.orientation.x = 0;
        drive_velocities.pose.orientation.y = 0;
        drive_velocities.pose.orientation.z = 0;
    } else { //如果路径有效，发布规划的速度
        drive_velocities.pose.position.x = result_traj_.xv_;
        drive_velocities.pose.position.y = result_traj_.yv_;
        drive_velocities.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, result_traj_.thetav_); //设置四元数的朝向角度
        tf2::convert(q, drive_velocities.pose.orientation); //将四元数转换为ROS消息格式
    }

    return result_traj_;
}

// 定时器回调：异步发布缓冲的可视化数据，避免阻塞主控制路径
// 定时器回调：异步发布缓冲的可视化数据，避免阻塞主控制路径
void HybridPlanner::vizTimerCB(const ros::TimerEvent &ev) {
    // 复制缓冲内容并尽快释放锁，避免在发布过程中阻塞 planner 线程
    visualization_msgs::MarkerArray local_markers;
    visualization_msgs::Marker local_path;
    bool do_publish = false;
    {
        boost::mutex::scoped_lock l(viz_mutex_);
        if (have_viz_data_) {
            local_markers = viz_marker_array_buffer_;
            local_path = viz_path_marker_buffer_;
            do_publish = true;
        }
    }
    if (!do_publish) return;

    // 发布到 namespaced 发布器和根命名空间发布器 (如果存在)
    if (graph_pub_global_) graph_pub_global_.publish(local_markers);
    if (graph_pub_root_) graph_pub_root_.publish(local_markers);

    if (astar_pub_global_) astar_pub_global_.publish(local_path);
    if (astar_pub_root_) astar_pub_root_.publish(local_path);
}
};  // namespace hlpmpccorridor_local_planner