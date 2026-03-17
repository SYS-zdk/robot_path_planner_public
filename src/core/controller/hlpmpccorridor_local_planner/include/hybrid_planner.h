/**
 * @file hybrid_planner.h
 * @brief Hybrid local planner core (graph search + trajectory scoring) with corridor support.
 * @author Zhang Dingkun (integration/adaptation)
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef HYBRID_LOCAL_PLANNER_HYBRID_PLANNER_H_
#define HYBRID_LOCAL_PLANNER_HYBRID_PLANNER_H_

#include <dwa_local_planner/DWAPlannerConfig.h>

#include <Eigen/Core>
#include <vector>

// for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

// for obstacle data access
#include <Node.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/twirling_cost_function.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// safety corridor
#include <common/safety_corridor/convex_safety_corridor.h>

#include <ros/ros.h>

#include <pedsim_msgs/TrackedPersons.h>

#include <dynamic_obstacle_cost.h>
#include <social_cost_function.h>

namespace hlpmpccorridor_local_planner {
const double CAP_TIME = 2.0; // 轨迹生成的最大时间限制
const double PROCESSING_TIME = 0.035; // 单次规划周期的时间预算

const int DJK_LAYERS = 20; // Dijkstra 算法的分层数量
const int DJK_LAYER_NODES = 60; // 每层的节点数
const double DIST_COST_SCALE = 120; // 距离成本缩放因子
const double DEAD_EDGE_COST = 3000; // 无效边的惩罚成本
const double GOAL_DEC_TIME = 2.0; // 目标点决策时间窗口
const int NUM_ANGLES = 60; // 角度采样分辨率
const int NUM_X_DIVS = 60; // X 方向空间离散化数量
const int NUM_Y_DIVS = 60; // Y 方向空间离散化数量
const double VEL_TH_MULTIPLYER = 1.2; // 速度阈值乘数

/**
 * @class HybridPlanner
 * @brief A class implementing a local planner using the Dijkstra's algorithm and Hybrid A Star algorithm
 */
class HybridPlanner {
   public:
    /**
     * @brief  Constructor for the planner 构造函数
     * @param name The name of the planner
     * @param costmap_ros A pointer to the costmap instance the planner should use
     * @param global_frame the frame id of the tf frame to use
     */
    HybridPlanner(std::string name, base_local_planner::LocalPlannerUtil* planner_util); //LocalPlannerUtil 提供代价地图、坐标系转换等基础设施

    /**
     * @brief Reconfigures the trajectory planner
     */
    void reconfigure(dwa_local_planner::DWAPlannerConfig& cfg); //动态调整算法参数（如速度限制、加速度限制）

    /**
     * @brief  Check if a trajectory is legal for a position/velocity pair 轨迹合法性检查，验证给定状态下的轨迹是否可行（无碰撞、满足动力学约束）
     * @param pos The robot's position 机器人当前位置（x, y, theta）
     * @param vel The robot's velocity 当前速度（vx, vy, vtheta）
     * @param vel_samples The desired velocity 期望速度
     * @return True if the trajectory is valid, false otherwise
     */
    bool checkTrajectory(
        const Eigen::Vector3f pos,
        const Eigen::Vector3f vel,
        const Eigen::Vector3f vel_samples);

    /**
     * @brief Given the current position and velocity of the robot, find the best trajectory to exectue 核心路径规划方法，生成最优路径并返回速度指令
     * @param global_pose The current position of the robot 全局坐标系下的机器人位姿
     * @param global_vel The current velocity of the robot 全局坐标系下的速度
     * @param drive_velocities The velocities to send to the robot base 输出的驱动速度（线速度、角速度）
     * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
     */
    base_local_planner::Trajectory findBestPath(
        const geometry_msgs::PoseStamped& global_pose,
        const geometry_msgs::PoseStamped& global_vel,
        geometry_msgs::PoseStamped& drive_velocities); //1.状态采样：在速度空间中生成候选速度样本；2.轨迹生成：结合 Hybrid A* 和 Dijkstra 分层搜索，生成候选轨迹；3.成本评估：计算每条轨迹的成本（距离、障碍物、速度平滑性）；4.选择最优：返回成本最低的可行轨迹

    /**
     * @brief  Update the cost functions before planning 代价更新，更新全局路径和机器人轮廓，重新计算局部代价，确保规划器响应环境变化（如动态障碍物）
     * @param  global_pose The robot's current pose
     * @param  new_plan The new global plan
     * @param  footprint_spec The robot's footprint
     *
     * The obstacle cost function gets the footprint.
     * The path and goal cost functions get the global_plan
     * The alignment cost functions get a version of the global plan
     *   that is modified based on the global_pose
     */
    void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                                 const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                 const std::vector<geometry_msgs::Point>& footprint_spec);

    /**
     * @brief Get the period at which the local planner is expected to run 
     * @return The simulation period 模拟周期获取，提供规划周期，用于控制算法执行频率
     */
    double getSimPeriod() { return sim_period_; } 

    /**
     * @brief Compute the components and total cost for a map grid cell 计算指定地图单元格的路径成本、目标成本、占据成本及总成本
     * @param cx The x coordinate of the cell in the map grid 单元格的坐标
     * @param cy The y coordinate of the cell in the map grid
     * @param path_cost Will be set to the path distance component of the cost function 路径距离成本（如到参考路径的距离）
     * @param goal_cost Will be set to the goal distance component of the cost function 目标距离成本（如到终点的欧氏距离）
     * @param occ_cost Will be set to the costmap value of the cell 占据成本（来自代价地图的值）
     * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters 综合加权总成本
     * @return True if the cell is traversible and therefore a legal location for the robot to move to
     */
    bool getCellCosts(int cx, int cy, float& path_cost, float& goal_cost, float& occ_cost, float& total_cost);
    /**
     * sets new plan and resets state
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan); //更新全局路径并重置内部状态

    /**
     * @brief Provide access to costmap ROS wrapper so the planner can build safety corridor
     */
    void setCostmapRos(costmap_2d::Costmap2DROS* costmap_ros);

   private:
    bool prepareCritics(); //初始化或更新路径规划中的成本评估模块
    std::vector<Node> getDijkstraPath(const double& sim_x,
                                      const double& sim_y,
                                      const double& sim_th,
                                      const double& orientation_vel,
                                      const geometry_msgs::PoseStamped& global_vel); //通过 Dijkstra 算法生成参考路径；sim_x, sim_y, sim_th：模拟起始位姿；orientation_vel：当前朝向速度；global_vel：全局速度（用于动力学约束）

    void getAllowedVelocityAndGoalNodeIndex(std::vector<Node>& nodes_in_path, //nodes_in_path：参考路径节点序列
                            const geometry_msgs::PoseStamped& global_vel,
                            const double sim_period_, //sim_period_：规划周期
                            double& allowed_velocity, //输出的允许速度
                            int& goal_node_index); //输出的目标节点索引，该函数确定当前允许的最大速度及路径跟踪目标节点

    ros::Publisher marker_pub; //发布可视化标记
    ros::Publisher marker_pub_; // 类成员发布器
    visualization_msgs::MarkerArray marker_array_; // 存储所有标记
    visualization_msgs::Marker astar_path; //存储用于可视化的 A* 路径信息，通过 marker_pub 发布路径标记
    visualization_msgs::Marker hybrid_astar_path; // 新增：混合A*路径专属标记
    double layer_display_time_;

    base_local_planner::LocalPlannerUtil* planner_util_; //访问本地规划器工具，如坐标变换、代价地图

    double stop_time_buffer_;  ///< @brief How long before hitting something we're going to enforce that the robot stop 机器人在接近障碍物时提前停止的时间缓冲
    double path_distance_bias_, goal_distance_bias_, occdist_scale_; //路径距离、目标距离和占据距离的成本权重，平衡不同成本因素，影响规划器对路径的选择
    Eigen::Vector3f vsamples_; //存储速度样本（线速度、角速度），用于速度采样，生成候选轨迹

    double sim_period_;  ///< @brief The number of seconds to use to compute max/min vels for dwa 规划周期，用于计算速度的最大最小值，确保规划器在固定周期内完成速度计算
    base_local_planner::Trajectory result_traj_; //存储规划器生成的最优轨迹，作为最终输出，提供给机器人执行

    double forward_point_distance_;

    int step_count;

    std::vector<geometry_msgs::PoseStamped> global_plan_; //存储全局路径的各个点，作为局部规划的参考，确保路径整体趋向目标

    boost::mutex configuration_mutex_; //保护配置参数的访问，确保多线程环境下配置参数的原子操作
    std::string frame_id_;
    ros::Publisher traj_cloud_pub_; //发布轨迹的点云信息
    bool publish_cost_grid_pc_;  ///< @brief Whether or not to build and publish a PointCloud 控制是否发布代价网格的点云信息
    bool publish_traj_pc_; //控制是否发布轨迹的点云信息

    // --- 非阻塞可视化相关成员 ---
    ros::Timer viz_timer_;                       // 定时器，用于定时发布可视化数据
    double viz_rate_;                            // 可视化发布频率 (Hz)
    boost::mutex viz_mutex_;                     // 保护可视化缓冲区的互斥锁
    visualization_msgs::MarkerArray viz_marker_array_buffer_; // 缓冲图可视化数据
    visualization_msgs::Marker viz_path_marker_buffer_;       // 缓冲路径可视化数据
    ros::Publisher graph_pub_global_;            // 全局 MarkerArray 发布器（/graph_visualization）
    ros::Publisher astar_pub_global_;            // 全局单一 Marker 发布器（路径）
    ros::Publisher graph_pub_root_;              // 额外：在根命名空间上发布 graph_visualization（向后兼容 RViz 配置）
    ros::Publisher astar_pub_root_;              // 额外：在根命名空间上发布单一 visualization_marker
    bool have_viz_data_ = false;                 // 标志：是否有可视化数据可发布

    double cheat_factor_; //调整规划器行为的经验因子

    base_local_planner::MapGridVisualizer map_viz_;  ///< @brief The map grid visualizer for outputting the potential field generated by the cost function 将代价网格转换为可视化数据

    // see constructor body for explanations
    base_local_planner::SimpleTrajectoryGenerator generator_; //生成候选轨迹，为规划器提供多种可能的路径选项
    base_local_planner::OscillationCostFunction oscillation_costs_; //计算路径震荡带来的成本，防止机器人在原地打转，提高路径效率
    base_local_planner::ObstacleCostFunction obstacle_costs_; //计算机器人与障碍物之间的距离成本，确保路径安全，避免碰撞
    base_local_planner::MapGridCostFunction path_costs_; //计算路径偏离参考路径的成本，确保路径平滑，接近全局规划目标
    base_local_planner::MapGridCostFunction goal_costs_; //计算机器人到目标点的距离成本，引导机器人朝目标点移动
    base_local_planner::MapGridCostFunction goal_front_costs_; //计算前方路径到目标点的成本，确保路径具有前瞻性，避免局部最优
    base_local_planner::MapGridCostFunction alignment_costs_; //计算机器人朝向与路径方向的对齐程度，确保机器人沿着路径行驶，提高路径跟踪精度
    base_local_planner::TwirlingCostFunction twirling_costs_; //计算路径旋转带来的成本，防止机器人频繁旋转，提高运动效率

    base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_; //通过采样方法生成候选路径，并根据成本函数评分，选择最优路径，作为规划器的输出

    std::vector<base_local_planner::TrajectoryCostFunction*> critics;//存储各种轨迹成本函数，综合评估每条候选轨迹的成本，选择最优路径

    ros::Publisher current_pose_pub_;  // 新增：位姿发布者
    geometry_msgs::PoseStamped current_pose_;  // 新增：当前位姿存储
    void vizTimerCB(const ros::TimerEvent &ev); // 定时发布回调
    // --- Corridor members ---
    std::unique_ptr<rpp::common::safety_corridor::ConvexSafetyCorridor> safety_corridor_; //凸安全走廊对象
    std::vector<rpp::common::geometry::Polygon2d> corridor_polygons_; //当前分解得到的多边形序列
    ros::Publisher corridor_pub_; //用于发布 MarkerArray
    double safety_range_ = 0.6; //默认安全范围

    // dynamic pedestrians
    ros::Subscriber ped_sub_;
    std::vector<DynamicPed> dynamic_peds_;
    boost::mutex ped_mutex_;
    DynamicObstacleCostFunction dynamic_obstacle_costs_;

    SocialCostFunction social_costs_;

    void pedCallback(const pedsim_msgs::TrackedPersons::ConstPtr& msg);
};
};  // namespace hlpmpccorridor_local_planner
#endif
