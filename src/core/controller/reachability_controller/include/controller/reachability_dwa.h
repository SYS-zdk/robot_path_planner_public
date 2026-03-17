/**
 * @file reachability_dwa.h
 * @brief Reachability-aware trajectory sampling and scoring core (DWA/MPPI-style).
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef RMP_CONTROLLER_REACHABILITY_DWA_H_
#define RMP_CONTROLLER_REACHABILITY_DWA_H_

#include <random>
#include <vector>

#include <Eigen/Core>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/trajectory.h>

#include <localreachability_layer/localreachability_layer.h>

#include "reachability_controller/ReachabilityControllerConfig.h"

namespace rpp
{
namespace controller
{
/**
 * @class ReachabilityDWA
 * @brief MPPI core (kept class name for compatibility)
 */
class ReachabilityDWA
{
public:
  /**
   * @brief  Constructor for the planner
   * @param name The name of the planner
   * @param costmap_ros A pointer to the costmap instance the planner should use
   * @param global_frame the frame id of the tf frame to use
   */
  ReachabilityDWA(std::string name, base_local_planner::LocalPlannerUtil* planner_util,
                  costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Reconfigures the trajectory planner
   */
  void reconfigure(reachability_controller::ReachabilityControllerConfig& cfg);

  /**
   * @brief  Check if a trajectory is legal for a position/velocity pair
   * @param pos The robot's position
   * @param vel The robot's velocity
   * @param vel_samples The desired velocity
   * @return True if the trajectory is valid, false otherwise
   */
  bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);

  /**
   * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
   * @param global_pose The current position of the robot
   * @param global_vel The current velocity of the robot
   * @param drive_velocities The velocities to send to the robot base
   * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
   */
  base_local_planner::Trajectory findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                              const geometry_msgs::PoseStamped& global_vel,
                                              geometry_msgs::PoseStamped& drive_velocities);

  /**
   * @brief  Update the cost functions before planning
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
   * @return The simulation period
   */
  double getSimPeriod()
  {
    return sim_period_;
  }

  /**
   * @brief Compute the components and total cost for a map grid cell
   * @param cx The x coordinate of the cell in the map grid
   * @param cy The y coordinate of the cell in the map grid
   * @param path_cost Will be set to the path distance component of the cost function
   * @param goal_cost Will be set to the goal distance component of the cost function
   * @param occ_cost Will be set to the costmap value of the cell
   * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
   * @return True if the cell is traversible and therefore a legal location for the robot to move to
   */
  bool getCellCosts(int cx, int cy, float& path_cost, float& goal_cost, float& occ_cost, float& total_cost);

  /**
   * sets new plan and resets state
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

private:
  struct MPPICostWeights
  {
    double path{ 5.0 };
    double goal{ 2.0 };
    double lookahead_goal{ 2.0 };
    double heading{ 1.0 };
    double path_heading{ 0.0 };
    double obstacle{ 1.0 };
    double control{ 0.1 };
    double smooth{ 0.0 };
    double forward{ 0.0 };
    double reachability{ 5.0 };
  };

  void ensureControlSequence();
  void shiftControlSequence();
  void refreshReachabilityCache();
  double costToPath(double x, double y) const;

  bool computeLookaheadTarget(const geometry_msgs::PoseStamped& reference_pose, double lookahead_dist,
                              double& target_x, double& target_y) const;

  void publishSampledTrajectories(const geometry_msgs::PoseStamped& global_pose,
                                  const std::vector<double>& costs,
                                  const std::vector<std::vector<double>>& eps_v,
                                  const std::vector<std::vector<double>>& eps_w,
                                  double v_min,
                                  double v_max,
                                  double w_min,
                                  double w_max);

  base_local_planner::Trajectory rolloutTrajectory(const geometry_msgs::PoseStamped& start_pose,
                                                   const std::vector<double>& seq_v,
                                                   const std::vector<double>& seq_w) const;

  base_local_planner::LocalPlannerUtil* planner_util_{ nullptr };
  costmap_2d::Costmap2DROS* costmap_ros_{ nullptr };

  boost::shared_ptr<costmap_2d::LocalReachabilityLayer> reach_layer_;
  bool reach_cache_valid_{ false };
  unsigned int reach_size_x_{ 0 };
  unsigned int reach_size_y_{ 0 };
  std::vector<double> reach_field_cache_;

  double min_reachability_{ 0.0 };
  bool reachability_use_min_{ true };

  int mppi_num_samples_{ 200 };
  int mppi_horizon_steps_{ 30 };
  int mppi_iterations_{ 1 };
  double mppi_dt_{ 0.05 };
  double mppi_lambda_{ 1.0 };
  double mppi_noise_v_std_{ 0.2 };
  double mppi_noise_w_std_{ 0.6 };
  MPPICostWeights w_;

  double mppi_lookahead_dist_{ 0.8 };
  double mppi_update_alpha_{ 1.0 };

  std::vector<double> u_v_;
  std::vector<double> u_w_;

  mutable std::mt19937 rng_;
  mutable std::normal_distribution<double> noise_v_;
  mutable std::normal_distribution<double> noise_w_;

  double sim_period_{ 0.05 };
  base_local_planner::Trajectory result_traj_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;

  bool publish_traj_pc_{ false };
  int traj_viz_num_samples_{ 50 };
  int traj_viz_stride_{ 1 };
  std::string traj_cloud_frame_;
  ros::Publisher traj_cloud_pub_;

  boost::mutex configuration_mutex_;
};
};  // namespace controller
}  // namespace rpp
#endif