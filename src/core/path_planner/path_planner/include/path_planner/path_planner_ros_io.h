/**
 * @file: path_planner_ros_io.h
 * @brief: ROS I/O + visualization helper for global planner plugins.
 *
 * This component owns publishers/services and consumes PathPlannerEngine's DebugData.
 */
#ifndef RPP_PATH_PLANNER_PATH_PLANNER_ROS_IO_H_
#define RPP_PATH_PLANNER_PATH_PLANNER_ROS_IO_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>

#include "path_planner/path_planner_engine.h"

namespace rpp
{
namespace path_planner
{

class PathPlannerRosIO
{
public:
  PathPlannerRosIO() = default;

  void initialize(const std::string& instance_name, PathPlannerEngine* engine);

  void publish();

private:
  bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

private:
  bool initialized_{ false };
  std::string instance_name_;

  PathPlannerEngine* engine_{ nullptr };

  ros::NodeHandle io_nh_;

  ros::Publisher plan_pub_;
  ros::Publisher plan_opt_pub_;
  ros::Publisher expand_pub_;
  ros::Publisher points_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher tree_pub_;
  ros::Publisher particles_pub_;
  ros::Publisher nag_candidates_pub_;
  ros::Publisher nag_cutpoints_pub_;
  ros::Publisher nag_cpr_pub_;
  ros::Publisher raystar_candidates_pub_;
  ros::Publisher raystar_poly_obstacles_pub_;
  ros::Publisher gkvm_landmarks_pub_;
  ros::Publisher gkvm_route_pub_;
  ros::Publisher rcc_marker_pub_;
  ros::Publisher parallel_curves_marker_pub_;
  ros::ServiceServer make_plan_srv_;
  ros::Publisher prune_plan_pub_;
  ros::Publisher initial_plan_pub_;
  ros::Publisher initial_plan_pub2_;
  ros::Publisher plan_time_pub_;

  bool has_published_gkvm_{ false };

  bool is_initial_plan_recorded_{ false };
  bool is_initial_plan_recorded2_{ false };
  std::vector<PathPlanner::Point3d> initial_plan_;
  std::vector<PathPlanner::Point3d> initial_plan2_;
};

}  // namespace path_planner
}  // namespace rpp

#endif
