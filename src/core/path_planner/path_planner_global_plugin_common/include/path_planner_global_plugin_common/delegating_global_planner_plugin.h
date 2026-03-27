#ifndef RPP_PATH_PLANNER_GLOBAL_PLUGIN_COMMON_DELEGATING_GLOBAL_PLANNER_PLUGIN_H_
#define RPP_PATH_PLANNER_GLOBAL_PLUGIN_COMMON_DELEGATING_GLOBAL_PLANNER_PLUGIN_H_

#include <utility>

#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include "path_planner/path_planner_engine.h"
#include "path_planner/path_planner_ros_io.h"

namespace rpp
{
namespace path_planner
{

class DelegatingGlobalPlannerPlugin : public nav_core::BaseGlobalPlanner
{
public:
  DelegatingGlobalPlannerPlugin() = default;

  DelegatingGlobalPlannerPlugin(std::string instance_name, std::string fixed_planner_name)
    : fixed_planner_name_(std::move(fixed_planner_name)), instance_name_(std::move(instance_name))
  {
  }

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override
  {
    if (fixed_planner_name_.empty())
    {
      ROS_ERROR("DelegatingGlobalPlannerPlugin: fixed_planner_name is empty.");
      return;
    }

    // Prefer our fixed instance namespace (matches launch-side rosparam ns=...).
    // If empty, fall back to move_base provided name, then to "PathPlanner".
    const std::string instance_name =
        !instance_name_.empty() ? instance_name_ : (name.empty() ? std::string("PathPlanner") : name);

    // Per-plugin I/O toggle (visualization + make_plan service). Default enabled.
    {
      ros::NodeHandle private_nh("~/" + instance_name);
      private_nh.param("enable_io", enable_io_, true);
    }

    impl_.initializeWithFixedPlanner(instance_name, costmap_ros, fixed_planner_name_);

    if (enable_io_)
    {
      io_.initialize(instance_name, &impl_);
    }
  }

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override
  {
    const bool ok = impl_.makePlan(start, goal, plan);
    if (enable_io_)
    {
      io_.publish();
    }
    return ok;
  }

protected:
  PathPlannerEngine impl_;
  PathPlannerRosIO io_;
  bool enable_io_{ true };
  std::string fixed_planner_name_;
  std::string instance_name_;
};

}  // namespace path_planner
}  // namespace rpp

#endif
