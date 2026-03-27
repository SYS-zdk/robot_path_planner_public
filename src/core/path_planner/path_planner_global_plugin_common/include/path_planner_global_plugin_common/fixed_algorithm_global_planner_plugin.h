#ifndef RPP_PATH_PLANNER_GLOBAL_PLUGIN_COMMON_FIXED_ALGORITHM_GLOBAL_PLANNER_PLUGIN_H_
#define RPP_PATH_PLANNER_GLOBAL_PLUGIN_COMMON_FIXED_ALGORITHM_GLOBAL_PLANNER_PLUGIN_H_

#include <memory>
#include <string>
#include <utility>

#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include "path_planner/path_planner.h"
#include "path_planner/path_planner_engine.h"
#include "path_planner/path_planner_ros_io.h"

namespace rpp
{
namespace path_planner
{

/**
 * @brief Base class for "one algorithm per global-planner plugin" packages.
 *
 * - Concrete plugin creates the algorithm instance (in its own package)
 * - This base class wires it into PathPlannerEngine + optional ROS IO
 */
class FixedAlgorithmGlobalPlannerPlugin : public nav_core::BaseGlobalPlanner
{
public:
  FixedAlgorithmGlobalPlannerPlugin() = default;

  FixedAlgorithmGlobalPlannerPlugin(std::string instance_name, std::string planner_name,
                                    PathPlannerEngine::PLANNER_TYPE planner_type)
    : planner_name_(std::move(planner_name))
    , instance_name_(std::move(instance_name))
    , planner_type_(planner_type)
  {
  }

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override
  {
    if (planner_name_.empty())
    {
      ROS_ERROR("FixedAlgorithmGlobalPlannerPlugin: planner_name is empty.");
      return;
    }

    // Prefer our fixed instance namespace (matches launch-side rosparam ns=...).
    // If empty, fall back to move_base provided name, then to "PathPlanner".
    const std::string instance_name =
        !instance_name_.empty() ? instance_name_ : (name.empty() ? std::string("PathPlanner") : name);

    ros::NodeHandle private_nh("~/" + instance_name);

    // Per-plugin I/O toggle (visualization + make_plan service). Default enabled.
    private_nh.param("enable_io", enable_io_, true);

    // Use WARN so this line is still visible when move_base is configured with rosconsole>=WARN.
    ROS_WARN("FixedAlgorithmGlobalPlannerPlugin: instance=~/%s planner=%s enable_io=%s", instance_name.c_str(),
         planner_name_.c_str(), enable_io_ ? "true" : "false");

    // Keep obstacle_factor in the same place as previous engine-driven implementation.
    double obstacle_factor = 0.5;
    private_nh.param("obstacle_factor", obstacle_factor, 0.5);

    auto planner = createPlanner(private_nh, costmap_ros, obstacle_factor);
    if (!planner)
    {
      ROS_ERROR_STREAM("FixedAlgorithmGlobalPlannerPlugin: createPlanner() returned null. planner=" << planner_name_);
      return;
    }

    impl_.initializeWithPlanner(instance_name, costmap_ros, planner_name_, planner_type_, std::move(planner));

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
  const std::string& plannerName() const { return planner_name_; }

  virtual std::shared_ptr<PathPlanner> createPlanner(const ros::NodeHandle& private_nh,
                                                     costmap_2d::Costmap2DROS* costmap_ros,
                                                     double obstacle_factor) = 0;

protected:
  PathPlannerEngine impl_;
  PathPlannerRosIO io_;
  bool enable_io_{ true };

private:
  std::string planner_name_;
  std::string instance_name_;
  PathPlannerEngine::PLANNER_TYPE planner_type_{ PathPlannerEngine::GRAPH_PLANNER };
};

}  // namespace path_planner
}  // namespace rpp

#endif  // RPP_PATH_PLANNER_GLOBAL_PLUGIN_COMMON_FIXED_ALGORITHM_GLOBAL_PLANNER_PLUGIN_H_
