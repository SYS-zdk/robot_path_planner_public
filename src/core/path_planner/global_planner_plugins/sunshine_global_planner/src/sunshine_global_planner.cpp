#include <pluginlib/class_list_macros.h>

#include "path_planner_global_plugin_common/fixed_algorithm_global_planner_plugin.h"

#include "sunshine_global_planner/sunshine_planner.h"

namespace rpp
{
namespace path_planner
{

class SunshinePlanner : public FixedAlgorithmGlobalPlannerPlugin
{
public:
  SunshinePlanner() : FixedAlgorithmGlobalPlannerPlugin("SunshinePlanner", "sunshine", PathPlannerEngine::SAMPLE_PLANNER) {}

protected:
  std::shared_ptr<PathPlanner> createPlanner(const ros::NodeHandle& private_nh, costmap_2d::Costmap2DROS* costmap_ros,
                                             double obstacle_factor) override
  {
    SunshinePathPlanner::Params params;
    const std::string ns = plannerName();
    private_nh.param(ns + "/max_iterations", params.max_iterations, 20000);
    private_nh.param(ns + "/theta_step", params.theta_step, 0.17453292519943295);
    private_nh.param(ns + "/length_step", params.length_step, 1.0);
    private_nh.param(ns + "/length_diff_threshold", params.length_diff_threshold, 8.0);
    private_nh.param(ns + "/forward_distance", params.forward_distance, 6.0);
    private_nh.param(ns + "/max_ray_length", params.max_ray_length, 200.0);
    private_nh.param(ns + "/enable_bidirectional_opt", params.enable_bidirectional_opt, true);
    private_nh.param(ns + "/optimize_iterations", params.optimize_iterations, 5);
    return std::make_shared<SunshinePathPlanner>(costmap_ros, obstacle_factor, params);
  }
};

}  // namespace path_planner
}  // namespace rpp

PLUGINLIB_EXPORT_CLASS(rpp::path_planner::SunshinePlanner, nav_core::BaseGlobalPlanner)
