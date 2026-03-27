#include <pluginlib/class_list_macros.h>

#include "path_planner_global_plugin_common/fixed_algorithm_global_planner_plugin.h"

#include "raystar_global_planner/raystar_planner.h"

namespace rpp
{
namespace path_planner
{

class RaystarPlanner : public FixedAlgorithmGlobalPlannerPlugin
{
public:
  RaystarPlanner() : FixedAlgorithmGlobalPlannerPlugin("RaystarPlanner", "raystar", PathPlannerEngine::GRAPH_PLANNER) {}

protected:
  std::shared_ptr<PathPlanner> createPlanner(const ros::NodeHandle& private_nh, costmap_2d::Costmap2DROS* costmap_ros,
                                             double obstacle_factor) override
  {
    RaystarPathPlanner::Params params;
    const std::string ns = plannerName();

    private_nh.param(ns + "/implementation", params.implementation, std::string("grid"));

    private_nh.param(ns + "/k_paths", params.k_paths, 5);
    private_nh.param(ns + "/max_tries", params.max_tries, 24);

    private_nh.param(ns + "/allow_unknown", params.traverse_unknown, true);
    private_nh.param(ns + "/allow_diagonal", params.allow_diagonal, true);
    private_nh.param(ns + "/use_theta_star", params.use_theta_star, true);

    private_nh.param(ns + "/max_expansions", params.max_expansions, 250000);
    private_nh.param(ns + "/max_planning_time_ms", params.max_planning_time_ms, 1500);

    private_nh.param(ns + "/use_costmap_weight", params.use_costmap_weight, true);
    private_nh.param(ns + "/cost_penalty_weight", params.cost_penalty_weight, 1.0);

    private_nh.param(ns + "/penalty_weight", params.penalty_weight, 8.0);
    private_nh.param(ns + "/penalty_increment", params.penalty_increment, 1.0);
    private_nh.param(ns + "/penalty_radius_cells", params.penalty_radius_cells, 2);

    private_nh.param(ns + "/component_min_cells", params.component_min_cells, 12);
    private_nh.param(ns + "/max_components", params.max_components, 32);
    private_nh.param(ns + "/component_search_margin_cells", params.component_search_margin_cells, 60);

    private_nh.param(ns + "/store_debug_data", params.store_debug_data, true);
    private_nh.param(ns + "/publish_poly_obstacles", params.publish_poly_obstacles, true);

    private_nh.param(ns + "/visibility_ray_eps", params.visibility_ray_eps, 1e-6);
    private_nh.param(ns + "/visibility_max_rays", params.visibility_max_rays, 8000);
    private_nh.param(ns + "/visibility_max_vertices", params.visibility_max_vertices, 4096);
    private_nh.param(ns + "/polymap_min_loop_vertices", params.polymap_min_loop_vertices, 6);

    return std::make_shared<RaystarPathPlanner>(costmap_ros, params, obstacle_factor);
  }
};

}  // namespace path_planner
}  // namespace rpp

PLUGINLIB_EXPORT_CLASS(rpp::path_planner::RaystarPlanner, nav_core::BaseGlobalPlanner)
