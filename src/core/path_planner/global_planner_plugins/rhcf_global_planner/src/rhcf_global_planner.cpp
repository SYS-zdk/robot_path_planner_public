#include <pluginlib/class_list_macros.h>

#include "path_planner_global_plugin_common/fixed_algorithm_global_planner_plugin.h"

#include "rhcf_global_planner/rhcf_planner.h"

namespace rpp
{
namespace path_planner
{

class RhcfPlanner : public FixedAlgorithmGlobalPlannerPlugin
{
public:
  RhcfPlanner() : FixedAlgorithmGlobalPlannerPlugin("RhcfPlanner", "rhcf", PathPlannerEngine::GRAPH_PLANNER) {}

protected:
  std::shared_ptr<PathPlanner> createPlanner(const ros::NodeHandle& private_nh, costmap_2d::Costmap2DROS* costmap_ros,
                                             double obstacle_factor) override
  {
    RhcfPathPlanner::Params params;
    const std::string ns = plannerName();
    private_nh.param(ns + "/K_homotopy_classes", params.K_homotopy_classes, 5);
    private_nh.param(ns + "/discounting_factor", params.discounting_factor, 0.8);
    private_nh.param(ns + "/max_random_walks", params.max_random_walks, 200);
    private_nh.param(ns + "/max_vertex_steps_per_walk", params.max_vertex_steps_per_walk, 2000);
    private_nh.param(ns + "/w_length", params.w_length, 1.0);
    private_nh.param(ns + "/w_clearance", params.w_clearance, 2.0);
    private_nh.param(ns + "/w_costmap", params.w_costmap, 0.01);
    private_nh.param(ns + "/w_heuristic", params.w_heuristic, 1.0);
    private_nh.param(ns + "/visited_vertex_weight", params.visited_vertex_weight, 0.1);
    private_nh.param(ns + "/edge_similarity_threshold", params.edge_similarity_threshold, 0.7);
    private_nh.param(ns + "/voronoi_connect_diagonal", params.voronoi_connect_diagonal, false);
    private_nh.param(ns + "/traverse_unknown", params.traverse_unknown, true);
    private_nh.param(ns + "/use_fixed_seed", params.use_fixed_seed, false);
    private_nh.param(ns + "/fixed_seed", params.fixed_seed, 1);

    const double circ_radius = costmap_ros && costmap_ros->getLayeredCostmap() ?
                                   costmap_ros->getLayeredCostmap()->getCircumscribedRadius() :
                                   0.0;
    return std::make_shared<RhcfPathPlanner>(costmap_ros, circ_radius, params, obstacle_factor);
  }
};

}  // namespace path_planner
}  // namespace rpp

PLUGINLIB_EXPORT_CLASS(rpp::path_planner::RhcfPlanner, nav_core::BaseGlobalPlanner)
