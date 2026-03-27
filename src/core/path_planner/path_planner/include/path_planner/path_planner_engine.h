/**
 * @file: path_planner_engine.h
 * @brief: Core global-planner pipeline (algorithm + toolchain). ROS I/O is owned by plugin wrappers.
 * @author: ZhangDingkun
 * @date: 2026.03.22
 */
#ifndef RPP_PATH_PLANNER_PATH_PLANNER_ENGINE_H_
#define RPP_PATH_PLANNER_PATH_PLANNER_ENGINE_H_

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/point.h"
#include "path_planner/path_planner.h"
#include "path_planner/debug_data.h"
#include "path_planner/path_simplify/path_simplify.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rpp
{
namespace path_planner
{

/**
 * @brief Shared planning pipeline for global planner plugins.
 *
 * This class owns the algorithm instance + toolchain and implements the planning computation.
 * Plugin wrappers manage their own publishers/services and may visualize using the debug data
 * exposed by this pipeline.
 */
class PathPlannerEngine
{
public:
  enum PLANNER_TYPE
  {
    GRAPH_PLANNER = 0,
    SAMPLE_PLANNER = 1,
    EVOLUTION_PLANNER = 2,
  };

  using DebugData = ::rpp::path_planner::DebugData;

  PathPlannerEngine();

  PathPlannerEngine(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  ~PathPlannerEngine() = default;

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);

  /**
   * @brief Initialize pipeline with a concrete planner instance provided by the plugin.
   *
   * This is the preferred initialization path for the "one algorithm per plugin package" structure,
   * matching the local-planner/controller pattern.
   */
  void initializeWithPlanner(std::string instance_name, costmap_2d::Costmap2DROS* costmapRos,
                             const std::string& planner_name, PLANNER_TYPE planner_type,
                             std::shared_ptr<PathPlanner> planner);

  void initializeWithFixedPlanner(std::string name, costmap_2d::Costmap2DROS* costmapRos,
                                  const std::string& fixed_planner_name);

  void initialize(std::string name);

  void initializeWithFixedPlanner(std::string name, const std::string& fixed_planner_name);

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief 根据全局规划方法配置对应的工具链（RDP、优化器、走廊等）
   */
  void configurePlannerToolchain();

  const DebugData& debugData() const;

  bool isInitialized() const;
  const std::string& frameId() const;
  const std::string& plannerName() const;
  PLANNER_TYPE plannerType() const;
  bool isExpandEnabled() const;
  bool shouldShowSafetyCorridor() const;
  costmap_2d::Costmap2DROS* costmapRos() const;
  PathPlanner* planner() const;

protected:
  bool _getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan);

protected:
  bool initialized_;                        // initialization flag
  costmap_2d::Costmap2DROS* costmap_ros_;   // costmap(ROS wrapper)
  std::string frame_id_;                    // costmap frame ID
  std::string planner_name_;                // planner name
  std::string instance_name_;               // global planner plugin instance name (private namespace)
  std::string forced_planner_name_;         // optional: force planner_name_ without relying on rosparam
  PLANNER_TYPE planner_type_;               // planner type
  std::string optimizer_name_;              // optimizer name
  std::shared_ptr<PathPlanner> g_planner_;  // global graph planner

  DebugData debug_data_;                    // last planning debug snapshot

private:
  bool is_outline_;            // whether outline the boudary of map
  bool is_expand_;             // whether publish expand map or not
  bool auto_safety_corridor_;  // whether auto-toggle show_safety_corridor_ in toolchain
  bool show_safety_corridor_;  // whether visualize safety corridor
  bool use_safety_corridor_;   // whether use safety corridor constraint in optimization
  double tolerance_;           // tolerance
  double factor_;              // obstacle inflation factor

private:
  std::shared_ptr<PathSimplifier> pruner_;
  std::shared_ptr<rpp::trajectory_optimization::Optimizer> optimizer_;
};

}  // namespace path_planner
}  // namespace rpp

#endif
