
/**
 * *********************************************************
 *
 * @file: ilqr_controller.h
 * @brief: Contains the iterative LQR (iLQR) local controller class
 * @author: Zhang Dingkun (generated)
 * @date: 2026-02-04
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef ILQR_CONTROLLER_H
#define ILQR_CONTROLLER_H

#include <tf2/utils.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <memory>

#include "controller/controller.h"
#include "common/geometry/vec2d.h"
#include "common/geometry/line_segment2d.h"
#include "common/geometry/polygon2d.h"
#include "common/safety_corridor/convex_safety_corridor.h"
#include "common/safety_corridor/ackermann_config.h"

namespace costmap_2d
{
class DistanceLayer;
}  // namespace costmap_2d

namespace rpp
{
namespace controller
{
class ILQRController : public nav_core::BaseLocalPlanner, public Controller
{
public:
  ILQRController();
  ILQRController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  ~ILQRController();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool isGoalReached();
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  static constexpr int kNx = 5;  // (x, y, theta, v_prev, w_prev)
  static constexpr int kNu = 2;  // (v, w)
  using State = Eigen::Matrix<double, kNx, 1>;
  using Control = Eigen::Matrix<double, kNu, 1>;
  using MatA = Eigen::Matrix<double, kNx, kNx>;
  using MatB = Eigen::Matrix<double, kNx, kNu>;
  using MatK = Eigen::Matrix<double, kNu, kNx>;

  struct ILQRParams
  {
    int horizon_steps{30};
    int max_iter{10};
    double dt{0.1};

    double lambda_init{1e-3};
    double lambda_max{1e6};

    double v_ref{0.4};
    double ds_ref{0.04};

    double w_track{5.0};
    double w_heading{2.0};
    double w_u{0.1};
    double w_du{0.5};
    double w_obs{1.0};

    bool use_corridor{false};
    bool corridor_hard_constraint{true};
    double w_corridor{2.0};
    double corridor_safety_range{0.6};
    double corridor_wheelbase{0.3};
    double corridor_max_steer_angle{0.83};
    double corridor_track_width{0.3};

    double obs_cost_scale{1.0};
    double obs_cost_gain{5.0};
    double obs_cost_threshold{253.0};
    double obs_collision_cost{50.0};

    std::vector<double> alphas{ 1.0, 0.5, 0.25, 0.1, 0.05 };
  };

  struct Trajectory
  {
    std::vector<State> x;    // (x, y, theta, v_prev, w_prev)
    std::vector<Control> u;  // (v, w)
    double cost{0.0};
  };

  bool buildReference(const geometry_msgs::PoseStamped& robot_pose_map,
                      const std::vector<geometry_msgs::PoseStamped>& prune_plan,
                      std::vector<Eigen::Vector3d>& x_ref);

  double evaluateTrajectoryCost(const Trajectory& traj, const std::vector<Eigen::Vector3d>& x_ref);
  double obstacleCost(const Eigen::Vector2d& p, Eigen::Vector2d* grad);
  bool isCollision(const Eigen::Vector2d& p);

  bool buildCorridorFromReference(const std::vector<Eigen::Vector3d>& x_ref);
  double corridorCost(const Eigen::Vector2d& p, Eigen::Vector2d* grad, bool* outside);
  bool isInsideCorridor(const Eigen::Vector2d& p) const;
  void publishCorridorMarkers() const;
  void publishTrajectoryPath(const Trajectory& traj) const;

  bool solveILQR(const State& x0,
                 const std::vector<Eigen::Vector3d>& x_ref,
                 Trajectory& traj);

  State stepDynamics(const State& x, const Control& u);

  void linearizeDynamics(const State& x, const Control& u, MatA& A, MatB& B);

  void clampControl(Control& u);

  double obstacleCostCostmap(const Eigen::Vector2d& p, Eigen::Vector2d* grad);
  double obstacleCostESDF(const Eigen::Vector2d& p, Eigen::Vector2d* grad);

private:
  bool initialized_{false};
  bool goal_reached_{false};
  tf2_ros::Buffer* tf_{nullptr};

  bool use_esdf_{false};
  double esdf_sigma_m_{0.30};
  double esdf_collision_dist_m_{0.05};
  costmap_2d::DistanceLayer* distance_layer_{nullptr};

  ILQRParams params_;

  std::unique_ptr<rpp::common::safety_corridor::ConvexSafetyCorridor> safety_corridor_;
  std::vector<rpp::common::geometry::Polygon2d> corridor_polygons_;

  ros::Publisher debug_ref_pub_;
  ros::Publisher corridor_pub_;
  ros::Publisher corridor_pub_ns_;
  ros::Publisher traj_pub_;

  // goal parameters
  double goal_x_{0.0}, goal_y_{0.0};
  Eigen::Vector3d goal_rpy_{0.0, 0.0, 0.0};
};
}  // namespace controller
}  // namespace rpp

#endif
