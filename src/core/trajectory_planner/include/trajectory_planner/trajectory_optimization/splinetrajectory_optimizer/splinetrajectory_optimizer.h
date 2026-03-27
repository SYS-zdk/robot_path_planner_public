#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/curve/quintic_polynomial.h"
#include "common/geometry/point.h"
#include "common/structure/trajectory.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{

// Minimum-jerk (quintic) clamped spline trajectory generator.
// Core idea matches MINCO-style formulation but solved via block tridiagonal (Thomas) algorithm.
class SplineTrajectoryOptimizer : public Optimizer
{
private:
  using Point3d = rpp::common::geometry::Point3d;
  using Points3d = rpp::common::geometry::Points3d;
  using Trajectory3d = rpp::common::structure::Trajectory3d;
  using QuinticPolynomial = rpp::common::geometry::QuinticPolynomial;

  struct Segment
  {
    QuinticPolynomial px;
    QuinticPolynomial py;
    double T{ 0.0 };
  };

public:
  // Lightweight config structs (mirrors upstream SplineTrajectory concepts).
  struct Boundary2D
  {
    double vx0{ 0.0 };
    double ax0{ 0.0 };
    double vxN{ 0.0 };
    double axN{ 0.0 };

    double vy0{ 0.0 };
    double ay0{ 0.0 };
    double vyN{ 0.0 };
    double ayN{ 0.0 };
  };

  struct TimeConfig
  {
    enum class Mode
    {
      Trapezoidal,
      Uniform,
      FromParam,
    };

    Mode mode{ Mode::Trapezoidal };
    double uniform_segment_time{ 0.0 };
    double min_segment_time{ 1e-3 };
    std::vector<double> segments;
  };

  SplineTrajectoryOptimizer(int sample_points, double vel_max, double acc_max);
  SplineTrajectoryOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int sample_points, double vel_max, double acc_max);
  ~SplineTrajectoryOptimizer() override = default;

  bool run(const Points3d& waypoints) override;
  bool run(const Trajectory3d& traj) override;
  bool getTrajectory(Trajectory3d& traj) override;

private:
  static Eigen::Matrix3d computeQabc(double T);

  // Solve 1D minimum-jerk spline for velocities/accelerations at knots.
  static bool solveMinJerk1D(const std::vector<double>& p,
                            const std::vector<double>& T,
                            double v0,
                            double a0,
                            double vN,
                            double aN,
                            std::vector<double>& v_out,
                            std::vector<double>& a_out);

  bool buildSegmentsFromWaypoints(const Points3d& waypoints, const Boundary2D& bc, const TimeConfig& time_cfg);

private:
  int sample_points_{ 100 };
  double vel_max_{ 1.0 };
  double acc_max_{ 2.0 };

  std::vector<double> time_allocations_;
  std::vector<Segment> segments_;
  bool has_solution_{ false };
};

}  // namespace trajectory_optimization
}  // namespace rpp
