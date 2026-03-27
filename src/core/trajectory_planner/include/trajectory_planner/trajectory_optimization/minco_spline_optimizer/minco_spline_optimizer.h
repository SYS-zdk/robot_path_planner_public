#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/point.h"
#include "common/structure/trajectory.h"
#include "trajectory_planner/trajectory_optimization/minco_spline_optimizer/minco_core.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{

class MincoSplineOptimizer : public Optimizer
{
private:
  using Point3d = rpp::common::geometry::Point3d;
  using Points3d = rpp::common::geometry::Points3d;
  using Trajectory3d = rpp::common::structure::Trajectory3d;

public:
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

  MincoSplineOptimizer(int sample_points, double vel_max, double acc_max);
  MincoSplineOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int sample_points, double vel_max, double acc_max);
  ~MincoSplineOptimizer() override = default;

  bool run(const Points3d& waypoints) override;
  bool run(const Trajectory3d& traj) override;
  bool getTrajectory(Trajectory3d& traj) override;

private:
  static TimeConfig::Mode parseTimeMode(const std::string& s);
  static Boundary2D loadBoundaryFromParams();
  static TimeConfig loadTimeConfigFromParams(int num_segments);

  bool buildAndSample(const Points3d& waypoints, const Boundary2D& bc, const TimeConfig& time_cfg);

private:
  int sample_points_{ 120 };
  double vel_max_{ 1.0 };
  double acc_max_{ 2.0 };

  bool has_solution_{ false };
  Trajectory3d traj_opt_;
};

}  // namespace trajectory_optimization
}  // namespace rpp
