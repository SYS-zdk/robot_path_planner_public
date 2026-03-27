#include "trajectory_planner/trajectory_optimization/minco_spline_optimizer/minco_spline_optimizer.h"

#include <algorithm>
#include <cmath>

#include <ros/console.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace
{
bool getDoubleVectorParam(const std::string& key, std::vector<double>& out)
{
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(key, v))
  {
    return false;
  }
  if (v.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return false;
  }

  out.clear();
  out.reserve(v.size());
  for (int i = 0; i < v.size(); ++i)
  {
    if (v[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      out.push_back(static_cast<int>(v[i]));
    }
    else if (v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      out.push_back(static_cast<double>(v[i]));
    }
    else
    {
      return false;
    }
  }
  return true;
}
}

namespace rpp
{
namespace trajectory_optimization
{

MincoSplineOptimizer::MincoSplineOptimizer(int sample_points, double vel_max, double acc_max)
  : Optimizer()
  , sample_points_(sample_points)
  , vel_max_(vel_max)
  , acc_max_(acc_max)
{
}

MincoSplineOptimizer::MincoSplineOptimizer(costmap_2d::Costmap2DROS* costmap_ros,
                                         int sample_points,
                                         double vel_max,
                                         double acc_max)
  : Optimizer(costmap_ros)
  , sample_points_(sample_points)
  , vel_max_(vel_max)
  , acc_max_(acc_max)
{
}

bool MincoSplineOptimizer::run(const Points3d& waypoints)
{
  has_solution_ = false;
  traj_opt_.clear();

  if (waypoints.size() < 2)
  {
    ROS_WARN("[MincoSplineOptimizer] waypoints < 2");
    return false;
  }

  const auto bc = loadBoundaryFromParams();
  const auto time_cfg = loadTimeConfigFromParams(static_cast<int>(waypoints.size()) - 1);

  if (!buildAndSample(waypoints, bc, time_cfg))
    return false;

  has_solution_ = true;
  return true;
}

bool MincoSplineOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

bool MincoSplineOptimizer::getTrajectory(Trajectory3d& traj)
{
  if (!has_solution_)
    return false;
  traj = traj_opt_;
  return true;
}

MincoSplineOptimizer::TimeConfig::Mode MincoSplineOptimizer::parseTimeMode(const std::string& s)
{
  if (s == "uniform")
    return TimeConfig::Mode::Uniform;
  if (s == "from_param" || s == "param" || s == "segments")
    return TimeConfig::Mode::FromParam;
  return TimeConfig::Mode::Trapezoidal;
}

MincoSplineOptimizer::Boundary2D MincoSplineOptimizer::loadBoundaryFromParams()
{
  Boundary2D bc;

  ros::param::param("/move_base/Optimizer/minco_bc/vx0", bc.vx0, 0.0);
  ros::param::param("/move_base/Optimizer/minco_bc/ax0", bc.ax0, 0.0);
  ros::param::param("/move_base/Optimizer/minco_bc/vxN", bc.vxN, 0.0);
  ros::param::param("/move_base/Optimizer/minco_bc/axN", bc.axN, 0.0);

  ros::param::param("/move_base/Optimizer/minco_bc/vy0", bc.vy0, 0.0);
  ros::param::param("/move_base/Optimizer/minco_bc/ay0", bc.ay0, 0.0);
  ros::param::param("/move_base/Optimizer/minco_bc/vyN", bc.vyN, 0.0);
  ros::param::param("/move_base/Optimizer/minco_bc/ayN", bc.ayN, 0.0);

  return bc;
}

MincoSplineOptimizer::TimeConfig MincoSplineOptimizer::loadTimeConfigFromParams(int num_segments)
{
  TimeConfig cfg;

  std::string mode_str;
  ros::param::param<std::string>("/move_base/Optimizer/minco_time/mode", mode_str, std::string("trapezoidal"));
  cfg.mode = parseTimeMode(mode_str);

  ros::param::param("/move_base/Optimizer/minco_time/uniform_segment_time", cfg.uniform_segment_time, 0.0);
  ros::param::param("/move_base/Optimizer/minco_time/min_segment_time", cfg.min_segment_time, 1e-3);

  if (cfg.mode == TimeConfig::Mode::FromParam)
  {
    std::vector<double> segments;
    if (getDoubleVectorParam("/move_base/Optimizer/minco_time/segments", segments) &&
        static_cast<int>(segments.size()) == num_segments)
    {
      cfg.segments = std::move(segments);
    }
    else
    {
      ROS_WARN_STREAM("[MincoSplineOptimizer] minco_time.mode=from_param but 'segments' missing/size mismatch; "
                      << "fallback to trapezoidal. expected=" << num_segments << " got=" << segments.size());
      cfg.mode = TimeConfig::Mode::Trapezoidal;
    }
  }

  if (cfg.mode == TimeConfig::Mode::Uniform)
  {
    if (cfg.uniform_segment_time <= 0.0)
    {
      ROS_WARN_STREAM("[MincoSplineOptimizer] minco_time.mode=uniform but uniform_segment_time<=0; "
                      << "fallback to trapezoidal.");
      cfg.mode = TimeConfig::Mode::Trapezoidal;
    }
  }

  cfg.min_segment_time = std::max(1e-4, cfg.min_segment_time);

  return cfg;
}

bool MincoSplineOptimizer::buildAndSample(const Points3d& waypoints, const Boundary2D& bc, const TimeConfig& time_cfg)
{
  const int piece_num = static_cast<int>(waypoints.size()) - 1;
  if (piece_num <= 0)
    return false;

  // Build 2D boundary conditions (z ignored; z copied from waypoints).
  rpp::trajectory_optimization::minco_spline::MinJerkOpt2D opt;

  Eigen::Matrix<double, 2, 3> head_pva;
  Eigen::Matrix<double, 2, 3> tail_pva;
  head_pva.setZero();
  tail_pva.setZero();

  head_pva(0, 0) = waypoints.front().x();
  head_pva(1, 0) = waypoints.front().y();
  head_pva(0, 1) = bc.vx0;
  head_pva(0, 2) = bc.ax0;
  head_pva(1, 1) = bc.vy0;
  head_pva(1, 2) = bc.ay0;

  tail_pva(0, 0) = waypoints.back().x();
  tail_pva(1, 0) = waypoints.back().y();
  tail_pva(0, 1) = bc.vxN;
  tail_pva(0, 2) = bc.axN;
  tail_pva(1, 1) = bc.vyN;
  tail_pva(1, 2) = bc.ayN;

  opt.reset(head_pva, tail_pva, piece_num);

  Eigen::MatrixXd inner_pos(std::max(0, piece_num - 1), 2);
  for (int i = 1; i < piece_num; ++i)
  {
    inner_pos(i - 1, 0) = waypoints[i].x();
    inner_pos(i - 1, 1) = waypoints[i].y();
  }

  Eigen::VectorXd T(piece_num);
  if (time_cfg.mode == TimeConfig::Mode::Uniform)
  {
    const double dt = std::max(time_cfg.uniform_segment_time, time_cfg.min_segment_time);
    T.setConstant(dt);
  }
  else if (time_cfg.mode == TimeConfig::Mode::FromParam)
  {
    for (int i = 0; i < piece_num; ++i)
      T[i] = std::max(time_cfg.segments[i], time_cfg.min_segment_time);
  }
  else
  {
    // Trapezoidal time allocation based on distance and vel_max.
    for (int i = 0; i < piece_num; ++i)
    {
      const double dx = waypoints[i + 1].x() - waypoints[i].x();
      const double dy = waypoints[i + 1].y() - waypoints[i].y();
      const double dist = std::hypot(dx, dy);
      const double dt = dist / std::max(vel_max_, 1e-3);
      T[i] = std::max(dt, time_cfg.min_segment_time);
    }
  }

  opt.generate(inner_pos, T);

  // Sample trajectory at fixed number of points.
  // Use piecewise polynomial pos as the output path points.
  const auto traj2d = opt.getTrajectory();

  const double total_time = T.sum();
  if (total_time <= 0.0)
    return false;

  const int N = std::max(sample_points_, 2);
  traj_opt_.reset(N);
  traj_opt_.clear();
  traj_opt_.time.reserve(N);
  traj_opt_.position.reserve(N);

  const double dt = total_time / static_cast<double>(N - 1);
  for (int i = 0; i < N; ++i)
  {
    const double t = dt * static_cast<double>(i);
    const Eigen::Vector2d p = traj2d.pos(t);
    const Eigen::Vector2d v = traj2d.vel(t);
    const double theta = std::atan2(v.y(), v.x());

    traj_opt_.time.push_back(t);
    traj_opt_.position.emplace_back(p.x(), p.y(), theta);
  }

  return true;
}

}  // namespace trajectory_optimization
}  // namespace rpp
