#include "trajectory_planner/trajectory_optimization/splinetrajectory_optimizer/splinetrajectory_optimizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <ros/console.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "path_planner/path_simplify/rdp_path_simplifier.h"
#include "trajectory_planner/trajectory_optimization/time_allocation.h"

namespace
{
constexpr double kPruneDelta = 0.25;

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

rpp::trajectory_optimization::SplineTrajectoryOptimizer::TimeConfig::Mode parseTimeMode(const std::string& s)
{
  using Mode = rpp::trajectory_optimization::SplineTrajectoryOptimizer::TimeConfig::Mode;
  if (s == "uniform")
    return Mode::Uniform;
  if (s == "from_param" || s == "param" || s == "segments")
    return Mode::FromParam;
  return Mode::Trapezoidal;
}

rpp::trajectory_optimization::SplineTrajectoryOptimizer::Boundary2D loadBoundaryFromParams()
{
  rpp::trajectory_optimization::SplineTrajectoryOptimizer::Boundary2D bc;
  ros::param::param("/move_base/Optimizer/spline_bc/vx0", bc.vx0, 0.0);
  ros::param::param("/move_base/Optimizer/spline_bc/ax0", bc.ax0, 0.0);
  ros::param::param("/move_base/Optimizer/spline_bc/vxN", bc.vxN, 0.0);
  ros::param::param("/move_base/Optimizer/spline_bc/axN", bc.axN, 0.0);

  ros::param::param("/move_base/Optimizer/spline_bc/vy0", bc.vy0, 0.0);
  ros::param::param("/move_base/Optimizer/spline_bc/ay0", bc.ay0, 0.0);
  ros::param::param("/move_base/Optimizer/spline_bc/vyN", bc.vyN, 0.0);
  ros::param::param("/move_base/Optimizer/spline_bc/ayN", bc.ayN, 0.0);
  return bc;
}

rpp::trajectory_optimization::SplineTrajectoryOptimizer::TimeConfig loadTimeConfigFromParams(int num_segments)
{
  using TimeConfig = rpp::trajectory_optimization::SplineTrajectoryOptimizer::TimeConfig;

  TimeConfig cfg;
  std::string mode_str;
  ros::param::param<std::string>("/move_base/Optimizer/spline_time/mode", mode_str, std::string("trapezoidal"));
  cfg.mode = parseTimeMode(mode_str);
  ros::param::param("/move_base/Optimizer/spline_time/uniform_segment_time", cfg.uniform_segment_time, 0.0);
  ros::param::param("/move_base/Optimizer/spline_time/min_segment_time", cfg.min_segment_time, 1e-3);

  if (cfg.mode == TimeConfig::Mode::FromParam)
  {
    std::vector<double> segs;
    if (getDoubleVectorParam("/move_base/Optimizer/spline_time/segments", segs) &&
        static_cast<int>(segs.size()) == num_segments)
    {
      cfg.segments = std::move(segs);
    }
    else
    {
      ROS_WARN_STREAM("[SplineTrajectoryOptimizer] spline_time.mode=from_param but 'segments' missing/size mismatch; "
                      << "fallback to trapezoidal. expected=" << num_segments << " got=" << segs.size());
      cfg.mode = TimeConfig::Mode::Trapezoidal;
    }
  }

  if (cfg.mode == TimeConfig::Mode::Uniform)
  {
    if (cfg.uniform_segment_time <= 0.0)
    {
      ROS_WARN_STREAM("[SplineTrajectoryOptimizer] spline_time.mode=uniform but uniform_segment_time<=0; "
                      << "fallback to trapezoidal.");
      cfg.mode = TimeConfig::Mode::Trapezoidal;
    }
  }

  cfg.min_segment_time = std::max(1e-4, cfg.min_segment_time);
  return cfg;
}

inline void symmetrize(Eigen::Matrix2d& A)
{
  A = 0.5 * (A + A.transpose());
}

bool invert2x2Regularized(const Eigen::Matrix2d& A_in, Eigen::Matrix2d& A_inv)
{
  Eigen::Matrix2d A = A_in;
  symmetrize(A);
  double det = A.determinant();
  if (std::abs(det) < 1e-12)
  {
    A += 1e-9 * Eigen::Matrix2d::Identity();
    det = A.determinant();
    if (std::abs(det) < 1e-12)
    {
      return false;
    }
  }
  A_inv = A.inverse();
  return true;
}
}

namespace rpp
{
namespace trajectory_optimization
{

SplineTrajectoryOptimizer::SplineTrajectoryOptimizer(int sample_points, double vel_max, double acc_max)
  : Optimizer(), sample_points_(sample_points), vel_max_(vel_max), acc_max_(acc_max)
{
}

SplineTrajectoryOptimizer::SplineTrajectoryOptimizer(costmap_2d::Costmap2DROS* costmap_ros,
                                                     int sample_points,
                                                     double vel_max,
                                                     double acc_max)
  : Optimizer(costmap_ros), sample_points_(sample_points), vel_max_(vel_max), acc_max_(acc_max)
{
}

bool SplineTrajectoryOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

bool SplineTrajectoryOptimizer::run(const Points3d& waypoints)
{
  has_solution_ = false;
  segments_.clear();
  time_allocations_.clear();

  if (waypoints.size() < 2)
  {
    return false;
  }

  // Prune points to reduce conditioning issues and runtime.
  Points3d key_points;
  auto pruner = std::make_shared<rpp::path_planner::RDPPathSimplifier>(kPruneDelta);
  pruner->process(waypoints, key_points);
  if (key_points.size() < 2)
  {
    key_points = waypoints;
  }

  const Boundary2D bc = loadBoundaryFromParams();
  const TimeConfig time_cfg = loadTimeConfigFromParams(static_cast<int>(key_points.size()) - 1);

  if (!buildSegmentsFromWaypoints(key_points, bc, time_cfg))
  {
    return false;
  }

  has_solution_ = true;
  return true;
}

Eigen::Matrix3d SplineTrajectoryOptimizer::computeQabc(double T)
{
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T2 * T2;
  const double T5 = T4 * T;

  // W = \int_0^T [1, t, t^2]^T [1, t, t^2] dt
  Eigen::Matrix3d W;
  W << T, T2 / 2.0, T3 / 3.0,
      T2 / 2.0, T3 / 3.0, T4 / 4.0,
      T3 / 3.0, T4 / 4.0, T5 / 5.0;

  // [c0, c1, c2]^T = M [A, B, C]^T where jerk(t)=c0 + c1 t + c2 t^2
  // Derived from quintic coefficients with boundary p,v,a and A/B/C residuals.
  const double invT = 1.0 / T;
  const double invT2 = invT * invT;
  const double invT3 = invT2 * invT;
  const double invT4 = invT2 * invT2;
  const double invT5 = invT4 * invT;

  Eigen::Matrix3d M;
  M << 60.0 * invT3, -24.0 * invT2, 3.0 * invT,
      -360.0 * invT4, 168.0 * invT3, -24.0 * invT2,
      360.0 * invT5, -180.0 * invT4, 30.0 * invT3;

  Eigen::Matrix3d Q = M.transpose() * W * M;
  return 0.5 * (Q + Q.transpose());
}

bool SplineTrajectoryOptimizer::solveMinJerk1D(const std::vector<double>& p,
                                              const std::vector<double>& T,
                                              double v0,
                                              double a0,
                                              double vN,
                                              double aN,
                                              std::vector<double>& v_out,
                                              std::vector<double>& a_out)
{
  const int K = static_cast<int>(p.size());
  if (K < 2 || static_cast<int>(T.size()) != K - 1)
  {
    return false;
  }

  v_out.assign(K, 0.0);
  a_out.assign(K, 0.0);
  v_out.front() = v0;
  a_out.front() = a0;
  v_out.back() = vN;
  a_out.back() = aN;

  const int m = K - 2;  // internal knots count
  if (m <= 0)
  {
    return true;
  }

  std::vector<Eigen::Matrix2d> diag(m, Eigen::Matrix2d::Zero());
  std::vector<Eigen::Matrix2d> upper(m, Eigen::Matrix2d::Zero());
  std::vector<Eigen::Matrix2d> lower(m, Eigen::Matrix2d::Zero());
  std::vector<Eigen::Vector2d> b(m, Eigen::Vector2d::Zero());
  std::vector<Eigen::Vector2d> shift(m, Eigen::Vector2d::Zero());

  const auto isInternal = [K](int knot_idx) { return knot_idx >= 1 && knot_idx <= K - 2; };
  const auto toInternal = [](int knot_idx) { return knot_idx - 1; };

  for (int seg = 0; seg < K - 1; ++seg)
  {
    const int i = seg;
    const int j = seg + 1;
    const double dt = std::max(1e-3, T[seg]);
    const double dp = p[j] - p[i];

    const Eigen::Matrix3d Qabc = computeQabc(dt);

    // [A,B,C]^T = S * [vi, ai, vj, aj]^T + s0
    Eigen::Matrix<double, 3, 4> S;
    S.setZero();
    S(0, 0) = -dt;
    S(0, 1) = -0.5 * dt * dt;
    S(1, 0) = -1.0;
    S(1, 1) = -dt;
    S(1, 2) = 1.0;
    S(2, 1) = -1.0;
    S(2, 3) = 1.0;

    Eigen::Vector3d s0(dp, 0.0, 0.0);

    Eigen::Matrix4d H = S.transpose() * Qabc * S;
    H = 0.5 * (H + H.transpose());
    Eigen::Vector4d bb = S.transpose() * Qabc * s0;

    const Eigen::Matrix2d Hii = H.block<2, 2>(0, 0);
    const Eigen::Matrix2d Hij = H.block<2, 2>(0, 2);
    const Eigen::Matrix2d Hji = H.block<2, 2>(2, 0);
    const Eigen::Matrix2d Hjj = H.block<2, 2>(2, 2);
    const Eigen::Vector2d bi = bb.segment<2>(0);
    const Eigen::Vector2d bj = bb.segment<2>(2);

    Eigen::Vector2d xi_boundary(v0, a0);
    Eigen::Vector2d xj_boundary(vN, aN);

    const bool i_internal = isInternal(i);
    const bool j_internal = isInternal(j);

    if (!i_internal && i == 0)
    {
      xi_boundary << v0, a0;
    }
    else if (!i_internal && i == K - 1)
    {
      xi_boundary << vN, aN;
    }

    if (!j_internal && j == 0)
    {
      xj_boundary << v0, a0;
    }
    else if (!j_internal && j == K - 1)
    {
      xj_boundary << vN, aN;
    }

    if (i_internal)
    {
      const int ii = toInternal(i);
      diag[ii] += Hii;
      b[ii] += bi;

      if (j_internal)
      {
        const int jj = toInternal(j);
        upper[ii] += Hij;
        lower[jj] += Hji;
      }
      else
      {
        shift[ii] += Hij * xj_boundary;
      }
    }

    if (j_internal)
    {
      const int jj = toInternal(j);
      diag[jj] += Hjj;
      b[jj] += bj;

      if (!i_internal)
      {
        shift[jj] += Hji * xi_boundary;
      }
    }
  }

  std::vector<Eigen::Vector2d> rhs(m);
  for (int k = 0; k < m; ++k)
  {
    rhs[k] = -(b[k] + shift[k]);
    symmetrize(diag[k]);
  }

  // Block Thomas algorithm.
  for (int k = 1; k < m; ++k)
  {
    Eigen::Matrix2d inv_prev;
    if (!invert2x2Regularized(diag[k - 1], inv_prev))
    {
      return false;
    }
    const Eigen::Matrix2d mult = lower[k] * inv_prev;
    diag[k] -= mult * upper[k - 1];
    rhs[k] -= mult * rhs[k - 1];
    symmetrize(diag[k]);
  }

  std::vector<Eigen::Vector2d> sol(m);
  {
    Eigen::Matrix2d inv_last;
    if (!invert2x2Regularized(diag[m - 1], inv_last))
    {
      return false;
    }
    sol[m - 1] = inv_last * rhs[m - 1];
  }
  for (int k = m - 2; k >= 0; --k)
  {
    Eigen::Matrix2d inv_k;
    if (!invert2x2Regularized(diag[k], inv_k))
    {
      return false;
    }
    sol[k] = inv_k * (rhs[k] - upper[k] * sol[k + 1]);
  }

  for (int knot = 1; knot <= K - 2; ++knot)
  {
    const int idx = knot - 1;
    v_out[knot] = sol[idx](0);
    a_out[knot] = sol[idx](1);
  }

  return true;
}

bool SplineTrajectoryOptimizer::buildSegmentsFromWaypoints(const Points3d& waypoints,
                                                           const Boundary2D& bc,
                                                           const TimeConfig& time_cfg)
{
  const int K = static_cast<int>(waypoints.size());
  if (K < 2)
  {
    return false;
  }

  // Allocate time per segment.
  time_allocations_.clear();
  const int num_segments = K - 1;

  if (time_cfg.mode == TimeConfig::Mode::Trapezoidal)
  {
    TimeAllocator::trapezoidalAllocation<Point3d>(waypoints, vel_max_, acc_max_, time_allocations_);
    if (static_cast<int>(time_allocations_.size()) != num_segments)
    {
      return false;
    }
  }
  else if (time_cfg.mode == TimeConfig::Mode::Uniform)
  {
    const double dt = std::max(time_cfg.min_segment_time, time_cfg.uniform_segment_time);
    time_allocations_.assign(num_segments, dt);
  }
  else
  {
    if (static_cast<int>(time_cfg.segments.size()) != num_segments)
    {
      return false;
    }
    time_allocations_.resize(num_segments);
    for (int i = 0; i < num_segments; ++i)
    {
      time_allocations_[i] = std::max(time_cfg.min_segment_time, time_cfg.segments[i]);
    }
  }

  std::vector<double> px(K), py(K);
  for (int i = 0; i < K; ++i)
  {
    px[i] = waypoints[i].x();
    py[i] = waypoints[i].y();
  }

  std::vector<double> vx, ax, vy, ay;
  if (!solveMinJerk1D(px, time_allocations_, bc.vx0, bc.ax0, bc.vxN, bc.axN, vx, ax))
  {
    return false;
  }
  if (!solveMinJerk1D(py, time_allocations_, bc.vy0, bc.ay0, bc.vyN, bc.ayN, vy, ay))
  {
    return false;
  }

  segments_.clear();
  segments_.reserve(K - 1);

  for (int seg = 0; seg < K - 1; ++seg)
  {
    const double dt = std::max(1e-3, time_allocations_[seg]);

    Segment s;
    s.T = dt;

    s.px.solve({ px[seg], vx[seg], ax[seg] }, { px[seg + 1], vx[seg + 1], ax[seg + 1] }, dt);
    s.py.solve({ py[seg], vy[seg], ay[seg] }, { py[seg + 1], vy[seg + 1], ay[seg + 1] }, dt);

    segments_.push_back(s);
  }

  return true;
}

bool SplineTrajectoryOptimizer::getTrajectory(Trajectory3d& traj)
{
  if (!has_solution_ || segments_.empty())
  {
    return false;
  }

  const int N = std::max(2, sample_points_);
  traj.reset(N);

  std::vector<double> cum;
  cum.reserve(segments_.size());
  double total_T = 0.0;
  for (const auto& s : segments_)
  {
    total_T += s.T;
    cum.push_back(total_T);
  }

  const double dt_sample = total_T / static_cast<double>(N - 1);

  double last_theta = 0.0;
  for (int i = 0; i < N; ++i)
  {
    const double t = std::min(total_T, dt_sample * static_cast<double>(i));
    const auto it = std::lower_bound(cum.begin(), cum.end(), t);
    const int seg_idx = static_cast<int>(std::distance(cum.begin(), it));

    const double seg_start = (seg_idx > 0) ? cum[seg_idx - 1] : 0.0;
    const double tau = t - seg_start;
    const auto& s = segments_[seg_idx];

    const double x = s.px.x(tau);
    const double y = s.py.x(tau);
    const double vx = s.px.dx(tau);
    const double vy = s.py.dx(tau);
    const double ax = s.px.ddx(tau);
    const double ay = s.py.ddx(tau);

    double theta = last_theta;
    const double v2 = vx * vx + vy * vy;
    if (v2 > 1e-8)
    {
      theta = std::atan2(vy, vx);
      last_theta = theta;
    }

    traj.time.push_back(t);
    traj.position.emplace_back(x, y, theta);
    traj.velocity.emplace_back(vx, vy, 0.0);
    traj.acceletation.emplace_back(ax, ay, 0.0);
  }

  return true;
}

}  // namespace trajectory_optimization
}  // namespace rpp
