
/***********************************************************
 *
 * @file: lbfgs_optimizer.cpp
 * @brief: Trajectory optimization using L-BFGS method
 * @author: Zhang Dingkun
 * @date: 2026-02-06
 * @version: 1.1
 *
 * L-BFGS path smoother for Hybrid A* and other planners.
 *
 * Reference:
 *   - LBFGS-Lite: https://github.com/ZJU-FAST-Lab/LBFGS-Lite
 *   - hybridAstar_lbfgsSmooth: https://github.com/USE-jx/hybridAstar_lbfgsSmooth
 *
 **********************************************************/
#include <cmath>
#include <algorithm>
#include <lbfgs.hpp>

#include "common/math/math_helper.h"
#include "common/util/log.h"

#include "trajectory_planner/trajectory_optimization/lbfgs_optimizer/lbfgs_optimizer.h"

namespace rpp
{
namespace trajectory_optimization
{

// ======================== Construction ========================

LBFGSOptimizer::LBFGSOptimizer(costmap_2d::Costmap2DROS* costmap_ros,
                               int max_iter,
                               double obs_dist_max, double k_max,
                               double w_obstacle, double w_smooth, double w_curvature)
  : Optimizer(costmap_ros)
  , max_iter_(max_iter)
  , obs_dist_max_(obs_dist_max)
  , k_max_(k_max)
  , w_obstacle_(w_obstacle)
  , w_smooth_(w_smooth)
  , w_curvature_(w_curvature)
{
}

// ======================== Public interface ========================

bool LBFGSOptimizer::run(const Points3d& waypoints)
{
  path_opt_ = waypoints;
  return optimize(waypoints);
}

bool LBFGSOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

bool LBFGSOptimizer::getTrajectory(Trajectory3d& traj)
{
  traj.reset(0);

  if (path_opt_.size() < 2)
  {
    return false;
  }

  // Resample polyline to a dense trajectory (minimumsnap returns ~100 samples).
  // This helps controllers that expect a sufficiently dense global plan.
  constexpr int kTimeSamples = 100;
  std::vector<double> s;
  s.reserve(path_opt_.size());
  s.push_back(0.0);
  for (size_t i = 1; i < path_opt_.size(); ++i)
  {
    const double ds = std::hypot(path_opt_[i].x() - path_opt_[i - 1].x(), path_opt_[i].y() - path_opt_[i - 1].y());
    s.push_back(s.back() + ds);
  }

  const double total_len = s.back();
  if (!(total_len > 1e-9))
  {
    // Degenerate: all points identical.
    traj.time.push_back(0.0);
    traj.position.emplace_back(path_opt_.front().x(), path_opt_.front().y(), path_opt_.front().theta());
    return true;
  }

  const int samples = std::max(2, kTimeSamples);
  const double ds = total_len / static_cast<double>(samples - 1);
  const double dt = 0.1;

  traj.reset(samples);

  size_t seg = 1;
  for (int i = 0; i < samples; ++i)
  {
    const double si = ds * static_cast<double>(i);
    while (seg < s.size() && s[seg] < si)
    {
      ++seg;
    }

    if (seg >= s.size())
    {
      seg = s.size() - 1;
    }

    const size_t i1 = seg;
    const size_t i0 = (i1 > 0) ? (i1 - 1) : 0;
    const double s0 = s[i0];
    const double s1 = s[i1];
    const double denom = std::max(1e-9, s1 - s0);
    const double u = std::min(1.0, std::max(0.0, (si - s0) / denom));

    const double x = path_opt_[i0].x() + u * (path_opt_[i1].x() - path_opt_[i0].x());
    const double y = path_opt_[i0].y() + u * (path_opt_[i1].y() - path_opt_[i0].y());

    traj.time.push_back(static_cast<double>(i) * dt);
    traj.position.emplace_back(x, y, 0.0);
  }

  // Compute a simple heading from positions.
  for (size_t i = 1; i < traj.position.size(); ++i)
  {
    const double dx = traj.position[i].x() - traj.position[i - 1].x();
    const double dy = traj.position[i].y() - traj.position[i - 1].y();
    if (std::hypot(dx, dy) > 1e-9)
    {
      traj.position[i - 1].setTheta(std::atan2(dy, dx));
    }
  }
  if (traj.position.size() >= 2)
  {
    traj.position.back().setTheta(traj.position[traj.position.size() - 2].theta());
  }

  // Vel/acc (optional, used mainly for debugging/visualization).
  for (size_t i = 0; i + 1 < traj.position.size(); ++i)
  {
    const double vx = (traj.position[i + 1].x() - traj.position[i].x()) / dt;
    const double vy = (traj.position[i + 1].y() - traj.position[i].y()) / dt;
    traj.velocity.emplace_back(vx, vy);
  }
  for (size_t i = 0; i + 2 < traj.position.size(); ++i)
  {
    const double vx0 = (traj.position[i + 1].x() - traj.position[i].x()) / dt;
    const double vy0 = (traj.position[i + 1].y() - traj.position[i].y()) / dt;
    const double vx1 = (traj.position[i + 2].x() - traj.position[i + 1].x()) / dt;
    const double vy1 = (traj.position[i + 2].y() - traj.position[i + 1].y()) / dt;
    traj.acceletation.emplace_back((vx1 - vx0) / dt, (vy1 - vy0) / dt);
  }

  return true;
}

// ======================== Core optimization ========================

bool LBFGSOptimizer::optimize(const Points3d& waypoints)
{
  const int total_points = static_cast<int>(waypoints.size());

  // Need at least 5 points (2 fixed at each end + 1 free)
  if (total_points < 5)
  {
    R_WARN << "[LBFGSOptimizer] Too few waypoints (" << total_points << "), skipping optimization.";
    return true;  // keep original path
  }

  // ---------- Acquire distance layer ----------
  distance_layer_.reset();
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    auto dl = boost::dynamic_pointer_cast<costmap_2d::DistanceLayer>(*layer);
    if (dl)
    {
      distance_layer_ = dl;
      break;
    }
  }
  if (!distance_layer_)
  {
    R_WARN << "[LBFGSOptimizer] Distance layer not found, obstacle cost disabled.";
  }

  // ---------- Build decision variables ----------
  // Fix first 2 and last 2 points; optimize interior points.
  const int fixed_ends = 2;
  const int free_points = total_points - 2 * fixed_ends;
  const int dim = 2 * free_points;

  Eigen::VectorXd x(dim);
  for (int i = 0; i < free_points; ++i)
  {
    const auto& pt = path_opt_[i + fixed_ends];
    x(i) = pt.x();
    x(i + free_points) = pt.y();
  }

  // ---------- Configure L-BFGS parameters ----------
  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 256;
  params.past = 3;
  params.delta = 1.0e-5;
  params.g_epsilon = 0.0;
  params.max_iterations = (max_iter_ > 0) ? max_iter_ : 0;
  params.min_step = 1.0e-32;

  // ---------- Run L-BFGS ----------
  double final_cost = 0.0;
  int ret = lbfgs::lbfgs_optimize(x, final_cost,
                                  &LBFGSOptimizer::costFunction,
                                  nullptr,
                                  nullptr,
                                  this,
                                  params);

  if (ret < 0 && ret != lbfgs::LBFGSERR_MAXIMUMITERATION)
  {
    R_WARN << "[LBFGSOptimizer] L-BFGS returned error code " << ret
           << " (" << lbfgs::lbfgs_strerror(ret) << "), using best result so far.";
  }
  else
  {
    R_INFO << "[LBFGSOptimizer] Converged: cost=" << final_cost << ", code=" << ret;
  }

  // ---------- Write back optimized positions (with safety clamp) ----------
  double ox = costmap_ros_->getCostmap()->getOriginX();
  double oy = costmap_ros_->getCostmap()->getOriginY();
  double map_w = costmap_ros_->getCostmap()->getSizeInMetersX();
  double map_h = costmap_ros_->getCostmap()->getSizeInMetersY();

  for (int i = 0; i < free_points; ++i)
  {
    double px = x(i);
    double py = x(i + free_points);

    // Clamp to map bounds to prevent downstream crash
    px = std::max(ox + 0.01, std::min(ox + map_w - 0.01, px));
    py = std::max(oy + 0.01, std::min(oy + map_h - 0.01, py));

    // NaN/Inf guard: revert to original if invalid
    if (std::isnan(px) || std::isinf(px) || std::isnan(py) || std::isinf(py))
    {
      px = path_opt_[i + fixed_ends].x();
      py = path_opt_[i + fixed_ends].y();
    }

    path_opt_[i + fixed_ends].setX(px);
    path_opt_[i + fixed_ends].setY(py);
  }

  // ---------- Recompute heading (theta) from optimized positions ----------
  for (int i = 1; i < total_points; ++i)
  {
    double dx = path_opt_[i].x() - path_opt_[i - 1].x();
    double dy = path_opt_[i].y() - path_opt_[i - 1].y();
    if (std::hypot(dx, dy) > 1e-6)
    {
      path_opt_[i - 1].setTheta(std::atan2(dy, dx));
    }
  }
  if (total_points >= 2)
  {
    path_opt_[total_points - 1].setTheta(path_opt_[total_points - 2].theta());
  }

  return true;
}

// ======================== L-BFGS cost callback ========================
//
// Follows the structure of USE-jx/hybridAstar_lbfgsSmooth:
//   - Obstacle cost: per-point, using distance field
//   - Smooth cost:   5-point stencil (fourth-order finite difference)
//   - Curvature cost: per-triplet with 3 overlapping kappa contributions
//

double LBFGSOptimizer::costFunction(void* instance,
                                    const Eigen::VectorXd& x,
                                    Eigen::VectorXd& g)
{
  auto* self = static_cast<LBFGSOptimizer*>(instance);
  const int N = static_cast<int>(self->path_opt_.size());
  const int fixed_ends = 2;
  const int free_points = N - 2 * fixed_ends;

  // Reconstruct full 2D path: [fixed_head | free | fixed_tail]
  Eigen::Matrix2Xd pts(2, N);
  // Fixed head
  for (int i = 0; i < fixed_ends; ++i)
  {
    pts(0, i) = self->path_opt_[i].x();
    pts(1, i) = self->path_opt_[i].y();
  }
  // Free interior
  for (int i = 0; i < free_points; ++i)
  {
    pts(0, i + fixed_ends) = x(i);
    pts(1, i + fixed_ends) = x(i + free_points);
  }
  // Fixed tail
  for (int i = 0; i < fixed_ends; ++i)
  {
    int idx = N - 1 - i;
    pts(0, idx) = self->path_opt_[idx].x();
    pts(1, idx) = self->path_opt_[idx].y();
  }

  // Gradient accumulator for free points only (2 x free_points)
  Eigen::Matrix2Xd grad = Eigen::Matrix2Xd::Zero(2, free_points);

  double cost = 0.0;

  // ==================== 1. Obstacle cost ====================
  if (self->distance_layer_)
  {
    const double resolution = self->costmap_ros_->getCostmap()->getResolution();

    for (int i = 0; i < free_points; ++i)
    {
      int pi = i + fixed_ends;  // index in full path
      double wx = pts(0, pi);
      double wy = pts(1, pi);

      unsigned int mx, my;
      if (!self->costmap_ros_->getCostmap()->worldToMap(wx, wy, mx, my))
        continue;

      // ✅ FIX: 移除错误的Y轴坐标转换
      // EDF（距离场）直接使用costmap的行列索引，不需要反转Y轴
      // 之前的 `ny_ - my - 1` 会导致访问错误的距离值和梯度方向
      double obs_dist = self->distance_layer_->getDistance(mx, my) * resolution;

      if (obs_dist < self->obs_dist_max_ && obs_dist > 1e-6)
      {
        double diff = obs_dist - self->obs_dist_max_;  // negative

        cost += self->w_obstacle_ * diff * diff;

        // Gradient direction: from nearest obstacle toward the point
        // Use distance layer's nearest-obstacle info to compute direction
        double gx_raw, gy_raw;
        self->distance_layer_->getGradient(mx, my, gx_raw, gy_raw);  // ✅ FIX: 移除Y轴转换

        // The gradient from distance field points in the direction of increasing distance
        // (away from obstacles). We want d(cost)/d(point) = w * 2 * (d-d_max) * d(d)/d(point)
        // d(d)/d(point) is the unit vector away from obstacle, scaled by resolution
        // ✅ FIX: EDF的梯度已经是正确方向，不需要flip Y轴
        Eigen::Vector2d obs_dir(gx_raw, gy_raw);
        double dir_len = obs_dir.norm();
        if (dir_len > 1e-6)
          obs_dir /= dir_len;

        Eigen::Vector2d g_obs = obs_dir * (self->w_obstacle_ * 2.0 * diff * resolution);
        grad(0, i) += g_obs(0);
        grad(1, i) += g_obs(1);
      }
    }
  }

  // ==================== 2. Smoothness cost ====================
  // Reference uses 5-point stencil: err = x_{i-2} - 4*x_{i-1} + 6*x_i - 4*x_{i+1} + x_{i+2}
  // cost_smooth = w_smo * err^T * err
  // grad for x_i: w_smo * 2 * (x_{i-2} - 4*x_{i-1} + 6*x_i - 4*x_{i+1} + x_{i+2})
  //
  // Loop range: i in [2, N-3] to have 5-point window [i-2, i-1, i, i+1, i+2]
  {
    double smooth_cost = 0.0;
    Eigen::Matrix2Xd smooth_grad = Eigen::Matrix2Xd::Zero(2, free_points);

    for (int pi = 2; pi < N - 2; ++pi)
    {
      Eigen::Vector2d xp2 = pts.col(pi - 2);
      Eigen::Vector2d xp  = pts.col(pi - 1);
      Eigen::Vector2d xc  = pts.col(pi);
      Eigen::Vector2d xn  = pts.col(pi + 1);
      Eigen::Vector2d xn2 = pts.col(pi + 2);

      Eigen::Vector2d err = xp + xn - 2.0 * xc;
      smooth_cost += self->w_smooth_ * err.squaredNorm();

      // The gradient for center point of the 5-point stencil
      // d/d(xc) of w*||xp2 - 4xp + 6xc - 4xn + xn2||^2
      // = w * 2 * (xp2 - 4xp + 6xc - 4xn + xn2)
      Eigen::Vector2d g_smooth = self->w_smooth_ * 2.0 * (xp2 - 4.0 * xp + 6.0 * xc - 4.0 * xn + xn2);

      int fi = pi - fixed_ends;  // free index
      if (fi >= 0 && fi < free_points)
      {
        smooth_grad(0, fi) += g_smooth(0);
        smooth_grad(1, fi) += g_smooth(1);
      }
    }

    cost += smooth_cost;
    grad += smooth_grad;
  }

  // ==================== 3. Curvature cost ====================
  // Reference: for each center point, compute curvatures from 4 segments around it,
  //   kappa_p (from segments i-2→i-1, i-1→i) and kappa_n (from segments i→i+1, i+1→i+2)
  //   plus kappa_c (from segments i-1→i, i→i+1).
  //   If any kappa > k_max, penalize with weight distribution [0.25, 0.5, 0.25].
  {
    double curv_cost = 0.0;
    Eigen::Matrix2Xd curv_grad = Eigen::Matrix2Xd::Zero(2, free_points);

    for (int pi = 2; pi < N - 2; ++pi)
    {
      Eigen::Vector2d xp2 = pts.col(pi - 2);
      Eigen::Vector2d xp  = pts.col(pi - 1);
      Eigen::Vector2d xc  = pts.col(pi);
      Eigen::Vector2d xn  = pts.col(pi + 1);
      Eigen::Vector2d xn2 = pts.col(pi + 2);

      // 4 segment vectors
      Eigen::Vector2d delta_xp = xp - xp2;   // i-2 → i-1
      Eigen::Vector2d delta_xc = xc - xp;    // i-1 → i
      Eigen::Vector2d delta_xn = xn - xc;    // i   → i+1
      Eigen::Vector2d delta_xn2 = xn2 - xn;  // i+1 → i+2

      double norm_xp = delta_xp.norm();
      double norm_xc = delta_xc.norm();
      double norm_xn = delta_xn.norm();
      double norm_xn2 = delta_xn2.norm();

      if (norm_xp < 1e-6 || norm_xc < 1e-6 || norm_xn < 1e-6 || norm_xn2 < 1e-6)
        continue;

      // Helper: orthogonal complement ort(a, b) = a - b * dot(a,b) / ||b||^2
      auto compute_ort = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) -> Eigen::Vector2d {
        double b_sq = b.squaredNorm();
        if (b_sq < 1e-12) return Eigen::Vector2d::Zero();
        return a - b * (a.dot(b) / b_sq);
      };

      // Helper: d(delta_phi)/d(cos_phi) = -1/sin(phi)
      auto compute_d_delta_phi = [](double delta_phi) -> double {
        double sin_phi = std::sin(delta_phi);
        if (std::abs(sin_phi) < 1e-8) return 0.0;
        return -1.0 / sin_phi;
      };

      // --- kappa_p: curvature at (i-1) from segments delta_xp, delta_xc ---
      double cos_phi_p = std::max(-1.0, std::min(1.0,
          delta_xp.dot(delta_xc) / (norm_xp * norm_xc)));
      double delta_phi_p = std::acos(cos_phi_p);
      double kappa_p = delta_phi_p / norm_xp;

      // --- kappa_c: curvature at (i) from segments delta_xc, delta_xn ---
      double cos_phi_c = std::max(-1.0, std::min(1.0,
          delta_xc.dot(delta_xn) / (norm_xc * norm_xn)));
      double delta_phi_c = std::acos(cos_phi_c);
      double kappa_c = delta_phi_c / norm_xc;

      // --- kappa_n: curvature at (i+1) from segments delta_xn, delta_xn2 ---
      double cos_phi_n = std::max(-1.0, std::min(1.0,
          delta_xn.dot(delta_xn2) / (norm_xn * norm_xn2)));
      double delta_phi_n = std::acos(cos_phi_n);
      double kappa_n = delta_phi_n / norm_xn;

      // Only penalize if any curvature exceeds k_max
      if (kappa_p <= self->k_max_ && kappa_c <= self->k_max_ && kappa_n <= self->k_max_)
        continue;

      // Cost contribution (only from kappa_c, matching reference)
      if (kappa_c > self->k_max_)
      {
        curv_cost += self->w_curvature_ * std::pow(kappa_c - self->k_max_, 2);
      }

      // --- Gradient computation (reference approach) ---
      // d(kappa)/d(x_c) for each of the 3 curvatures:
      //   d(kappa_p)/d(xc), d(kappa_c)/d(xc), d(kappa_n)/d(xc)

      // kappa_p gradient w.r.t. xc:
      //   kappa_p = delta_phi_p / ||delta_xp||
      //   delta_phi_p depends on cos(phi_p) = dot(delta_xp, delta_xc)/(||delta_xp||*||delta_xc||)
      //   xc appears in delta_xc = xc - xp
      //   d(cos_phi_p)/d(xc) = ort(delta_xp, delta_xc) / (||delta_xp|| * ||delta_xc||)
      Eigen::Vector2d k_p_grad = Eigen::Vector2d::Zero();
      if (kappa_p > self->k_max_)
      {
        double d_dphi_p = compute_d_delta_phi(delta_phi_p);
        Eigen::Vector2d d_cos_p = compute_ort(delta_xp, delta_xc) / (norm_xp * norm_xc);
        Eigen::Vector2d dk_p = (1.0 / norm_xp) * d_dphi_p * d_cos_p;
        k_p_grad = 2.0 * (kappa_p - self->k_max_) * dk_p;
      }

      // kappa_c gradient w.r.t. xc:
      //   xc appears in both delta_xc = xc - xp and delta_xn = xn - xc
      //   d(cos_phi_c)/d(xc) = ort(delta_xn, delta_xc*(-1)) / (||delta_xc||*||delta_xn||)
      //                       - ort(delta_xc, delta_xn*(-1)) / (||delta_xc||*||delta_xn||)
      //   Also kappa_c = delta_phi_c / ||delta_xc||, so d(1/||delta_xc||)/d(xc) contributes
      Eigen::Vector2d k_c_grad = Eigen::Vector2d::Zero();
      if (kappa_c > self->k_max_)
      {
        double d_dphi_c = compute_d_delta_phi(delta_phi_c);
        Eigen::Vector2d neg_xc = -delta_xc;
        Eigen::Vector2d neg_xn = -delta_xn;
        Eigen::Vector2d d_cos_c_part1 = compute_ort(delta_xn, neg_xc) / (norm_xc * norm_xn);  // from delta_xn side
        Eigen::Vector2d d_cos_c_part2 = compute_ort(delta_xc, neg_xn) / (norm_xc * norm_xn);  // from delta_xc side
        // xc is the endpoint of delta_xc and startpoint of delta_xn
        // d(delta_xc)/d(xc) = +I, d(delta_xn)/d(xc) = -I
        Eigen::Vector2d d_cos_c = d_cos_c_part1 - d_cos_c_part2;

        // d(kappa_c)/d(xc) = d(delta_phi_c / ||delta_xc||)/d(xc)
        //                   = 1/||delta_xc|| * d(delta_phi_c)/d(xc) + delta_phi_c * d(1/||delta_xc||)/d(xc)
        // d(1/||delta_xc||)/d(xc) = -delta_xc / ||delta_xc||^3 * d(delta_xc)/d(xc)
        //                         = -delta_xc / ||delta_xc||^3  (since d(delta_xc)/d(xc) = +1)
        // Wait, that's a scalar→vector issue. Let's be precise:
        // d(1/||delta_xc||)/d(xc) = delta_xc / ||delta_xc||^3  (points TOWARD xp, sign from delta_xc = xc-xp)
        Eigen::Vector2d dk_c = (1.0 / norm_xc) * d_dphi_c * d_cos_c
                               - delta_xc * (delta_phi_c / (norm_xc * norm_xc * norm_xc));
        k_c_grad = 2.0 * (kappa_c - self->k_max_) * dk_c;
      }

      // kappa_n gradient w.r.t. xc:
      //   xc appears in delta_xn = xn - xc
      //   d(cos_phi_n)/d(xc) = -ort(delta_xn2, delta_xn) / (||delta_xn||*||delta_xn2||)
      //   d(kappa_n)/d(xc) also has d(1/||delta_xn||)/d(xc) = delta_xn / ||delta_xn||^3
      Eigen::Vector2d k_n_grad = Eigen::Vector2d::Zero();
      if (kappa_n > self->k_max_)
      {
        double d_dphi_n = compute_d_delta_phi(delta_phi_n);
        Eigen::Vector2d d_cos_n = -compute_ort(delta_xn2, delta_xn) / (norm_xn * norm_xn2);
        Eigen::Vector2d dk_n = (1.0 / norm_xn) * d_dphi_n * d_cos_n
                               + delta_xn * (delta_phi_n / (norm_xn * norm_xn * norm_xn));
        k_n_grad = 2.0 * (kappa_n - self->k_max_) * dk_n;
      }

      // Weighted combination (matching reference: 0.25*k_p + 0.5*k_c + 0.25*k_n)
      Eigen::Vector2d total_curv_grad = self->w_curvature_ * (0.25 * k_p_grad + 0.5 * k_c_grad + 0.25 * k_n_grad);

      // NaN guard
      if (std::isnan(total_curv_grad(0)) || std::isnan(total_curv_grad(1)) ||
          std::isinf(total_curv_grad(0)) || std::isinf(total_curv_grad(1)))
      {
        total_curv_grad = Eigen::Vector2d::Zero();
      }

      int fi = pi - fixed_ends;
      if (fi >= 0 && fi < free_points)
      {
        curv_grad(0, fi) += total_curv_grad(0);
        curv_grad(1, fi) += total_curv_grad(1);
      }
    }

    cost += curv_cost;
    grad += curv_grad;
  }

  // ==================== Assemble output gradient ====================
  g.setZero();
  g.head(free_points) = grad.row(0).transpose();
  g.tail(free_points) = grad.row(1).transpose();

  return cost;
}

}  // namespace trajectory_optimization
}  // namespace rpp
