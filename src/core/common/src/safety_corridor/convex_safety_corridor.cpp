
/**
 * *********************************************************
 *
 * @file: convex_safety_corridor.cpp
 * @brief: convex safety corridor class
 * @author: Yang Haodong
 * @date: 2025-01-12
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/math/math_helper.h"
#include "common/geometry/line2d.h"
#include "common/safety_corridor/convex_safety_corridor.h"
#include "common/safety_corridor/ackermann_config.h"

namespace rpp
{
namespace common
{
namespace safety_corridor
{
using Line2d = rpp::common::geometry::Line2d;

/**
 * @brief Empty constructor.
 */
ConvexSafetyCorridor::ConvexSafetyCorridor() : SafetyCorridor(){};

/**
 * @brief Constructor which takes a vector of obstacle points as its barrier.
 * @param obstacle_pts The obstacle points to construct the safety corridor.
 * @param safety_range The safety range to limit polygon space [m]
 */
ConvexSafetyCorridor::ConvexSafetyCorridor(const Points3d& obstacle_pts, double safety_range)
  : SafetyCorridor(obstacle_pts), safety_range_(safety_range)
{
}

ConvexSafetyCorridor::ConvexSafetyCorridor(const Points3d& obstacle_pts, double safety_range,
                                           const rpp::AckermannConfig& ackermann_cfg)
  : SafetyCorridor(obstacle_pts), ackermann_config_(ackermann_cfg), safety_range_(safety_range)
{
}
/**
 * @brief Constructor which takes a vector of obstacle points as its barrier.
 * @param costmap_ros costmap ROS wrapper
 * @param safety_range The safety range to limit polygon space [m]
 */
ConvexSafetyCorridor::ConvexSafetyCorridor(costmap_2d::Costmap2DROS* costmap_ros, double safety_range, const rpp::AckermannConfig& ackermann_cfg)
  : SafetyCorridor(costmap_ros), ackermann_config_(ackermann_cfg), safety_range_(safety_range)
{
}
/**
 * @brief Construct a safety corridor composed of polygons based on waypoints and environmental obstacles。
 * @param waypoints The waypoints
 * @param decomp_polygons The decomposed polygons to construct the safety corridor.
 * @return true if successful construction.
 */
bool ConvexSafetyCorridor::decompose(const Points3d& waypoints, std::vector<Polygon2d>& decomp_polygons) const
{
  decomp_polygons.clear();
  int nums = static_cast<int>(waypoints.size());
  for (int i = 0; i < nums - 1; i++)
  {
    // calculate waypoints segment
    const Vec2d pf(waypoints[i].x(), waypoints[i].y());
    const Vec2d pr(waypoints[i + 1].x(), waypoints[i + 1].y());
    // construct initial polygon
    auto init_polygon = _initPolygon(pf, pr);
    // filter obstacle points
    Point3d query(0.5 * (pf.x() + pr.x()), 0.5 * (pf.y() + pr.y()));
    double query_radius = std::hypot(0.5 * std::hypot(pr.x() - pf.x(), pr.y() - pf.y()) + safety_range_, safety_range_);
    // double path_length = (pr - pf).length();
    // double query_radius = path_length + 2.0 * safety_range_;
    auto local_obs_idx = obs_kd_tree_.radiusSearch(query, query_radius);
    std::vector<Vec2d> local_obs_points;
    for (auto idx : local_obs_idx)
    {
      const Vec2d local_obstacle(obs_kd_tree_[idx].x(), obs_kd_tree_[idx].y());
      if (init_polygon.isPointIn(local_obstacle))
      {
        local_obs_points.emplace_back(local_obstacle.x(), local_obstacle.y());
      }
    }

    // construct ellipse
    auto ellipse = _buildEllipse(pf, pr, local_obs_points);

    // construct polygon
    auto polygon = _buildPolygon(ellipse, init_polygon, local_obs_points);

    // save
    decomp_polygons.push_back(std::move(polygon));
  }
  return true;
}

/**
 * @brief Polygon initialization.
 * @param pf waypoint front
 * @param pr waypoint rear
 * @return initial polygon
 */
ConvexSafetyCorridor::Polygon2d ConvexSafetyCorridor::_initPolygon(const Vec2d& pf, const Vec2d& pr) const
{
  // Directional vector (pf to pr)
  Vec2d v_dir = pr - pf;
  v_dir.normalize();
  // Normal vector
  Vec2d v_nor(v_dir.y(), -v_dir.x());

  // 计算急刹车距离
  const double v_max = 1.0;  // 最大速度 (m/s) 与minimum-snap保持一致，由于此处没有构建snap对象，故不能调用特定的速度、加速度属性，得自己重新定义
  const double a_max = 2.0;  // 最大减速度 (m/s^2)
  const double d_brake = v_max * v_max / (2 * a_max);  // 急刹车距离 (m) 计算得到0.25
  const double R_min = ackermann_config_.minTurnRadius();
  const double dynamic_safety = std::max(safety_range_,R_min * 0.5);
  const double break_safety = std::max(safety_range_,d_brake ); 

  // Parallel range
  auto p_1 = pf + dynamic_safety * v_nor;
  auto p_2 = pf - dynamic_safety * v_nor;
  // Vertical range
  auto p_3 = pr + break_safety * v_dir;
  auto p_4 = pf - break_safety * v_dir;

  auto vertical_1 = p_1 + p_3 - pf;
  auto vertical_2 = p_1 + p_4 - pf;
  auto vertical_3 = p_2 + p_4 - pf;
  auto vertical_4 = p_2 + p_3 - pf;

  Polygon2d init_polygon({ vertical_1, vertical_2, vertical_3, vertical_4 });
  return init_polygon;
}

/**
 * @brief Build ellipse of corridor.
 * @param pf waypoint front
 * @param pr waypoint rear
 * @param obs_points obstacles that inside the initial polygon
 * @return maximum ellipse
 */
ConvexSafetyCorridor::Ellipse2d ConvexSafetyCorridor::_buildEllipse(const Vec2d& pf, const Vec2d& pr,
                                                                    const std::vector<Vec2d>& obs_points) const
{
  // ball initialization
  auto pf_to_pr = pr - pf;
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  const double min_car_long = ackermann_config_.wheelbase * 1.2;
  A(0, 0) = std::max(min_car_long, 0.5 * pf_to_pr.length()); //将矩阵 A 的第 (0, 0) 位置的元素设置为路径点 pf 和 pr 之间距离的一半
  A(1, 1) = A(0, 0);

  const double min_ellipse_minor_axis = ackermann_config_.track_width * 1.2;
  A(1,1) = std::max(A(1,1),min_ellipse_minor_axis);

  // rotation matrix
  double yaw = pf_to_pr.angle();
  Eigen::Rotation2D<double> rotation(yaw);
  auto R = rotation.matrix();
  Eigen::Vector2d mid(0.5 * (pr.x() + pf.x()), 0.5 * (pr.y() + pf.y()));
  auto ellipse = Ellipse2d(R * A * R.transpose(), mid);
  std::vector<Vec2d> inside_obs_points;
  ellipse.insidePoints(obs_points, inside_obs_points);
  while (!inside_obs_points.empty())
  {
    // get the closest obstacle
    auto closest_obs_point = R.transpose() * (ellipse.closestPoint(inside_obs_points) - ellipse.d());
    if (rpp::common::math::less(closest_obs_point[0], A(0, 0)))
    {
      A(1, 1) = std::fabs(closest_obs_point[1]) / std::sqrt(1 - std::pow(closest_obs_point[0] / A(0, 0), 2));
    }
    // update ellipse
    ellipse.setC(R * A * R.transpose());
    auto cur_obs_points = inside_obs_points;
    ellipse.insidePoints(cur_obs_points, inside_obs_points, false);
  }
  return ellipse;
}

/**
 * @brief Build polygon of corridor.
 * @param ellipse maximum ellipse
 * @param init_polygon initial polygon
 * @param obs_points obstacles that inside the initial polygon
 * @return maximum polygon
 */
ConvexSafetyCorridor::Polygon2d ConvexSafetyCorridor::_buildPolygon(const Ellipse2d& ellipse,
                                                                    const Polygon2d& init_polygon,
                                                                    const std::vector<Vec2d>& obs_points) const
{
  // initial hyper planes
  std::vector<Line2d> polygon_planes;
  for (int i = 0; i < init_polygon.num_points(); i++)
  {
    const auto& pt = init_polygon.points()[i];
    const auto& next_pt = init_polygon.points()[init_polygon.next(i)];
    Vec2d nor(next_pt.y() - pt.y(), -(next_pt.x() - pt.x()));
    nor.normalize();
    polygon_planes.emplace_back(nor, pt);
  }
  // ramaining obstacles inside the polygon
  auto remain_obs = obs_points;

  while (!remain_obs.empty())
  {
    // get the closest obstacle
    auto closest_point = ellipse.closestPoint(remain_obs);
    // Calculate the normal vector of the tangent plane to the closest obstacle on the ellipse
    auto C_inv = ellipse.C().inverse();
    Eigen::Vector2d norm_vector = C_inv * C_inv * (closest_point - ellipse.d());
    norm_vector.normalize();
    // build the hyper plane
    auto hyper_plane = Line2d({ norm_vector.x(), norm_vector.y() }, { closest_point.x(), closest_point.y() });
    // save
    polygon_planes.emplace_back(hyper_plane);
    // remove the obstacles outside the polygon
    remain_obs.erase(std::remove_if(remain_obs.begin(), remain_obs.end(),
                                    [&](const Vec2d& p) {
                                      return rpp::common::math::less(hyper_plane.distTo(p), 0.0) ? false : true;
                                    }),
                     remain_obs.end());
  }

  std::vector<Vec2d> vertices;
  Line2d::getVertexOfFeasibleDomain(polygon_planes, vertices);
  Polygon2d new_polygon(vertices);
  return new_polygon;
}

}  // namespace safety_corridor
}  // namespace common
}  // namespace rpp