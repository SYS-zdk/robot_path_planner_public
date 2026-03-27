/**
 * *********************************************************
 *
 * @file: visualizer.cpp
 * @brief: Contains visualization functions
 * @author: REMOVED
 * @date: 2024-09-24
 * @version: 1.0
 *
 * Copyright (c) 2024, REMOVED.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/util/visualizer.h"

namespace rpp
{
namespace common
{
namespace util
{
std_msgs::ColorRGBA Visualizer::RED = Visualizer::_colorInit(1.0, 0.0, 0.0, 1.0);
std_msgs::ColorRGBA Visualizer::DARK_GREEN = Visualizer::_colorInit(0.43, 0.54, 0.24, 0.5);
std_msgs::ColorRGBA Visualizer::PURPLE = Visualizer::_colorInit(1.0, 0.0, 1.0, 1.0);
std_msgs::ColorRGBA Visualizer::BLUE = Visualizer::_colorInit(1, 0.6078, 0.6078, 1.0);//自己加的

/**
 * @brief publish lines
 */
void Visualizer::publishLines2d(const Lines2d& lines, const ros::Publisher& publisher, const std::string& frame_id,
                                const std::string& ns, std_msgs::ColorRGBA color, double scale)
{
  int cnt = 0;
  visualization_msgs::MarkerArray line_array;
  for (const auto& line : lines)
  {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = ns + "_" + "line_marker";
    line_marker.id = cnt;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start, end;
    start.x = line.first.x();
    start.y = line.first.y();
    end.x = line.second.x();
    end.y = line.second.y();
    line_marker.points.push_back(start);
    line_marker.points.push_back(end);
    line_marker.lifetime = ros::Duration(0.5);
    line_marker.scale.x = scale;
    line_marker.scale.y = scale;
    line_marker.scale.z = scale;
    line_marker.color.r = color.r;
    line_marker.color.g = color.g;
    line_marker.color.b = color.b;
    line_marker.color.a = color.a;
    cnt++;
    line_array.markers.push_back(line_marker);
  }
  publisher.publish(line_array);
  line_array.markers.clear();
}

void Visualizer::publishVariableRadiusDisks2d(const Points3d& centers_with_radius_m, const ros::Publisher& publisher,
                                              const std::string& frame_id, const std::string& ns,
                                              std_msgs::ColorRGBA centers_color, std_msgs::ColorRGBA disks_color,
                                              double centers_scale, double disks_height, int max_disks, bool delete_all,
                                              bool disks_color_gradient, double radius_min_m, double radius_max_m)
{
  visualization_msgs::MarkerArray arr;

  if (delete_all)
  {
    visualization_msgs::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp = ros::Time::now();
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(std::move(del));
  }

  if (centers_with_radius_m.empty())
  {
    publisher.publish(arr);
    return;
  }

  max_disks = std::max(1, max_disks);
  const int stride = std::max(1, static_cast<int>((centers_with_radius_m.size() + max_disks - 1) / max_disks));

  // auto radius range if requested (or invalid)
  if (disks_color_gradient)
  {
    if (!(radius_max_m > radius_min_m))
    {
      radius_min_m = std::numeric_limits<double>::infinity();
      radius_max_m = 0.0;
      for (const auto& pt : centers_with_radius_m)
      {
        const double r = std::max(0.0, pt.theta());
        radius_min_m = std::min(radius_min_m, r);
        radius_max_m = std::max(radius_max_m, r);
      }
      if (!std::isfinite(radius_min_m))
        radius_min_m = 0.0;
      if (!(radius_max_m > radius_min_m))
      {
        // avoid divide-by-zero
        radius_max_m = radius_min_m + 1e-3;
      }
    }
  }

  auto color_green_yellow_red = [&](double t01) {
    // t01: 0 -> green, 0.5 -> yellow, 1 -> red
    t01 = std::max(0.0, std::min(1.0, t01));
    std_msgs::ColorRGBA c;
    c.a = disks_color.a;
    if (t01 <= 0.5)
    {
      const double u = t01 / 0.5;
      c.r = static_cast<float>(u);
      c.g = 1.0f;
      c.b = 0.0f;
    }
    else
    {
      const double u = (t01 - 0.5) / 0.5;
      c.r = 1.0f;
      c.g = static_cast<float>(1.0 - u);
      c.b = 0.0f;
    }
    return c;
  };

  // centers (single POINTS marker)
  visualization_msgs::Marker centers;
  centers.header.frame_id = frame_id;
  centers.header.stamp = ros::Time::now();
  centers.ns = ns + "_centers";
  centers.id = 0;
  centers.type = visualization_msgs::Marker::POINTS;
  centers.action = visualization_msgs::Marker::ADD;
  centers.pose.orientation.w = 1.0;
  centers.scale.x = centers_scale;
  centers.scale.y = centers_scale;
  centers.color = centers_color;

  int disk_id = 0;
  for (size_t i = 0; i < centers_with_radius_m.size(); i += stride)
  {
    const auto& pt = centers_with_radius_m[i];
    const double r_m = std::max(0.0, pt.theta());

    geometry_msgs::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = 0.0;
    centers.points.push_back(p);

    visualization_msgs::Marker disk;
    disk.header.frame_id = frame_id;
    disk.header.stamp = ros::Time::now();
    disk.ns = ns + "_disks";
    disk.id = disk_id++;
    disk.type = visualization_msgs::Marker::CYLINDER;
    disk.action = visualization_msgs::Marker::ADD;
    disk.pose.position.x = pt.x();
    disk.pose.position.y = pt.y();
    disk.pose.position.z = 0.0;
    disk.pose.orientation.w = 1.0;
    disk.scale.x = std::max(0.02, 2.0 * r_m);
    disk.scale.y = std::max(0.02, 2.0 * r_m);
    disk.scale.z = std::max(0.001, disks_height);
    if (disks_color_gradient)
    {
      const double t = (r_m - radius_min_m) / (radius_max_m - radius_min_m);
      // small radius -> red, large radius -> green (so invert then use green->yellow->red)
      disk.color = color_green_yellow_red(1.0 - t);
    }
    else
    {
      disk.color = disks_color;
    }
    disk.lifetime = ros::Duration(0.0);
    arr.markers.push_back(std::move(disk));
  }

  arr.markers.push_back(std::move(centers));
  publisher.publish(arr);
}

std_msgs::ColorRGBA Visualizer::_colorInit(double r, double g, double b, double a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace util
}  // namespace common
}  // namespace rpp