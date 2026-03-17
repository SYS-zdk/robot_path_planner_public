/**
 * @file: social_layer.cpp
 * @brief: Implementation of SocialLayer
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include "social_layer/social_layer.h"

#include <algorithm>
#include <cmath>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_list_macros.h>

namespace social_layer
{

SocialLayer::SocialLayer()
  : enabled_(true)
  , ellipse_major_(2.5)
  , ellipse_minor_(1.0)
  , circle_radius_(0.8)
  , cutoff_distance_(3.0)
  , max_cost_(costmap_2d::LETHAL_OBSTACLE)
  , publish_debug_(true)
{
}

void SocialLayer::onInitialize()
{
  nh_ = ros::NodeHandle("~" + getName());

  nh_.param("enabled", enabled_, true);
  nh_.param("ellipse_major", ellipse_major_, 2.5);
  nh_.param("ellipse_minor", ellipse_minor_, 1.0);
  nh_.param("circle_radius", circle_radius_, 0.8);
  nh_.param("cutoff_distance", cutoff_distance_, 3.0);
  nh_.param("publish_debug", publish_debug_, true);
  nh_.param("debug_topic", debug_topic_, std::string("social_layer/debug_costmap"));

  if (publish_debug_)
  {
    debug_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(debug_topic_, 1, true);
  }

  ros::NodeHandle g_nh;
  ped_sub_ = g_nh.subscribe("/ped_visualization", 1, &SocialLayer::pedestriansCallback, this);

  current_ = true;
}

void SocialLayer::pedestriansCallback(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  persons_.clear();
  persons_.reserve(msg->tracks.size());

  for (const auto& t : msg->tracks)
  {
    PersonState p;
    p.x = t.pose.pose.position.x;
    p.y = t.pose.pose.position.y;
    p.vx = t.twist.twist.linear.x;
    p.vy = t.twist.twist.linear.y;
    persons_.push_back(p);
  }
}

void SocialLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                               double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  boost::mutex::scoped_lock lock(mutex_);

  for (const auto& p : persons_)
  {
    double r = cutoff_distance_;
    *min_x = std::min(*min_x, p.x - r);
    *min_y = std::min(*min_y, p.y - r);
    *max_x = std::max(*max_x, p.x + r);
    *max_y = std::max(*max_y, p.y + r);
  }
}

void SocialLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                              int max_i, int max_j)
{
  if (!enabled_)
    return;

  boost::mutex::scoped_lock lock(mutex_);

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      double cost = computeSocialCost(wx, wy);
      if (cost <= 0.0)
        continue;

      unsigned char old_cost = master_grid.getCost(i, j);
      unsigned char new_cost = static_cast<unsigned char>(std::min<double>(max_cost_, cost));
      if (new_cost > old_cost)
      {
        master_grid.setCost(i, j, new_cost);
      }
    }
  }

  if (publish_debug_ && debug_pub_)
  {
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = layered_costmap_->getGlobalFrameID();

    unsigned int size_x = master_grid.getSizeInCellsX();
    unsigned int size_y = master_grid.getSizeInCellsY();
    grid.info.resolution = master_grid.getResolution();
    grid.info.width = size_x;
    grid.info.height = size_y;

    double origin_x, origin_y;
    master_grid.mapToWorld(0, 0, origin_x, origin_y);
    grid.info.origin.position.x = origin_x - 0.5 * grid.info.resolution;
    grid.info.origin.position.y = origin_y - 0.5 * grid.info.resolution;
    grid.info.origin.position.z = 0.0;

    grid.data.resize(size_x * size_y);

    for (unsigned int j = 0; j < size_y; ++j)
    {
      for (unsigned int i = 0; i < size_x; ++i)
      {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        double c = computeSocialCost(wx, wy);
        int8_t val = 0;
        if (c > 0.0)
        {
          double norm = c / static_cast<double>(max_cost_);
          norm = std::max(0.0, std::min(1.0, norm));
          val = static_cast<int8_t>(std::round(norm * 100.0));
        }
        grid.data[j * size_x + i] = val;
      }
    }

    debug_pub_.publish(grid);
  }
}

double SocialLayer::computeSocialCost(double wx, double wy) const
{
  double max_c = 0.0;

  for (const auto& p : persons_)
  {
    double dx = wx - p.x;
    double dy = wy - p.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist > cutoff_distance_)
      continue;

    double speed = std::sqrt(p.vx * p.vx + p.vy * p.vy);
    double theta_h = (speed > 1e-3) ? std::atan2(p.vy, p.vx) : 0.0;

    // Rotate into pedestrian local frame (x axis along heading)
    double rel_x = dx * std::cos(theta_h) + dy * std::sin(theta_h);
    double rel_y = -dx * std::sin(theta_h) + dy * std::cos(theta_h);

    double c = 0.0;
    if (rel_x >= 0.0)
    {
      // Front elliptical comfortable region
      double e = (rel_x * rel_x) / (ellipse_major_ * ellipse_major_) +
                 (rel_y * rel_y) / (ellipse_minor_ * ellipse_minor_);
      if (e <= 1.0)
      {
        c = (1.0 - e) * static_cast<double>(max_cost_);
      }
    }
    else
    {
      // Back circular region
      double d = std::sqrt(rel_x * rel_x + rel_y * rel_y);
      if (d <= circle_radius_)
      {
        c = (1.0 - d / circle_radius_) * static_cast<double>(max_cost_);
      }
    }

    if (c > max_c)
      max_c = c;
  }

  return max_c;
}

unsigned char SocialLayer::getSocialCostAt(double wx, double wy) const
{
  boost::mutex::scoped_lock lock(mutex_);
  double cost = computeSocialCost(wx, wy);
  return static_cast<unsigned char>(std::min<double>(max_cost_, cost));
}

}  // namespace social_layer

PLUGINLIB_EXPORT_CLASS(social_layer::SocialLayer, costmap_2d::Layer)
