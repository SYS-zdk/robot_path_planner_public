
/**
 * *********************************************************
 *
 * @file: distance_layer.cpp
 * @brief: euclidean distance layer plugin for costmap
 * @author: Yang Haodong
 * @date: 2024-06-28
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "nav_msgs/OccupancyGrid.h"
#include "pluginlib/class_list_macros.h"

#include "distance_layer.h"

PLUGINLIB_EXPORT_CLASS(costmap_2d::DistanceLayer, costmap_2d::Layer)

namespace costmap_2d
{
void DistanceLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  edf_manager_ = std::make_unique<euclidean_distance_field::EuclideanDistanceField>();
  distance_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("distance_field", 1);
  dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
      boost::bind(&DistanceLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

const std::vector<std::vector<double>>& DistanceLayer::getEDF() const
{
  return edf_;
}

double DistanceLayer::getDistance(double x, double y)
{
  return edf_manager_->getDistance(x, y);
}

void DistanceLayer::getGradient(double x, double y, double& gx, double& gy)
{
  edf_manager_->getGradient(x, y, gx, gy);
}

boost::mutex& DistanceLayer::getMutex()
{
  return mutex_;
}

void DistanceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                 double* max_x, double* max_y)
{
  if (!enabled_)
  {
    return;
  }
}

void DistanceLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }

  boost::unique_lock<boost::mutex> lock(mutex_);
  edf_manager_->setGridMap(master_grid.getCharMap(), master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY());
  edf_manager_->compute2d();
  edf_ = edf_manager_->getEDF();
  publishDistanceMap(master_grid);
}

void DistanceLayer::publishDistanceMap(const costmap_2d::Costmap2D& master_grid)
{
  unsigned int nx = master_grid.getSizeInCellsX();
  unsigned int ny = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();
  nav_msgs::OccupancyGrid grid;

  grid.header.frame_id = "map";
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution;
  grid.info.width = nx;
  grid.info.height = ny;
  grid.info.origin.position.x = master_grid.getOriginX();
  grid.info.origin.position.y = master_grid.getOriginY();
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(nx * ny);

  double max_dist = -std::numeric_limits<double>::max();
  for (unsigned int y = 0; y < ny; y++)
    max_dist = std::max(max_dist, *std::max_element(edf_[y].begin(), edf_[y].end()));

  for (unsigned int x = 0; x < nx; x++)
    for (unsigned int y = 0; y < ny; y++)
      grid.data[x + y * nx] = static_cast<unsigned int>(-100.0f / max_dist * edf_[y][x] + 100.0f);
  distance_grid_pub_.publish(grid);
}

void DistanceLayer::reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

}  // namespace costmap_2d