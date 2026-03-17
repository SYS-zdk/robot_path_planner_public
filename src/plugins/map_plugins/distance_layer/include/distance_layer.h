
/**
 * *********************************************************
 *
 * @file: distance_layer.h
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
#ifndef DISTANCE_LAYER_H
#define DISTANCE_LAYER_H

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "costmap_2d/layer.h"
#include "costmap_2d/GenericPluginConfig.h"
#include "dynamic_reconfigure/server.h"

#include "euclidean_distance_field.h"

namespace costmap_2d
{
class DistanceLayer : public Layer
{
public:
  DistanceLayer() = default;
  virtual ~DistanceLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  const std::vector<std::vector<double>>& getEDF() const;
  double getDistance(double x, double y);
  void getGradient(double x, double y, double& gx, double& gy);
  boost::mutex& getMutex();

private:
  void publishDistanceMap(const costmap_2d::Costmap2D& master_grid);

  void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level);

private:
  boost::mutex mutex_;
  ros::Publisher distance_grid_pub_;
  std::vector<std::vector<double>> edf_;
  std::unique_ptr<euclidean_distance_field::EuclideanDistanceField> edf_manager_;
  std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_ = nullptr;
};

}  // namespace costmap_2d

#endif