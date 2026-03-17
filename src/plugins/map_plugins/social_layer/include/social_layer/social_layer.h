































































































/**
 * @file: social_layer.h
 * @brief: Costmap layer plugin that adds social-navigation costs around tracked persons
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef SOCIAL_LAYER_SOCIAL_LAYER_H_
#define SOCIAL_LAYER_SOCIAL_LAYER_H_

#include <vector>

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/obstacle_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <boost/thread/mutex.hpp>

namespace social_layer
{

struct PersonState
{
  double x;
  double y;
  double vx;
  double vy;
};

class SocialLayer : public costmap_2d::Layer
{
public:
  SocialLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                           int max_i, int max_j);

  virtual void reset() { }

  // Public interface for querying social cost at a world coordinate
  // Used by SocialCostFunction in HybridPlanner for trajectory evaluation
  unsigned char getSocialCostAt(double wx, double wy) const;

private:
  void pedestriansCallback(const pedsim_msgs::TrackedPersons::ConstPtr& msg);

  double computeSocialCost(double wx, double wy) const;

  ros::NodeHandle nh_;
  ros::Subscriber ped_sub_;
  ros::Publisher debug_pub_;

  mutable boost::mutex mutex_;
  std::vector<PersonState> persons_;

  bool enabled_;
  double ellipse_major_;
  double ellipse_minor_;
  double circle_radius_;
  double cutoff_distance_;
  unsigned char max_cost_;
  bool publish_debug_;
  std::string debug_topic_;
};

}  // namespace social_layer

#endif  // SOCIAL_LAYER_SOCIAL_LAYER_H_
