/**
 * @file: social_layer_global_debug_node.cpp
 * @brief: Debug node for visualizing social-layer cost on the global costmap
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pedsim_msgs/TrackedPersons.h>

#include <cmath>
#include <vector>

struct GlobalPersonState
{
  double x;
  double y;
  double vx;
  double vy;
};

class SocialGlobalDebugNode
{
public:
  SocialGlobalDebugNode()
  {
    ros::NodeHandle private_nh("~");

    // 和 social_layer 插件保持一致的参数意义
    private_nh.param("ellipse_major", ellipse_major_, 2.5);
    private_nh.param("ellipse_minor", ellipse_minor_, 1.0);
    private_nh.param("circle_radius", circle_radius_, 0.8);
    private_nh.param("cutoff_distance", cutoff_distance_, 3.0);

    // 专门订阅 global costmap（不再通过参数切换）
    costmap_topic_ = "/move_base/global_costmap/costmap";

    map_sub_ = nh_.subscribe(costmap_topic_, 1,
                 &SocialGlobalDebugNode::mapCallback, this);
    ped_sub_ = nh_.subscribe("/ped_visualization", 1,
                             &SocialGlobalDebugNode::pedsCallback, this);

    // 单独的 debug 话题，避免和 local 版混淆
    debug_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/social_layer/global_debug_costmap", 1, true);
  }

private:
  void pedsCallback(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
  {
    persons_.clear();
    persons_.reserve(msg->tracks.size());

    for (const auto& t : msg->tracks)
    {
      GlobalPersonState p;
      p.x = t.pose.pose.position.x;
      p.y = t.pose.pose.position.y;
      p.vx = t.twist.twist.linear.x;
      p.vy = t.twist.twist.linear.y;
      persons_.push_back(p);
    }

    // 若有缓存的全局 costmap，则在行人更新时立即重算发布（global costmap 更新频率低时也能动起来）
    if (have_map_)
    {
      publishFromMap(last_map_);
    }
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    // 缓存最新的全局 costmap
    last_map_ = *msg;
    have_map_ = true;

    publishFromMap(last_map_);
  }

  void publishFromMap(const nav_msgs::OccupancyGrid& base_map)
  {
    // 没有人时仍发布一张全 0 的地图，保证 RViz 有框架
    if (persons_.empty())
    {
      nav_msgs::OccupancyGrid grid = base_map;
      std::fill(grid.data.begin(), grid.data.end(), 0);
      grid.header.stamp = ros::Time::now();
      debug_pub_.publish(grid);
      return;
    }

    nav_msgs::OccupancyGrid grid;
    grid.header = base_map.header;
    grid.info = base_map.info;
    grid.data.resize(grid.info.width * grid.info.height);

    for (unsigned int j = 0; j < grid.info.height; ++j)
    {
      for (unsigned int i = 0; i < grid.info.width; ++i)
      {
        double wx = grid.info.origin.position.x + (i + 0.5) * grid.info.resolution;
        double wy = grid.info.origin.position.y + (j + 0.5) * grid.info.resolution;

        double c = computeSocialCost(wx, wy);
        int8_t val = 0;
        if (c > 0.0)
        {
          if (c > 1.0) c = 1.0;
          if (c < 0.0) c = 0.0;
          val = static_cast<int8_t>(std::round(c * 100.0));
        }
        grid.data[j * grid.info.width + i] = val;
      }
    }

    grid.header.stamp = ros::Time::now();
    debug_pub_.publish(grid);
  }

  double computeSocialCost(double wx, double wy) const
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

      // 旋转到行人局部坐标系：x 轴沿行走方向
      double rel_x = dx * std::cos(theta_h) + dy * std::sin(theta_h);
      double rel_y = -dx * std::sin(theta_h) + dy * std::cos(theta_h);

      double c = 0.0;
      if (rel_x >= 0.0)
      {
        // 前方椭圆舒适区
        double e = (rel_x * rel_x) / (ellipse_major_ * ellipse_major_) +
                   (rel_y * rel_y) / (ellipse_minor_ * ellipse_minor_);
        if (e <= 1.0)
        {
          c = 1.0 - e;  // 0~1
        }
      }
      else
      {
        // 后方圆形区
        double d = std::sqrt(rel_x * rel_x + rel_y * rel_y);
        if (d <= circle_radius_)
        {
          c = 1.0 - d / circle_radius_;
        }
      }

      if (c > max_c)
        max_c = c;
    }

    return max_c;
  }

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber ped_sub_;
  ros::Publisher debug_pub_;

  std::string costmap_topic_;

  std::vector<GlobalPersonState> persons_;

  // 缓存最新的全局 costmap
  nav_msgs::OccupancyGrid last_map_;
  bool have_map_ = false;

  double ellipse_major_;
  double ellipse_minor_;
  double circle_radius_;
  double cutoff_distance_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "social_layer_global_debug_node");
  SocialGlobalDebugNode node;
  ros::spin();
  return 0;
}
