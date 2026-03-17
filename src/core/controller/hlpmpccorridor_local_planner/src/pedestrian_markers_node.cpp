/**
 * @file pedestrian_markers_node.cpp
 * @brief Small visualization node that publishes pedestrian markers from TrackedPersons.
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <ros/ros.h>

#include <pedsim_msgs/TrackedPersons.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pedestrian_markers_node");
  ros::NodeHandle nh;

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/pedestrian_markers", 1);

  auto cb = [&](const pedsim_msgs::TrackedPersons::ConstPtr& msg)
  {
    visualization_msgs::MarkerArray ma;
    ros::Time now = ros::Time::now();

    int id = 0;
    for (const auto& person : msg->tracks)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = msg->header.frame_id;
      m.header.stamp = now;
      m.ns = "pedestrians";
      m.id = id++;
      m.type = visualization_msgs::Marker::CYLINDER;
      m.action = visualization_msgs::Marker::ADD;
      m.pose = person.pose.pose;
      m.scale.x = 0.5;
      m.scale.y = 0.5;
      m.scale.z = 1.0;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 0.6;
      m.lifetime = ros::Duration(0.2);
      ma.markers.push_back(m);

      visualization_msgs::Marker v;
      v.header.frame_id = msg->header.frame_id;
      v.header.stamp = now;
      v.ns = "pedestrians_velocity";
      v.id = id++;
      v.type = visualization_msgs::Marker::ARROW;
      v.action = visualization_msgs::Marker::ADD;
      v.pose.orientation.w = 1.0;  // identity

      geometry_msgs::Point p0, p1;
      // 起点：行人当前位置（稍微抬高一点避免和地面重合）
      p0 = person.pose.pose.position;
      p0.z += 0.5;

      // 终点：当前位置 + 缩放后的速度向量
      const double scale = 0.8;  // 速度箭头长度缩放
      p1 = p0;
      p1.x += person.twist.twist.linear.x * scale;
      p1.y += person.twist.twist.linear.y * scale;

      v.points.push_back(p0);
      v.points.push_back(p1);
      v.scale.x = 0.05;
      v.scale.y = 0.1;
      v.scale.z = 0.1;
      v.color.r = 0.0;
      v.color.g = 0.0;
      v.color.b = 1.0;
      v.color.a = 0.8;
      v.lifetime = ros::Duration(0.2);
      ma.markers.push_back(v);
    }

    marker_pub.publish(ma);
  };

  ros::Subscriber sub = nh.subscribe<pedsim_msgs::TrackedPersons>("/ped_visualization", 1, cb);

  ros::spin();
  return 0;
}
