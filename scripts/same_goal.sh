#!/bin/bash
# 启动第一个终端并检测错误
gnome-terminal -- bash -c "cd /home/zhangdingkun/robot_path_planner; source ./devel/setup.bash; roslaunch send_goals launch_randomizer.launch"

# 两个roslaunch之间需要间隔一段时间
sleep 12s
gnome-terminal -- bash -c "cd /home/zhangdingkun/robot_path_planner; ./src/send_goals/current_pose_publisher.py"

# 启动第二个终端并检测错误
gnome-terminal -- bash -c "cd /home/zhangdingkun/robot_path_planner/bag_record; rosbag record /current_pose /cmd_vel /move_base/PathPlanner/initial_plan /move_base/PathPlanner/initial_plan2 /move_base/PathPlanner/plan_time /move_base/PathPlanner/plan_opt"

# 启动第三个终端并检测错误
gnome-terminal -- bash -c "cd /home/zhangdingkun/robot_path_planner; source ./devel/setup.bash; roslaunch send_goals send_goals.launch"
