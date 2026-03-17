#!/bin/bash

# 定义你的sh文件路径
SH_FILE="/home/zhangdingkun/robot_path_planner/scripts/same_goal.sh"

# 定义循环次数
MAX_ITERATIONS=150
COUNT=0

# 检查脚本文件是否存在
if [ ! -f "$SH_FILE" ]; then
    echo "Error: Script file $SH_FILE does not exist."
    exit 1
fi

echo "Starting loop with $MAX_ITERATIONS iterations..."

while [ $COUNT -lt $MAX_ITERATIONS ]; do
    echo "Starting same_goal.sh (Iteration $((COUNT + 1)) of $MAX_ITERATIONS)..."
    bash "$(dirname "$0")/clear_tf_cache.sh"

    # 执行same_goal.sh脚本
    gnome-terminal -- bash -c "bash $SH_FILE; exit"
    
    echo "Waiting for 2 minutes..."
    sleep 200s #30 120 60
    
    echo "Closing all related terminals..."
    # 关闭所有由same_goal.sh启动的终端
    pkill -SIGINT -f "bash $SH_FILE"
    pkill -SIGINT -f "/home/zhangdingkun/robot_path_planner/bag_record"
    pkill -SIGINT -f "/home/zhangdingkun/robot_path_planner/send_goals"
    pkill -SIGINT -f "roslaunch send_goals"
    pkill -SIGINT -f "rosbag record"
    pkill -SIGINT -f "./src/send_goals/current_pose_publisher.py"
    
    # 确保所有终端关闭
    
    echo "Waiting for 40 seconds..."
    sleep 30s #30 40
    
    echo "Preparing for the next iteration..."
    COUNT=$((COUNT + 1))
done

echo "Loop completed successfully. Exiting..."




# ------------------------------------------------------------------------------------------------------------------------------
# #!/bin/bash
# # 定义存放.bag.active文件的路径
# BAG_PATH="/home/zhangdingkun/robot_path_planner/bag_record"

# # 检查路径是否存在
# if [ ! -d "$BAG_PATH" ]; then
#   echo "指定路径不存在: $BAG_PATH"
#   exit 1
# fi

# # 初始化计数器
# COUNTER=1

# # 遍历路径下的所有.bag.active文件
# for FILE in "$BAG_PATH"/*.bag.active; do
#   if [ -f "$FILE" ]; then
#     echo "正在处理文件: $FILE"
    
#     # 提取文件名（不包含路径）
#     FILENAME=$(basename "$FILE")
    
#     # 定义目标文件名
#     RESULT_FILE="$BAG_PATH/data$COUNTER.bag"
    
#     # 使用rosbag reindex和rosbag fix命令恢复文件
#     rosbag reindex "$FILE"
#     rosbag fix "$FILE" "$RESULT_FILE"
    
#     echo "文件恢复完成，结果保存为: $RESULT_FILE"
#     ((COUNTER++))
#   fi
# done

# echo "所有文件处理完成！"