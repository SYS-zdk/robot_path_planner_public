#!/bin/bash
# 定义目录路径
BAG_DIR="/home/zhangdingkun/robot_path_planner/bag_record"

# 检查目录是否存在
if [ ! -d "$BAG_DIR" ]; then
    echo "Error: Directory $BAG_DIR does not exist."
    exit 1
fi

# 进入目标目录
cd "$BAG_DIR"

# 初始化计数器
COUNT=1

# 遍历目录下的所有.bag文件
for BAG_FILE in *.bag; do
    if [ -f "$BAG_FILE" ]; then
        # 定义输出的CSV文件名
        OUTPUT_FILE="data${COUNT}.csv"
        
        # 清空或创建输出文件
        > "$OUTPUT_FILE"
        
        # 定义要提取的话题列表
        # TOPICS=("/current_pose" "/cmd_vel" "/move_base/PathPlanner/initial_plan" "/move_base/PathPlanner/initial_plan2" "/move_base/PathPlanner/plan_time" "/move_base/PathPlanner/plan_opt")
        # TOPICS=("/current_pose" "/cmd_vel" "/move_base/PathPlanner/initial_plan" "/move_base/PathPlanner/initial_plan2")
        TOPICS=("/current_pose" "/cmd_vel")
        
        # 遍历每个话题并提取数据
        for TOPIC in "${TOPICS[@]}"; do
            # 使用rostopic echo提取数据并追加到CSV文件
            rostopic echo -b "$BAG_FILE" -p "$TOPIC" >> "$OUTPUT_FILE"
        done
        
        # 输出当前处理的文件信息
        echo "Processed $BAG_FILE and saved as $OUTPUT_FILE"
        
        # 计数器加1
        ((COUNT++))
    fi
done

echo "All .bag files have been processed."