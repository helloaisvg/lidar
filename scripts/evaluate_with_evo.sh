#!/bin/bash
# EVO轨迹评估脚本

echo "=== 使用EVO评估激光雷达里程计轨迹 ==="

# 安装EVO
if ! command -v evo_ape &> /dev/null; then
    echo "正在安装EVO..."
    pip3 install evo --upgrade --no-binary determined-system
fi

# 检查轨迹文件
TRAJECTORY_FILE="direct_processing_results/estimated_trajectory.txt"

if [ ! -f "$TRAJECTORY_FILE" ]; then
    echo "错误: 找不到轨迹文件 $TRAJECTORY_FILE"
    echo "请先运行: python3 scripts/direct_bag_processor.py ./data.bag"
    exit 1
fi

echo "找到轨迹文件: $TRAJECTORY_FILE"

# 创建评估结果目录
EVAL_DIR="evo_evaluation_results"
mkdir -p $EVAL_DIR

# 1. 可视化轨迹
echo ""
echo "1. 可视化轨迹..."
evo_traj tum $TRAJECTORY_FILE --plot --save_plot $EVAL_DIR/trajectory_plot.png

# 2. 如果有真值，进行APE评估
if [ -f "groundtruth.txt" ]; then
    echo ""
    echo "2. 进行绝对位姿误差(APE)评估..."
    evo_ape tum groundtruth.txt $TRAJECTORY_FILE -va --plot --save_plot $EVAL_DIR/ape_plot.png --save_results $EVAL_DIR/ape_stats.zip
    
    echo ""
    echo "3. 进行相对位姿误差(RPE)评估..."
    evo_rpe tum groundtruth.txt $TRAJECTORY_FILE -va --plot --save_plot $EVAL_DIR/rpe_plot.png --save_results $EVAL_DIR/rpe_stats.zip
else
    echo ""
    echo "未找到真值文件(groundtruth.txt)，跳过误差评估"
fi

# 3. 轨迹信息统计
echo ""
echo "4. 轨迹信息统计..."
evo_info tum $TRAJECTORY_FILE --save_statistics $EVAL_DIR/statistics.zip

echo ""
echo "=== 评估完成 ==="
echo "结果保存在: $EVAL_DIR/"
echo ""
echo "文件列表:"
ls -lh $EVAL_DIR/
