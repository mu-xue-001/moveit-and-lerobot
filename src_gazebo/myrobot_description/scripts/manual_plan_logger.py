#!/usr/bin/env python3
"""
手动规划监听 + 分阶段自动统计
用法：
1. 修改变量：CURRENT_PLANNER, STAGE_LIMITS
2. 运行脚本
3. 在 RViz2 中按顺序对每个目标点击 Plan（每次成功后输入规划时间）
4. 脚本会在达到每个阶段限制时自动打印该阶段的统计结果
5. 切换规划器时，修改 CURRENT_PLANNER 并重新运行脚本
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import csv
import math
from datetime import datetime

# ========== 手动配置 ==========
CURRENT_PLANNER = "RRT-Connect**"   # 每次切换规划器时修改

# 阶段规划次数限制（累计成功次数）
# 例如：5个目标各1次 = 5；接着5个目标各3次 = 15（累计20）；接着5个目标各5次 = 25（累计45）
STAGE_LIMITS = [5, 20, 45]       # 三个阶段分别到达的总成功次数

# 可选：输出文件名前缀
OUTPUT_PREFIX = "plan_log"
# =============================

class ManualPlanLogger(Node):
    def __init__(self):
        super().__init__('manual_plan_logger')
        self.records = []          # 所有记录
        self.trial_num = 0         # 总成功次数
        self.current_stage = 0     # 0,1,2
        self.stage_records = []    # 当前阶段内的记录
        
        # 订阅规划成功轨迹
        self.sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.on_trajectory,
            10
        )
        
        print(f"\n=== 规划器: {CURRENT_PLANNER} ===")
        print(f"阶段计划: 共{len(STAGE_LIMITS)}个阶段，每个阶段成功次数上限: {STAGE_LIMITS}")
        print("请在 RViz2 中依次点击 Plan，每成功一次输入时间（毫秒）")
        print("脚本将自动累计并在每个阶段结束时打印统计\n")
    
    def compute_path_length(self, traj):
        points = traj.joint_trajectory.points
        if len(points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(points)):
            diff = sum((a-b)**2 for a,b in zip(points[i].positions, points[i-1].positions))
            total += math.sqrt(diff)
        return total
    
    def print_stage_stats(self):
        if not self.stage_records:
            return
        n = len(self.stage_records)
        plan_times = [r['planning_time_ms'] for r in self.stage_records if r.get('planning_time_ms') is not None]
        path_lens = [r['path_length_rad'] for r in self.stage_records]
        num_pts = [r['num_points'] for r in self.stage_records]
        
        print(f"\n===== 阶段{self.current_stage+1} 统计 (共{n}次成功) =====")
        if plan_times:
            avg_t = sum(plan_times)/len(plan_times)
            var_t = sum((t-avg_t)**2 for t in plan_times)/len(plan_times)
            # 修改点1：规划时间显示四位小数
            print(f"规划时间: 平均 {avg_t:.4f} ms, 标准差 {math.sqrt(var_t):.4f} ms")
        if path_lens:
            avg_l = sum(path_lens)/len(path_lens)
            var_l = sum((l-avg_l)**2 for l in path_lens)/len(path_lens)
            print(f"路径长度: 平均 {avg_l:.4f} rad, 标准差 {math.sqrt(var_l):.4f} rad")
        if num_pts:
            print(f"轨迹点数: 平均 {sum(num_pts)/len(num_pts):.1f}")
        print("====================================\n")
        # 清空阶段数据
        self.stage_records = []
    
    def on_trajectory(self, msg):
        if not msg.trajectory:
            self.get_logger().warn("空的 trajectory")
            return
        traj = msg.trajectory[0]
        if not traj.joint_trajectory.points:
            self.get_logger().warn("没有关节点")
            return
        
        self.trial_num += 1
        path_len = self.compute_path_length(traj)
        num_points = len(traj.joint_trajectory.points)
        
        print(f"\n[成功] 第 {self.trial_num} 次规划")
        print(f"  路径长度: {path_len:.4f} rad")
        print(f"  轨迹点数: {num_points}")
        time_input = input("  规划时间 (毫秒，回车留空): ").strip()
        # 修改点2：存储时保留四位小数
        plan_time_ms = round(float(time_input), 4) if time_input else None
        
        record = {
            'planner': CURRENT_PLANNER,
            'trial': self.trial_num,
            'success': True,
            'path_length_rad': round(path_len, 4),
            'num_points': num_points,
            'planning_time_ms': plan_time_ms,
            'timestamp': datetime.now().isoformat()
        }
        self.records.append(record)
        self.stage_records.append(record)
        
        # 检查阶段是否完成
        if self.current_stage < len(STAGE_LIMITS) and self.trial_num >= STAGE_LIMITS[self.current_stage]:
            self.print_stage_stats()
            self.current_stage += 1
            if self.current_stage >= len(STAGE_LIMITS):
                print("所有阶段已完成！")
    
    def save_results(self):
        if not self.records:
            print("没有记录")
            return
        filename = f"{OUTPUT_PREFIX}_{CURRENT_PLANNER}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as f:
            fieldnames = ['planner','trial','success','path_length_rad','num_points','planning_time_ms','timestamp']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)
        print(f"结果已保存至 {filename}")
        # 总体统计
        print(f"\n=== {CURRENT_PLANNER} 总体统计 ===")
        times = [r['planning_time_ms'] for r in self.records if r['planning_time_ms']]
        if times:
            # 修改点3：总体平均时间也保留四位小数
            print(f"平均时间: {sum(times)/len(times):.4f} ms")

def main():
    rclpy.init()
    node = ManualPlanLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()