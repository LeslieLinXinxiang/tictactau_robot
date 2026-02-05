#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
import time

# === 目标配置 ===
TARGET_CUBE_POS = [0.2, 0.4, 0.025]  

# === 调试偏移量 ===
SHIFT_X = 0.0
SHIFT_Y = -0.06  # 用户设定的 -2cm 偏移
SHIFT_Z = 0.15   # 悬停高度

class MoveItTiltPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_tilt_planner', anonymous=True)
        
        self.move_group = moveit_commander.MoveGroupCommander("jaka_zu3")
        self.cmd_pub = rospy.Publisher('/mujoco/target_joints', JointState, queue_size=10)
        
        # === Pilz PTP 工业模式 ===
        self.move_group.set_planner_id("PTP")
        self.move_group.set_num_planning_attempts(1)
        self.move_group.set_planning_time(5.0)
        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        self.move_group.clear_path_constraints()

    def get_tilt_target(self, obj_pos):
        pose = Pose()
        
        # 1. 计算位置
        pose.position.x = obj_pos[0] + SHIFT_X
        pose.position.y = obj_pos[1] + SHIFT_Y
        pose.position.z = obj_pos[2] + SHIFT_Z
        
        # 2. 计算姿态 (核心修改)
        # 基础是向下 (180度)，现在要绕 X 轴再转 50 度
        # 180 + 50 = 230 度
        # 如果你发现转反了，就把 50 改成 -50 (即 130 度)
        tilt_angle = 180 + 50
        
        q = R.from_euler('x', tilt_angle, degrees=True).as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        print(f">>> [Debug] Target Info:")
        print(f"    Pos : [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]")
        print(f"    Rot : X-Axis Rotation {tilt_angle} deg (180 + 50)")
        
        return pose

    def run(self):
        time.sleep(1)
        
        print(">>> [Brain] Calculating Tilted Target...")
        goal_pose = self.get_tilt_target(TARGET_CUBE_POS)
        
        self.move_group.set_pose_target(goal_pose)
        
        print(f">>> [Brain] Planning with Pilz PTP...")
        success, plan, _, _ = self.move_group.plan()
        
        if not success:
            print(">>> [Brain] ERROR: Planning failed!")
            print("    Analysis: The 50-degree tilt might be hitting a joint limit.")
            print("    Try changing '180 + 50' to '180 - 50' in the code.")
            return

        points = plan.joint_trajectory.points
        print(f">>> [Brain] Path found! Length: {len(points)} points.")
        
        rate = rospy.Rate(50) 
        for point in points:
            if rospy.is_shutdown(): break
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = plan.joint_trajectory.joint_names
            msg.position = point.positions
            self.cmd_pub.publish(msg)
            rate.sleep()
        print(">>> [Brain] Finished.")

if __name__ == '__main__':
    try:
        MoveItTiltPlanner().run()
    except rospy.ROSInterruptException: pass