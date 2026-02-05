#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import numpy as np
from scipy.spatial.transform import Rotation as R
# === 修复点：确保引入了 JointState ===
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import time

# === 配置 ===
# 目标：机械臂正前方，稍微抬起一点
TARGET_CUBE_POS = [0.2, 0.4, 0.025] 
TARGET_CUBE_YAW = 0.0 

# TCP 偏移 (根据你的 Marker 数据)
TCP_OFFSET_POS = [0.06, 0.017, 0.11]
TCP_OFFSET_ROT = R.from_euler('y', -50, degrees=True)

class MoveItDirectPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_direct_planner', anonymous=True)
        
        self.move_group = moveit_commander.MoveGroupCommander("jaka_zu3")
        
        # === 关键：手动推流通道 ===
        # 这个 Topic 必须和 mujoco_node.py 里订阅的一模一样
        self.cmd_pub = rospy.Publisher('/mujoco/target_joints', JointState, queue_size=10)
        
        # TCP 矩阵预计算
        self.T_hand_tcp = np.eye(4)
        self.T_hand_tcp[:3, :3] = TCP_OFFSET_ROT.as_matrix()
        self.T_hand_tcp[:3, 3] = TCP_OFFSET_POS
        self.T_tcp_hand = np.linalg.inv(self.T_hand_tcp)

    def get_wrist_target(self, obj_pos, obj_yaw_deg):
        r_obj = R.from_euler('z', obj_yaw_deg, degrees=True)
        r_flip = R.from_euler('y', 180, degrees=True) 
        T_world_obj = np.eye(4)
        T_world_obj[:3, :3] = r_obj.as_matrix()
        T_world_obj[:3, 3] = obj_pos
        
        # 变换: Wrist = Object * TCP_Inverse
        T_world_wrist = np.dot(T_world_obj, self.T_tcp_hand)
        
        pose = geometry_msgs.msg.Pose()
        pose.position.x = T_world_wrist[0, 3]
        pose.position.y = T_world_wrist[1, 3]
        pose.position.z = T_world_wrist[2, 3]
        
        q = R.from_matrix(T_world_wrist[:3, :3]).as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def run(self):
        # 等待发布者建立连接
        time.sleep(1)
        
        print(">>> [Brain] Calculating IK...")
        goal_pose = self.get_wrist_target(TARGET_CUBE_POS, TARGET_CUBE_YAW)
        self.move_group.set_pose_target(goal_pose)
        
        # 1. 仅规划 (Plan Only)
        print(">>> [Brain] Planning path to target...")
        success, plan, _, _ = self.move_group.plan()
        
        if not success:
            print(">>> [Brain] ERROR: Planning failed. Target out of reach.")
            return

        print(f">>> [Brain] Plan found with {len(plan.joint_trajectory.points)} points.")
        print(">>> [Brain] STREAMING TO MUJOCO STARTING NOW...")

        # 2. 手动推流 (Manual Streaming)
        # 提高频率到 50Hz，让动作更顺滑
        rate = rospy.Rate(50) 
        
        for i, point in enumerate(plan.joint_trajectory.points):
            if rospy.is_shutdown(): break
            
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = plan.joint_trajectory.joint_names
            msg.position = point.positions
            
            # 发送给 MuJoCo
            self.cmd_pub.publish(msg)
            
            # 简单的进度条
            if i % 10 == 0:
                print(f"Streaming point {i}/{len(plan.joint_trajectory.points)}")
            
            rate.sleep()

        print(">>> [Brain] Execution finished. Check MuJoCo window.")

if __name__ == '__main__':
    try:
        MoveItDirectPlanner().run()
    except rospy.ROSInterruptException:
        pass