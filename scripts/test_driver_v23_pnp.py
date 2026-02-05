#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import time

# === 你验证过的黄金偏移量 (Direct Offset) ===
SHIFT_X = 0.04
SHIFT_Y = -0.065
SHIFT_Z = 0.14
TILT_X  = 230  # (180 + 50)

# === 任务坐标 ===
POS_A = [0.3, 0.3, 0.025]     # 抓取点 (棋子 A)
POS_B = [0.1, 0.4, 0.025]     # 释放点 (棋子 B)
SAFE_EXTRA_Z = 0.1            # 抬升高度 (在 SHIFT_Z 之上再加 10cm)

# === 夹爪角度 (h1-h6) ===
HAND_OPEN  = [1.21, 0.0, 0.0, 0.0, 0.0, 0.0]
HAND_CLOSE = [1.21, 0.25, 0.5, 0.5, 0.0, 0.0]
INITIAL_Q  = [1.7, 1.57, -1.57, 1.57, 3.14, -1.57]

class EmpiricalPnP:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('empirical_pnp')
        
        self.move_group = moveit_commander.MoveGroupCommander("jaka_zu3")
        self.arm_pub = rospy.Publisher('/mujoco/target_joints', JointState, queue_size=10)
        self.hand_pub = rospy.Publisher('/mujoco/target_gripper', JointState, queue_size=10)
        
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    def get_empirical_pose(self, obj_pos, extra_z=0.0):
        """
        核心：完全套用你给出的计算逻辑
        Wrist_Pos = Object_Pos + SHIFT + [0, 0, extra_z]
        """
        pose = Pose()
        pose.position.x = obj_pos[0] + SHIFT_X
        pose.position.y = obj_pos[1] + SHIFT_Y
        pose.position.z = obj_pos[2] + SHIFT_Z + extra_z
        
        q = R.from_euler('x', TILT_X, degrees=True).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        return pose

    def solve_ik(self, target_pose, seed_q):
        req = GetPositionIKRequest()
        req.ik_request.group_name = "jaka_zu3"
        req.ik_request.pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.robot_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        req.ik_request.robot_state.joint_state.position = seed_q
        req.ik_request.timeout = rospy.Duration(0.1)
        
        res = self.ik_service(req)
        return list(res.solution.joint_state.position[:6]) if res.error_code.val == 1 else None

    def trace_move(self, start_pose, end_pose, duration=2.0):
        """直线追踪移动 (防止出现你说的先左后右的怪动作)"""
        steps = int(duration * 50)
        last_q = self.move_group.get_current_joint_values()
        
        for i in range(1, steps + 1):
            alpha = i / float(steps)
            curr_p = Pose()
            # 线性插值位置
            curr_p.position.x = start_pose.position.x*(1-alpha) + end_pose.position.x*alpha
            curr_p.position.y = start_pose.position.y*(1-alpha) + end_pose.position.y*alpha
            curr_p.position.z = start_pose.position.z*(1-alpha) + end_pose.position.z*alpha
            curr_p.orientation = end_pose.orientation # 姿态保持最终态
            
            q_arm = self.solve_ik(curr_p, last_q)
            if q_arm:
                msg = JointState()
                msg.position = q_arm
                self.arm_pub.publish(msg)
                last_q = q_arm
            time.sleep(0.02)

    def set_gripper(self, angles, label):
        print(f">>> [Gripper] {label}")
        msg = JointState()
        msg.position = angles
        self.hand_pub.publish(msg)
        time.sleep(1.0)

    def run_cycle(self):
        print(">>> [System] Starting Empirical PnP Cycle...")
        
        # 1. 准备：获取关键点位姿
        p_home_wrist = self.move_group.get_current_pose().pose
        p_safe_a = self.get_empirical_pose(POS_A, extra_z=SAFE_EXTRA_Z)
        p_pick_a = self.get_empirical_pose(POS_A)
        p_safe_b = self.get_empirical_pose(POS_B, extra_z=SAFE_EXTRA_Z)
        p_place_b = self.get_empirical_pose(POS_B)

        # 2. 执行动作序列
        # (1) 飞向 A 上方
        self.trace_move(p_home_wrist, p_safe_a)
        # (2) 下降抓取
        self.trace_move(p_safe_a, p_pick_a, duration=1.0)
        # (3) 闭合夹爪
        self.set_gripper(HAND_CLOSE, "GRAB")
        # (4) 抬升
        self.trace_move(p_pick_a, p_safe_a, duration=1.0)
        # (5) 水平移向 B 上方
        self.trace_move(p_safe_a, p_safe_b)
        # (6) 下降放置
        self.trace_move(p_safe_b, p_place_b, duration=1.0)
        # (7) 松开夹爪
        self.set_gripper(HAND_OPEN, "RELEASE")
        # (8) 抬升
        self.trace_move(p_place_b, p_safe_b, duration=1.0)
        
        # 3. 回家 (同步关节空间)
        print(">>> [System] Returning to Home...")
        start_q = self.move_group.get_current_joint_values()
        for i in range(100):
            alpha = i/100.0
            q = [start_q[j]*(1-alpha) + INITIAL_Q[j]*alpha for j in range(6)]
            msg = JointState(); msg.position = q; self.arm_pub.publish(msg)
            time.sleep(0.02)
        
        print(">>> [System] Cycle Complete.")

if __name__ == '__main__':
    pnp = EmpiricalPnP()
    pnp.run_cycle()