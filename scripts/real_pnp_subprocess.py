#!/usr/bin/env python3
import sys
import os
import rospy
import numpy as np
import time
import math
import subprocess
from jaka_msgs.srv import Move, MoveRequest
from geometry_msgs.msg import PoseStamped 

# ==========================================
# 1. 硬件参数与路径
# ==========================================
# SDK 里的 CLI 工具路径
HAND_SCRIPT_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk/o6_cli.py"

# 机械臂几何
SHIFT_X = 0.02; SHIFT_Y = -0.065; SHIFT_Z = 0.14
TILT_X_DEG = 50; MOUNT_OFFSET_DEG = -90.0 
POS_A = [-0.601, 0.058, 0.025]; POS_B = [-0.361, -0.221, 0.025]     
YAW_A_DEG = 0.0; YAW_B_DEG = 0.0
SAFE_EXTRA_Z = 0.1            

HOME_JOINTS = [1.7, 1.57, -1.57, 1.57, 3.14, -1.57]
LINEAR_SPEED_MM = 50.0; LINEAR_ACC_MM = 20.0
JOINT_SPEED_RAD = 0.5; JOINT_ACC_RAD = 0.5

class JakaSubprocessPnP:
    def __init__(self):
        rospy.init_node('jaka_subprocess_pnp', anonymous=True)
        self.current_tcp_mm = None
        rospy.Subscriber('/jaka_driver/tool_position', PoseStamped, self.feedback_cb)
        
        print(">>> [System] Connecting to JAKA Services...")
        try:
            rospy.wait_for_service('/jaka_driver/linear_move', timeout=5)
            rospy.wait_for_service('/jaka_driver/joint_move', timeout=5)
            self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
            self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
            print(">>> [System] JAKA Connected.")
        except:
            print(">>> [System] ERROR: JAKA Driver died."); sys.exit(1)

        # 检查手是否在线 (通过调用 CLI)
        print(">>> [System] Checking Hand Status...")
        res = self.call_hand("check")
        if "OK" in res:
            print(">>> [Hand] Hand is READY (via Subprocess).")
        else:
            print(f">>> [Hand] Hand Check Failed: {res}")
            # sys.exit(1) # 可以选择不强制退出，继续调试手臂

        print(">>> [System] Waiting for robot feedback...")
        while self.current_tcp_mm is None and not rospy.is_shutdown():
            time.sleep(0.1)

    def feedback_cb(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        scale = 1000.0 if abs(z) < 2.0 else 1.0
        self.current_tcp_mm = [x * scale, y * scale, z * scale]

    def get_smart_pose_mm(self, obj_pos_m, obj_yaw_deg=0.0, extra_z_m=0.0):
        raw_x, raw_y, raw_z = obj_pos_m
        arm_heading_rad = math.atan2(raw_y, raw_x)
        final_tcp_yaw_rad = arm_heading_rad + np.deg2rad(MOUNT_OFFSET_DEG) + np.deg2rad(obj_yaw_deg)
        s_x = SHIFT_X * math.cos(final_tcp_yaw_rad) - SHIFT_Y * math.sin(final_tcp_yaw_rad)
        s_y = SHIFT_X * math.sin(final_tcp_yaw_rad) + SHIFT_Y * math.cos(final_tcp_yaw_rad)
        return [(raw_x + s_x)*1000, (raw_y + s_y)*1000, (raw_z + SHIFT_Z + extra_z_m)*1000, 
                np.deg2rad(180 + TILT_X_DEG), 0.0, final_tcp_yaw_rad]

    # --- 核心隔离调用 ---
    def call_hand(self, action):
        """调用外部 Python 脚本控制手，实现进程隔离"""
        try:
            # 这里的 python3 使用系统环境，确保有 can 库
            cmd = ["python3", HAND_SCRIPT_PATH, action]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.stdout.strip()
        except Exception as e:
            return str(e)

    def move_joint(self, joint_angles, label):
        print(f">>> [Joint] {label}...")
        req = MoveRequest()
        req.pose = joint_angles
        req.has_ref = False
        req.mvvelo = JOINT_SPEED_RAD
        req.mvacc = JOINT_ACC_RAD
        req.coord_mode = 0 
        try:
            self.joint_client(req)
            time.sleep(4.0) 
        except rospy.ServiceException as e:
            print(f">>> [Joint] Failed: {e}")

    def move_linear(self, target_pose, label):
        print(f">>> [Linear] {label}: {np.round(target_pose[:3], 1)}")
        req = MoveRequest()
        req.pose = target_pose
        req.has_ref = False
        req.mvvelo = LINEAR_SPEED_MM
        req.mvacc = LINEAR_ACC_MM 
        req.coord_mode = 1 
        try:
            self.linear_client(req)
            start_time = time.time()
            while not rospy.is_shutdown():
                curr = np.array(self.current_tcp_mm[:3])
                targ = np.array(target_pose[:3])
                if np.linalg.norm(curr - targ) < 15.0: break
                if time.time() - start_time > 15.0: break
                time.sleep(0.1)
        except rospy.ServiceException as e:
            print(f">>> [Linear] Failed: {e}")

    def run(self):
        print("\n=== STARTING SUBPROCESS PNP ===\n")
        
        # 0. 预备
        print(">>> [Hand] Initializing Open...")
        self.call_hand("open")
        self.move_joint(HOME_JOINTS, "HOME_ARM")
        
        # 1. 轨迹
        p_safe_a = self.get_smart_pose_mm(POS_A, YAW_A_DEG, SAFE_EXTRA_Z)
        p_pick_a = self.get_smart_pose_mm(POS_A, YAW_A_DEG)
        p_safe_b = self.get_smart_pose_mm(POS_B, YAW_B_DEG, SAFE_EXTRA_Z)
        p_place_b = self.get_smart_pose_mm(POS_B, YAW_B_DEG)

        # 2. 抓 A
        self.move_linear(p_safe_a, "GOTO_A_TOP")
        self.move_linear(p_pick_a, "DESCEND_A")
        
        print(">>> [Hand] Closing...")
        res = self.call_hand("close")
        print(f"    -> Result: {res}")
        
        self.move_linear(p_safe_a, "LIFT_A")

        # 3. 放 B
        self.move_linear(p_safe_b, "TRANSIT_B")
        self.move_linear(p_place_b, "DESCEND_B")
        
        print(">>> [Hand] Opening...")
        res = self.call_hand("open")
        print(f"    -> Result: {res}")
        
        self.move_linear(p_safe_b, "LIFT_B")
        
        # 4. 归位
        self.move_joint(HOME_JOINTS, "END_HOME")
        print("\n=== MISSION SUCCESSFUL ===\n")

if __name__ == '__main__':
    pnp = JakaSubprocessPnP()
    pnp.run()