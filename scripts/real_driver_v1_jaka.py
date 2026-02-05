#!/usr/bin/env python3
import sys
import os
import rospy
import numpy as np
import time
import math
from jaka_msgs.srv import Move, MoveRequest
from geometry_msgs.msg import PoseStamped 

# ==========================================
# 1. SDK 路径注入
# ==========================================
SDK_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk"
if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)

try:
    from LinkerHand.linker_hand_api import LinkerHandApi
    print(">>> [System] LinkerHand SDK 加载成功。")
except ImportError:
    print(f">>> [Error] 无法在 {SDK_PATH} 找到 SDK。")
    sys.exit(1)

# ==========================================
# 2. 配置参数
# ==========================================

# --- 灵巧手 0-255 刻度 ---
HAND_OPEN  = [255, 48, 255, 255, 255, 255]
HAND_CLOSE = [140, 48, 160, 160, 255, 255]
HAND_TOLERANCE = 20
THUMB_BEND_INDEX = 0 

# --- 机械臂几何参数 ---
SHIFT_X = 0.02; SHIFT_Y = -0.065; SHIFT_Z = 0.14
TILT_X_DEG = 50; MOUNT_OFFSET_DEG = -90.0 
POS_A = [-0.601, 0.058, 0.025]; POS_B = [-0.361, -0.221, 0.025]     
YAW_A_DEG = 0.0; YAW_B_DEG = 90.0 # 假设放置时旋转90度
SAFE_EXTRA_Z = 0.1            

# --- 速度与 HOME ---
HOME_JOINTS = [1.7, 1.57, -1.57, 1.57, 3.14, -1.57]
LINEAR_SPEED_MM = 50.0; LINEAR_ACC_MM = 20.0
JOINT_SPEED_RAD = 0.5; JOINT_ACC_RAD = 0.5

class JakaO6IntegratedPnP:
    def __init__(self):
        rospy.init_node('jaka_o6_pnp_mission', anonymous=True)
        self.current_tcp_mm = None
        rospy.Subscriber('/jaka_driver/tool_position', PoseStamped, self.feedback_cb)
        
        # A. 连接 JAKA 服务 (增加 joint_move 的显式等待)
        print(">>> [System] Connecting to JAKA Services...")
        try:
            rospy.wait_for_service('/jaka_driver/linear_move', timeout=5)
            rospy.wait_for_service('/jaka_driver/joint_move', timeout=5)
            self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
            self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
            print(">>> [System] JAKA Services Connected.")
        except:
            print(">>> [System] ERROR: JAKA Driver not responding."); sys.exit(1)

        # B. 等待机器人反馈 (确保闭环逻辑有数据)
        print(">>> [System] Waiting for robot position feedback...")
        while self.current_tcp_mm is None and not rospy.is_shutdown():
            time.sleep(0.1)

        # C. 初始化灵巧手
        print(">>> [System] 初始化 Linker Hand O6 (右手)...")
        try:
            self.hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
            self.hand.set_enable()
            time.sleep(1)
            print(f">>> [Hand] 序列号: {self.hand.get_serial_number()}")
        except Exception as e:
            print(f">>> [Hand] 初始化失败: {e}"); sys.exit(1)

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

    # --- 核心修复：还原为“逐字段赋值”模式 ---
    def move_joint(self, joint_angles, label):
        print(f">>> [Joint] {label}...")
        req = MoveRequest() # 不在构造函数里传参
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
        req = MoveRequest() # 不在构造函数里传参
        req.pose = target_pose
        req.has_ref = False
        req.mvvelo = LINEAR_SPEED_MM
        req.mvacc = LINEAR_ACC_MM 
        req.coord_mode = 1 
        try:
            self.linear_client(req)
            # 闭环检测到位
            start_time = time.time()
            while not rospy.is_shutdown():
                curr = np.array(self.current_tcp_mm[:3])
                targ = np.array(target_pose[:3])
                if np.linalg.norm(curr - targ) < 15.0: break
                if time.time() - start_time > 15.0: break
                time.sleep(0.1)
        except rospy.ServiceException as e:
            print(f">>> [Linear] Failed: {e}")

    def execute_hand_action(self, target_pose, label):
        print(f">>> [Hand] {label} -> {target_pose}")
        self.hand.finger_move(target_pose)
        target_val = target_pose[THUMB_BEND_INDEX]
        start_wait = time.time()
        while not rospy.is_shutdown():
            states = self.hand.get_state()
            if states and len(states) > THUMB_BEND_INDEX:
                if abs(states[THUMB_BEND_INDEX] - target_val) <= HAND_TOLERANCE:
                    print(f"    [OK] 手指已到位 ({states[THUMB_BEND_INDEX]})")
                    break
            if time.time() - start_wait > 3.0: break
            time.sleep(0.1)

    def run(self):
        print("\n=== STARTING INTEGRATED PNP ===\n")
        self.execute_hand_action(HAND_OPEN, "INIT_HAND")
        self.move_joint(HOME_JOINTS, "HOME_ARM")
        
        # 轨迹点计算
        p_safe_a = self.get_smart_pose_mm(POS_A, YAW_A_DEG, SAFE_EXTRA_Z)
        p_pick_a = self.get_smart_pose_mm(POS_A, YAW_A_DEG)
        p_safe_b = self.get_smart_pose_mm(POS_B, YAW_B_DEG, SAFE_EXTRA_Z)
        p_place_b = self.get_smart_pose_mm(POS_B, YAW_B_DEG)

        # 任务循环
        self.move_linear(p_safe_a, "SAFE_A")
        self.move_linear(p_pick_a, "PICK_DOWN")
        self.execute_hand_action(HAND_CLOSE, "GRAB")
        self.move_linear(p_safe_a, "PICK_UP")
        
        self.move_linear(p_safe_b, "TRANSIT_B")
        self.move_linear(p_place_b, "PLACE_DOWN")
        self.execute_hand_action(HAND_OPEN, "RELEASE")
        self.move_linear(p_safe_b, "PLACE_UP")
        
        self.move_joint(HOME_JOINTS, "END_HOME")
        print("\n=== TASK COMPLETE ===\n")

if __name__ == '__main__':
    pnp = JakaO6IntegratedPnP()
    pnp.run()