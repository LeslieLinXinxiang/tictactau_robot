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
# 1. 跨目录路径注入 (核心：让项目找到 SDK)
# ==========================================
# 指向你在 Docker 内部下载 SDK 的绝对路径
SDK_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk"
if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)

try:
    from LinkerHand.linker_hand_api import LinkerHandApi
    print(">>> [System] LinkerHand SDK 加载成功。")
except ImportError:
    print(f">>> [Error] 无法在 {SDK_PATH} 找到 SDK。")
    print("请确认 Docker 路径是否正确。")
    sys.exit(1)

# ==========================================
# 2. 硬件与逻辑参数
# ==========================================

# --- 灵巧手 0-255 刻度 (已校准) ---
# [拇指弯, 拇指旋, 食指弯, 中指弯, 无名, 小指]
HAND_OPEN  = [255, 48, 255, 255, 255, 255]
HAND_CLOSE = [140, 48, 160, 160, 255, 255]
HAND_TOLERANCE = 20    # 容差带
THUMB_BEND_INDEX = 0   # 判定关节索引

# --- 机械臂几何参数 ---
SHIFT_X = 0.02; SHIFT_Y = -0.065; SHIFT_Z = 0.14
TILT_X_DEG = 50; MOUNT_OFFSET_DEG = -90.0 
POS_A = [-0.601, 0.058, 0.025]; POS_B = [-0.361, -0.221, 0.025]     
YAW_A_DEG = 0.0; YAW_B_DEG = 0.0
SAFE_EXTRA_Z = 0.1            

# --- 机械臂运动参数 ---
HOME_JOINTS = [1.7, 1.57, -1.57, 1.57, 3.14, -1.57]
LINEAR_SPEED_MM = 50.0; LINEAR_ACC_MM = 20.0
JOINT_SPEED_RAD = 0.5; JOINT_ACC_RAD = 0.5

class JakaO6ProjectPnP:
    def __init__(self):
        # A. 初始化 ROS 节点
        rospy.init_node('jaka_o6_pnp_task', anonymous=True)
        self.current_tcp_mm = None
        rospy.Subscriber('/jaka_driver/tool_position', PoseStamped, self.feedback_cb)
        
        # B. 建立 JAKA 服务连接
        print(">>> [System] Connecting to JAKA Services...")
        try:
            rospy.wait_for_service('/jaka_driver/linear_move', timeout=5)
            self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
            self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
        except:
            print(">>> [System] ERROR: JAKA Driver not responding."); sys.exit(1)

        # C. 初始化灵巧手连接
        print(">>> [System] 初始化 Linker Hand O6 (右手)...")
        try:
            # 显式参数初始化，跳过 SDK 自动检测可能带来的 bug
            self.hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
            self.hand.set_enable()
            time.sleep(1)
            print(f">>> [Hand] 序列号: {self.hand.get_serial_number()}")
        except Exception as e:
            print(f">>> [Hand] 初始化失败: {e}"); sys.exit(1)

    # --- 机械臂核心辅助 ---
    def feedback_cb(self, msg):
        z = msg.pose.position.z
        scale = 1000.0 if abs(z) < 2.0 else 1.0
        self.current_tcp_mm = [msg.pose.position.x * scale, msg.pose.position.y * scale, z * scale]

    def get_smart_pose_mm(self, obj_pos_m, obj_yaw_deg=0.0, extra_z_m=0.0):
        raw_x, raw_y, raw_z = obj_pos_m
        arm_heading_rad = math.atan2(raw_y, raw_x)
        final_tcp_yaw_rad = arm_heading_rad + np.deg2rad(MOUNT_OFFSET_DEG) + np.deg2rad(obj_yaw_deg)
        s_x = SHIFT_X * math.cos(final_tcp_yaw_rad) - SHIFT_Y * math.sin(final_tcp_yaw_rad)
        s_y = SHIFT_X * math.sin(final_tcp_yaw_rad) + SHIFT_Y * math.cos(final_tcp_yaw_rad)
        return [(raw_x + s_x)*1000, (raw_y + s_y)*1000, (raw_z + SHIFT_Z + extra_z_m)*1000, 
                np.deg2rad(180 + TILT_X_DEG), 0.0, final_tcp_yaw_rad]

    def move_joint(self, joint_angles, label):
        print(f">>> [Joint] {label}...")
        req = MoveRequest(pose=joint_angles, has_ref=False, mvvelo=JOINT_SPEED_RAD, mvacc=JOINT_ACC_RAD, coord_mode=0)
        self.joint_client(req)
        time.sleep(3.5)

    def move_linear(self, target_pose, label):
        print(f">>> [Linear] {label}: {np.round(target_pose[:3], 1)}")
        req = MoveRequest(pose=target_pose, has_ref=False, mvvelo=LINEAR_SPEED_MM, mvacc=LINEAR_ACC_MM, coord_mode=1)
        self.linear_client(req)
        # 阻塞直到位移误差小于 15mm
        while not rospy.is_shutdown():
            if self.current_tcp_mm and np.linalg.norm(np.array(self.current_tcp_mm[:3]) - np.array(target_pose[:3])) < 15.0:
                break
            time.sleep(0.1)

    # --- 灵巧手反馈判定逻辑 ---
    def execute_hand_action(self, target_pose, action_label):
        print(f">>> [Hand] {action_label} -> {target_pose}")
        self.hand.finger_move(target_pose)
        
        target_val = target_pose[THUMB_BEND_INDEX]
        start_wait = time.time()
        
        # 闭环判定循环 (10Hz)
        while not rospy.is_shutdown():
            current_states = self.hand.get_state()
            if current_states and len(current_states) > THUMB_BEND_INDEX:
                current_val = current_states[THUMB_BEND_INDEX]
                error = abs(current_val - target_val)
                if error <= HAND_TOLERANCE:
                    print(f"    [OK] 拇指已达标 (Current:{current_val})")
                    break
            
            if time.time() - start_wait > 3.5: # 安全超时
                print(f"    [Timeout] 拇指未进容差带，强制执行下一步。")
                break
            time.sleep(0.1)

    def run(self):
        print("\n=== STARTING O6-INTEGRATED PNP MISSION ===\n")
        
        # 0. 预备
        self.execute_hand_action(HAND_OPEN, "HOME_HAND")
        self.move_joint(HOME_JOINTS, "HOME_ARM")
        
        # 1. 离线计算航点
        p_safe_a = self.get_smart_pose_mm(POS_A, obj_yaw_deg=YAW_A_DEG, extra_z_m=SAFE_EXTRA_Z)
        p_pick_a = self.get_smart_pose_mm(POS_A, obj_yaw_deg=YAW_A_DEG)
        p_safe_b = self.get_smart_pose_mm(POS_B, obj_yaw_deg=YAW_B_DEG, extra_z_m=SAFE_EXTRA_Z)
        p_place_b = self.get_smart_pose_mm(POS_B, obj_yaw_deg=YAW_B_DEG)

        # 2. 抓取循环
        # A 点取
        self.move_linear(p_safe_a, "GOTO_A_TOP")
        self.move_linear(p_pick_a, "DESCEND_A")
        self.execute_hand_action(HAND_CLOSE, "GRASP")
        self.move_linear(p_safe_a, "LIFT_A")

        # B 点放
        self.move_linear(p_safe_b, "TRANSIT_B")
        self.move_linear(p_place_b, "DESCEND_B")
        self.execute_hand_action(HAND_OPEN, "RELEASE")
        self.move_linear(p_safe_b, "LIFT_B")
        
        # 3. 归位
        self.move_joint(HOME_JOINTS, "PARK")
        print("\n=== MISSION SUCCESSFUL ===\n")

if __name__ == '__main__':
    try:
        pnp = JakaO6ProjectPnP()
        pnp.run()
    except rospy.ROSInterruptException:
        pass