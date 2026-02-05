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
# 1. 配置参数
# ==========================================
HAND_SCRIPT_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk/o6_cli.py"

# --- 几何参数 ---
SHIFT_X = -0.04; SHIFT_Y = -0.1; SHIFT_Z = 0.13
TILT_X_DEG = 50; MOUNT_OFFSET_DEG = -60.0 
# 更新为你的最新坐标
POS_A = [-0.55860, 0.17829, 0.025]; POS_B = [-0.28448, -0.06299, 0.025]     
YAW_A_DEG = 0.0; YAW_B_DEG = 0.0
SAFE_EXTRA_Z = 0.1 

HOME_JOINTS = [1.7, 0.685, -1.49, 3.34, 3.14, -0.75]

# --- 速度设置 (已按你要求改为 50) ---
LINEAR_SPEED_MM = 100.0   
LINEAR_ACC_MM   = 50.0   
JOINT_SPEED_RAD = 0.5     
JOINT_ACC_RAD   = 0.5    

AUTO_TOLERANCE_MM = 5.0

class JakaFlowPnP:
    def __init__(self):
        rospy.init_node('jaka_flow_pnp', anonymous=True)
        self.current_tcp_mm = None
        rospy.Subscriber('/jaka_driver/tool_position', PoseStamped, self.feedback_cb)
        
        try:
            self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
            self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
        except: pass

        # 这里只是在等待数据流建立，并没有做任何位置判定逻辑
        # 这一步不能删，否则下面 get_smart_pose 可能还没数据就报错
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

    def wait_until_reached(self, target_pose):
        while not rospy.is_shutdown():
            if self.current_tcp_mm:
                if np.linalg.norm(np.array(self.current_tcp_mm[:3]) - np.array(target_pose[:3])) < AUTO_TOLERANCE_MM:
                    return True
            time.sleep(0.01)

    def call_hand(self, action):
        print(f">>> [Hand] {action.upper()}")
        subprocess.run(["python3", HAND_SCRIPT_PATH, action], check=True)

    def move_linear_no_wait(self, target_pose, label):
        """只发指令，不等待"""
        print(f">>> [Flow] {label}")
        req = MoveRequest()
        req.pose = target_pose
        req.has_ref = False; req.mvvelo = LINEAR_SPEED_MM; req.mvacc = LINEAR_ACC_MM; req.coord_mode = 1 
        try: self.linear_client(req)
        except: pass
        time.sleep(0.05) 

    def move_linear_with_wait(self, target_pose, label):
        """发送并阻塞等待到位"""
        print(f">>> [Linear] {label}")
        req = MoveRequest()
        req.pose = target_pose
        req.has_ref = False; req.mvvelo = LINEAR_SPEED_MM; req.mvacc = LINEAR_ACC_MM; req.coord_mode = 1 
        try: self.linear_client(req)
        except: pass 
        self.wait_until_reached(target_pose)

    def run(self):
        print("\n=== STARTING STABLE PNP ===\n")
        p_safe_a = self.get_smart_pose_mm(POS_A, YAW_A_DEG, SAFE_EXTRA_Z)
        p_pick_a = self.get_smart_pose_mm(POS_A, YAW_A_DEG)
        p_safe_b = self.get_smart_pose_mm(POS_B, YAW_B_DEG, SAFE_EXTRA_Z)
        p_place_b = self.get_smart_pose_mm(POS_B, YAW_B_DEG)

        self.call_hand("open")
        
        # --- 关键修改：第一步入场，必须是阻塞的 (with_wait) ---
        # 这样机器人会先稳稳地飞到 A 点上方，把手腕姿态调整好，停稳。
        # 彻底消除了“一边飞一边扭”的怪异动作。
        self.move_linear_with_wait(p_safe_a, "1. Stable Entry to A") 
        
        # --- 接下来的动作依然保持 Flow (无等待)，除了抓取点 ---
        self.move_linear_with_wait(p_pick_a, "2. Descend to Pick A") # 下压抓取必须准
        self.call_hand("close") 
        
        # --- 抬升平移：流动连贯 ---
        self.move_linear_no_wait(p_safe_a, "3. Lifting...") # 抬起不等
        self.move_linear_no_wait(p_safe_b, "4. Transiting to B...") # 平移不等
        self.move_linear_with_wait(p_place_b, "5. Descend to Place B") # 放置点必须准
        self.call_hand("open") 
        
        # --- 退出 ---
        self.move_linear_no_wait(p_safe_b, "6. Exiting...")
        
        # 归位 (可选)
        req = MoveRequest(pose=HOME_JOINTS, mvvelo=JOINT_SPEED_RAD, mvacc=JOINT_ACC_RAD, coord_mode=0)
        self.joint_client(req)
        print("\n=== DONE ===\n")

if __name__ == '__main__':
    try: JakaFlowPnP().run()
    except rospy.ROSInterruptException: pass