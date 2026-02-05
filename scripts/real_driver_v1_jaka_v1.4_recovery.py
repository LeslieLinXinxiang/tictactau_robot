#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from jaka_msgs.srv import Move, MoveRequest 
import time

# === 坐标配置 (单位: 毫米 mm) ===
# 目标：机械臂正前方
READY_JOINTS = [1.7, 0.685, -1.49, 3.34, 3.14, -0.75] # 一个标准的“伸手”姿态

# 你的目标 A (mm)
TARGET_A_MM = [350.0, 0.0, 300.0, np.deg2rad(180+50), 0.0, 0.0] 

class JakaRealRecovery:
    def __init__(self):
        rospy.init_node('jaka_real_recovery', anonymous=True)
        print(">>> [System] Connecting to JAKA Services...")
        rospy.wait_for_service('/jaka_driver/linear_move')
        rospy.wait_for_service('/jaka_driver/joint_move')
        self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
        self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)

    def move_joint(self, q_array):
        """关节运动：最安全，不走直线，不穿过奇异点"""
        print(f">>> [Action] Moving to READY pose (Joint Space): {q_array}")
        req = MoveRequest()
        req.pose = q_array
        req.has_ref = False
        req.mvvelo = 0.2  # 20% 速度
        req.mvacc = 0.2
        req.coord_mode = 0 
        return self.joint_client(req)

    def move_linear(self, p_array):
        """直线运动：末端走直线"""
        print(f">>> [Action] Moving Linear (Cartesian Space): {p_array}")
        req = MoveRequest()
        req.pose = p_array
        req.has_ref = False
        req.mvvelo = 50.0 # 50mm/s
        req.mvacc = 50.0
        req.coord_mode = 1 # 必须为 1 (代表 Base 坐标系直线)
        return self.linear_client(req)

    def run(self):
        print("\n" + "!"*40)
        print("  SAFETY CHECK: ROBOT WILL MOVE NOW!")
        print("  HOLD THE EMERGENCY STOP BUTTON!")
        print("!"*40 + "\n")
        
        input("Press Enter to move to READY pose (Joint Move)...")
        # 1. 先复位到一个舒服的姿态，解开当前的“扭曲”
        self.move_joint(READY_JOINTS)
        time.sleep(3)
        
        input("Press Enter to attempt LINEAR move to Target A...")
        # 2. 从舒服的姿态开始走直线
        res = self.move_linear(TARGET_A_MM)
        print(f">>> Result: {res}")

if __name__ == '__main__':
    JakaRealRecovery().run()
