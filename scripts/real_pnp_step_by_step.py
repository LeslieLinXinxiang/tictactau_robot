#!/usr/bin/env python3
import sys
import os
import rospy
import numpy as np
import transformations as tf
import time
import math
import subprocess
import select
from jaka_msgs.srv import Move, MoveRequest
from geometry_msgs.msg import PoseStamped 

# ==========================================
# 配置部分
# ==========================================
HAND_SCRIPT_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk/o6_cli.py"

# 几何参数
SHIFT_X = 0; SHIFT_Y = 0; SHIFT_Z = 0
#SHIFT_X = 0.02; SHIFT_Y = -0.055; SHIFT_Z = 0.14
TILT_X_DEG = 50; MOUNT_OFFSET_DEG = -90.0 
# POS_A = [-0.6, 0, 0.35]
POS_B = [-0.55, 0, 0.25]     
YAW_A_DEG = 0.0; YAW_B_DEG = 90.0
SAFE_EXTRA_Z = 0.1            
HOME_JOINTS = [1.7, 1.57, -1.57, 1.57, 3.14, -1.57]

class JakaStrictSerial:
    def __init__(self):
        rospy.init_node('jaka_strict_serial', anonymous=True)
        self.current_tcp_mm = None
        rospy.Subscriber('/jaka_driver/tool_position', PoseStamped, self.feedback_cb)
        
        print(">>> [System] Connecting to JAKA...")
        try:
            rospy.wait_for_service('/jaka_driver/linear_move', timeout=2)
            self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
            self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
        except:
            print(">>> [System] Warning: Driver service not healthy, but proceeding as requested.")

        print(">>> [System] Waiting for robot feedback...")
        while self.current_tcp_mm is None and not rospy.is_shutdown():
            time.sleep(0.1)
        print(f">>> [System] Feedback received. Z={self.current_tcp_mm[2]:.1f}")

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


    def transform(self, hand_pose):
        
        origin = [0, 0, 0]
        xaxis = [1, 0, 0]
        yaxis = [0, 1, 0]
        zaxis = [0, 0, 1]
        hand2ee_pos = tf.translation_matrix([-0.075, 0.01, 0])
        Rx = tf.rotation_matrix(0, xaxis)
        Ry = tf.rotation_matrix(0, yaxis)
        Rz = tf.rotation_matrix(0, zaxis)
        R = tf.concatenate_matrices(Rx, Ry, Rz)
        hand2ee_rot = R 
        hand2ee_trans = tf.concatenate_matrices(hand2ee_pos)

        inverse_matrix = np.linalg.inv(hand2ee_trans)
        target_ee_pose = np.matmul(inverse_matrix, hand_pose)
        return target_ee_pose

    # --- 核心：无论是否报错，必须暂停 ---
    def manual_wait(self, target_pose, label):
        print(f"\n======== [CHECK POINT] {label} ========")
        print(">>> 请观察真机：机械臂是否到位？")
        print(">>> 按 [Enter] 确认到位并继续...")
        
        while not rospy.is_shutdown():
            # 计算并打印误差
            dist_str = "Unknown"
            if target_pose is not None and self.current_tcp_mm is not None:
                curr = np.array(self.current_tcp_mm[:3])
                targ = np.array(target_pose[:3])
                dist = np.linalg.norm(curr - targ)
                dist_str = f"{dist:.1f} mm"
            
            sys.stdout.write(f"\r>>> 实时误差: {dist_str}  (等待回车...)")
            sys.stdout.flush()

            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                sys.stdin.readline()
                print("\n>>> [CONFIRMED] -> NEXT STEP\n")
                break
            time.sleep(0.1)

    def call_hand(self, action):
        # 在动手之前，再加一道确认，防止误触
        print(f"\n======== [HAND ACTION] 准备执行: {action.upper()} ========")
        input(">>> 机械臂停稳了吗？按 [Enter] 立即执行手部动作...")
        
        print(f">>> [Hand] Executing {action}...")
        try:
            cmd = ["python3", HAND_SCRIPT_PATH, action]
            subprocess.run(cmd, check=True)
            print(">>> [Hand] Done.")
        except Exception as e:
            print(f">>> [Hand] Error: {e}")

    def move_linear(self, target_pose, label):
        print(f">>> [Linear] Command: {label}")
        req = MoveRequest()
        req.pose = target_pose
        print(f">>> [Linear] Target Pose: {req.pose }")
        req.has_ref = False; req.mvvelo = 50.0; req.mvacc = 20.0; req.coord_mode = 1 
        
        try:
            self.linear_client(req)
        except rospy.ServiceException:
            # 捕获异常但不退出，继续执行 manual_wait
            print(f">>> [Linear] Service reported error (Ignored as requested).")
        
        # 强制暂停，等待人工确认
        self.manual_wait(target_pose, label)

    def run(self):

        tg_hand_pos = tf.translation_matrix([-0.51403, 0.21085, 0.15])
        origin = [0, 0, 0]
        xaxis = [1, 0, 0]
        yaxis = [0, 1, 0]
        zaxis = [0, 0, 1]
        Rx = tf.rotation_matrix(0, xaxis)
        Ry = tf.rotation_matrix(0, yaxis)
        Rz = tf.rotation_matrix(0, zaxis)
        R = tf.concatenate_matrices(Rx, Ry, Rz)
        tg_hand_rot = R
        target_hand_pose = tf.concatenate_matrices(tg_hand_pos, tg_hand_rot)
        target_ee_pose = self.transform(target_hand_pose)
        scale, shear, angles, translate, perspective = tf.decompose_matrix(target_ee_pose)
        print(f"Target EE Pose Translation (m): {translate}")
        print("\n=== STRICT SERIAL DEBUG MODE ===\n")

        target_place_pos = [-0.49623,-0.15097,0.15]

        # 1. 预备
        self.call_hand("open")
        # POS_A = [translate[0], translate[1], translate[2]]
        print(f">>> [DEBUG] Using POS_A = {translate} ")
        # 2. 计算
        p_safe_a = self.get_smart_pose_mm(translate, YAW_A_DEG, SAFE_EXTRA_Z)
        print(f">>> [DEBUG] Calculated p_safe_a = {p_safe_a} ")
        p_pick_a = self.get_smart_pose_mm(translate, YAW_A_DEG)
        p_safe_b = self.get_smart_pose_mm(target_place_pos, YAW_B_DEG, SAFE_EXTRA_Z)
        p_place_b = self.get_smart_pose_mm(target_place_pos, YAW_B_DEG)

        # 3. 步骤
        self.move_linear(p_safe_a, "1. GOTO Safe A")
        self.move_linear(p_pick_a, "2. DESCEND to Pick")
        
        self.call_hand("close") # 这里现在会再卡一次回车
        
        self.move_linear(p_safe_a, "3. LIFT A")
        self.move_linear(p_safe_b, "4. TRANSIT to B")
        self.move_linear(p_place_b, "5. DESCEND to Place")
        
        self.call_hand("open") # 这里也会再卡一次
        
        self.move_linear(p_safe_b, "6. LIFT B")
        
        print("\n=== DONE ===\n")

if __name__ == '__main__':
    try:
        app = JakaStrictSerial()
        app.run()
    except rospy.ROSInterruptException: pass