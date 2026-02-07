#!/usr/bin/env python3
import sys
import os
import rospy
import numpy as np
import time
import math
import subprocess
import argparse
import tf
from jaka_msgs.srv import Move, MoveRequest
from geometry_msgs.msg import PoseStamped 

# ==========================================
# 1. 配置参数
# ==========================================
HAND_SCRIPT_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk/o6_cli.py"
# 物理偏移参数保持不变
SHIFT_X = -0.02; SHIFT_Y = -0.065; SHIFT_Z = 0.10
TILT_X_DEG = 50; MOUNT_OFFSET_DEG = -30.0 

# 初始默认值
POS_A = [-0.43942, 0.42619, 0.07]
POS_B = [-0.59937, -0.24758, 0.07]    

Height_Place = 0.01
# [x,y,z,qx,qy,qz,qw]
POS_00 = [-0.553,-0.215,Height_Place,-0.24838786376988045,-0.8827268648748214,0.3590423670639823,0.3590423670639823]
POS_01 = [-0.602,-0.109,Height_Place,-0.3413772425171733,-0.8579467720461618,0.3602110415469914,0.13280406686006893] 
POS_02 = [-0.617,0.0024,Height_Place,-0.41563805260688547,-0.8231691476955079,0.34759241043346095,0.16975594162344787]

# Other points do not need rotation
POS_10 = [-0.460,-0.162,Height_Place] 
POS_11 = [-0.485,-0.053,Height_Place] 
POS_12 = [-0.500,0.054,Height_Place] 
POS_20 = [-0.350,-0.132,Height_Place] 
POS_21 = [-0.378,-0.025,Height_Place] 
POS_22 = [-0.382,-0.082,Height_Place] 

# ✨ 删除了全局的 YAW_A_DEG = 0.0，以消除干扰 ✨
# 全局变量：
YAW_B_DEG = None  

SAFE_EXTRA_Z = 0.1
HOME_JOINTS = [1.3, 0.685, -1.49, 3.34, 3.14, -0.75]
LINEAR_SPEED_MM = 150.0; LINEAR_ACC_MM = 50.0; JOINT_SPEED_RAD = 0.5; JOINT_ACC_RAD = 0.5
AUTO_TOLERANCE_MM = 5.0

class JakaFreeYawPnP:
    def __init__(self):
        rospy.init_node('jaka_free_yaw_pnp_exec', anonymous=True)
        self.current_tcp_mm = None
        rospy.Subscriber('/jaka_driver/tool_position', PoseStamped, self.feedback_cb)
        try:
            self.linear_client = rospy.ServiceProxy('/jaka_driver/linear_move', Move)
            self.joint_client = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
        except: pass
        
        while self.current_tcp_mm is None and not rospy.is_shutdown():
            time.sleep(0.1)

    def feedback_cb(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        scale = 1000.0 if abs(z) < 2.0 else 1.0
        self.current_tcp_mm = [x * scale, y * scale, z * scale]

    def get_smart_pose_mm(self, obj_pos_m, obj_yaw_deg=0.0, extra_z_m=0.0):
        """接收传入的 obj_yaw_deg (来自命令行)"""
        raw_x, raw_y, raw_z = obj_pos_m
        arm_heading_rad = math.atan2(raw_y, raw_x)
        
        if obj_yaw_deg is None:
            final_tcp_yaw_rad = arm_heading_rad + np.deg2rad(MOUNT_OFFSET_DEG)
            print(f"    [Geom] B-Pose: Free Rotation Mode (Natural Heading)")
        else:
            # 使用传入的变量，而不是全局变量
            final_tcp_yaw_rad = arm_heading_rad + np.deg2rad(MOUNT_OFFSET_DEG) + np.deg2rad(obj_yaw_deg)
            print(f"    [Geom] Object Yaw: {obj_yaw_deg:>5.1f}° | Target TCP Yaw: {math.degrees(final_tcp_yaw_rad):>5.1f}°")
        
        s_x = SHIFT_X * math.cos(final_tcp_yaw_rad) - SHIFT_Y * math.sin(final_tcp_yaw_rad)
        s_y = SHIFT_X * math.sin(final_tcp_yaw_rad) + SHIFT_Y * math.cos(final_tcp_yaw_rad)
        
        return [(raw_x + s_x)*1000, (raw_y + s_y)*1000, (raw_z + SHIFT_Z + extra_z_m)*1000, 
                np.deg2rad(180 + TILT_X_DEG), 0.0, final_tcp_yaw_rad]

    def wait_until_reached(self, target_pose):
        while not rospy.is_shutdown():
            if self.current_tcp_mm and np.linalg.norm(np.array(self.current_tcp_mm[:3]) - np.array(target_pose[:3])) < AUTO_TOLERANCE_MM:
                return True
            time.sleep(0.01)

    def call_hand(self, action):
        print(f">>> [Hand] {action.upper()}")
        subprocess.run(["python3", HAND_SCRIPT_PATH, action], check=True)

    def move_linear_no_wait(self, target_pose, label):
        req = MoveRequest(pose=target_pose, has_ref=False, mvvelo=LINEAR_SPEED_MM, mvacc=LINEAR_ACC_MM, coord_mode=1)
        try: self.linear_client(req)
        except: pass
        time.sleep(0.05) 

    def move_linear_with_wait(self, target_pose, label):
        print(f">>> [Linear] {label}")
        req = MoveRequest(pose=target_pose, has_ref=False, mvvelo=LINEAR_SPEED_MM, mvacc=LINEAR_ACC_MM, coord_mode=1)
        try: self.linear_client(req)
        except: pass 
        self.wait_until_reached(target_pose)
        
    def get_place_abs_pose(self, target):
        
        if target == "00":
            eulers = tf.transformations.euler_from_quaternion([POS_00[6], POS_00[3], POS_00[4], POS_00[5]])
            pos = [POS_00[0], POS_00[1], POS_00[2], eulers[0], eulers[1], eulers[2]]
            return pos
        elif target == "01":
            eulers = tf.transformations.euler_from_quaternion([POS_01[6], POS_01[3], POS_01[4], POS_01[5]])
            pos = [POS_01[0], POS_01[1], POS_01[2], eulers[0], eulers[1], eulers[2]]
            return pos
        elif target == "02":
            eulers = tf.transformations.euler_from_quaternion([POS_02[6], POS_02[3], POS_02[4], POS_02[5]])
            pos = [POS_02[0], POS_02[1], POS_02[2], eulers[0], eulers[1], eulers[2]]
            return pos
        elif target == "10": 
            return POS_10
        elif target == "11":
            return POS_11
        elif target == "12":
            return POS_12
        elif target == "20":
            return POS_20
        elif target == "21":
            return POS_21
        elif target == "22":
            return POS_22

    # ✨ 接收外部传入的 yaw_a 变量
    def run(self, pos_a, target_code, yaw_a):
        print(f"\n=== EXECUTION START (YAW_A: {yaw_a}°) ===\n")
        
        # 将传入的 yaw_a 变量传递给计算函数
        p_safe_a = self.get_smart_pose_mm(pos_a, yaw_a, SAFE_EXTRA_Z)
        p_pick_a = self.get_smart_pose_mm(pos_a, yaw_a)
        
        # Move to fixed target position given target
        target = target_code
        target_place = self.get_place_abs_pose(target)
        print(target_place)
        
        pos_b = [target_place[0], target_place[1], target_place[2]]
        
        p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
        p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
        
        if target == "00":
            
            pos_b[0] = target_place[0]
            pos_b[1] = target_place[1]            
            p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
            p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
            p_safe_b[0] += -43
            p_safe_b[1] += -43
            p_place_b[0] += -43
            p_place_b[1] += -43
            
        elif target == "01":
            
            pos_b[0] = target_place[0]
            pos_b[1] = target_place[1]            
            p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
            p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
            p_safe_b[0] += -30
            p_safe_b[1] += -30
            p_place_b[0] += -30
            p_place_b[1] += -30
            
        elif target == "12":
            
            pos_b[0] = target_place[0]
            pos_b[1] = target_place[1]            
            p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
            p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
            p_safe_b[0] += -50
            p_safe_b[1] += -35
            p_place_b[0] += -50
            p_place_b[1] += -35
            
        elif target == "21":
            
            pos_b[0] = target_place[0]
            pos_b[1] = target_place[1]            
            p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
            p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
            p_safe_b[0] += -30
            p_safe_b[1] += -40
            p_place_b[0] += -30
            p_place_b[1] += -40

        elif target == "22":
            
            pos_b[0] = target_place[0]
            pos_b[1] = target_place[1]            
            p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
            p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
            p_safe_b[0] += -68
            p_safe_b[1] += 130
            p_place_b[0] += -68
            p_place_b[1] += 130
            
            
        elif target == "02":
            
            pos_b[0] = target_place[0]
            pos_b[1] = target_place[1]            
            p_safe_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, SAFE_EXTRA_Z)
            p_place_b = self.get_smart_pose_mm(pos_b, YAW_B_DEG, extra_z_m=0.05)
            p_safe_b[0] += -35
            p_safe_b[1] += -35
            p_place_b[0] += -35
            p_place_b[1] += -35
            
            p_place_b[5] += -0.1
            p_safe_b[5] += -0.1
            
        else:
            
            p_safe_b[0] = target_place[0]*1000
            p_safe_b[1] = target_place[1]*1000
            p_place_b[0] = target_place[0]*1000
            p_place_b[1] = target_place[1]*1000
            
        print(p_safe_b)
        print(p_place_b)
        
        p_place_b[2] = 180
        p_safe_b[2] = 250
        self.call_hand("open")
        self.move_linear_with_wait(p_safe_a, "1. Entry to A") 
        self.move_linear_with_wait(p_pick_a, "2. Pick A")
        self.call_hand("close") 
        
        self.move_linear_with_wait(p_safe_a, "3. Lifting")
        self.move_linear_with_wait(p_safe_b, "4. Transiting to B") 
        
        # Place at fixed target position
        self.move_linear_with_wait(p_place_b, "5. Place B")
        self.call_hand("open") 
        
        
        self.move_linear_with_wait(p_safe_b, "6. Exiting")
        req = MoveRequest(pose=HOME_JOINTS, mvvelo=JOINT_SPEED_RAD, mvacc=JOINT_ACC_RAD, coord_mode=0)
        self.joint_client(req)
        print("\n=== DONE ===\n")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--place_code", type=str, default="11", help="Target Grid ID (e.g. 00, 11)")
    parser.add_argument("--pick_x", type=float, help="Pick X coordinate (m)")
    parser.add_argument("--pick_y", type=float, help="Pick Y coordinate (m)")
    parser.add_argument("--place_x", type=float, help="Place X coordinate (m)")
    parser.add_argument("--place_y", type=float, help="Place Y coordinate (m)")
    # 接收参数
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle (degrees)")
    
    args, unknown = parser.parse_known_args()

    current_pick_pos = [args.pick_x, args.pick_y, POS_A[2]] if args.pick_x else POS_A
    current_place_pos = [args.place_x, args.place_y, POS_B[2]] if args.place_x else POS_B
    
    # ✨ 直接读取变量，不再依赖任何全局默认值
    received_yaw = args.yaw
    received_code = args.place_code
    
    print(f"🛠️ [DEBUG] Received YAW from Driver: {received_yaw}")

    try: 
        app = JakaFreeYawPnP()
        app.run(current_pick_pos, received_code, received_yaw)
    except rospy.ROSInterruptException: 
        pass