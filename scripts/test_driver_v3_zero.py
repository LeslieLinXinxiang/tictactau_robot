#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest # <--- 导入 IK 服务
import time

# === 目标配置 ===
TARGET_CUBE_POS = [0.3, 0.3, 0.025]  

# === 偏移量 (保持你的设定) ===
SHIFT_X = 0.02
SHIFT_Y = -0.065
SHIFT_Z = 0.14

class MoveItSyncPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_sync_planner', anonymous=True)
        
        self.move_group = moveit_commander.MoveGroupCommander("jaka_zu3")
        self.cmd_pub = rospy.Publisher('/mujoco/target_joints', JointState, queue_size=10)
        
        # === 连接 IK 服务 (只用它算角度，不用它规划) ===
        print(">>> [Brain] Connecting to IK Service...")
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        print(">>> [Brain] IK Service Ready!")

    def get_tilt_target(self, obj_pos):
        pose = Pose()
        pose.position.x = obj_pos[0] + SHIFT_X
        pose.position.y = obj_pos[1] + SHIFT_Y
        pose.position.z = obj_pos[2] + SHIFT_Z
        
        # 姿态：180 + 50 = 230度
        tilt_angle = 180 + 50
        q = R.from_euler('x', tilt_angle, degrees=True).as_quat()
        pose.orientation.x = q[0]; pose.orientation.y = q[1]
        pose.orientation.z = q[2]; pose.orientation.w = q[3]
        return pose

    def solve_ik(self, pose):
        """调用底层服务求解逆运动学"""
        req = GetPositionIKRequest()
        req.ik_request.group_name = "jaka_zu3"
        req.ik_request.pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.pose_stamped.header.stamp = rospy.Time.now()
        req.ik_request.timeout = rospy.Duration(1.0)
        
        # 使用当前状态作为种子，防止解算出这一圈以外的怪异角度
        req.ik_request.robot_state = self.move_group.get_current_state()
        req.ik_request.avoid_collisions = True # 仍然要避障
        
        try:
            res = self.ik_service(req)
            if res.error_code.val == 1: # 1 = SUCCESS
                return res.solution.joint_state.position[:6] # 返回前6个关节 (手臂)
            else:
                print(f">>> [Brain] IK Failed with error code: {res.error_code.val}")
                return None
        except rospy.ServiceException as e:
            print(f">>> [Brain] Service call failed: {e}")
            return None

    def interpolate_joints(self, start_joints, end_joints, steps=100):
        """
        核心魔法：手动生成同步轨迹
        start_joints: 起始角度列表
        end_joints: 目标角度列表
        steps: 插补点数 (越多越慢)
        """
        trajectory = []
        start_arr = np.array(start_joints)
        end_arr = np.array(end_joints)
        
        # === 智能处理 360 度跳变 ===
        # 如果从 3.14 跳到 -3.14，实际只动了一点点，不能转大圈
        diff = end_arr - start_arr
        for i in range(len(diff)):
            if diff[i] > np.pi:
                end_arr[i] -= 2*np.pi
            elif diff[i] < -np.pi:
                end_arr[i] += 2*np.pi
        
        # 线性插值 (Linear Interpolation)
        # 这保证了所有关节 0% 同时开始，100% 同时结束
        for t in np.linspace(0, 1, steps):
            # 公式: q(t) = start + t * (end - start)
            q_t = start_arr + t * (end_arr - start_arr)
            trajectory.append(q_t)
            
        return trajectory, end_arr

    def run(self):
        time.sleep(1)
        
        print(">>> [Brain] 1. Calculating Target Pose...")
        goal_pose = self.get_tilt_target(TARGET_CUBE_POS)
        
        print(">>> [Brain] 2. Solving Inverse Kinematics (IK)...")
        target_joints = self.solve_ik(goal_pose)
        
        if target_joints is None:
            print(">>> [Brain] ERROR: Target is unreachable (IK Failed).")
            print("    Try reducing the tilt angle or moving the target closer.")
            return
            
        print(f">>> [Brain] IK Solution Found: {np.round(target_joints, 2)}")
        
        print(">>> [Brain] 3. Generating Synchronized Trajectory...")
        start_joints = self.move_group.get_current_joint_values()
        
        # 生成 200 个点的插值轨迹 (约 4秒)
        points, final_joints = self.interpolate_joints(start_joints, target_joints, steps=200)
        
        print(f">>> [Brain] 4. Streaming {len(points)} points (Synchronized)...")
        
        rate = rospy.Rate(50) 
        joint_names = self.move_group.get_active_joints()
        
        for q in points:
            if rospy.is_shutdown(): break
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = joint_names
            msg.position = q
            self.cmd_pub.publish(msg)
            rate.sleep()
            
        print(">>> [Brain] Motion Complete.")
        
        # 更新 MoveIt 内部状态，防止下一次规划报错
        # (虽然我们这次绕过了它，但保持状态同步是个好习惯)
        # self.move_group.go(final_joints, wait=False) 

if __name__ == '__main__':
    try:
        MoveItSyncPlanner().run()
    except rospy.ROSInterruptException: pass