import numpy as np
from scipy.spatial.transform import Rotation as R

# 定义你的 Marker 偏移 (相对于 hand_base_link)
# Q: "t(0.06 0.017 0.11) d(130 0 -1 0)d(-90 0 0 1)"
TCP_OFFSET_POS = [0.06, 0.017, 0.11]
# 计算旋转矩阵 (顺序: 绕-Y转130度 -> 绕Z转-90度)
r1 = R.from_euler('y', -130, degrees=True)
r2 = R.from_euler('z', -90, degrees=True)
# 组合旋转 (注意乘法顺序，取决于rai是内旋还是外旋，通常是 r1 * r2)
TCP_OFFSET_ROT = r1 * r2 

# 构建 4x4 变换矩阵 T_hand_tcp
T_hand_tcp = np.eye(4)
T_hand_tcp[:3, :3] = TCP_OFFSET_ROT.as_matrix()
T_hand_tcp[:3, 3] = TCP_OFFSET_POS

# 计算逆矩阵 T_tcp_hand (这就是我们要乘的 "代换量")
T_tcp_hand = np.linalg.inv(T_hand_tcp)

def get_wrist_target(object_pos, object_yaw_degrees):
    """
    输入: 物体的位置 [x, y, z] 和 偏航角 Yaw (度)
    输出: 机械臂手腕(Link_6)应该去的 [x, y, z, qx, qy, qz, qw]
    """
    # 1. 构建物体在世界坐标系下的矩阵 T_world_object
    # 假设物体 Z 轴朝上，只绕 Z 轴旋转 Yaw
    r_obj = R.from_euler('z', object_yaw_degrees, degrees=True)
    T_world_object = np.eye(4)
    T_world_object[:3, :3] = r_obj.as_matrix()
    T_world_object[:3, 3] = object_pos

    # 2. 核心代换: Wrist = Object * (TCP_to_Hand)
    # 意思是：把 TCP 对齐到物体上，倒推出 Hand 在哪
    T_world_hand = np.dot(T_world_object, T_tcp_hand)

    # 3. 提取结果
    target_pos = T_world_hand[:3, 3]
    target_quat = R.from_matrix(T_world_hand[:3, :3]).as_quat() # [x, y, z, w]
    
    return np.concatenate((target_pos, target_quat))

# === 使用示例 ===
# 假设 CV 看到方块在 [0.4, -0.2, 0.025], 歪了 45 度
# wrist_goal = get_wrist_target([0.4, -0.2, 0.025], 45)
# move_group.set_pose_target(wrist_goal)
# move_group.go()