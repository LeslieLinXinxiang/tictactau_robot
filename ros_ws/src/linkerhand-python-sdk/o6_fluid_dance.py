import sys
import os
import time
import math

# 1. SDK 路径注入 (请确认路径与你之前的环境一致)
SDK_PATH = "/home/leslie/ros_ws/src/linkerhand-python-sdk"
sys.path.append(SDK_PATH)
from LinkerHand.linker_hand_api import LinkerHandApi

# ==========================================
# 2. 舞蹈参数配置 (0-255 刻度)
# ==========================================

# 关节映射: [J1(拇指弯), J2(拇指旋), J3(食指), J4(中指), J5(无名), J6(小指)]
# 我们定义每一个关节的摆动区间 [最小值, 最大值]
RANGES = {
    'J1': [140, 255],  # 拇指弯曲
    'J2': [48,  80],   # 拇指旋转 (增加侧向灵动感)
    'J3': [160, 255],  # 食指
    'J4': [160, 255],  # 中指
    'J5': [160, 255],  # 无名指
    'J6': [160, 255],  # 小指
}

# 阶梯相位差 (单位: 弧度)
# 选项 B: 小指先行 -> ... -> 拇指收尾
# 我们让小指相位为 0，每向前一根手指延迟 0.5 弧度
PHASE_OFFSETS = [
    2.0,  # J1 拇指弯 (最后动)
    2.0,  # J2 拇指旋 (与 J1 同步)
    1.5,  # J3 食指
    1.0,  # J4 中指
    0.5,  # J5 无名指
    0.0,  # J6 小指 (最先动)
]

TEMPO = 0.8  # 频率: 数值越小越慢 (0.5-0.8 适合慢节奏)

def fluid_dance():
    print("="*50)
    print("  Linker Hand O6 'Zen Breathing' Dance")
    print("  Style: Fluid Cascading (Pinky to Thumb)")
    print("  Press [Ctrl+C] to Stop Safe")
    print("="*50)

    try:
        # 初始化
        hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
        hand.set_enable()
        time.sleep(1)

        start_time = time.time()
        
        while True:
            t = (time.time() - start_time) * TEMPO
            current_pose = []

            # 为 6 个关节计算当前的正弦位置
            for i in range(6):
                joint_key = f'J{i+1}'
                min_val, max_val = RANGES[joint_key]
                phase = PHASE_OFFSETS[i]
                
                # 正弦波映射到 [min, max] 空间
                # (sin + 1) / 2 将 [-1, 1] 转为 [0, 1]
                val = min_val + (max_val - min_val) * (math.sin(t - phase) + 1.0) / 2.0
                current_pose.append(int(val))

            # 发送指令 (高速插帧)
            hand.finger_move(current_pose)
            
            # 20Hz 控制频率
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n>>> [System] 停止舞蹈，正在恢复初始位...")
        try:
            hand.finger_move([255, 48, 255, 255, 255, 255])
            time.sleep(1)
            hand.set_disable()
        except: pass
        print(">>> [System] 已安全退出。")

if __name__ == "__main__":
    fluid_dance()
