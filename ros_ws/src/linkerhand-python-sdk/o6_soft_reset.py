import sys
import os
import time

abs_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abs_path)
from LinkerHand.linker_hand_api import LinkerHandApi

def soft_reset():
    print(">>> [System] 启动软复位程序 (无需断电)...")
    try:
        # 1. 强制初始化 API
        hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
        
        # 2. 强制断电 (Disable)，释放可能存在的自锁力矩
        print(">>> [Step 1] 正在释放电机使能...")
        hand.set_disable()
        time.sleep(1.0)
        
        # 3. 清除内部固件错误标志位
        print(">>> [Step 2] 正在清除寄存器故障 (clear_faults)...")
        hand.clear_faults()
        time.sleep(1.0)
        
        # 4. 重新使能
        print(">>> [Step 3] 重新开启使能 (set_enable)...")
        hand.set_enable()
        time.sleep(1.0)
        
        # 5. 设置一个极慢的速度进行回位，防止二次撞击
        # O6 有 6 个电机，尝试设置低速 (0-100 范围，设为 20)
        print(">>> [Step 4] 设置低速安全模式...")
        try:
            hand.set_joint_speed([20]*6) 
        except: pass
        
        # 6. 发送张开位置的中间安全值 (根据你的 YAML, 尝试发 200 左右)
        # 这是一个保守的中位值，确保它能从极限位置拉回来
        SAFE_MID_POS = [200, 150, 200, 200, 200, 200]
        print(f">>> [Step 5] 尝试缓慢移动至安全位: {SAFE_MID_POS}")
        hand.finger_move(SAFE_MID_POS)
        
        time.sleep(3)
        print(">>> [Success] 软复位完成！请观察手指是否已离开物理限位。")

    except Exception as e:
        print(f">>> [Error] 复位失败: {e}")

if __name__ == "__main__":
    soft_reset()
