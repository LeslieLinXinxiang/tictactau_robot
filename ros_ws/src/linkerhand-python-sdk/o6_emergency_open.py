import sys
import os
import time

abs_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abs_path)

from LinkerHand.linker_hand_api import LinkerHandApi

def emergency_open():
    print(">>> [System] 正在执行紧急张开程序...")
    try:
        # 尝试以右手 O6 初始化（既然刚才动了，说明 ID 已经对上了）
        # 如果这个不动，就换成 hand_type="left"
        hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
        
        print(">>> [Action] 发送使能并归零...")
        hand.set_enable()
        time.sleep(0.5)
        
        # 发送全 0 指令 (回到初始张开位置)
        hand.finger_move([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        print(">>> [Action] 张开指令已发送！")
        
        time.sleep(2)
        hand.set_disable() # 动作完成后断电，防止电机持续堵转
        print(">>> [System] 电机已断电保护。")

    except Exception as e:
        print(f">>> [Error] {e}")

if __name__ == "__main__":
    emergency_open()
