import sys
import os
import time

abs_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abs_path)

from LinkerHand.linker_hand_api import LinkerHandApi

# 增加一点闭合幅度，确保肉眼可见 (假设是弧度)
# 1.0 rad 约等于 57度
HAND_OPEN  = [255, 48, 255, 255, 255, 255]
HAND_CLOSE = [140, 48, 160, 160, 255, 255] 

def test_hand():
    print(">>> [System] 正在初始化 Linker Hand O6 (右手)...")
    try:
        hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
        
        # --- 关键步骤：开启电机使能 ---
        print(">>> [System] 正在发送使能指令 (set_enable)...")
        hand.set_enable() 
        time.sleep(1) # 给刹车松开一点时间

        print("\n--- 执行循环测试 ---")
        
        for i in range(2): # 循环两次
            print(f">>> [Action] 闭合中... {HAND_CLOSE}")
            hand.finger_move(HAND_CLOSE)
            time.sleep(2)
            
            print(f">>> [Action] 张开中... {HAND_OPEN}")
            hand.finger_move(HAND_OPEN)
            time.sleep(2)

        print("\n>>> [Success] 测试完成。")

    except Exception as e:
        print(f"\n>>> [Error] 运行出错: {e}")
    finally:
        # 绕过官方 SDK 的 close_can Bug
        print(">>> [System] 正在清理退出...")
        try:
            hand.set_disable() # 退出前断电保护
            # 不直接调用 hand.close_can() 以免触发那个 NameError
        except: pass

if __name__ == "__main__":
    test_hand()
