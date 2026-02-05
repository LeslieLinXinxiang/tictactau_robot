import sys
import os
import time
import termios
import tty

# 1. SDK 路径补全
abs_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abs_path)
from LinkerHand.linker_hand_api import LinkerHandApi

# === 你校准好的右手 0-255 刻度 ===
# [拇指旋, 拇指弯, 食指弯, 中指弯, 无名, 小指]
HAND_OPEN  = [255, 48, 255, 255, 255, 255]
HAND_CLOSE = [140, 48, 160, 160, 255, 255] 

def getch():
    """读取单个字符而不回显，不需要按回车"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def interactive_control():
    print("="*50)
    print("  Linker Hand O6 交互式调试工具 (0-255 Mode)")
    print("  [Space] : 切换 开/合 状态")
    print("  [E]     : 紧急断电并退出")
    print("="*50)

    try:
        # 初始化
        hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
        hand.set_enable()
        time.sleep(1)
        
        # 初始状态设为张开
        is_open = True
        hand.finger_move(HAND_OPEN)
        print(">>> [Status] 初始状态：已张开")

        while True:
            char = getch()
            
            if char == ' ':  # 空格键
                if is_open:
                    print(f">>> [Action] 执行：闭合 {HAND_CLOSE}")
                    hand.finger_move(HAND_CLOSE)
                    is_open = False
                else:
                    print(f">>> [Action] 执行：张开 {HAND_OPEN}")
                    hand.finger_move(HAND_OPEN)
                    is_open = True
            
            elif char.lower() == 'e':  # E 键退出
                print("\n>>> [System] 正在断电并安全退出...")
                hand.set_disable()
                break

    except Exception as e:
        print(f"\n>>> [Error] 发生错误: {e}")
    finally:
        print(">>> [System] 交互结束。")

if __name__ == "__main__":
    interactive_control()
