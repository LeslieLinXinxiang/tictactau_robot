#!/usr/bin/env python3
import sys
import os
import time
import argparse

# 路径自举
ABS_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ABS_PATH)
try:
    from LinkerHand.linker_hand_api import LinkerHandApi
except ImportError:
    sys.exit(1)

# === 0-255 刻度配置 ===
HAND_OPEN  = [255, 0, 255, 255, 255, 255]
HAND_CLOSE = [140, 0, 160, 160, 160, 160]
TOLERANCE  = 20
THUMB_INDEX = 0

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['open', 'close', 'check'], help="动作类型")
    args = parser.parse_args()

    try:
        # 1. 建立连接
        hand = LinkerHandApi(hand_type="right", hand_joint="O6", can="can0")
        
        if args.action == 'check':
            hand.set_enable()
            print("OK")
            return

        # 2. 发送指令前确保使能
        hand.set_enable()
        time.sleep(0.2)

        target = []
        label = ""
        if args.action == 'open':
            target = HAND_OPEN
            label = "OPEN"
        elif args.action == 'close':
            target = HAND_CLOSE
            label = "CLOSE"

        # 3. 发送动作指令
        hand.finger_move(target)
        
        # 4. === 关键修复：强制阻塞等待 ===
        # 先死等 1.0 秒，让机械结构飞一会儿
        time.sleep(1.0) 
        
        # 5. 闭环检查 (最多再等 2 秒)
        start_t = time.time()
        final_status = "TIMEOUT"
        
        while time.time() - start_t < 2.0:
            states = hand.get_state()
            if states and len(states) > 0:
                curr = states[THUMB_INDEX]
                # 检查是否进入误差带
                if abs(curr - target[THUMB_INDEX]) < TOLERANCE:
                    final_status = "REACHED"
                    break
            time.sleep(0.1)
            
        # 打印结果供主程序读取
        print(f"DONE:{final_status}")

    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
