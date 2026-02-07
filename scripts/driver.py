#!/usr/bin/env python3
import json
import os
import time
import sys
import subprocess

# ==========================================
# 配置区域
# ==========================================
JSON_CMD_PATH = "/root/share_data/robot_cmd.json"
JSON_STATUS_PATH = "/root/share_data/robot_status.json" 
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
EXEC_SCRIPT_PATH = os.path.join(CURRENT_DIR, "real_tictactau_exec.py")

def update_status(status_str, last_id):
    try:
        status_data = {"status": status_str, "last_task_id": last_id, "timestamp": time.time()}
        with open(JSON_STATUS_PATH, "w") as f: json.dump(status_data, f)
        print(f"📊 [Status] {status_str} | ID: {last_id}")
    except Exception as e:
        print(f"❌ Status write error: {e}")

def driver_loop():
    print(f"\n🚗 [Driver] Listening...")
    last_processed_id = 0
    if os.path.exists(JSON_CMD_PATH):
        try:
            with open(JSON_CMD_PATH, "r") as f:
                last_processed_id = json.load(f).get("task_id", 0)
        except: pass

    update_status("idle", last_processed_id)

    while True:
        try:
            if os.path.exists(JSON_CMD_PATH):
                with open(JSON_CMD_PATH, "r") as f:
                    try: cmd = json.load(f)
                    except: time.sleep(0.1); continue

                task_id = cmd.get("task_id", 0)

                if task_id > last_processed_id:
                    print(f"\n⚡ [New Task] ID: {task_id}")
                    update_status("busy", last_processed_id)

                    data = cmd.get("data", {})
                    try:
                        pick_x = float(data["pick"]["x"]) / 1000.0
                        pick_y = float(data["pick"]["y"]) / 1000.0
                        # ✨ 关键：提取 yaw
                        pick_yaw = float(data["pick"].get("yaw", 0.0))
                        
                        # ✨ 处理 place: 无论它是字符串代码 ("12") 还是 对象 ({x:..})，都转为字符串代码
                        # 如果是坐标对象，这里需要额外的逻辑处理吗？根据需求，这里应该是代码
                        raw_place = data["place"]
                        if isinstance(raw_place, dict):
                            # 如果未来支持坐标，这里保留兼容，但目前 real_tictactau_exec 只接受 code
                            # 为了不报错，我们暂时假设它必须是 code
                            raise ValueError("Expected place code string, got dict")
                        
                        place_code = str(raw_place)

                    except Exception as e:
                        print(f"❌ Data error: {e}"); last_processed_id = task_id; update_status("idle", last_processed_id); continue

                    # ✨ 关键：增加 --yaw 参数，把 --place_x/y 替换为 --place_code
                    cmd_args = ["python3", EXEC_SCRIPT_PATH, 
                                "--pick_x", str(pick_x), "--pick_y", str(pick_y),
                                "--place_code", place_code,
                                "--yaw", str(pick_yaw)]
                    
                    subprocess.run(cmd_args)
                    last_processed_id = task_id
                    update_status("idle", last_processed_id)
            time.sleep(0.5)
        except KeyboardInterrupt: sys.exit(0)
        except Exception as e: print(f"❌ Error: {e}"); time.sleep(1)

if __name__ == "__main__":
    driver_loop()