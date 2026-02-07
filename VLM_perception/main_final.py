import pyrealsense2 as rs
import numpy as np
import cv2
import json
import re
import dashscope
from dashscope import MultiModalConversation
import time
import os
from datetime import datetime
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from collections import deque

# ==========================================
# 1. 配置区域
# ==========================================
dashscope.api_key = "sk-605ea3cbaae74c45a720b88e23ceab06"

LATEST_CALIB = {
    "translation": [-384.1665, 66.0822, 962.9691],
    "quaternion": [-0.6232, -0.7818, 0.0131, 0.0157]
}

SHARE_DIR = "/app/share_data"
JSON_FILE = "robot_cmd.json"
JSON_STATUS_FILE = "robot_status.json"
CONFIG_FILE = "board_config.json"
VLM_LOG_DIR = "/home/congquan/wytdino+sam+vla/VLM_interaction"

BLUE_LOWER = np.array([100, 100, 50]); BLUE_UPPER = np.array([130, 255, 255])
RED_LOWER1 = np.array([0, 100, 70]); RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 100, 70]); RED_UPPER2 = np.array([180, 255, 255])

STABILITY_TIME = 1.5  
CHANGE_THRESHOLD = 500
VERIFY_DIFF_TH = 1500  

# ==========================================
# 2. 核心系统类
# ==========================================
class VisualRobotAgent:
    def __init__(self):
        rot_matrix = R.from_quat(LATEST_CALIB["quaternion"]).as_matrix()
        self.T_CAM2BASE = np.eye(4)
        self.T_CAM2BASE[:3, :3] = rot_matrix
        self.T_CAM2BASE[:3, 3] = LATEST_CALIB["translation"]

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        self.profile = self.pipeline.start(self.config)
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.roi_board = None
        self.roi_blue_stock = None
        self.grid_cells = {} 
        
        self.prev_gray = None           
        self.last_move_time = time.time()
        self.last_player_piece_count = 0 
        self.base_depth_m = 0.95 

        self.yaw_buffer = deque(maxlen=10)
        self.pre_move_board_img = None 
        self.last_logic_rc = (None, None) 

        if not os.path.exists(VLM_LOG_DIR):
            os.makedirs(VLM_LOG_DIR, exist_ok=True)

    def get_robot_pos(self, u, v, depth_frame):
        u, v = int(u), int(v)
        dist = depth_frame.get_distance(u, v)
        if dist <= 0: dist = self.base_depth_m
        p_cam = rs.rs2_deproject_pixel_to_point(self.intr, [u, v], dist)
        p_cam_mm = np.array([p_cam[0]*1000, p_cam[1]*1000, p_cam[2]*1000, 1.0])
        p_base = self.T_CAM2BASE @ p_cam_mm
        return [round(p_base[0], 2), round(p_base[1], 2), round(p_base[2], 2)]

    def check_robot_status(self):
        status_path = os.path.join(SHARE_DIR, JSON_STATUS_FILE)
        if not os.path.exists(status_path): return False
        try:
            with open(status_path, 'r') as f:
                return json.load(f).get("status") == "idle"
        except: return False

    def load_calibration(self):
        config_path = os.path.join(SHARE_DIR, CONFIG_FILE)
        if not os.path.exists(config_path): config_path = CONFIG_FILE 

        if not os.path.exists(config_path):
            print(f"❌ [Error] Config file not found: {config_path}")
            return False
        
        try:
            with open(config_path, 'r') as f:
                data = json.load(f)
            self.roi_blue_stock = tuple(data["stock_roi"])
            self.roi_board = tuple(data["board_roi"])
            self.grid_cells = {}
            for k, v in data["grid"].items():
                r, c = map(int, k.split(','))
                self.grid_cells[(r, c)] = {"robot": v}
            print(f"✅ Loaded Config from {config_path}")
            return True
        except Exception as e:
            print(f"❌ [Error] Failed to load config: {e}")
            return False

# ...existing code...
    def ask_ai_pure_vision(self, full_img):
        """✨ 修改：增加 game_over 状态识别"""
        if self.roi_board is None: return None, None, False # 增加 False
        bx, by, bw, bh = self.roi_board
        board_crop = full_img[by:by+bh, bx:bx+bw]
        
        self.pre_move_board_img = cv2.cvtColor(board_crop, cv2.COLOR_BGR2GRAY)
        
        # ==========================================
        # 📸 [显式保存] 图片保存逻辑
        # ==========================================
        if not os.path.exists(VLM_LOG_DIR):
            os.makedirs(VLM_LOG_DIR, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. 保存传给 VLM 的【局部裁剪图】 (模型看到的就是这张)
        save_img_path = os.path.join(VLM_LOG_DIR, f"{ts}_input_crop.jpg")
        cv2.imwrite(save_img_path, board_crop)
        
        # 2. 保存【原始全景图】 (方便您回顾当时的整体环境)
        save_full_path = os.path.join(VLM_LOG_DIR, f"{ts}_input_full.jpg")
        cv2.imwrite(save_full_path, full_img)
        
        print(f"📸 [Snapshot] Saved Crop: {os.path.basename(save_img_path)} | Full: {os.path.basename(save_full_path)}")
        # ==========================================

        # ✨ 1. 修改 Prompt，增加 game_over 字段要求
        prompt_text = (
            "Role: Tic-Tac-Toe Supreme Referee & Player (Side: BLUE).\n"
            "Task: Analyze the board image accurately based on physical geometry.\n\n"
            "VISUAL RULES (Strict):\n"
            "1. GRID: 3x3 matrix. Rows 0-2 (Top-Down), Cols 0-2 (Left-Right).\n"
            "2. RED CUBES: Opponent pieces. These locations are FORBIDDEN.\n"
            "3. BLUE CUBES: Your pieces.\n"
            "4. EMPTY WHITE SQUARES: The ONLY valid move targets.\n\n"
            "EXECUTION PROTOCOL:\n"
            "Phase 1 [AUDIT]: List coordinates of ALL Red pieces and ALL Blue pieces.\n"
            "Phase 2 [LEGALITY]: Calculate the list of EMPTY coordinates. \n"
            "   >>> CRITICAL: If a coordinate appears in Phase 1, it CANNOT be in Phase 2.\n"
            "Phase 3 [REFEREE]: Check specifically for 3-in-a-row (Row/Col/Diag).\n"
            "   - If 3 Red: Winner=Red, GameOver=True.\n"
            "   - If 3 Blue: Winner=Blue, GameOver=True.\n"
            "   - If No Empty Squares: Winner=Draw, GameOver=True.\n"
            "Phase 4 [ACTION]:\n"
            "   - If GameOver=True: Set row=-1, col=-1.\n"
            "   - If GameOver=False: Pick ONE coordinate from the [LEGALITY] list in Phase 2.\n"
            "     (Prioritize: Blocking Red > Winning > Center > Corner).\n\n"
            "RESPONSE FORMAT (Strict JSON):\n"
            "{\n"
            "  \"observation_red_coords\": [[r,c], ...],\n"
            "  \"observation_blue_coords\": [[r,c], ...],\n"
            "  \"valid_empty_slots\": [[r,c], ...],  <-- MUST calculate this first!\n"
            "  \"game_over\": true/false,\n"
            "  \"winner\": \"red\"/\"blue\"/\"draw\"/null,\n"
            "  \"reason\": \"Step-by-step logic...\",\n"
            "  \"row\": [0-2] or -1,\n"
            "  \"col\": [0-2] or -1\n"
            "}"
        )

        messages = [{'role': 'user', 'content': [{'image': Path(save_img_path).as_uri()}, {'text': prompt_text}]}]
        try:
            response = MultiModalConversation.call(model='qwen-vl-max', messages=messages)
            if response.status_code == 200:
                content = response.output.choices[0].message.content
                if isinstance(content, list): content = content[0]['text']
                print(f"🤖 VLM Thinking:\n{content}\n")
                
                match = re.search(r'\{.*\}', content, re.DOTALL)
                if match:
                    res = json.loads(match.group())
                    # ✨ 2. 解析新增字段
                    r = int(res.get("row", -1))
                    c = int(res.get("col", -1))
                    is_over = res.get("game_over", False)
                    return r, c, is_over
        except Exception as e:
            print(f"❌ [VLM Error] {e}")
        return None, None, False
    
    def initialize(self):
        print("\n--- [System Startup] ---")
        if not self.load_calibration():
            raise RuntimeError("Calibration missing")
        for _ in range(15): self.pipeline.wait_for_frames()
        frames = self.pipeline.wait_for_frames()
        depth_f = self.align.process(frames).get_depth_frame()
        bx, by, bw, bh = self.roi_board
        self.base_depth_m = depth_f.get_distance(bx + bw//2, by + bh//2)
        if self.base_depth_m <= 0: self.base_depth_m = 0.96 
        print(f"✅ Base Depth: {self.base_depth_m:.3f}m")

    def run(self):
        try:
            self.initialize()
        except RuntimeError: return

        needs_retry = False 

        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_f = self.align.process(frames).get_color_frame()
                depth_f = self.align.process(frames).get_depth_frame()
                if not color_f: continue
                img = np.asanyarray(color_f.get_data())
                
                is_idle = self.check_robot_status()
                trigger = False

                # 1. 储备区检测
                hsv_full = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                bm = cv2.inRange(hsv_full, BLUE_LOWER, BLUE_UPPER)
                sx, sy, sw, sh = self.roi_blue_stock
                stock_mask = np.zeros_like(bm); stock_mask[sy:sy+sh, sx:sx+sw] = bm[sy:sy+sh, sx:sx+sw]
                cnts_stock, _ = cv2.findContours(stock_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                target_piece = None
                if cnts_stock:
                    piece_candidates = []
                    for c in cnts_stock:
                        if cv2.contourArea(c) < 400: continue
                        M = cv2.moments(c)
                        piece_candidates.append({"cx": M["m10"]/M["m00"], "cy": M["m01"]/M["m00"], "cnt": c})
                    if piece_candidates:
                        piece_candidates.sort(key=lambda p: (-p["cx"], -p["cy"]))
                        target_piece = piece_candidates[0]
                        rect = cv2.minAreaRect(target_piece["cnt"])
                        raw_angle = rect[2]
                        w, h = rect[1]
                        if w < h: raw_angle -= 90
                        clean_yaw = (raw_angle % 45) - 45
                        self.yaw_buffer.append(clean_yaw)

                # 2. 状态监测
                if is_idle:
                    if self.pre_move_board_img is not None:
                        bx, by, bw, bh = self.roi_board
                        current_board_gray = cv2.cvtColor(img[by:by+bh, bx:bx+bw], cv2.COLOR_BGR2GRAY)
                        diff = cv2.absdiff(self.pre_move_board_img, current_board_gray)
                        diff_score = cv2.countNonZero(cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)[1])
                        if diff_score < VERIFY_DIFF_TH:
                            needs_retry = True; trigger = True
                        self.pre_move_board_img = None

                    if not needs_retry:
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        if self.prev_gray is None: self.prev_gray = gray; continue
                        diff_mon = cv2.absdiff(self.prev_gray, gray)
                        change_score = cv2.countNonZero(cv2.threshold(diff_mon, 25, 255, cv2.THRESH_BINARY)[1])
                        if change_score > CHANGE_THRESHOLD: self.last_move_time = time.time()
                        elif (time.time() - self.last_move_time) > STABILITY_TIME:
                            bx, by, bw, bh = self.roi_board
                            hsv_b = cv2.cvtColor(img[by:by+bh, bx:bx+bw], cv2.COLOR_BGR2HSV)
                            red_mask = cv2.bitwise_or(cv2.inRange(hsv_b, RED_LOWER1, RED_UPPER1), cv2.inRange(hsv_b, RED_LOWER2, RED_UPPER2))
                            red_cnt = len(cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
                            if red_cnt > self.last_player_piece_count:
                                trigger = True; self.last_player_piece_count = red_cnt
                        self.prev_gray = gray

                if cv2.waitKey(1) & 0xFF == ord('p'): trigger = True

                if trigger:
                    print("\n🚀 [Action] Generating Robot Commands...")
                    if target_piece:
                        pick_xyz = self.get_robot_pos(target_piece["cx"], target_piece["cy"], depth_f)
                        final_yaw = sum(self.yaw_buffer) / len(self.yaw_buffer) if self.yaw_buffer else 0.0
                        
                        if needs_retry:
                            r, c = self.last_logic_rc
                            needs_retry = False
                            is_over = False # 重试默认游戏未结束
                        else:
                            # ✨ 3. 接收 game_over 信号
                            r, c, is_over = self.ask_ai_pure_vision(img)
                            self.last_logic_rc = (r, c)

                        # ✨ 4. 判定结束
                        if is_over:
                            print("\n🏁 [GAME OVER] The AI decided the game has ended. Stopping script.")
                            break # 跳出 While 循环，结束脚本

                        if r is not None and c is not None and r != -1:
                            # 校验坐标有效性
                            if (r, c) in self.grid_cells:
                                # place_xyz = self.grid_cells[(r, c)]["robot"] # 不再需要物理坐标转换
                                grid_id = f"{r}{c}" # 组合成 "00", "01" ... "22" 格式
                                
                                cmd = {
                                    "task_id": int(time.time()),
                                    "data": {
                                        "pick": {"x": pick_xyz[0], "y": pick_xyz[1], "z": 40.0, "yaw": round(float(final_yaw), 2)},
                                        "place": grid_id 
                                    }
                                }
                                with open(os.path.join(SHARE_DIR, JSON_FILE), 'w') as f:
                                    json.dump(cmd, f, indent=4)
                                print(f"📨 [Command] Sent! Move to Grid {grid_id} | Yaw: {final_yaw:.2f}")
                                time.sleep(1.0)
                            else:
                                print(f"❌ [Error] Invalid Coordinates ({r},{c})")
                    else:
                        print("⚠️ No Blue piece found.")
                        needs_retry = False

                if self.roi_board:
                    bx, by, bw, bh = self.roi_board
                    cv2.rectangle(img, (bx, by), (bx+bw, by+bh), (0, 255, 0), 2)
                if self.roi_blue_stock:
                    sx, sy, sw, sh = self.roi_blue_stock
                    cv2.rectangle(img, (sx, sy), (sx+sw, sy+sh), (255, 0, 0), 2)

                cv2.imshow("MENACE AI - Production", img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
        finally:
            self.pipeline.stop(); cv2.destroyAllWindows()

if __name__ == "__main__":
    agent = VisualRobotAgent(); agent.run()