import cv2
import json
import os
import time

# ================= 配置区域 =================
# 你的摄像头 ID 列表 (根据你的实际情况修改)
CAM_IDS = [0, 1, 2] 

# 图片保存路径
SAVE_DIR = "dataset_captured"

# 状态文件路径 (读取电机当前的位置)
STATE_FILE = "scan_progress.json"
# ===========================================

def load_state():
    if not os.path.exists(STATE_FILE):
        print(f"[错误] 找不到 {STATE_FILE}！")
        print("请先运行 run_step_scan.py 让机器人运动到位。")
        return None
    try:
        with open(STATE_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"[错误] 读取 JSON 失败: {e}")
        return None

def main():
    # 1. 读取当前机器人状态
    state = load_state()
    if state is None: 
        return

    # 从 JSON 里提取信息
    # next_step_idx 是下一步的索引，所以当前刚刚完成的是 next_step_idx - 1
    current_idx = state.get("next_step_idx", 1) - 1
    combo = state.get("last_combo", [])
    
    if not combo:
        print("[警告] JSON 中没有 last_combo 信息，无法生成文件名。")
        return

    # 构造文件名前缀: step_005_0_260_520
    # 格式: step_{索引}_{电机7角度}_{电机8角度}_{电机9角度}
    combo_str = "_".join(map(str, combo))
    filename_prefix = f"step_{current_idx:03d}_{combo_str}"
    
    print(f"--- 准备拍照: {filename_prefix} ---")
    
    # 2. 确保保存目录存在
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)

    # 3. 串行拍照 (逐个打开，拍完就关，节省 USB 带宽)
    for cam_id in CAM_IDS:
        print(f"正在尝试打开摄像头 ID: {cam_id} ... ", end="")
        
        # 使用 CAP_DSHOW (DirectShow) 在 Windows 上启动通常更快
        cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
        
        if not cap.isOpened():
            print("失败!")
            continue

        # 设置分辨率 (可选，如果不设置则使用默认)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # 【防黑屏策略】虽然你说不用预热，但为了防止第一帧全黑或自动曝光未完成
        # 建议至少空读 1-2 帧。这只需要几十毫秒。
        cap.read() 
        
        # 正式读取
        ret, frame = cap.read()
        
        if ret:
            save_path = os.path.join(SAVE_DIR, f"{filename_prefix}_cam{cam_id}.jpg")
            cv2.imwrite(save_path, frame)
            print(f"成功 -> 保存为: ..._cam{cam_id}.jpg")
        else:
            print("读取帧失败 (空帧)!")

        # 【关键】立刻释放摄像头，把 USB 资源让给下一个
        cap.release()
        
        # 小睡一下，防止 USB 总线瞬间切换过快导致驱动崩溃
        time.sleep(0.2)

    print("=== 本组拍摄完成 ===")

if __name__ == "__main__":
    main()