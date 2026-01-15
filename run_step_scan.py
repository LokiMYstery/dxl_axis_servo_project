import time
import json
import os
import itertools
import numpy as np
import sys
from servo_controller import load_config, AxisServo

# ================= 配置区域 =================
STATE_FILE = "scan_progress.json" # 用于存储进度和零点文件
TARGET_IDS = [7, 8, 9]            # 确保只操作这三个电机
POINTS_PER_AXIS = 4               # 每个轴遍历4个点
MAX_DEG = 780                     # 最大角度

# 生成 4 个采样点: [0, 260, 520, 780]
POSITIONS = np.linspace(0, MAX_DEG, POINTS_PER_AXIS).astype(int).tolist()
# ===========================================

def load_state():
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, 'r') as f:
                return json.load(f)
        except Exception:
            return None
    return None

def save_state(state):
    with open(STATE_FILE, 'w') as f:
        json.dump(state, f, indent=2)

def main():
    # 1. 生成所有目标组合 (64种组合)
    combinations = list(itertools.product(POSITIONS, repeat=len(TARGET_IDS)))
    total_steps = len(combinations)
    
    # 2. 读取当前状态
    state = load_state()
    
    # 初始化变量
    current_step_idx = 0
    home_offsets = {}

    # 3. 连接电机
    try:
        cfg = load_config("config.json")
        servo = AxisServo(cfg)
        servo.connect()
    except Exception as e:
        print(f"[ERROR] 无法连接电机: {e}")
        print("请检查 USB 连接或电源。")
        return

    print(f"--- 步进式扫描控制器 (总计 {total_steps} 步) ---")

    # ================= 状态分支处理 =================
    
    # 分支 A: 第一次运行 (没有状态文件)
    if state is None:
        print("[初始化] 检测到首次运行...")
        
        # 确保进入多圈模式
        servo.ensure_extended_mode(TARGET_IDS)
        
        # 读取当前位置作为【永久零点】
        print("正在读取当前位置作为由于零点 (Home Offsets)...")
        start_pos, _ = servo.bulk_read(want_current=False)
        home_offsets = {str(mid): start_pos[mid] for mid in TARGET_IDS} # JSON key 必须是字符串
        
        print(f"零点已锁定: {home_offsets}")
        print("请勿断电或移动电机位置，直到整个扫描完成！")
        
        current_step_idx = 0 # 准备执行第 0 步

    # 分支 B: 后续运行 (读取状态)
    else:
        current_step_idx = state.get("next_step_idx", 0)
        home_offsets = state.get("home_offsets", {})
        
        if not home_offsets:
            print("[错误] 状态文件损坏，丢失零点信息！")
            print(f"请删除 {STATE_FILE} 并将机器人复位后重新开始。")
            return

        print(f"[进度] 准备执行第 {current_step_idx + 1} / {total_steps} 步")

    # ================= 检查是否完成 =================
    if current_step_idx >= total_steps:
        print("\n=== 恭喜！所有遍历已完成 ===")
        user_input = input("是否要回到零点？(y/n): ")
        if user_input.lower() == 'y':
            print("正在归零...")
            # 归零逻辑：直接去 Offset 的位置
            for mid_str, offset_val in home_offsets.items():
                servo.cmd_ticks[int(mid_str)] = offset_val
            servo.execute_time_synced(duration_s=3.0)
            print("已归零。你可以删除 json 文件来开始新的测试。")
        return

    # ================= 执行运动 =================
    
    # 获取当前的目标组合
    target_combo = combinations[current_step_idx]
    print(f"\n>>> 正在移动到目标: {target_combo} (度)")
    
    # 计算绝对 Ticks
    for i, mid in enumerate(TARGET_IDS):
        deg = target_combo[i]
        offset = home_offsets[str(mid)] # 从 JSON 读出来的是 int
        
        # 绝对位置 = 零点 + 角度转换
        target_tick = offset + servo.deg_to_ticks(deg)
        servo.cmd_ticks[mid] = target_tick

    # 发送指令 (给予充足的稳定时间)
    result = servo.execute_time_synced(
        duration_s=1.5, 
        poll_hz=50, 
        eps_deg=2.0, 
        settle_n=10
    )

    if result['settled']:
        print("[成功] 机器人已到位并稳定。")
    else:
        print("[警告] 机器人未完全稳定 (超时)，但已停止指令发送。")

    # ================= 保存进度 =================
    
    # 指向下一步
    next_idx = current_step_idx + 1
    
    new_state = {
        "next_step_idx": next_idx,
        "home_offsets": home_offsets,
        "last_combo": target_combo
    }
    save_state(new_state)
    
    print(f"\n=== 第 {current_step_idx + 1} 步执行完毕 ===")
    print(f"当前状态: {target_combo}")
    print(">> 现在你可以进行拍照了 <<")
    print(f"拍完后，请再次运行此脚本以执行第 {next_idx + 1} 步。")
    
    # 关闭连接，但扭矩保持 (Torque stays ON)
    servo.close()

if __name__ == "__main__":
    main()