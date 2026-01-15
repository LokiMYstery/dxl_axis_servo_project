import time
import csv
import os
import itertools
import numpy as np
from servo_controller import load_config, AxisServo
from camera_manager import CameraGroup

# ================= 配置区域 =================
# 定义当前要测试的单元ID (Unit 1)
TARGET_IDS = [1, 2, 3] 

# 定义采样点 (0度 到 780度)
# 方案A: 线性采样 (均匀)
# POSITIONS = np.linspace(0, 780, 6).astype(int).tolist() # [0, 156, 312, ...]

# 方案B: 非线性采样 (针对你说的"后面弯曲明显"，在后面采得更密)
# 例如: 0, 200, 400, 550, 650, 720, 780
POSITIONS = [0, 200, 400, 550, 650, 720, 780]

SAVE_DIR = "dataset_unit1_v1"
# ===========================================

def main():
    # 1. 初始化电机
    cfg = load_config("config.json")
    servo = AxisServo(cfg)
    servo.connect()
    servo.scan_ids()
    servo.ensure_extended_mode(TARGET_IDS)
    
    # 归零 (假设当前位置是松弛状态，设为0)
    # 也可以用 servo.soft_reset("to_present")
    print("Resetting coords to present (Zeroing)...")
    servo.soft_reset("to_present") 

    # 2. 初始化摄像头 (假设是 0, 1, 2 号)
    # 如果你的电脑有自带摄像头，USB摄像头可能是 1, 2, 3
    cams = CameraGroup(cam_indices=[0, 1, 2]) 

    # 3. 准备CSV记录数据
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
    
    csv_path = os.path.join(SAVE_DIR, "labels.csv")
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    # 写入表头: id_1_deg, id_2_deg, id_3_deg, img_cam0, img_cam1, img_cam2
    writer.writerow([f"motor_{i}_deg" for i in TARGET_IDS] + ["img_cam0", "img_cam1", "img_cam2"])

    # 4. 生成遍历列表 (笛卡尔积)
    # 如果每个电机有 7 个点，总共 7^3 = 343 个状态
    combinations = list(itertools.product(POSITIONS, repeat=len(TARGET_IDS)))
    print(f"Total scan points: {len(combinations)}")

    try:
        for i, combo in enumerate(combinations):
            print(f"\n--- Progress [{i+1}/{len(combinations)}] ---")
            print(f"Target (Deg): {combo}")

            # 4.1 发送运动指令
            targets = {motor_id: deg for motor_id, deg in zip(TARGET_IDS, combo)}
            
            # 使用绝对控制，确保位置准确
            servo.set_targets_abs_deg(targets)
            
            # 执行运动 (给予足够的时间稳定，这里设为 1.5秒移动，精度2度)
            # 根据软体机器人的晃动情况，可能需要加大 settle_n
            result = servo.execute_time_synced(
                duration_s=1.5, 
                poll_hz=50, 
                eps_deg=2.0, 
                settle_n=10  # 增加稳定检测次数，防止软体抖动
            )

            # 4.2 额外的物理稳定时间 (软体机器人特有的阻尼震荡)
            time.sleep(1.0) 

            # 4.3 拍照
            # 文件名前缀: step_001_100_200_300
            prefix = f"step_{i:04d}_{'_'.join(map(str, combo))}"
            saved_paths = cams.capture_and_save(SAVE_DIR, prefix)

            # 4.4 记录到CSV
            # 只记录相对路径，方便以后迁移文件夹
            rel_paths = [os.path.basename(p) for p in saved_paths]
            writer.writerow(list(combo) + rel_paths)
            csv_file.flush() # 实时保存，防止中断丢失

    except KeyboardInterrupt:
        print("Scan interrupted by user!")
    finally:
        # 5. 收尾工作
        print("Returning to Zero...")
        # 缓慢归零，防止弹射
        servo.set_targets_abs_deg({mid: 0 for mid in TARGET_IDS})
        servo.execute_time_synced(duration_s=3.0)
        
        servo.close()
        cams.release()
        csv_file.close()
        print("Done.")

if __name__ == "__main__":
    main()