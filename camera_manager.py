import cv2
import time
import os

class CameraGroup:
    def __init__(self, cam_indices=[0, 1, 2], warmup_frames=10):
        """
        初始化多个摄像头
        cam_indices: 摄像头的索引列表，通常是 0, 1, 2... 需根据实际插拔顺序确认
        """
        self.caps = {}
        self.indices = cam_indices
        self.warmup_frames = warmup_frames
        
        print(f"[VISION] Opening cameras: {cam_indices}...")
        for idx in cam_indices:
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                print(f"[VISION] Warning: Camera {idx} failed to open!")
            else:
                # 设置分辨率（可选，根据你的摄像头支持情况）
                # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                self.caps[idx] = cap
        
        # 预热摄像头（自动白平衡和曝光适应）
        self._warmup()

    def _warmup(self):
        print("[VISION] Warming up sensors...")
        for _ in range(self.warmup_frames):
            for cap in self.caps.values():
                cap.read()
        time.sleep(1.0) 
        print("[VISION] Ready.")

    def capture_and_save(self, save_dir, filename_prefix):
        """
        拍照并保存
        filename_prefix: 例如 "step_001_pos_0_780_300"
        """
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        saved_files = []
        for idx, cap in self.caps.items():
            ret, frame = cap.read()
            if ret:
                # 文件名格式: prefix_camX.jpg
                fname = f"{filename_prefix}_cam{idx}.jpg"
                path = os.path.join(save_dir, fname)
                cv2.imwrite(path, frame)
                saved_files.append(path)
            else:
                print(f"[VISION] Error reading from cam {idx}")
        
        return saved_files

    def release(self):
        for cap in self.caps.values():
            cap.release()