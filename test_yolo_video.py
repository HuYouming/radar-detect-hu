"""
简单的 YOLO 两阶段检测测试脚本
stage_one: 目标检测 + 追踪（识别车辆）
stage_two: 装甲板分类（识别具体编号）
"""
import cv2
import math
import time
from ultralytics import YOLO

# ============ 配置 ============
VIDEO_PATH = "/home/py/Hust_Radar_2025/data/test_video_trimmed.mp4"
STAGE_ONE_PATH = "/home/py/Hust_Radar_2025/weight/stage_one.pt"
STAGE_TWO_PATH = "/home/py/Hust_Radar_2025/weight/stage_two.pt"
TRACKER_CFG = "/home/py/Hust_Radar_2025/configs/bytetrack.yaml"

STAGE_ONE_CONF = 0.1       # 一阶段置信度阈值
STAGE_TWO_CONF = 0.5       # 二阶段置信度阈值
LABELS = ["B1", "B2", "B3", "B4", "B5", "B7", "R1", "R2", "R3", "R4", "R5", "R7"]
SHOW_SCALE = 0.5            # 显示缩放比例（原图太大时缩小）
# ==============================

print("加载模型...")
model_detect = YOLO(STAGE_ONE_PATH, task="detect")
model_classify = YOLO(STAGE_TWO_PATH)
print("模型加载完成")

# 追踪器投票字典
Track_value = {}
for i in range(10000):
    Track_value[i] = [0] * len(LABELS)

# 灰色装甲板映射
Gray2Blue = {12: 5, 13: 1, 14: 0, 15: 3, 16: 2, 17: 4}
Gray2Red = {12: 11, 13: 7, 14: 6, 15: 9, 16: 8, 17: 10}
Blue2Gray = {v: k for k, v in Gray2Blue.items()}
Red2Gray = {v: k for k, v in Gray2Red.items()}
GRAY_THRESH = 15

cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"无法打开视频: {VIDEO_PATH}")
    exit(1)

fps_list = []
frame_count = 0

print("开始检测，按 q 退出...")
while True:
    ret, frame = cap.read()
    if not ret:
        print("视频播放完毕")
        break

    t0 = time.time()
    frame_count += 1

    # === 一阶段：目标检测 + 追踪 ===
    results = model_detect.track(frame, persist=True, tracker=TRACKER_CFG,
                                  conf=STAGE_ONE_CONF, verbose=False)

    if results is None or results[0].boxes.id is None:
        # 无检测结果，直接显示
        pass
    else:
        boxes = results[0].boxes.xywh.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        confs = results[0].boxes.conf.cpu().numpy()

        # === 二阶段：批量分类 ===
        roi_list = []
        meta_list = []  # (track_id, box, conf)

        for box, track_id, conf in zip(boxes, track_ids, confs):
            x, y, w, h = box
            x1 = max(int(x - w / 2), 0)
            y1 = max(int(y - h / 2), 0)
            x2 = min(int(x + w / 2), frame.shape[1])
            y2 = min(int(y + h / 2), frame.shape[0])
            roi = frame[y1:y2, x1:x2]
            if roi.size > 0:
                roi_list.append(roi)
                meta_list.append((track_id, box, conf))

        if len(roi_list) > 0:
            cls_results = model_classify.predict(roi_list, conf=STAGE_TWO_CONF,
                                                  iou=0.7, device=0, verbose=False)

            for (track_id, box, det_conf), cls_result, roi in zip(meta_list, cls_results, roi_list):
                x, y, w, h = box
                data = cls_result.boxes.data

                # 取置信度最高的分类结果
                classify_label = -1
                cls_conf = 0
                for i in range(len(data)):
                    if data[i][4] > cls_conf:
                        cls_conf = float(data[i][4])
                        classify_label = int(data[i][5])

                # 灰色装甲板判定
                if classify_label != -1 and classify_label > 11:
                    x1, y1 = int(data[0][0]), int(data[0][1])
                    x2, y2 = int(data[0][2]), int(data[0][3])
                    gray_roi = roi[y1:y2, x1:x2]
                    if gray_roi.size > 0:
                        gray_val = cv2.cvtColor(gray_roi, cv2.COLOR_BGR2GRAY)
                        if cv2.mean(gray_val)[0] < GRAY_THRESH:
                            best = Track_value[track_id].index(max(Track_value[track_id]))
                            if best < 6:
                                classify_label = Gray2Blue.get(classify_label, classify_label)
                            else:
                                classify_label = Gray2Red.get(classify_label, classify_label)

                # 投票累加
                if classify_label != -1 and classify_label < len(LABELS):
                    Track_value[track_id][classify_label] += 0.5 + cls_conf * 0.5

                # 取投票最多的类别
                best_label_idx = Track_value[track_id].index(max(Track_value[track_id]))
                all_same = all(v == Track_value[track_id][0] for v in Track_value[track_id])
                label_str = LABELS[best_label_idx] if not all_same else "?"

                # === 画框 ===
                x1 = int(x - w / 2)
                y1 = int(y - h / 2)
                x2 = int(x + w / 2)
                y2 = int(y + h / 2)

                color = (255, 128, 0)
                if label_str.startswith("B"):
                    color = (255, 200, 0)
                elif label_str.startswith("R"):
                    color = (0, 0, 255)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                cv2.putText(frame, f"{label_str}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
                cv2.putText(frame, f"id:{track_id}", (x2 + 5, y2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # FPS 计算
    dt = time.time() - t0
    fps = 1.0 / dt if dt > 0 else 0
    fps_list.append(fps)
    if len(fps_list) > 30:
        fps_list.pop(0)
    avg_fps = sum(fps_list) / len(fps_list)

    cv2.putText(frame, f"FPS: {avg_fps:.1f}  Frame: {frame_count}",
                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

    # 缩放显示
    show = cv2.resize(frame, (int(frame.shape[1] * SHOW_SCALE), int(frame.shape[0] * SHOW_SCALE)))
    cv2.imshow("YOLO Detection Test", show)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord(' '):  # 空格暂停
        cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()
print(f"共处理 {frame_count} 帧，平均 FPS: {sum(fps_list)/len(fps_list):.1f}")
