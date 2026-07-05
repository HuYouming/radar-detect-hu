import argparse
import datetime as dt
import os
import re
import sys

import cv2


LINE_RE = re.compile(
    r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})\]\s+Preparing to send all location info:\s+(.*)$"
)
PAIR_RE = re.compile(r"\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*\]")

FIELD_W = 28.0
FIELD_H = 15.0


def parse_entries(log_path):
    entries = []
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            m = LINE_RE.match(line)
            if not m:
                continue
            ts_str = m.group(1)
            pairs_str = m.group(2)
            try:
                ts = dt.datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S")
            except ValueError:
                continue
            pairs = [(float(x), float(y)) for x, y in PAIR_RE.findall(pairs_str)]
            if not pairs:
                continue
            entries.append((ts, pairs))
    return entries


def pick_ids(color, count):
    if color.lower() == "red":
        enemy_ids = [101, 102, 103, 104, 106, 107]
        my_ids = [1, 2, 3, 4, 6, 7]
    else:
        enemy_ids = [1, 2, 3, 4, 6, 7]
        my_ids = [101, 102, 103, 104, 106, 107]

    if count == 12:
        return enemy_ids + my_ids
    if count == 6:
        return enemy_ids
    return list(range(1, count + 1))


def draw_frame(base_img, points, ids, timestamp, draw_zero):
    img = base_img.copy()
    h, w = img.shape[:2]
    for (x, y), robot_id in zip(points, ids):
        if not draw_zero and x == 0.0 and y == 0.0:
            continue
        px = int((w * x) / FIELD_W)
        py = int(h * (FIELD_H - y) / FIELD_H)
        color = (200, 0, 0) if robot_id > 99 else (0, 0, 200)
        cv2.circle(img, (px, py), 18, color, -1)
        cv2.putText(
            img,
            str(robot_id),
            (px - 10, py + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

    ts_text = timestamp.strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(
        img,
        ts_text,
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )
    return img


def main():
    parser = argparse.ArgumentParser(
        description="Draw minimap video from Sender log entries."
    )
    parser.add_argument("log", help="Path to Sender_*.log")
    parser.add_argument("--color", default="red", choices=["red", "blue"], help="Our side color")
    parser.add_argument(
        "--map",
        dest="map_path",
        default=None,
        help="Map image path (defaults to camera_locator/map_red.jpg or map_blue.jpg)",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Output video FPS")
    parser.add_argument(
        "--output",
        default="minimap.mp4",
        help="Output video path (.mp4)",
    )
    parser.add_argument(
        "--draw-zero",
        action="store_true",
        help="Draw points with (0,0) instead of skipping them",
    )
    args = parser.parse_args()

    entries = parse_entries(args.log)
    if not entries:
        print("No location entries found in log.")
        return 1

    if args.map_path is None:
        map_name = "map_red.jpg" if args.color.lower() == "red" else "map_blue.jpg"
        args.map_path = os.path.join("camera_locator", map_name)

    base_img = cv2.imread(args.map_path)
    if base_img is None:
        print(f"Failed to read map image: {args.map_path}")
        return 1

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    h, w = base_img.shape[:2]
    writer = cv2.VideoWriter(args.output, fourcc, args.fps, (w, h))
    if not writer.isOpened():
        print(f"Failed to open video writer: {args.output}")
        return 1

    for i, (ts, points) in enumerate(entries):
        ids = pick_ids(args.color, len(points))
        frame = draw_frame(base_img, points, ids, ts, args.draw_zero)
        if i + 1 < len(entries):
            next_ts = entries[i + 1][0]
            dt_sec = max(0.0, (next_ts - ts).total_seconds())
        else:
            dt_sec = 1.0 / max(args.fps, 1.0)
        frames = max(1, int(round(dt_sec * args.fps)))
        for _ in range(frames):
            writer.write(frame)

    writer.release()
    print(f"Wrote video: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
