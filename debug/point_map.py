#!/usr/bin/env python3
"""在图片上交互式选择并编号像素点。

操作：
    鼠标左键       选择/移动当前点
    鼠标滚轮、+/-  缩放（以鼠标位置或窗口中心为中心）
    鼠标右键拖动   平移图片
    n              确认当前点并写入编号
    q              保存标注图和坐标后退出
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


Point = Tuple[int, int]


def read_image(path: Path) -> np.ndarray:
    """读取图片；使用 imdecode 以兼容包含中文的路径。"""
    path = Path(path)
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError as exc:
        raise ValueError(f"无法读取图片：{path} ({exc})") from exc

    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"不是有效图片或图片格式不受支持：{path}")
    return image


def write_image(path: Path, image: np.ndarray) -> None:
    """保存图片；使用 imencode 以兼容包含中文的路径。"""
    path = Path(path)
    suffix = path.suffix.lower() or ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise ValueError(f"不支持的输出图片格式：{suffix}")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        encoded.tofile(str(path))
    except OSError as exc:
        raise ValueError(f"无法保存图片：{path} ({exc})") from exc


def draw_numbered_circle(
    image: np.ndarray,
    point: Point,
    number: Optional[int],
    color: Tuple[int, int, int] = (0, 0, 255),
    alpha: float = 0.45,
) -> np.ndarray:
    """在原图像素坐标处绘制半径 5 px 的半透明实心圆及居中编号。"""
    result = image.copy()
    overlay = image.copy()
    cv2.circle(overlay, point, 36, color, thickness=-1, lineType=cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, result, 1.0 - alpha, 0, dst=result)

    if number is not None:
        text = str(number)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.30 if number < 10 else 0.24
        thickness = 1
        (text_w, text_h), baseline = cv2.getTextSize(
            text, font, font_scale, thickness
        )
        origin = (
            int(round(point[0] - text_w / 2)),
            int(round(point[1] + (text_h - baseline) / 2)),
        )
        cv2.putText(
            result,
            text,
            origin,
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )
    return result


class PointMapEditor:
    STATUS_HEIGHT = 48
    MAX_VIEW_WIDTH = 1280
    MAX_VIEW_HEIGHT = 800

    def __init__(self, image: np.ndarray, window_name: str = "Point map") -> None:
        self.original = image
        self.annotated = image.copy()
        self.window_name = window_name
        self.image_h, self.image_w = image.shape[:2]

        self.view_w = min(self.image_w, self.MAX_VIEW_WIDTH)
        self.view_h = min(self.image_h, self.MAX_VIEW_HEIGHT)
        fit_scale = min(
            self.view_w / self.image_w,
            self.view_h / self.image_h,
            1.0,
        )
        self.scale = fit_scale
        self.min_scale = max(0.02, fit_scale * 0.1)
        self.max_scale = max(20.0, fit_scale)
        self.pan = np.array([0.0, 0.0], dtype=np.float64)

        self.points: List[Point] = []
        self.pending_point: Optional[Point] = None
        self.dragging = False
        self.drag_start = np.zeros(2, dtype=np.float64)
        self.pan_at_drag_start = np.zeros(2, dtype=np.float64)
        self.dirty = True

    def _scaled_size(self) -> Tuple[int, int]:
        return (
            max(1, int(round(self.image_w * self.scale))),
            max(1, int(round(self.image_h * self.scale))),
        )

    def _image_offset(self) -> np.ndarray:
        scaled_w, scaled_h = self._scaled_size()
        return np.array(
            [
                max(0, (self.view_w - scaled_w) // 2),
                max(0, (self.view_h - scaled_h) // 2),
            ],
            dtype=np.float64,
        )

    def _clamp_pan(self) -> None:
        scaled_w, scaled_h = self._scaled_size()
        max_pan = np.array(
            [max(0, scaled_w - self.view_w), max(0, scaled_h - self.view_h)],
            dtype=np.float64,
        )
        self.pan = np.clip(self.pan, 0.0, max_pan)

    def screen_to_image(self, x: int, y: int) -> Optional[Point]:
        """把窗口坐标转换为原图整数像素坐标。"""
        image_y = y - self.STATUS_HEIGHT
        offset = self._image_offset()
        scaled_xy = np.array([x, image_y], dtype=np.float64) - offset + self.pan
        scaled_w, scaled_h = self._scaled_size()
        if not (0 <= scaled_xy[0] < scaled_w and 0 <= scaled_xy[1] < scaled_h):
            return None

        point = np.floor(scaled_xy / self.scale).astype(int)
        return (
            int(np.clip(point[0], 0, self.image_w - 1)),
            int(np.clip(point[1], 0, self.image_h - 1)),
        )

    def set_zoom(self, new_scale: float, screen_x: int, screen_y: int) -> None:
        """缩放时保持光标指向的原图位置不变。"""
        image_y = screen_y - self.STATUS_HEIGHT
        old_offset = self._image_offset()
        anchor = (
            np.array([screen_x, image_y], dtype=np.float64)
            - old_offset
            + self.pan
        ) / self.scale

        self.scale = float(np.clip(new_scale, self.min_scale, self.max_scale))
        new_offset = self._image_offset()
        self.pan = anchor * self.scale + new_offset - np.array(
            [screen_x, image_y], dtype=np.float64
        )
        self._clamp_pan()
        self.dirty = True

    def mouse_callback(self, event: int, x: int, y: int, flags: int, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            point = self.screen_to_image(x, y)
            if point is not None:
                self.pending_point = point
                self.dirty = True
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.dragging = True
            self.drag_start[:] = (x, y)
            self.pan_at_drag_start = self.pan.copy()
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            current = np.array([x, y], dtype=np.float64)
            self.pan = self.pan_at_drag_start + self.drag_start - current
            self._clamp_pan()
            self.dirty = True
        elif event == cv2.EVENT_RBUTTONUP:
            self.dragging = False
        elif event == cv2.EVENT_MOUSEWHEEL:
            # Python 的 OpenCV 绑定通常不提供 C++ 的 getMouseWheelDelta；
            # 回调中的 flags 自身已带符号，可直接判断滚轮方向。
            factor = 1.25 if flags > 0 else 0.8
            self.set_zoom(self.scale * factor, x, y)

    def confirm_pending(self) -> bool:
        if self.pending_point is None:
            return False
        self.points.append(self.pending_point)
        self.annotated = draw_numbered_circle(
            self.annotated, self.pending_point, len(self.points)
        )
        self.pending_point = None
        self.dirty = True
        return True

    def _render(self) -> np.ndarray:
        displayed = self.annotated
        if self.pending_point is not None:
            displayed = draw_numbered_circle(
                displayed, self.pending_point, None, color=(0, 255, 255), alpha=0.45
            )

        scaled_w, scaled_h = self._scaled_size()
        interpolation = cv2.INTER_AREA if self.scale < 1.0 else cv2.INTER_LINEAR
        scaled = cv2.resize(displayed, (scaled_w, scaled_h), interpolation=interpolation)

        canvas = np.full(
            (self.view_h + self.STATUS_HEIGHT, self.view_w, 3), 36, dtype=np.uint8
        )
        offset = self._image_offset().astype(int)
        src_x = int(round(self.pan[0]))
        src_y = int(round(self.pan[1]))
        copy_w = min(scaled_w - src_x, self.view_w - offset[0])
        copy_h = min(scaled_h - src_y, self.view_h - offset[1])
        if copy_w > 0 and copy_h > 0:
            canvas[
                self.STATUS_HEIGHT + offset[1] : self.STATUS_HEIGHT + offset[1] + copy_h,
                offset[0] : offset[0] + copy_w,
            ] = scaled[src_y : src_y + copy_h, src_x : src_x + copy_w]

        pending = (
            f"  current=({self.pending_point[0]}, {self.pending_point[1]})"
            if self.pending_point is not None
            else "  current=none"
        )
        status = (
            f"Left: select | Wheel/+/-: zoom | Right drag: pan | "
            f"N: confirm | Q: save & quit    points={len(self.points)}{pending}"
        )
        cv2.putText(
            canvas,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def run(self) -> None:
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        print("左键选点，滚轮或 +/- 缩放，右键拖动，n 确认，q 保存并退出。")

        try:
            while True:
                if self.dirty:
                    cv2.imshow(self.window_name, self._render())
                    self.dirty = False

                key = cv2.waitKeyEx(20)
                if key in (ord("n"), ord("N")):
                    if self.confirm_pending():
                        point = self.points[-1]
                        print(f"已确认点 {len(self.points)}: ({point[0]}, {point[1]})")
                    else:
                        print("当前没有待确认的点，请先在图片上单击。")
                elif key in (ord("+"), ord("=")):
                    self.set_zoom(
                        self.scale * 1.25,
                        self.view_w // 2,
                        self.STATUS_HEIGHT + self.view_h // 2,
                    )
                elif key in (ord("-"), ord("_")):
                    self.set_zoom(
                        self.scale * 0.8,
                        self.view_w // 2,
                        self.STATUS_HEIGHT + self.view_h // 2,
                    )
                elif key in (ord("q"), ord("Q")):
                    break

                try:
                    if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
        finally:
            cv2.destroyAllWindows()


def save_results(
    image_path: Path,
    coordinates_path: Path,
    image: np.ndarray,
    points: Sequence[Point],
) -> None:
    # 兼容调用处直接传入字符串路径。
    image_path = Path(image_path)
    coordinates_path = Path(coordinates_path)
    write_image(image_path, image)
    coordinates_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "image": str(image_path),
        "coordinate_system": "field coordinates; x in [0, 28], y in [0, 15]",
        "conversion": "x = pixel_x / 1258 * 28; y = (675 - pixel_y) / 675 * 15",
        "points": [
            {
                "number": index,
                "x": round(point[0] / 1258 * 28, 6),
                "y": round((675 - point[1]) / 675 * 15, 6),
                "pixel_x": point[0],
                "pixel_y": point[1],
            }
            for index, point in enumerate(points, start=1)
        ],
    }
    try:
        coordinates_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    except OSError as exc:
        raise ValueError(f"无法保存坐标：{coordinates_path} ({exc})") from exc


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取图片，缩放/平移后交互选点，并保存编号标注图和像素坐标。"
    )
    parser.add_argument("image", type=Path, help="输入图片路径")
    parser.add_argument(
        "-o", "--output", type=Path, help="输出图片路径（默认：原文件名_points.png）"
    )
    parser.add_argument(
        "-c",
        "--coordinates",
        type=Path,
        help="坐标 JSON 路径（默认：原文件名_points.json）",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    path = '/root/rm/radar-detect/debug/RM2026.png'
    output_path = '/root/rm/radar-detect/debug/RM2026point.png'
    coordinates_path = '/root/rm/radar-detect/debug/RM2026.json'

    try:
        image = read_image(path)
        editor = PointMapEditor(image)
        editor.run()
        save_results(output_path, coordinates_path, editor.annotated, editor.points)
    except (ValueError, cv2.error) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1

    print(f"已保存标注图片：{output_path}")
    print(f"已保存 {len(editor.points)} 个圆心坐标：{coordinates_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
