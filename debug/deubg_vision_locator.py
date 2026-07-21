import argparse
import os
import sys

import cv2
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from camera_locator.anchor import Anchor
from camera_locator.point_picker import PointsPicker
from detect.Video import Video
from Lidar.Converter import Converter
from Lidar.vision_locator import Vision_Locator


DEFAULT_VIDEO_PATH = os.path.join(REPO_ROOT, "data", "video.avi")
DEFAULT_CONVERTER_CONFIG = os.path.join(REPO_ROOT, "configs", "converter_config.yaml")
DEFAULT_CAMERA_CONFIG = os.path.join(REPO_ROOT, "configs", "bin_cam_config.yaml")
CALIBRATION_POINT_NAMES = [
    "enemy_Base_25",
    "enemy_Tower_25",
    "self_FORTRESS",
    "self_Tower_25",
    "enemy_HERO_HIGH",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="五点标定后，将 Vision_Locator 的所有区域多边形投影到图像并可视化。"
    )
    parser.add_argument("--image", default=None, help="用于标定和可视化的图片路径")
    parser.add_argument("--video", default=DEFAULT_VIDEO_PATH, help="用于取第一帧的视频路径")
    parser.add_argument("--camera", action="store_true", help="从工业相机取图，而不是图片/视频")
    parser.add_argument("--camera-name", default="new_cam", help="工业相机配置中的相机名")
    parser.add_argument("--camera-config", default=DEFAULT_CAMERA_CONFIG, help="工业相机配置路径")
    parser.add_argument("--converter-config", default=DEFAULT_CONVERTER_CONFIG, help="Converter 配置路径")
    parser.add_argument("--color", default="Red", choices=["Red", "Blue"], help="己方颜色，仅用于初始化 Converter")
    parser.add_argument("--output", default=os.path.join(REPO_ROOT, "debug", "vision_locator_projection.jpg"))
    return parser.parse_args()


def read_debug_frame(args):
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            raise RuntimeError(f"读取图片失败: {args.image}")
        return image

    if args.camera:
        from detect.Capture import Capture

        capture = Capture(args.camera_config, args.camera_name)
        try:
            image = capture.get_frame()
        finally:
            if hasattr(capture, "release"):
                capture.release()
        if image is None:
            raise RuntimeError("相机取图失败")
        return image

    video = Video(args.video)
    try:
        image = video.get_frame()
    finally:
        video.release()
    if image is None:
        raise RuntimeError(f"视频取帧失败: {args.video}")
    return image


def pick_five_points(image):
    anchor = Anchor()
    picker = PointsPicker()
    picker.caller(image, anchor)
    while len(anchor) < 5:
        picker.resume(anchor)

    pixel_points = np.array(anchor.vertexes, dtype=np.float32)
    if len(pixel_points) != 5:
        raise RuntimeError(f"需要 5 个图像点，当前只有 {len(pixel_points)} 个")
    return pixel_points


def solve_field_to_camera(converter, pixel_points):
    world_points = np.array(converter.real_points_26, dtype=np.float32)
    ok, rotation_vector, translation_vector = cv2.solvePnP(
        world_points,
        pixel_points,
        converter.intrinsic_matrix,
        converter.distortion_matrix,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok:
        raise RuntimeError("solvePnP EPNP failed")

    ok, rotation_vector, translation_vector = cv2.solvePnP(
        world_points,
        pixel_points,
        converter.intrinsic_matrix,
        converter.distortion_matrix,
        rotation_vector,
        translation_vector,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP ITERATIVE refine failed")

    rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
    field_to_camera_matrix = np.vstack(
        (
            np.hstack((rotation_matrix, translation_vector.reshape(3, 1))),
            np.array([0.0, 0.0, 0.0, 1.0]),
        )
    )
    return rotation_vector, translation_vector, field_to_camera_matrix


def draw_projected_regions(image, locator):
    canvas = image.copy()
    overlay = image.copy()
    colors = [
        (230, 25, 75),
        (60, 180, 75),
        (255, 225, 25),
        (0, 130, 200),
        (245, 130, 48),
        (145, 30, 180),
        (70, 240, 240),
        (240, 50, 230),
        (210, 245, 60),
        (250, 190, 190),
        (0, 128, 128),
        (230, 190, 255),
        (170, 110, 40),
    ]

    for index, (name, parser_points) in enumerate(locator.points_map.items()):
        points = np.array(parser_points.points_2d, dtype=np.int32)
        if len(points) < 3:
            continue

        color = colors[index % len(colors)]
        cv2.fillPoly(overlay, [points], color)
        cv2.polylines(canvas, [points], isClosed=True, color=color, thickness=3, lineType=cv2.LINE_AA)

        label_point = points.mean(axis=0).astype(int)
        cv2.putText(
            canvas,
            name,
            (int(label_point[0]), int(label_point[1])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )

    cv2.addWeighted(overlay, 0.22, canvas, 0.78, 0, canvas)
    return canvas


def draw_text_with_outline(image, text, origin, color, scale=0.65, thickness=2):
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_calibration_projection(image, converter, rotation_vector, translation_vector, pixel_points):
    canvas = image.copy()
    world_points = np.array(converter.real_points_25, dtype=np.float32)
    projected_points, _ = cv2.projectPoints(
        world_points,
        rotation_vector,
        translation_vector,
        converter.intrinsic_matrix,
        converter.distortion_matrix,
    )
    projected_points = projected_points.reshape(-1, 2)

    for index, projected_point in enumerate(projected_points):
        projected_xy = tuple(np.rint(projected_point).astype(int))
        clicked_xy = tuple(np.rint(pixel_points[index]).astype(int))
        error = float(np.linalg.norm(projected_point - pixel_points[index]))
        name = CALIBRATION_POINT_NAMES[index] if index < len(CALIBRATION_POINT_NAMES) else f"point_{index + 1}"

        cv2.line(canvas, clicked_xy, projected_xy, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.drawMarker(
            canvas,
            clicked_xy,
            (255, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
            line_type=cv2.LINE_AA,
        )
        cv2.circle(canvas, projected_xy, 9, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, projected_xy, 12, (0, 0, 0), 2, cv2.LINE_AA)
        draw_text_with_outline(
            canvas,
            f"{index + 1}: {name} {error:.1f}px",
            (projected_xy[0] + 12, projected_xy[1] - 10),
            (0, 255, 255),
            scale=0.6,
            thickness=2,
        )

    return canvas, projected_points


def main():
    args = parse_args()
    image = read_debug_frame(args)
    converter = Converter(args.color, args.converter_config)

    print("请按 Converter.real_points_25 的顺序点击 5 个标定点，按 q 结束当前点选窗口。")
    print("顺序: " + ", ".join(CALIBRATION_POINT_NAMES))
    pixel_points = pick_five_points(image)

    rotation_vector, translation_vector, field_to_camera_matrix = solve_field_to_camera(converter, pixel_points)
    locator = Vision_Locator(
        intrinsic_matrix=converter.intrinsic_matrix,
        dist_coeffs=converter.distortion_matrix,
        world_rvec=rotation_vector,
        world_tvec=translation_vector,
        extrinsic_matrix=field_to_camera_matrix,
        img=image,
    )

    result = draw_projected_regions(image, locator)
    result, projected_points = draw_calibration_projection(
        result,
        converter,
        rotation_vector,
        translation_vector,
        pixel_points,
    )
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    cv2.imwrite(args.output, result)
    print(f"field_to_camera_matrix:\n{field_to_camera_matrix}")
    print("calibration reprojection:")
    for index, projected_point in enumerate(projected_points):
        error = float(np.linalg.norm(projected_point - pixel_points[index]))
        print(
            f"  {index + 1}. {CALIBRATION_POINT_NAMES[index]} "
            f"clicked={pixel_points[index].tolist()} "
            f"projected={projected_point.tolist()} "
            f"error={error:.2f}px"
        )
    print(f"debug image saved: {args.output}")

    show = result
    max_width = 1600
    if show.shape[1] > max_width:
        scale = max_width / show.shape[1]
        show = cv2.resize(show, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    cv2.namedWindow("vision locator projection", cv2.WINDOW_NORMAL)
    cv2.imshow("vision locator projection", show)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
