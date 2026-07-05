import argparse
import logging
import os
import sys
import time
import traceback
import types
import faulthandler
from collections import deque

import cv2
import numpy as np
from ruamel.yaml import YAML

from communication.Messager import Messager
from detect.Detector import Detector
from detect.Video import Video
from detect.Capture import Capture
from Lidar.Converter import Converter
from Car.Car import CarList


def get_new_box(xyxy, xywh):
    x1, y1, x2, y2 = xyxy
    x, y, w1, h1 = xywh
    new_x = x + w1 / 2
    new_y = y2
    if y > 3036 / 2:
        w = x2 - x1
        h = y2 - y1
        new_x1 = x1 + w / 2
        new_y1 = y1 + h / 2 + h / 3
    else:
        w = x2 - x1
        h = y2 - y1
        new_x1 = x1 + w / 2
        new_y1 = y1 + h / 2 + h / 7
    return [new_x, new_y, new_x1, new_y1]


def build_logger(log_dir):
    os.makedirs(log_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    log_path = os.path.join(log_dir, f"debug_detector_pipeline_{ts}.log")

    logger = logging.getLogger("debug_detector_pipeline")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    return logger, log_path


def timed_call(logger, name, func, *args, **kwargs):
    start = time.time()
    try:
        result = func(*args, **kwargs)
    except Exception:
        cost = time.time() - start
        logger.exception("%s failed after %.3fs", name, cost)
        raise
    cost = time.time() - start
    logger.info("%s ok in %.3fs", name, cost)
    return result


def install_detector_hooks(detector, logger, debug_state):
    original_track_infer = detector.track_infer
    original_classify_infer = detector.classify_infer
    original_infer = detector.infer

    def wrapped_track_infer(self, frame):
        debug_state["last_stage"] = "track_infer:start"
        start = time.time()
        result = original_track_infer(frame)
        debug_state["last_stage"] = "track_infer:end"
        logger.info("track_infer cost %.3fs", time.time() - start)
        return result

    def wrapped_classify_infer(self, roi_list):
        debug_state["last_stage"] = f"classify_infer:start roi_count={len(roi_list)}"
        start = time.time()
        result = original_classify_infer(roi_list)
        debug_state["last_stage"] = "classify_infer:end"
        logger.info("classify_infer cost %.3fs", time.time() - start)
        if result == -1:
            logger.error("classify_infer returned -1, infer() will crash on tuple unpack")
        return result

    def wrapped_infer(self, frame):
        debug_state["last_stage"] = "infer:start"
        if frame is None:
            logger.warning("infer got None frame")
        else:
            logger.info("infer input frame shape=%s", getattr(frame, "shape", None))
        start = time.time()
        try:
            result = original_infer(frame)
        except Exception:
            debug_state["last_stage"] = "infer:exception"
            logger.exception("infer crashed")
            raise
        debug_state["last_stage"] = "infer:end"
        logger.info("infer total cost %.3fs", time.time() - start)
        return result

    detector.track_infer = types.MethodType(wrapped_track_infer, detector)
    detector.classify_infer = types.MethodType(wrapped_classify_infer, detector)
    detector.infer = types.MethodType(wrapped_infer, detector)

    original_detect_thread = detector.detect_thread

    def wrapped_detect_thread(self, capture):
        logger.info("detect_thread started")
        debug_state["thread_started_at"] = time.time()
        try:
            return original_detect_thread(capture)
        except Exception as exc:
            debug_state["thread_exception"] = repr(exc)
            debug_state["thread_traceback"] = traceback.format_exc()
            logger.exception("detect_thread crashed")
            raise
        finally:
            debug_state["thread_exited_at"] = time.time()
            logger.info("detect_thread exited")

    detector.detect_thread = types.MethodType(wrapped_detect_thread, detector)


def install_capture_hooks(capture, logger, debug_state):
    original_get_frame = capture.get_frame

    def wrapped_get_frame():
        start = time.time()
        frame = original_get_frame()
        cost = time.time() - start
        current_frame = getattr(capture, "current_frame", None)
        if frame is None:
            logger.warning("capture.get_frame -> None in %.3fs, current_frame=%s", cost, current_frame)
        else:
            logger.info(
                "capture.get_frame ok in %.3fs, current_frame=%s, shape=%s",
                cost,
                current_frame,
                getattr(frame, "shape", None),
            )
        debug_state["last_frame_index"] = current_frame
        debug_state["last_frame_time"] = time.time()
        return frame

    capture.get_frame = wrapped_get_frame


def install_converter_hooks(converter, logger, debug_state):
    original_detection_main = converter.detection_main
    original_get_distance = converter.get_distance
    original_camera_to_image = converter.camera_to_image
    original_angle_to_quadrant = converter.angle_to_quadrant

    def wrapped_detection_main(self, box, t, point_cloud=None):
        debug_state["last_stage"] = "converter.detection_main:start"
        start = time.time()
        result = original_detection_main(box, t, point_cloud)
        debug_state["last_stage"] = "converter.detection_main:end"
        logger.info("converter.detection_main cost %.3fs result=%s", time.time() - start, result)
        return result

    def wrapped_get_distance(self, point):
        start = time.time()
        result = original_get_distance(point)
        logger.info("converter.get_distance cost %.3fs result=%s", time.time() - start, result)
        return result

    def wrapped_camera_to_image(self, pc):
        start = time.time()
        result = original_camera_to_image(pc)
        logger.info(
            "converter.camera_to_image cost %.3fs input_shape=%s output_shape=%s",
            time.time() - start,
            getattr(pc, "shape", None),
            getattr(result, "shape", None),
        )
        return result

    def wrapped_angle_to_quadrant(self, angle):
        result = original_angle_to_quadrant(angle)
        logger.info("converter.angle_to_quadrant angle=%s quadrant=%s", angle, result)
        return result

    converter.detection_main = types.MethodType(wrapped_detection_main, converter)
    converter.get_distance = types.MethodType(wrapped_get_distance, converter)
    converter.camera_to_image = types.MethodType(wrapped_camera_to_image, converter)
    converter.angle_to_quadrant = types.MethodType(wrapped_angle_to_quadrant, converter)

    if getattr(converter, "vision_locator", None) is not None:
        original_post_process = converter.vision_locator.post_process
        original_visualize = converter.vision_locator.visualize

        def wrapped_post_process(xy, color):
            debug_state["last_stage"] = "vision_locator.post_process:start"
            start = time.time()
            result = original_post_process(xy, color)
            debug_state["last_stage"] = "vision_locator.post_process:end"
            logger.info("vision_locator.post_process cost %.3fs result=%s", time.time() - start, result)
            return result

        def wrapped_visualize(input_points):
            start = time.time()
            result = original_visualize(input_points)
            logger.info(
                "vision_locator.visualize cost %.3fs points=%s",
                time.time() - start,
                0 if input_points is None else len(input_points),
            )
            return result

        converter.vision_locator.post_process = wrapped_post_process
        converter.vision_locator.visualize = wrapped_visualize


def install_messager_hooks(messager, logger, debug_state):
    original_update_enemy = messager.update_enemy_car_infos
    original_update_our = messager.update_our_car_infos
    original_update_alert = messager.update_sentinel_alert_info
    original_start = messager.start
    original_stop = messager.stop

    def wrapped_update_enemy(enemy_car_infos):
        debug_state["last_stage"] = "messager.update_enemy_car_infos:start"
        start = time.time()
        result = original_update_enemy(enemy_car_infos)
        debug_state["last_stage"] = "messager.update_enemy_car_infos:end"
        logger.info("messager.update_enemy_car_infos cost %.3fs len=%s", time.time() - start, len(enemy_car_infos))
        return result

    def wrapped_update_our(*, our_car_infos):
        debug_state["last_stage"] = "messager.update_our_car_infos:start"
        start = time.time()
        result = original_update_our(our_car_infos=our_car_infos)
        debug_state["last_stage"] = "messager.update_our_car_infos:end"
        logger.info("messager.update_our_car_infos cost %.3fs len=%s", time.time() - start, len(our_car_infos))
        return result

    def wrapped_update_alert(sentinel_alert_info):
        debug_state["last_stage"] = "messager.update_sentinel_alert_info:start"
        start = time.time()
        result = original_update_alert(sentinel_alert_info)
        debug_state["last_stage"] = "messager.update_sentinel_alert_info:end"
        logger.info(
            "messager.update_sentinel_alert_info cost %.3fs value=%s",
            time.time() - start,
            sentinel_alert_info,
        )
        return result

    def wrapped_start():
        start = time.time()
        result = original_start()
        logger.info("messager.start cost %.3fs", time.time() - start)
        return result

    def wrapped_stop():
        start = time.time()
        result = original_stop()
        logger.info("messager.stop cost %.3fs", time.time() - start)
        return result

    messager.update_enemy_car_infos = wrapped_update_enemy
    messager.update_our_car_infos = wrapped_update_our
    messager.update_sentinel_alert_info = wrapped_update_alert
    messager.start = wrapped_start
    messager.stop = wrapped_stop


def build_capture(args):
    if args.mode == "video":
        return Video(args.video_path)
    if args.mode == "camera":
        return Capture(args.binocular_camera_cfg, "new_cam")
    raise ValueError(f"unsupported mode: {args.mode}")


def run_sync_probe(detector, capture, logger, args):
    logger.info("===== sync probe start =====")
    for idx in range(args.sync_frames):
        frame = timed_call(logger, f"sync get_frame[{idx}]", capture.get_frame)
        if frame is None:
            logger.error("sync probe stopped: frame is None at index %s", idx)
            return False
        try:
            infer_result = timed_call(logger, f"sync infer[{idx}]", detector.infer, frame)
        except Exception:
            logger.error("sync probe found detector exception on frame %s", idx)
            return False

        if infer_result is None:
            logger.error("sync probe infer_result is None at frame %s", idx)
            return False

        result_img, results = infer_result
        logger.info(
            "sync probe frame %s -> result_img=%s results_len=%s",
            idx,
            result_img is not None,
            None if results is None else len(results),
        )
    logger.info("===== sync probe end =====")
    return True


def start_detector_and_wait_for_result(detector, capture, logger, debug_state, args):
    logger.info("===== async probe start =====")
    detector.create(capture)
    detector.start()

    got_result = False
    first_result_time = None
    start = time.time()

    while time.time() - start < args.async_timeout:
        infer_result = detector.get_results()
        thread_alive = detector.threading.is_alive() if detector.threading is not None else False
        result_ready = (
            infer_result is not None
            and isinstance(infer_result, (list, tuple))
            and len(infer_result) == 2
            and infer_result[0] is not None
        )

        logger.info(
            "async poll thread_alive=%s result_ready=%s last_stage=%s last_frame_index=%s",
            thread_alive,
            result_ready,
            debug_state.get("last_stage"),
            debug_state.get("last_frame_index"),
        )

        if result_ready:
            got_result = True
            first_result_time = time.time() - start
            result_img, results = infer_result
            logger.info(
                "async first result in %.3fs, result_img_shape=%s, results_len=%s",
                first_result_time,
                getattr(result_img, "shape", None),
                None if results is None else len(results),
            )
            break

        if not thread_alive:
            logger.error("detector thread is dead before producing result")
            break

        time.sleep(args.poll_interval)

    logger.info("===== async probe end =====")
    return got_result, first_result_time


def stop_detector(detector):
    detector.stop()
    if detector.threading is not None and detector.threading.is_alive():
        detector.threading.join(timeout=2.0)


def run_basic_main_loop_probe(detector, car_list, logger, args):
    logger.info("===== main loop probe start =====")
    start = time.time()
    none_count = 0

    for loop_idx in range(args.main_loops):
        infer_result = detector.get_results()
        if infer_result is None:
            logger.warning("loop %s detector.get_results() is None", loop_idx)
            none_count += 1
            time.sleep(args.poll_interval)
            continue

        result_img, results = infer_result
        if result_img is None:
            logger.warning("loop %s result_img is None", loop_idx)
            none_count += 1
            time.sleep(args.poll_interval)
            continue

        car_list_results = []
        if results is not None:
            logger.info("loop %s consume results_len=%s", loop_idx, len(results))
        else:
            logger.info("loop %s consume results=None", loop_idx)

        car_list.update_car_info(car_list_results)
        time.sleep(args.poll_interval)

    logger.info(
        "main loop probe finished in %.3fs, none_count=%s/%s",
        time.time() - start,
        none_count,
        args.main_loops,
    )
    logger.info("===== main loop probe end =====")


def run_full_main_loop_probe(detector, converter, car_list, messager, logger, args, main_cfg):
    logger.info("===== full main loop probe start =====")
    global_my_color = main_cfg["global"]["my_color"]
    is_debug = main_cfg["global"]["is_debug"] and not args.disable_visual_debug
    start_time = time.time()
    fps_queue = deque(maxlen=10)
    all_detections = []
    frame_id = 1
    none_count = 0

    for loop_idx in range(args.main_loops):
        loop_start = time.time()
        now = time.time()
        fps = 1 / max(now - start_time, 1e-6)
        start_time = now
        fps_queue.append(fps)
        avg_fps = sum(fps_queue) / len(fps_queue)
        logger.info("loop %s avg_fps=%.3f", loop_idx, avg_fps)

        infer_result = detector.get_results()
        if infer_result is None:
            none_count += 1
            logger.warning("loop %s detector.get_results() is None", loop_idx)
            time.sleep(args.poll_interval)
            continue

        result_img, results = infer_result
        car_list_results = []
        debug_results = []

        if result_img is None:
            none_count += 1
            logger.warning("loop %s result_img is None", loop_idx)
            time.sleep(args.poll_interval)
            continue

        logger.info(
            "loop %s got infer_result result_img_shape=%s results_len=%s",
            loop_idx,
            getattr(result_img, "shape", None),
            None if results is None else len(results),
        )

        if results is not None:
            for result_idx, result in enumerate(results):
                stage_start = time.time()
                try:
                    xyxy_box, xywh_box, track_id, label, stamp = result
                    logger.info(
                        "loop %s result %s track_id=%s label=%s xywh=%s",
                        loop_idx,
                        result_idx,
                        track_id,
                        label,
                        xywh_box,
                    )
                    if label == "NULL":
                        logger.info("loop %s result %s skipped because label=NULL", loop_idx, result_idx)
                        continue

                    new_xywh_box = get_new_box(xyxy_box, xywh_box)
                    center = converter.detection_main(new_xywh_box, t=stamp)
                    center = converter.vision_locator.post_process(center, global_my_color)
                    distance = converter.get_distance(center)
                    if distance == 0:
                        logger.warning("loop %s result %s skipped because distance=0", loop_idx, result_idx)
                        continue

                    field_xyz = center
                    field_distance = converter.get_distance(field_xyz)

                    if is_debug:
                        cv2.putText(
                            result_img,
                            "distance: {:.2f}".format(field_distance),
                            (int(xyxy_box[0]), int(xyxy_box[1])),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.5,
                            (0, 255, 122),
                            2,
                        )

                    car_id = car_list.get_car_id(label)
                    car_list_results.append([track_id, car_id, xywh_box, 1, center, field_xyz])
                    debug_results.append([center, car_id])
                    logger.info(
                        "loop %s result %s packaged car_id=%s center=%s cost=%.3fs",
                        loop_idx,
                        result_idx,
                        car_id,
                        center,
                        time.time() - stage_start,
                    )
                except Exception:
                    logger.exception("loop %s result %s processing failed", loop_idx, result_idx)
                    raise

            if is_debug and len(debug_results) > 0:
                timed_call(logger, f"loop {loop_idx} visualize", converter.vision_locator.visualize, debug_results)

        timed_call(logger, f"loop {loop_idx} car_list.update_car_info", car_list.update_car_info, car_list_results)
        all_infos = timed_call(logger, f"loop {loop_idx} car_list.get_all_info", car_list.get_all_info)
        my_car_infos = []
        enemy_car_infos = []

        for all_info in all_infos:
            track_id, car_id, center_xy, camera_xyz, field_xyz, color, is_valid = all_info
            if color == global_my_color:
                if track_id == -1:
                    continue
                my_car_infos.append(all_info)
            else:
                enemy_car_infos.append(all_info)
                if track_id != -1:
                    all_detections.append([frame_id] + list(all_info))

        logger.info(
            "loop %s split infos my=%s enemy=%s all_detections=%s",
            loop_idx,
            len(my_car_infos),
            len(enemy_car_infos),
            len(all_detections),
        )

        if messager is not None:
            timed_call(logger, f"loop {loop_idx} messager.update_enemy_car_infos", messager.update_enemy_car_infos, enemy_car_infos)
            timed_call(
                logger,
                f"loop {loop_idx} messager.update_our_car_infos",
                messager.update_our_car_infos,
                our_car_infos=my_car_infos,
            )

        for my_idx, my_car_info in enumerate(my_car_infos):
            try:
                my_track_id, my_car_id, my_center_xy, my_camera_xyz, my_field_xyz, my_color, my_is_valid = my_car_info
                if my_car_id != car_list.sentinel_id or not my_is_valid:
                    continue

                min_distance_car_id = -1
                min_distance = 1000
                min_distance_angle = -1
                my_reprojected_point = None

                for enemy_idx, enemy_car_info in enumerate(enemy_car_infos):
                    enemy_track_id, enemy_car_id, enemy_center_xy, enemy_camera_xyz, enemy_field_xyz, enemy_color, enemy_is_valid = enemy_car_info
                    if not enemy_is_valid or enemy_track_id == -1:
                        continue

                    distance = np.linalg.norm(np.array(my_field_xyz) - np.array(enemy_field_xyz))
                    logger.info(
                        "loop %s sentinel pair my_idx=%s enemy_idx=%s enemy_car_id=%s distance=%.3f",
                        loop_idx,
                        my_idx,
                        enemy_idx,
                        enemy_car_id,
                        distance,
                    )

                    if is_debug:
                        if len(my_camera_xyz) != 3 or len(enemy_camera_xyz) != 3:
                            logger.warning("loop %s skip reprojection due to invalid camera_xyz", loop_idx)
                            continue
                        my_camera_xyz_arr = np.array(my_camera_xyz, dtype=np.float64).reshape(1, 3)
                        my_reprojected_point = converter.camera_to_image(my_camera_xyz_arr)[0]
                        enemy_camera_xyz_arr = np.array(enemy_camera_xyz, dtype=np.float64).reshape(1, 3)
                        enemy_reprojected_point = converter.camera_to_image(enemy_camera_xyz_arr)[0]
                        cv2.circle(result_img, (int(my_reprojected_point[0]), int(my_reprojected_point[1])), 5, (0, 0, 255), -1)
                        cv2.circle(result_img, (int(enemy_reprojected_point[0]), int(enemy_reprojected_point[1])), 5, (0, 0, 255), -1)

                    if distance < car_list.sentinel_min_alert_distance or distance > car_list.sentinel_max_alert_distance:
                        continue

                    if distance < min_distance:
                        angle = np.arctan2(
                            enemy_field_xyz[1] - my_field_xyz[1],
                            enemy_field_xyz[0] - my_field_xyz[0],
                        ) * 180 / np.pi
                        min_distance = distance
                        min_distance_angle = angle
                        min_distance_car_id = enemy_car_id

                if min_distance_car_id != -1 and messager is not None:
                    quadrant = converter.angle_to_quadrant(min_distance_angle)
                    sentinel_alert_info = [min_distance_car_id, min_distance, quadrant]
                    timed_call(
                        logger,
                        f"loop {loop_idx} messager.update_sentinel_alert_info",
                        messager.update_sentinel_alert_info,
                        sentinel_alert_info,
                    )
            except Exception:
                logger.exception("loop %s sentinel processing failed", loop_idx)
                raise

        logger.info("loop %s finished in %.3fs", loop_idx, time.time() - loop_start)
        frame_id += 1
        time.sleep(args.poll_interval)

    logger.info(
        "full main loop probe finished none_count=%s/%s",
        none_count,
        args.main_loops,
    )
    logger.info("===== full main loop probe end =====")


def maybe_init_converter(args, capture, logger, debug_state):
    if args.skip_converter:
        logger.info("skip converter init")
        return None

    main_cfg = YAML().load(open(args.main_config, encoding="utf-8", mode="r"))
    global_my_color = main_cfg["global"]["my_color"]
    converter = Converter(global_my_color, args.converter_config)
    logger.info("converter init begin, this step may require interactive point selection")
    timed_call(logger, "converter.camera_to_field_init", converter.camera_to_field_init, capture)
    install_converter_hooks(converter, logger, debug_state)

    if args.mode == "video" and hasattr(capture, "cap"):
        capture.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        capture.current_frame = 0
        logger.info("video rewound to frame 0 after converter init")

    return converter


def maybe_init_messager(main_cfg, logger, args, debug_state):
    if args.skip_messager:
        logger.info("skip messager init")
        return None

    draw_queue = deque(maxlen=10)
    try:
        messager = Messager(main_cfg, draw_queue)
        install_messager_hooks(messager, logger, debug_state)
        logger.info("messager init success")
        return messager
    except Exception as exc:
        logger.warning("messager init failed, continue without it: %s", exc)
        return None


def parse_args():
    parser = argparse.ArgumentParser(description="Debug detector pipeline for 26_main.py")
    parser.add_argument("--mode", default="video", choices=["video", "camera"])
    parser.add_argument(
        "--video-path",
        default="/home/radar/Radar/Videos/test/MV-CH120-10UC+DA7548439/train1.mp4",
    )
    parser.add_argument("--detector-config", default="./configs/detector_config.yaml")
    parser.add_argument("--binocular-camera-cfg", default="./configs/bin_cam_config.yaml")
    parser.add_argument("--main-config", default="./configs/main_config.yaml")
    parser.add_argument("--converter-config", default="./configs/converter_config.yaml")
    parser.add_argument("--log-dir", default="./debug_logs")
    parser.add_argument("--sync-frames", type=int, default=3)
    parser.add_argument("--async-timeout", type=float, default=15.0)
    parser.add_argument("--poll-interval", type=float, default=0.2)
    parser.add_argument("--main-loops", type=int, default=20)
    parser.add_argument("--skip-converter", action="store_true")
    parser.add_argument("--skip-main-loop", action="store_true")
    parser.add_argument("--skip-messager", action="store_true")
    parser.add_argument("--disable-visual-debug", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    logger, log_path = build_logger(args.log_dir)
    debug_state = {"last_stage": "init"}

    logger.info("debug script start")
    logger.info("argv=%s", sys.argv)
    logger.info("log_path=%s", log_path)

    with open(log_path, "a", encoding="utf-8") as fh:
        faulthandler.enable(fh)
        faulthandler.dump_traceback_later(20, repeat=True, file=fh)

        detector = None
        capture = None
        messager = None
        try:
            detector = Detector(args.detector_config)
            install_detector_hooks(detector, logger, debug_state)

            capture = build_capture(args)
            install_capture_hooks(capture, logger, debug_state)

            main_cfg = YAML().load(open(args.main_config, encoding="utf-8", mode="r"))
            converter = maybe_init_converter(args, capture, logger, debug_state)
            car_list = CarList(main_cfg)
            messager = maybe_init_messager(main_cfg, logger, args, debug_state)

            sync_ok = run_sync_probe(detector, capture, logger, args)
            if not sync_ok:
                logger.error("sync probe failed, root cause is very likely before main loop")
                return 1

            if args.mode == "video" and hasattr(capture, "cap"):
                capture.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                capture.current_frame = 0
                logger.info("video rewound to frame 0 before async probe")

            async_ok, first_result_time = start_detector_and_wait_for_result(detector, capture, logger, debug_state, args)
            if not async_ok:
                logger.error("async probe failed")
                if debug_state.get("thread_exception"):
                    logger.error("thread_exception=%s", debug_state["thread_exception"])
                    logger.error("thread_traceback=\n%s", debug_state["thread_traceback"])
                else:
                    logger.error(
                        "no thread exception captured, likely blocked near last_stage=%s",
                        debug_state.get("last_stage"),
                    )
                return 2

            logger.info("async probe success, first_result_time=%.3fs", first_result_time)

            if messager is not None:
                timed_call(logger, "messager.start", messager.start)

            if not args.skip_main_loop:
                if converter is not None:
                    run_full_main_loop_probe(detector, converter, car_list, messager, logger, args, main_cfg)
                else:
                    run_basic_main_loop_probe(detector, car_list, logger, args)

            stop_detector(detector)
            logger.info("debug script finished successfully")
            return 0
        finally:
            faulthandler.cancel_dump_traceback_later()
            if detector is not None:
                try:
                    detector.stop_save_video()
                except Exception:
                    logger.exception("detector.stop_save_video failed")
                try:
                    stop_detector(detector)
                except Exception:
                    logger.exception("detector.stop failed")
            if capture is not None:
                try:
                    capture.release()
                except Exception:
                    logger.exception("capture.release failed")
            if messager is not None:
                try:
                    if hasattr(messager, "receiver") and messager.receiver is not None:
                        messager.receiver.stop()
                except Exception:
                    logger.exception("messager.receiver.stop failed")
                try:
                    messager.stop()
                except Exception:
                    logger.exception("messager.stop failed")


if __name__ == "__main__":
    raise SystemExit(main())
