import argparse
import os
import sys
import time

import cv2


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_INPUT = os.path.join(REPO_ROOT, "data", "201357.avi")
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "data", "final.mp4")


def parse_args():
    parser = argparse.ArgumentParser(description="Trim the first part of a video and save the remainder.")
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Input video path")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output video path")
    parser.add_argument("--skip-seconds", type=float, default=80.0, help="Seconds to remove from the beginning")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output if it already exists")
    return parser.parse_args()


def open_writer(output_path, fps, width, height):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    codecs = ("mp4v", "avc1", "XVID")
    for codec in codecs:
        writer = cv2.VideoWriter(
            output_path,
            cv2.VideoWriter_fourcc(*codec),
            fps,
            (width, height),
        )
        if writer.isOpened():
            print(f"Using codec: {codec}")
            return writer
        writer.release()
    raise RuntimeError(f"Unable to open output video writer: {output_path}")


def trim_video(input_path, output_path, skip_seconds, overwrite=False):
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Input video does not exist: {input_path}")
    if os.path.exists(output_path) and not overwrite:
        raise FileExistsError(f"Output already exists, use --overwrite to replace it: {output_path}")

    capture = cv2.VideoCapture(input_path)
    if not capture.isOpened():
        raise RuntimeError(f"Unable to open input video: {input_path}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if fps <= 0 or width <= 0 or height <= 0:
        capture.release()
        raise RuntimeError("Invalid video metadata")

    skip_frames = int(round(skip_seconds * fps))
    if frame_count > 0 and skip_frames >= frame_count:
        capture.release()
        raise RuntimeError(
            f"Skip duration is longer than the video: skip_frames={skip_frames}, frame_count={frame_count}"
        )

    capture.set(cv2.CAP_PROP_POS_FRAMES, skip_frames)
    writer = open_writer(output_path, fps, width, height)

    print(f"Input: {input_path}")
    print(f"Output: {output_path}")
    print(f"FPS: {fps:.3f}, size: {width}x{height}, total frames: {frame_count}")
    print(f"Skipping first {skip_seconds:.3f}s ({skip_frames} frames)")

    written = 0
    start = time.time()
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            writer.write(frame)
            written += 1

            if written % 300 == 0:
                elapsed = max(time.time() - start, 1e-6)
                speed = written / elapsed
                print(f"Written {written} frames ({speed:.1f} fps)", flush=True)
    finally:
        writer.release()
        capture.release()

    print(f"Done. Written frames: {written}")


def main():
    args = parse_args()
    trim_video(args.input, args.output, args.skip_seconds, args.overwrite)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
