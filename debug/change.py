from pathlib import Path

import numpy as np
import yaml

# 读取/root/rm/radar-detect/Lidar/rm25_points.yaml的点，根据下面的旋转平移矩阵，读取然后转换坐标
# 新的坐标保存在/root/rm/radar-detect/debug/dji_point.yaml中
R = np.array([[0.001, 0.0, 0.0],
              [0.0, 0.001, 0.0],
              [0.0, 0.0, 0.001]])
T = np.array([14.000, -9.14434, 0])

BASE_DIR = Path(__file__).resolve().parents[1]
INPUT_PATH = BASE_DIR / "debug" / "dji.yaml"
OUTPUT_PATH = BASE_DIR / "debug" / "26_point.yaml"


def transform_point(point):
    xyz = np.array([point["x"], point["y"], point["z"]], dtype=float)
    transformed = R @ xyz + T

    new_point = dict(point)
    new_point["x"] = round(float(transformed[0]), 6)
    new_point["y"] = round(float(transformed[1]), 6)
    new_point["z"] = round(float(transformed[2]), 6)
    return new_point


def transform_data(data):
    if isinstance(data, dict):
        if {"x", "y", "z"}.issubset(data.keys()):
            return transform_point(data)
        return {key: transform_data(value) for key, value in data.items()}

    if isinstance(data, list):
        return [transform_data(item) for item in data]

    return data


def main():
    with INPUT_PATH.open("r", encoding="utf-8") as file:
        points_data = yaml.safe_load(file)

    transformed_data = transform_data(points_data)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_PATH.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            transformed_data,
            file,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )


if __name__ == "__main__":
    main()
