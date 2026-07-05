#!/usr/bin/env python3
"""
pcd_player_debug.py
诊断版 PCD 播放器 - 自动对焦、详细日志、故障排查
"""

import os
import sys
import re
import time
import argparse
from glob import glob

import numpy as np
import open3d as o3d


def parse_frame_info(filename):
    basename = os.path.basename(filename)
    match = re.match(r"frame_(\d+)_t([\d\.]+)\.(pcd|npy)", basename)
    if not match:
        return None
    return int(match.group(1)), float(match.group(2))


def load_frame(filepath, ext):
    if ext == '.pcd':
        pcd = o3d.io.read_point_cloud(filepath)
        pts = np.asarray(pcd.points, dtype=np.float32)
    elif ext == '.npy':
        pts = np.load(filepath)
        if pts.ndim != 2 or pts.shape[1] != 3:
            pts = pts.reshape(-1, 3)
    else:
        raise ValueError(f"不支持的格式: {ext}")
    
    valid = (np.linalg.norm(pts, axis=1) > 0.001) & np.all(np.isfinite(pts), axis=1)
    pts = pts[valid]
    return pts


def discover_frames(record_dir, subfolder, prefer_format):
    pcd_dir = os.path.join(record_dir, subfolder, "pcd")
    npy_dir = os.path.join(record_dir, subfolder, "numpy")
    candidates = []
    
    for fmt, dir_path, ext in [('pcd', pcd_dir, '.pcd'), ('numpy', npy_dir, '.npy')]:
        if prefer_format not in ('both', fmt):
            continue
        if not os.path.isdir(dir_path):
            print(f"[信息] 目录不存在，跳过: {dir_path}")
            continue
        files = sorted(glob(os.path.join(dir_path, f"*{ext}")))
        print(f"[信息] 在 {dir_path} 找到 {len(files)} 个 {ext} 文件")
        for f in files:
            info = parse_frame_info(f)
            if info:
                candidates.append((*info, f, ext))
    
    if not candidates:
        print(f"[错误] 未找到任何帧文件")
        print(f"        已检查: {pcd_dir}, {npy_dir}")
        sys.exit(1)
    
    if prefer_format == 'both':
        by_frame = {}
        for frame_num, timestamp, filepath, ext in candidates:
            if frame_num not in by_frame or ext == '.pcd':
                by_frame[frame_num] = (frame_num, timestamp, filepath, ext)
        candidates = list(by_frame.values())
    
    candidates.sort(key=lambda x: x[0])
    return candidates


def diagnose_points(pts, label=""):
    """诊断点云数据"""
    n = len(pts)
    if n == 0:
        print(f"  [{label}] ⚠️ 空点云 (0 点)")
        return None
    
    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    center = (bbox_min + bbox_max) / 2
    size = bbox_max - bbox_min
    
    print(f"  [{label}] ✅ {n} 点 | 中心: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}] | "
          f"尺寸: [{size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}] | "
          f"范围: X=[{bbox_min[0]:.2f},{bbox_max[0]:.2f}] Y=[{bbox_min[1]:.2f},{bbox_max[1]:.2f}] Z=[{bbox_min[2]:.2f},{bbox_max[2]:.2f}]")
    
    return {
        'center': center,
        'size': size,
        'min': bbox_min,
        'max': bbox_max
    }


def fit_view_to_points(vis, pts, zoom=0.8):
    """强制将相机对准点云包围盒"""
    if len(pts) == 0:
        return
    
    center = pts.mean(axis=0)
    max_dim = np.ptp(pts, axis=0).max()
    if max_dim < 0.01:
        max_dim = 1.0
    
    ctr = vis.get_view_control()
    # 设置观察点
    ctr.set_lookat(center.tolist())
    # 设置相机位置（从斜上方看）
    front = np.array([1, 1, 0.5])
    front = front / np.linalg.norm(front) * max_dim * 2.5
    camera_pos = center + front
    ctr.set_front((-front).tolist())
    ctr.set_up([0, 0, 1])
    ctr.set_zoom(zoom)
    
    # 强制重置相机
    ctr.change_field_of_view(step=0.01)
    ctr.change_field_of_view(step=-0.01)


class PCDPlayerDebug:
    def __init__(self, frames, args):
        self.frames = frames
        self.args = args
        self.total = len(frames)
        self.current_idx = 0
        self.paused = False
        self.step_request = False
        self.loop = args.loop
        
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.pcd = o3d.geometry.PointCloud()
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.bbox_lines = o3d.geometry.LineSet()
        
        self._register_keys()
    
    def _register_keys(self):
        self.vis.register_key_callback(ord(" "), lambda vis: self._toggle_pause())
        self.vis.register_key_callback(ord("N"), lambda vis: self._step())
        self.vis.register_key_callback(ord("R"), lambda vis: self._toggle_loop())
        self.vis.register_key_callback(ord("Q"), lambda vis: self._quit())
        self.vis.register_key_callback(ord("F"), lambda vis: self._force_fit())
        self.vis.register_key_callback(ord("n"), lambda vis: self._step())
        self.vis.register_key_callback(ord("r"), lambda vis: self._toggle_loop())
        self.vis.register_key_callback(ord("q"), lambda vis: self._quit())
        self.vis.register_key_callback(ord("f"), lambda vis: self._force_fit())
    
    def _toggle_pause(self):
        self.paused = not self.paused
        print(f"\n [{'暂停' if self.paused else '播放'}]")
    
    def _step(self):
        self.step_request = True
        self.paused = False
    
    def _toggle_loop(self):
        self.loop = not self.loop
        print(f"\n [循环{'开' if self.loop else '关'}]")
    
    def _quit(self):
        print("\n [退出]")
        self.vis.close()
        sys.exit(0)
    
    def _force_fit(self):
        """手动触发重新对焦"""
        print("\n [强制对焦到当前帧]")
        self._fit_called = True
    
    def _update_geometry(self, pts):
        if len(pts) == 0:
            self.pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
            self.bbox_lines.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
            self.bbox_lines.lines = o3d.utility.Vector2iVector(np.empty((0, 2)))
            return
        
        self.pcd.points = o3d.utility.Vector3dVector(pts)
        
        # 高度着色
        z = pts[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 0.001:
            t = np.zeros_like(z)
        else:
            t = np.clip((z - z_min) / (z_max - z_min), 0, 1)
        
        colors = np.zeros((len(pts), 3))
        for i, ti in enumerate(t):
            if ti < 0.25:
                colors[i] = [0, ti * 4, 1]
            elif ti < 0.5:
                colors[i] = [0, 1, 1 - (ti - 0.25) * 4]
            elif ti < 0.75:
                colors[i] = [(ti - 0.5) * 4, 1, 0]
            else:
                colors[i] = [1, 1 - (ti - 0.75) * 4, 0]
        self.pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 绘制包围盒线框（帮助确认点云位置）
        bbox_min = pts.min(axis=0) - 0.05
        bbox_max = pts.max(axis=0) + 0.05
        vertices = np.array([
            [bbox_min[0], bbox_min[1], bbox_min[2]],
            [bbox_max[0], bbox_min[1], bbox_min[2]],
            [bbox_max[0], bbox_max[1], bbox_min[2]],
            [bbox_min[0], bbox_max[1], bbox_min[2]],
            [bbox_min[0], bbox_min[1], bbox_max[2]],
            [bbox_max[0], bbox_min[1], bbox_max[2]],
            [bbox_max[0], bbox_max[1], bbox_max[2]],
            [bbox_min[0], bbox_max[1], bbox_max[2]],
        ])
        lines = np.array([
            [0,1],[1,2],[2,3],[3,0],
            [4,5],[5,6],[6,7],[7,4],
            [0,4],[1,5],[2,6],[3,7]
        ])
        self.bbox_lines.points = o3d.utility.Vector3dVector(vertices)
        self.bbox_lines.lines = o3d.utility.Vector2iVector(lines)
        self.bbox_lines.colors = o3d.utility.Vector3dVector([[1,1,1]] * len(lines))
    
    def run(self):
        self.vis.create_window(
            window_name="PCD DEBUG | 空格:暂停 N:步进 F:对焦 R:循环 Q:退出",
            width=1400, height=900
        )
        
        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(self.coord_frame)
        self.vis.add_geometry(self.bbox_lines)
        
        render = self.vis.get_render_option()
        render.point_size = 4.0  # 更大点
        render.background_color = np.array([0.05, 0.05, 0.1])
        render.show_coordinate_frame = True
        
        print("\n" + "=" * 60)
        print("诊断版播放器")
        print("  空格: 暂停/继续")
        print("  N: 步进一帧")
        print("  F: 强制重新对焦到当前帧（看不见时点这个）")
        print("  R: 循环开关")
        print("  Q: 退出")
        print("=" * 60)
        
        # 加载第一帧用于初始对焦
        first_info = self.frames[0]
        first_pts = load_frame(first_info[2], first_info[3])
        print(f"\n[首帧诊断] {os.path.basename(first_info[2])}")
        diag = diagnose_points(first_pts, "首帧")
        
        self._update_geometry(first_pts)
        self.vis.update_geometry(self.pcd)
        self.vis.update_geometry(self.bbox_lines)
        
        # 强制对焦
        if diag:
            fit_view_to_points(self.vis, first_pts, zoom=0.6)
            print(f"[初始视角] 已对焦到首帧中心")
        
        self.vis.poll_events()
        self.vis.update_renderer()
        
        last_time = time.time()
        self._fit_called = False
        
        while self.vis.poll_events():
            if self.step_request:
                self.step_request = False
            elif self.paused:
                if self._fit_called:
                    if len(first_pts) > 0:
                        fit_view_to_points(self.vis, first_pts, zoom=0.6)
                    self._fit_called = False
                self.vis.update_renderer()
                time.sleep(0.01)
                continue
            
            if self.current_idx >= self.total:
                if self.loop:
                    self.current_idx = 0
                    print("\n[循环]")
                else:
                    print("\n[完毕] 按 Q 退出")
                    self.paused = True
                    continue
            
            frame_info = self.frames[self.current_idx]
            try:
                pts = load_frame(frame_info[2], frame_info[3])
            except Exception as e:
                print(f"\n[错误] 加载失败: {e}")
                self.current_idx += 1
                continue
            
            # 打印诊断信息（每帧都打印，方便排查）
            diag = diagnose_points(pts, f"帧 {self.current_idx+1}/{self.total}")
            
            self._update_geometry(pts)
            self.vis.update_geometry(self.pcd)
            self.vis.update_geometry(self.bbox_lines)
            
            # 自动对焦（如果点云中心偏离太远）
            if diag and (self.current_idx == 0 or self._fit_called):
                fit_view_to_points(self.vis, pts, zoom=0.6)
                self._fit_called = False
            
            self.vis.poll_events()
            self.vis.update_renderer()
            
            # 帧率控制
            wait = 1.0 / self.args.fps
            time.sleep(max(0, wait - (time.time() - last_time)))
            last_time = time.time()
            
            self.current_idx += 1
        
        self.vis.destroy_window()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('record_dir', help='录制目录')
    parser.add_argument('--subfolder', default='world', choices=['raw_lidar', 'world'])
    parser.add_argument('--format', default='both', choices=['pcd', 'numpy', 'both'])
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--loop', action='store_true')
    args = parser.parse_args()
    
    if not os.path.isdir(args.record_dir):
        print(f"[错误] 目录不存在: {args.record_dir}")
        sys.exit(1)
    
    print(f"[扫描] {args.record_dir}/{args.subfolder}")
    frames = discover_frames(args.record_dir, args.subfolder, args.format)
    print(f"[结果] 共 {len(frames)} 帧")
    
    if len(frames) == 0:
        sys.exit(1)
    
    first = frames[0]
    last = frames[-1]
    print(f"[范围] #{first[0]:06d} ~ #{last[0]:06d}, 时长 {last[1]-first[1]:.2f}s")
    
    player = PCDPlayerDebug(frames, args)
    player.run()


if __name__ == '__main__':
    main()