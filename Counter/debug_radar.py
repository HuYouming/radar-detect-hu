import argparse
import glob
import os
import time
import sys

import numpy as np
import open3d as o3d
import yaml

# ============ Calibration parameters ============
TRANSITION_VECTOR = np.array([-0.04067, -0.24053, 0.0053])
ROTATION_MATRIX = np.array([
	[1, 0, 0],
	[0, 1, 0],
	[0, 0, 1]
])

# ============ Clustering parameters ============
EPS = 0.5
MIN_POINTS = 15
MAX_RADIUS = 2

# ============ Field range filtering ============
MIN_HEIGHT = 1.8
MAX_HEIGHT = 3.6
MIN_X = 5.0
MAX_X = 27.5
MIN_Y = 0.1
MAX_Y = 7.0

# ============ Queue parameters ============
MIN_FRAME_COUNT = 5
MAX_DEQUE_SIZE = 10
CALIBRATION_FRAME_COUNT = 15

# ============ Background removal ============
MAP_OVERLAP_THRESHOLD = 0.6
MAP_DISTANCE_THRESHOLD = 0.15

# ============ Flatness parameters ============
MIN_FLATNESS = 1.0
FLATNESS_WEIGHT = 0.20
FLATNESS_BONUS_DYNAMIC = 0.15

# ============ Tracking parameters ============
VELOCITY_THRESHOLD = 0.3
HISTORY_TRACKING_DISTANCE = 3.0
BACKGROUND_PENALTY_WEIGHT = 0.8
LOCK_TIMEOUT = 3.0
LOCK_SCORE_MARGIN = 0.15
LOCK_MAX_STATIC_FRAMES = 150

# ============ Angle parameters ============
YAW_OFFSET = 0.0
PITCH_OFFSET = -12.0


class TrackedTarget:
	"""Locked target tracked across frames."""
	def __init__(self, cluster_id, center_world, cluster_points, score, flatness, velocity, timestamp):
		self.cluster_id = cluster_id
		self.center_world = center_world
		self.cluster_points = cluster_points
		self.initial_score = score
		self.current_score = score
		self.flatness = flatness
		self.velocity = velocity
		self.first_seen = timestamp
		self.last_seen = timestamp
		self.lost_count = 0
		self.static_count = 0
		self.is_active = True
		self.history_positions = [center_world.copy()]

	def update(self, center_world, cluster_points, score, flatness, velocity, timestamp):
		self.center_world = center_world
		self.cluster_points = cluster_points
		self.current_score = score
		self.flatness = flatness
		self.velocity = velocity
		self.last_seen = timestamp
		self.lost_count = 0
		self.history_positions.append(center_world.copy())
		if len(self.history_positions) > 20:
			self.history_positions.pop(0)

		if velocity < VELOCITY_THRESHOLD:
			self.static_count += 1
		else:
			self.static_count = 0

	def mark_lost(self):
		self.lost_count += 1
		if self.lost_count > 30:
			self.is_active = False

	def get_smoothed_position(self):
		if len(self.history_positions) < 3:
			return self.center_world

		positions = np.array(self.history_positions)
		weights = np.exp(np.linspace(-1, 0, len(positions)))
		weights /= weights.sum()
		return np.average(positions, axis=0, weights=weights)

	def should_release(self, current_time):
		if self.lost_count > 30:
			return True, "lost too long"
		if self.static_count > LOCK_MAX_STATIC_FRAMES and self.initial_score < 0.6:
			return True, "static and low initial score"
		if current_time - self.last_seen > LOCK_TIMEOUT:
			return True, "timeout"
		return False, "active"


class OfflineLidarDebugger:
	def __init__(self, pcd_dir, map_path, world_points_path, calib_frames, playback_delay):
		self.pcd_dir = pcd_dir
		self.map_path = map_path
		self.world_points_path = world_points_path
		self.calib_frames = calib_frames
		self.playback_delay = playback_delay

		self.world_feature_points = self._load_world_feature_points()
		self.map_pcd_world = None
		self.map_kdtree = None

		self.point_queue = []
		self.T_lidar_to_world = None
		self.locked_target = None
		self.next_cluster_id = 0

		self.lidar2gimbal = np.eye(4)
		self.lidar2gimbal[:3, :3] = ROTATION_MATRIX
		self.lidar2gimbal[:3, 3] = TRANSITION_VECTOR

		self.vis = None
		self.vis_clusters_pcd = o3d.geometry.PointCloud()
		self.vis_map_pcd = None
		self.vis_range_box = None
		self.vis_tracked_bbox = o3d.geometry.LineSet()

	def _load_world_feature_points(self):
		if self.world_points_path and os.path.exists(self.world_points_path):
			with open(self.world_points_path, 'r') as f:
				config = yaml.safe_load(f)
			points = np.array(config['points'])
			return points

		return np.array([
			[0, 0, 0],
			[15, 0, 0],
			[0, 28, 0]
		], dtype=np.float64)

	def _load_pcd_file_list(self):
		if not os.path.isdir(self.pcd_dir):
			raise FileNotFoundError(f"PCD directory not found: {self.pcd_dir}")
		pcd_files = sorted(glob.glob(os.path.join(self.pcd_dir, "*.pcd")))
		if len(pcd_files) == 0:
			raise FileNotFoundError(f"No PCD files in: {self.pcd_dir}")
		return pcd_files

	def _read_pcd_points(self, pcd_path):
		pcd = o3d.io.read_point_cloud(pcd_path)
		if len(pcd.points) == 0:
			return None
		xyz = np.asarray(pcd.points, dtype=np.float32)
		valid_mask = (np.linalg.norm(xyz, axis=1) > 0.01) & np.all(np.isfinite(xyz), axis=1)
		xyz = xyz[valid_mask]
		if len(xyz) == 0:
			return None
		return xyz

	def load_map(self):
		if not self.map_path:
			return None
		if not os.path.exists(self.map_path):
			raise FileNotFoundError(f"Map file not found: {self.map_path}")

		map_pcd = o3d.io.read_point_cloud(self.map_path)
		if len(map_pcd.points) == 0:
			raise ValueError("Map point cloud is empty")

		pts = np.asarray(map_pcd.points)
		max_range = np.max(np.abs(pts))
		if max_range > 1000:
			map_pcd.points = o3d.utility.Vector3dVector(pts / 1000.0)

		map_pcd_down = map_pcd.voxel_down_sample(voxel_size=0.03)
		self.map_pcd_world = map_pcd_down
		self.map_kdtree = o3d.geometry.KDTreeFlann(map_pcd_down)
		return map_pcd_down

	@staticmethod
	def kabsch_algorithm(P, Q):
		if P.shape != Q.shape or P.shape[0] < 3:
			raise ValueError("Need at least 3 point pairs for Kabsch")
		centroid_P = np.mean(P, axis=0)
		centroid_Q = np.mean(Q, axis=0)
		P_centered = P - centroid_P
		Q_centered = Q - centroid_Q
		H = P_centered.T @ Q_centered
		U, _, Vt = np.linalg.svd(H)
		R = Vt.T @ U.T
		if np.linalg.det(R) < 0:
			Vt[-1, :] *= -1
			R = Vt.T @ U.T
		t = centroid_Q - R @ centroid_P
		error = np.mean(np.linalg.norm(Q - ((R @ P.T).T + t), axis=1))
		return R, t, error

	@staticmethod
	def build_transform_matrix(R, t):
		T = np.eye(4, dtype=np.float64)
		T[:3, :3] = R
		T[:3, 3] = t
		return T

	@staticmethod
	def color_by_height(points):
		z = points[:, 2]
		z_min = np.min(z)
		z_max = np.max(z)
		if z_max - z_min < 0.001:
			t = np.zeros_like(z)
		else:
			t = np.clip((z - z_min) / (z_max - z_min), 0, 1)
		colors = np.zeros((len(points), 3))
		for i, ti in enumerate(t):
			if ti < 0.25:
				colors[i] = [0, ti * 4, 1]
			elif ti < 0.5:
				colors[i] = [0, 1, 1 - (ti - 0.25) * 4]
			elif ti < 0.75:
				colors[i] = [(ti - 0.5) * 4, 1, 0]
			else:
				colors[i] = [1, 1 - (ti - 0.75) * 4, 0]
		return colors

	@staticmethod
	def create_range_box(min_x, max_x, min_y, max_y, min_z, max_z, color=(1.0, 0.8, 0.0)):
		"""Create a wireframe bounding box for the filtering range."""
		# 8 corners of the AABB
		corners = np.array([
			[min_x, min_y, min_z],
			[max_x, min_y, min_z],
			[max_x, max_y, min_z],
			[min_x, max_y, min_z],
			[min_x, min_y, max_z],
			[max_x, min_y, max_z],
			[max_x, max_y, max_z],
			[min_x, max_y, max_z],
		], dtype=np.float64)

		# 12 edges as line indices
		lines = np.array([
			[0, 1], [1, 2], [2, 3], [3, 0],  # bottom face
			[4, 5], [5, 6], [6, 7], [7, 4],  # top face
			[0, 4], [1, 5], [2, 6], [3, 7],  # vertical edges
		], dtype=np.int32)

		line_set = o3d.geometry.LineSet()
		line_set.points = o3d.utility.Vector3dVector(corners)
		line_set.lines = o3d.utility.Vector2iVector(lines)
		line_set.colors = o3d.utility.Vector3dVector(np.tile(color, (len(lines), 1)))
		return line_set

	@staticmethod
	def create_bounding_box(points, color=(0.0, 1.0, 0.0)):
		min_bound = np.min(points, axis=0)
		max_bound = np.max(points, axis=0)
		padding = 0.05
		min_bound -= padding
		max_bound += padding

		vertices = np.array([
			[min_bound[0], min_bound[1], min_bound[2]],
			[max_bound[0], min_bound[1], min_bound[2]],
			[max_bound[0], max_bound[1], min_bound[2]],
			[min_bound[0], max_bound[1], min_bound[2]],
			[min_bound[0], min_bound[1], max_bound[2]],
			[max_bound[0], min_bound[1], max_bound[2]],
			[max_bound[0], max_bound[1], max_bound[2]],
			[min_bound[0], max_bound[1], max_bound[2]],
		])

		lines = np.array([
			[0, 1], [1, 2], [2, 3], [3, 0],
			[4, 5], [5, 6], [6, 7], [7, 4],
			[0, 4], [1, 5], [2, 6], [3, 7]
		])

		bbox = o3d.geometry.LineSet()
		bbox.points = o3d.utility.Vector3dVector(vertices)
		bbox.lines = o3d.utility.Vector2iVector(lines)
		bbox.colors = o3d.utility.Vector3dVector(np.tile(color, (len(lines), 1)))
		return bbox

	def _calibrate_from_points(self, collected_points):
		if len(collected_points) == 0:
			raise ValueError("No points for calibration")

		merged_points = np.vstack(collected_points)
		merged_pcd = o3d.geometry.PointCloud()
		merged_pcd.points = o3d.utility.Vector3dVector(merged_points)

		if len(merged_points) > 50000:
			voxel_size = 0.05
			while True:
				display_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)
				if len(display_pcd.points) <= 50000 or voxel_size > 1.0:
					break
				voxel_size += 0.05
		else:
			display_pcd = merged_pcd

		vis_edit = o3d.visualization.VisualizerWithEditing()
		vis_edit.create_window(
			window_name="Calibration: pick 3 feature points (Shift+Left, Q to finish)",
			width=1400,
			height=900
		)

		opt = vis_edit.get_render_option()
		if opt is None:
			vis_edit.destroy_window()
			raise RuntimeError(
				"Open3D window creation failed. "
				"Check GPU driver or DISPLAY/X11 forwarding."
			)

		display_pts = np.asarray(display_pcd.points)
		display_pcd.colors = o3d.utility.Vector3dVector(self.color_by_height(display_pts))
		vis_edit.add_geometry(display_pcd)
		vis_edit.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))
		opt.point_size = 3.0
		opt.background_color = np.array([0.1, 0.1, 0.1])

		ctr = vis_edit.get_view_control()
		ctr.set_zoom(0.5)
		ctr.set_lookat([0, 0, 2])

		vis_edit.run()
		picked_indices = vis_edit.get_picked_points()
		vis_edit.destroy_window()

		if len(picked_indices) != 3:
			raise ValueError(f"Need 3 points, got {len(picked_indices)}")

		lidar_points = np.asarray(display_pcd.points)[picked_indices]
		R, t, error = self.kabsch_algorithm(lidar_points, self.world_feature_points)
		self.T_lidar_to_world = self.build_transform_matrix(R, t)
		return error

	def transform_lidar_to_world(self, points_lidar):
		ones = np.ones((points_lidar.shape[0], 1))
		points_homo = np.hstack([points_lidar, ones])
		points_world_homo = (self.T_lidar_to_world @ points_homo.T).T
		return points_world_homo[:, :3]

	def transform_points_lidar_to_gimbal(self, points_lidar):
		ones = np.ones((points_lidar.shape[0], 1))
		points_homo = np.hstack([points_lidar, ones])
		points_gimbal_homo = (self.lidar2gimbal @ points_homo.T).T
		return points_gimbal_homo[:, :3]

	def calculate_yaw_pitch(self, center_gimbal):
		x, y, z = center_gimbal
		distance_xy = np.sqrt(x ** 2 + y ** 2)
		if distance_xy < 0.001:
			return 0.0, 0.0
		yaw = -(np.arctan2(y, x))
		pitch = np.arctan2(z, distance_xy)
		return yaw, pitch

	def remove_background_points(self, points_world):
		if self.map_kdtree is None:
			return points_world

		foreground_mask = np.ones(len(points_world), dtype=bool)
		batch_size = 100
		for i in range(0, len(points_world), batch_size):
			batch = points_world[i:i + batch_size]
			for j, pt in enumerate(batch):
				k, _, _ = self.map_kdtree.search_radius_vector_3d(pt, MAP_DISTANCE_THRESHOLD)
				if k > 3:
					foreground_mask[i + j] = False
		return points_world[foreground_mask]

	def check_cluster_background_overlap(self, cluster_points_world):
		if self.map_kdtree is None or len(cluster_points_world) == 0:
			return 0.0, False

		query_points = cluster_points_world
		if len(cluster_points_world) > 500:
			pcd_temp = o3d.geometry.PointCloud()
			pcd_temp.points = o3d.utility.Vector3dVector(cluster_points_world)
			pcd_temp = pcd_temp.voxel_down_sample(voxel_size=0.05)
			query_points = np.asarray(pcd_temp.points)

		map_near_count = 0
		for pt in query_points:
			k, _, _ = self.map_kdtree.search_radius_vector_3d(pt, MAP_DISTANCE_THRESHOLD)
			if k > 0:
				map_near_count += 1

		overlap_ratio = map_near_count / len(query_points) if len(query_points) > 0 else 0.0
		is_background = overlap_ratio > MAP_OVERLAP_THRESHOLD
		return overlap_ratio, is_background

	def calculate_score(self, point_count, height, radius, center, velocity,
							  overlap_ratio, flatness_score, flatness_ratio):
		velocity_score = min(velocity / 3.0, 1.0)
		dynamic_score = velocity_score * 0.30

		flatness_base = flatness_score * 0.15
		flatness_bonus = FLATNESS_BONUS_DYNAMIC if velocity > 0.5 and flatness_ratio > 5.0 else 0.0
		flatness_total = min(flatness_base + flatness_bonus, 0.20)

		point_score = min(point_count / 50.0, 1.0) * 0.15
		height_score = np.exp(-((height - 2.5) ** 2) / 0.5) * 0.15
		radius_score = np.exp(-((radius - 0.5) ** 2) / 0.2) * 0.10

		distance = np.linalg.norm(center)
		distance_score = max(0, 1.0 - distance / 50.0) * 0.10

		background_penalty = 1.0 - (overlap_ratio * BACKGROUND_PENALTY_WEIGHT)
		background_penalty = max(0.1, background_penalty)

		if velocity > 1.0:
			background_penalty = 1.0 - (overlap_ratio * BACKGROUND_PENALTY_WEIGHT * 0.3)
			background_penalty = max(0.5, background_penalty)

		base_score = dynamic_score + flatness_total + point_score + height_score + radius_score + distance_score
		return base_score * background_penalty

	def find_matching_cluster(self, candidates, locked_target):
		if locked_target is None or not locked_target.is_active:
			return None, float("inf")

		best_idx = None
		best_dist = HISTORY_TRACKING_DISTANCE
		for i, cand in enumerate(candidates):
			dist = np.linalg.norm(cand['center'] - locked_target.center_world)
			if dist < best_dist:
				height_diff = abs(cand['center'][2] - locked_target.center_world[2])
				if height_diff < 1.0:
					best_dist = dist
					best_idx = i

		return best_idx, best_dist

	def should_switch_target(self, locked_target, best_candidate, current_time):
		if locked_target is None or not locked_target.is_active:
			return True, "no active lock"

		locked_current_score = locked_target.current_score
		new_score = best_candidate['score']

		if new_score > locked_current_score + LOCK_SCORE_MARGIN:
			if best_candidate['velocity'] > VELOCITY_THRESHOLD or best_candidate['flatness_ratio'] > MIN_FLATNESS:
				return True, "higher score"

		if locked_target.lost_count > 10 and new_score > 0.3:
			return True, "locked target lost"

		if locked_target.static_count > LOCK_MAX_STATIC_FRAMES and locked_target.initial_score < 0.6:
			if new_score > locked_current_score:
				return True, "locked target static and low score"

		return False, "keep locked"

	def calculate_flatness(self, cluster_points):
		if len(cluster_points) < 3:
			return 0.0, 0.0

		x_min, x_max = np.min(cluster_points[:, 0]), np.max(cluster_points[:, 0])
		y_min, y_max = np.min(cluster_points[:, 1]), np.max(cluster_points[:, 1])
		z_min, z_max = np.min(cluster_points[:, 2]), np.max(cluster_points[:, 2])

		x_span = x_max - x_min
		y_span = y_max - y_min
		z_span = z_max - z_min

		z_span_safe = max(z_span, 0.01)
		xy_area = x_span * y_span
		flatness_ratio = xy_area / z_span_safe

		flatness_score = np.tanh(flatness_ratio / 5.0)
		flatness_bonus = FLATNESS_BONUS_DYNAMIC if flatness_ratio > 5.0 else 0.0
		flatness_score = min(flatness_score * FLATNESS_WEIGHT + flatness_bonus,
						FLATNESS_WEIGHT + FLATNESS_BONUS_DYNAMIC)
		return flatness_score, flatness_ratio

	def process_clusters(self, points_world):
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(points_world)
		pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
		down_points = np.asarray(pcd_down.points)

		if len(down_points) < MIN_POINTS:
			return down_points, np.zeros((len(down_points), 3)), None, None

		labels = np.array(pcd_down.cluster_dbscan(eps=EPS, min_points=MIN_POINTS))
		unique_labels = np.unique(labels)
		current_time = time.time()

		candidates = []
		colors = np.zeros((len(down_points), 3))

		for label in unique_labels:
			if label == -1:
				colors[labels == label] = [0.4, 0.4, 0.4]
				continue

			indices = np.where(labels == label)[0]
			cluster_points = down_points[indices]

			if len(cluster_points) < MIN_POINTS:
				colors[indices] = [0.2, 0.2, 0.2]
				continue

			center = np.mean(cluster_points, axis=0)
			x_mean, y_mean, z_mean = center

			if not (MIN_X <= x_mean <= MAX_X and MIN_Y <= y_mean <= MAX_Y and
					MIN_HEIGHT <= z_mean <= MAX_HEIGHT):
				colors[indices] = [0.2, 0.2, 0.2]
				continue

			distances = np.linalg.norm(cluster_points - center, axis=1)
			max_radius = np.max(distances)
			if max_radius > MAX_RADIUS:
				colors[indices] = [0.2, 0.2, 0.2]
				continue

			flatness_score, flatness_ratio = self.calculate_flatness(cluster_points)
			if flatness_ratio < MIN_FLATNESS:
				colors[indices] = [0.2, 0.2, 0.2]
				continue

			overlap_ratio, _ = self.check_cluster_background_overlap(cluster_points)
			velocity = 0.0

			candidates.append({
				'points': cluster_points,
				'center': center,
				'flatness_score': flatness_score,
				'flatness_ratio': flatness_ratio,
				'max_radius': max_radius,
				'overlap_ratio': overlap_ratio,
				'velocity': velocity,
				'score': 0.0,
			})

			rng = np.random.RandomState(seed=int(label) + 7)
			base_color = rng.rand(3)
			strength = min(max(flatness_score / max(FLATNESS_WEIGHT, 1e-6), 0.2), 1.0)
			colors[indices] = base_color * strength

		# Update velocities and scores
		for cand in candidates:
			if self.locked_target is not None and self.locked_target.is_active:
				dist = np.linalg.norm(cand['center'] - self.locked_target.center_world)
				if dist < HISTORY_TRACKING_DISTANCE:
					dt = current_time - self.locked_target.last_seen
					if dt > 0.001:
						cand['velocity'] = np.linalg.norm((cand['center'] - self.locked_target.center_world)[:2]) / dt
					else:
						cand['velocity'] = self.locked_target.velocity
				else:
					cand['velocity'] = 0.5
			else:
				cand['velocity'] = 0.5

			cand['score'] = self.calculate_score(
				point_count=len(cand['points']),
				height=cand['center'][2],
				radius=cand['max_radius'],
				center=cand['center'],
				velocity=cand['velocity'],
				overlap_ratio=cand['overlap_ratio'],
				flatness_score=cand['flatness_score'],
				flatness_ratio=cand['flatness_ratio']
			)

		# Locking logic
		if self.locked_target is not None and self.locked_target.is_active:
			matched_idx, _ = self.find_matching_cluster(candidates, self.locked_target)
			if matched_idx is not None:
				cand = candidates[matched_idx]
				self.locked_target.update(
					center_world=cand['center'],
					cluster_points=cand['points'],
					score=cand['score'],
					flatness=cand['flatness_ratio'],
					velocity=cand['velocity'],
					timestamp=current_time
				)
			else:
				self.locked_target.mark_lost()
				should_release, _ = self.locked_target.should_release(current_time)
				if should_release:
					self.locked_target = None

		if (self.locked_target is None or not self.locked_target.is_active) and len(candidates) > 0:
			candidates.sort(key=lambda x: x['score'], reverse=True)
			best = candidates[0]
			self.next_cluster_id += 1
			self.locked_target = TrackedTarget(
				cluster_id=self.next_cluster_id,
				center_world=best['center'],
				cluster_points=best['points'],
				score=best['score'],
				flatness=best['flatness_ratio'],
				velocity=best['velocity'],
				timestamp=current_time
			)
		elif self.locked_target is not None and self.locked_target.is_active and len(candidates) > 0:
			candidates.sort(key=lambda x: x['score'], reverse=True)
			best_new = candidates[0]
			should_switch, _ = self.should_switch_target(self.locked_target, best_new, current_time)
			if should_switch:
				self.next_cluster_id += 1
				self.locked_target = TrackedTarget(
					cluster_id=self.next_cluster_id,
					center_world=best_new['center'],
					cluster_points=best_new['points'],
					score=best_new['score'],
					flatness=best_new['flatness_ratio'],
					velocity=best_new['velocity'],
					timestamp=current_time
				)

		return down_points, colors, self.locked_target, current_time

	def _init_visualization(self):
		self.vis = o3d.visualization.Visualizer()
		self.vis.create_window(
			window_name="Offline LiDAR Clustering",
			width=1400,
			height=900
		)
		render_option = self.vis.get_render_option()
		if render_option is None:
			self.vis.destroy_window()
			raise RuntimeError(
				"Open3D window creation failed. "
				"Check GPU driver or DISPLAY/X11 forwarding."
			)

		if self.map_pcd_world is not None:
			self.vis_map_pcd = o3d.geometry.PointCloud()
			self.vis_map_pcd.points = self.map_pcd_world.points
			map_colors = np.tile([0.3, 0.5, 0.8], (len(self.map_pcd_world.points), 1))
			map_colors = map_colors * 0.5
			self.vis_map_pcd.colors = o3d.utility.Vector3dVector(map_colors)
			self.vis.add_geometry(self.vis_map_pcd)

		# Add filtering range wireframe box
		self.vis_range_box = self.create_range_box(
			MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_HEIGHT, MAX_HEIGHT,
			color=(1.0, 0.8, 0.0)  # yellow-orange wireframe
		)
		self.vis.add_geometry(self.vis_range_box)

		self.vis.add_geometry(self.vis_clusters_pcd)
		self.vis.add_geometry(self.vis_tracked_bbox)
		self.vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

		render_option.point_size = 3.0
		render_option.background_color = np.array([0.05, 0.05, 0.1])

		ctr = self.vis.get_view_control()
		ctr.set_zoom(0.6)
		ctr.set_front([-1, 0, -1])
		ctr.set_up([0, 0, 1])
		ctr.set_lookat([0, 0, 2])

	def _update_visualization(self, points, colors, locked_target):
		self.vis_clusters_pcd.points = o3d.utility.Vector3dVector(points)
		self.vis_clusters_pcd.colors = o3d.utility.Vector3dVector(colors)
		self.vis.update_geometry(self.vis_clusters_pcd)

		if locked_target is not None and locked_target.is_active:
			bbox = self.create_bounding_box(locked_target.cluster_points)
			self.vis_tracked_bbox.points = bbox.points
			self.vis_tracked_bbox.lines = bbox.lines
			self.vis_tracked_bbox.colors = bbox.colors
		else:
			self.vis_tracked_bbox.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
			self.vis_tracked_bbox.lines = o3d.utility.Vector2iVector(np.empty((0, 2)))
			self.vis_tracked_bbox.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))

		self.vis.update_geometry(self.vis_tracked_bbox)
		self.vis.poll_events()
		self.vis.update_renderer()

	def run(self):
		pcd_files = self._load_pcd_file_list()

		if len(pcd_files) < self.calib_frames:
			raise ValueError(f"Need at least {self.calib_frames} frames for calibration")

		self.load_map()

		calib_points = []
		for pcd_path in pcd_files[:self.calib_frames]:
			points = self._read_pcd_points(pcd_path)
			if points is not None:
				calib_points.append(points)

		error = self._calibrate_from_points(calib_points)
		print(f"Calibration error: {error:.4f} m")

		self._init_visualization()

		frame_count = 0
		for pcd_path in pcd_files[self.calib_frames:]:
			points = self._read_pcd_points(pcd_path)
			if points is None:
				continue

			points_world = self.transform_lidar_to_world(points)
			points_world = self.remove_background_points(points_world)

			self.point_queue.append(points_world)
			if len(self.point_queue) > MAX_DEQUE_SIZE:
				self.point_queue.pop(0)

			if len(self.point_queue) < MIN_FRAME_COUNT:
				continue

			all_points_world = np.vstack(self.point_queue)
			clustered_points, colors, locked_target, _ = self.process_clusters(all_points_world)
			self._update_visualization(clustered_points, colors, locked_target)

			if locked_target is not None and locked_target.is_active:
				smooth_center = locked_target.get_smoothed_position()
				T_inv = np.linalg.inv(self.T_lidar_to_world)
				center_lidar_homo = T_inv @ np.array([smooth_center[0], smooth_center[1], smooth_center[2], 1.0])
				center_lidar = center_lidar_homo[:3]
				center_gimbal = self.transform_points_lidar_to_gimbal(center_lidar.reshape(1, -1))[0]
				yaw, pitch = self.calculate_yaw_pitch(center_gimbal)
				yaw_deg = np.degrees(yaw) + YAW_OFFSET
				pitch_deg = -(np.degrees(pitch)) + PITCH_OFFSET
				sys.stdout.write(
					f"\rYaw: {yaw_deg:6.2f} deg | Pitch: {pitch_deg:6.2f} deg | "
					f"Lock ID: {locked_target.cluster_id}      "
				)
				sys.stdout.flush()

			frame_count += 1
			if self.playback_delay > 0:
				time.sleep(self.playback_delay)

		if self.vis is not None:
			self.vis.destroy_window()


def parse_args():
	parser = argparse.ArgumentParser(description="Offline LiDAR clustering playback")
	parser.add_argument("--pcd_dir", required=True, help="Directory with per-frame PCD files")
	parser.add_argument("--map_path", default="/home/radar/Radar/code/Hust_Radar_2026/RM2026_map.pcd", help="Optional map PCD path for background removal")
	parser.add_argument("--world_points", default="/home/radar/Radar/code/Hust_Radar_2026/configs/world_points.yaml", help="YAML with world feature points")
	parser.add_argument("--calib_frames", type=int, default=CALIBRATION_FRAME_COUNT)
	parser.add_argument("--playback_delay", type=float, default=0.03)
	return parser.parse_args()


def main():
	args = parse_args()
	debugger = OfflineLidarDebugger(
		pcd_dir=args.pcd_dir,
		map_path=args.map_path,
		world_points_path=args.world_points,
		calib_frames=args.calib_frames,
		playback_delay=args.playback_delay
	)
	debugger.run()


if __name__ == "__main__":
	main()
