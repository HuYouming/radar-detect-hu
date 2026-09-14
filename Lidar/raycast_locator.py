"""Ray-casting localization against the RoboMaster field mesh."""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence, Union

import cv2
import numpy as np

from .native_runtime import NATIVE_COMPUTE_LOCK
from .units import require_meter_unit


class RaycastLocator:
    """Convert an image pixel into a point in the mesh coordinate system.

    Mesh vertices, PnP calibration points and extrinsics must all live in the
    coordinate system declared by ``coordinate_system`` (metres). For the
    project's RM2026_map mesh that is ``"field"``, so returned intersections
    are directly in field coordinates (28 x 15 m, red origin at (0, 0));
    ``"solidwork"`` is kept only as a legacy reference-model option.
    """

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        field_to_camera_R: np.ndarray,
        field_to_camera_T: np.ndarray,
        mesh_path: Union[str, Path],
        coordinate_system: str = "field",
        unit: str = "m",
    ) -> None:
        self.o3d = self._import_open3d()
        self.unit = require_meter_unit(unit, "Raycast mesh and extrinsics")
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        # solvePnP returns field -> camera: X_camera = R X_field + T.
        self.rotation = np.asarray(field_to_camera_R, dtype=np.float64).reshape(3, 3)
        self.translation = np.asarray(field_to_camera_T, dtype=np.float64).reshape(3, 1)
        if not np.all(np.isfinite(self.rotation)) or not np.all(np.isfinite(self.translation)):
            raise ValueError("Raycast extrinsics contain non-finite values")
        if not np.isfinite(np.linalg.det(self.rotation)) or abs(np.linalg.det(self.rotation)) < 1e-6:
            raise ValueError("Raycast rotation matrix is singular")
        if np.linalg.norm(self.translation) > 100.0:
            raise ValueError(
                "Raycast translation is implausibly large for meter units; "
                "check that PLY vertices and extrinsics are in meters."
            )
        self.mesh_path = Path(mesh_path).expanduser().resolve()
        self.coordinate_system = coordinate_system
        self.mesh = self._load_mesh()
        self.scene = self.o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(
            self.o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)
        )

    @staticmethod
    def _import_open3d():
        try:
            import open3d as o3d
        except ImportError as exc:
            raise ImportError(
                "Raycast localization requires the open3d package"
            ) from exc
        return o3d

    def _load_mesh(self):
        if not self.mesh_path.exists():
            raise FileNotFoundError(f"Raycast mesh does not exist: {self.mesh_path}")
        mesh = self.o3d.io.read_triangle_mesh(str(self.mesh_path))
        if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
            raise ValueError(f"Raycast mesh has no triangles: {self.mesh_path}")
        if self.coordinate_system not in {"solidwork", "field"}:
            raise ValueError(
                f"Unsupported raycast coordinate_system: {self.coordinate_system}"
            )
        return mesh

    def pixel_to_world(self, pixel: Sequence[float]) -> Optional[np.ndarray]:
        """Return ``[x, y, z]`` in the mesh coordinate system, in meters."""
        if len(pixel) != 2:
            raise ValueError(f"pixel must contain (u, v), got {pixel!r}")
        points = np.array([[[float(pixel[0]), float(pixel[1])]]], dtype=np.float64)
        if np.any(self.dist_coeffs):
            points = cv2.undistortPoints(
                points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix
            )
        u, v = points[0, 0]
        camera_direction = np.linalg.solve(
            self.camera_matrix, np.array([u, v, 1.0], dtype=np.float64)
        )
        world_direction = self.rotation.T @ camera_direction
        origin = (-self.rotation.T @ self.translation).reshape(3)
        if not np.all(np.isfinite(origin)) or not np.all(np.isfinite(world_direction)):
            return None

        rays = self.o3d.core.Tensor(
            [[*origin, *world_direction]], dtype=self.o3d.core.Dtype.Float32
        )
        with NATIVE_COMPUTE_LOCK:
            result = self.scene.cast_rays(rays)
        distance = float(result["t_hit"].numpy()[0])
        if not np.isfinite(distance):
            return None
        hit = origin + distance * world_direction
        if not np.all(np.isfinite(hit)):
            return None
        # Return coordinates in the mesh's own frame. Converter performs any
        # project-specific coordinate mapping before this point.
        return hit.astype(np.float64)

    def plane_intersect(self, pixel, z_plane):
        """Ray-plane intersection with the horizontal plane z = z_plane (mesh frame).

        用于分层定位: 命中 z 归入已知高度层后, 用层平面交点给出稳定的
        水平坐标, 避免 mesh 地面起伏/孔洞导致 x,y 抖动。
        """
        points = np.array([[[float(pixel[0]), float(pixel[1])]]], dtype=np.float64)
        if np.any(self.dist_coeffs):
            points = cv2.undistortPoints(
                points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix
            )
        u, v = points[0, 0]
        direction = self.rotation.T @ np.linalg.solve(
            self.camera_matrix, np.array([u, v, 1.0], dtype=np.float64)
        )
        origin = (-self.rotation.T @ self.translation).reshape(3)
        if abs(direction[2]) < 1e-9:
            return None
        t = (float(z_plane) - origin[2]) / direction[2]
        if not np.isfinite(t) or t <= 0.0:
            return None
        return (origin + t * direction).astype(np.float64)

    __call__ = pixel_to_world
