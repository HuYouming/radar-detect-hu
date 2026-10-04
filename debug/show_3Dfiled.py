import open3d as o3d, numpy as np
m = o3d.io.read_triangle_mesh(
    "RM2026_field_lowpoly.ply")

verts = np.asarray(m.vertices)
tris  = np.asarray(m.triangles)
edges = np.vstack([tris[:, [0,1]], tris[:, [1,2]], tris[:, [2,0]]])

m.paint_uniform_color([1.0, 1.0, 0.0])        # 模型面：黄色

lines = o3d.geometry.LineSet()
lines.points = o3d.utility.Vector3dVector(verts)
lines.lines  = o3d.utility.Vector2iVector(edges)
lines.paint_uniform_color([1.0, 0.0, 0.0])    # 网格线：红色

o3d.visualization.draw_geometries(
    [m, lines],
    window_name="黄面红网格",
    mesh_show_back_face=True,
)