import open3d as o3d
import numpy as np

# 读取PCD文件
pcd_path = "/home/radar/Radar/code/Hust_Radar_2026/RM2026_map.pcd"
pcd = o3d.io.read_point_cloud(pcd_path)

# 打印点云信息
print(f"点云加载成功！")
print(f"点云包含 {len(pcd.points)} 个点")
print(f"点云是否有颜色: {pcd.has_colors()}")
print(f"点云是否有法向量: {pcd.has_normals()}")

# 可视化点云
print("正在打开可视化窗口...")
o3d.visualization.draw_geometries([pcd],
                                  window_name="RM2026 点云",
                                  width=1280,
                                  height=720,
                                  left=50,
                                  top=50,
                                  point_show_normal=False)
