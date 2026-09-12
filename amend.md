1. 新建了data文件夹并保存了shifan视频
2. main文件修改了修改了视频路径，以及输入源改为video
3. detect/Detector.py 修改337行，0改为cpu
4. 启动三个终端 
    1. roscore
    2. 启动main python3 26_main.py
    3. 启动Detector python3 -m detect.Detector

--- raycasting 迁移(仿 main 分支) ---
5. 添加 Lidar/units.py：米制单位校验(require_meter_unit/validate_meter_vector)，防止毫米数据混入定位链路
6. 添加 Lidar/native_runtime.py：open3d 原生调用串行锁(RLock)
7. 添加 Lidar/raycast_locator.py：RaycastLocator，图像像素→RM2026 场地 mesh 射线求交，输出场地系米制点
8. 添加 configs/raycast_config.yaml：raycast 开关、mesh 路径(RM2026_map_m.ply)、坐标系(field)、单位(m)
9. 添加 RM2026_map_m.ply：RM2026_map.pcd(mm) 重建的米制场地 mesh(几何坐标系不变；从 main 分支复制，两分支 pcd 相同)
10. 修改 Lidar/Converter.py：
    - 6-7 行：import units 与 RaycastLocator
    - 95-104 行：加载 raycast_config 并校验 mesh 单位必须为米
    - 151-157 行：标定 PnP 平移加米制量级拦截(毫米数值会提示重标，防止 mm 混入)
    - 179 行：标定完成后调用 _init_raycast_locator 初始化 raycast
    - 181-229 行：新增 _legacy_to_field_pose(标定/透视在 legacy 系而 mesh 在场系，桥接 T_field=T_leg−R·(0,15,0))、_init_raycast_locator、_raycast_result
    - 238-241 行：camera_results 改为 raycast 优先，未命中/未启用时回退原透视定位
11. 其余不动：real_points_25/26 数值、两步 PnP、26_point.yaml 透视链、get_new_box、Detector、26_main 均保持原样

--- 实测后校准 ---
12. 修改 26_main.py 的 get_new_box(21-31 行)：定位像素由"框中心下移 h/9(框内 61% 高度)"改为"检测框底边中心"。
    原因：mesh 中无车辆模型，射线对准车体像素会穿透车身命中车后方地面/结构，导致坐标系统性偏大；
    改取底边(≈车轮着地点像素)后射线命中车正下方地面，消除该项偏差(实测部分车定位不准由此引起)。
13. 修复 debug/deubg_vision_locator.py(该脚本与实际项目有意使用不同点集——debug 用 real_points_26,项目用 real_points_25)
14. 标定 GUI 顺序引导(把标定 SOP 固化进代码):
    - camera_locator/point_picker.py：caller/resume 增加可选 names 参数；点击时打印"第 n 点已记录: 名称"，窗口左上角实时显示"请点第 n/5 点: 名称"；不传 names 时行为与原来完全一致
    - Lidar/Converter.py：新增 _calibration_hints_25()，按红/蓝阵营打印 real_points_25 五点顺序与方位提示，并传给点选窗口
    - debug/deubg_vision_locator.py：pick_five_points 传入 CALIBRATION_POINT_NAMES(26 组)，点选窗口同步显示顺序
    原因：25/26 两套点易记混(曾把第1点大符/基地点反导致 PnP 错配 ~12m)，把顺序提示固化进代码避免人为记序
15. 添加 DEPLOY.md：Ubuntu 20.04 + NVIDIA GPU 部署清单(驱动/CUDA/ROS Noetic/conda 依赖/项目拷贝清单/三终端启动/常见检查/实机补充)