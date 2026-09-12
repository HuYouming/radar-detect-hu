# 部署到新主机(Ubuntu 20.04 + NVIDIA GPU)

适用于将本仓库(ros 分支)部署到另一台 Ubuntu 20.04、带 NVIDIA GPU 的主机。
项目所有路径按仓库根解析(`Tools.Paths`),拷贝到任意位置均可运行,无需改代码。

---

## 1. 前置: NVIDIA 驱动 + CUDA

```bash
# 驱动(若未装)
sudo ubuntu-drivers autoinstall
sudo reboot
nvidia-smi   # 确认能看到 GPU 与 CUDA 版本(建议 CUDA 12.x)

# CUDA 工具链(以 12.1 为例)
wget https://developer.download.nvidia.com/compute/cuda/12.1.1/local_installers/cuda_12.1.1_530.30.02_linux.run
sudo sh cuda_12.1.1_530.30.02_linux.run --toolkit --silent
echo 'export PATH=/usr/local/cuda-12.1/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

## 2. ROS Noetic(Ubuntu 20.04 官方支持)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. conda 环境(GPU 版)

```bash
conda create -n Radar python=3.8 -y
conda activate Radar

# GPU 版 torch(CUDA 12.1)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# 其余依赖
pip install opencv-python numpy pyyaml ruamel.yaml ultralytics PyQt5 shapely
pip install open3d        # raycast 射线求交(CPU 实现,无需 CUDA 版)
```

> 注意: open3d 装官方普通 wheel 即可;RaycastingScene 是 CPU 计算,不依赖 CUDA。

## 4. 拷贝项目

```bash
scp -r "radar-detect(ros分支)" user@新主机:~/
```

以下文件必须带全(缺了会跑不起来或定位错误):

| 路径 | 用途 |
|---|---|
| `weights/*.pt` | YOLO stage1/stage2 权重 |
| `RM2026_map_m.ply` | raycast 场地 mesh |
| `configs/` | 全部 yaml(内参/标定点/raycast 配置) |
| `Lidar/26_point.yaml` | 透视回退区域点 |
| `data/` | 调试视频(如 shifan.mp4) |

## 5. 启动(三个终端,顺序固定)

```bash
# 终端 1: ROS 主节点
source /opt/ros/noetic/setup.bash && roscore

# 终端 2: 主程序(先起,标定后发 ready)
conda activate Radar && source /opt/ros/noetic/setup.bash
cd ~/radar-detect\(ros分支\)
python3 26_main.py

# 终端 3: 检测进程(收到 ready 后读视频推理、弹 frame 窗口)
conda activate Radar && source /opt/ros/noetic/setup.bash
cd ~/radar-detect\(ros分支\)
python3 -m detect.Detector
```

标定时注意: 点选窗口左上角会提示"请点第 n/5 点",终端会先打印 5 点顺序与方位;25/26 两套点勿混。

## 6. 常见检查

```bash
# GPU 是否被 YOLO 使用(Detector 终端应无 CUDA 报错)
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

# ROS master 是否就绪
rostopic list

# 端口 11311 被占(上次进程未杀干净)
ss -ltnp | grep 11311

# raycast 是否初始化(26_main 标定完成后终端应有)
#   Raycast locator initialized: .../RM2026_map_m.ply
```

## 7. 实机补充(相机/雷达)

- 相机: 项目内 `stereo_camera/MvImport/` 已含 SDK,确认对应驱动库可用、USB 权限(`sudo usermod -aG video $USER`);
- 雷达: 配置 Livox workspace(`ws_livox`),供 `Counter/init_angle_sender.py` 使用;
- 需要标定 GUI 图形界面(有桌面环境即可,无头服务器需 X11 转发)。
