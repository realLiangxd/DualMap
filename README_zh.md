# DualMap: 实时开放词汇语义建图系统 | 中文文档

<h3>
  <a href="https://eku127.github.io/DualMap/">项目主页</a> |
  <a href="https://arxiv.org/abs/2506.01950">arXiv 论文</a> 
</h3>

<p align="center">
  <img src="resources/image/optimized-gif.gif" width="70%">
</p>

---

**DualMap** 是一个前沿的在线开放词汇建图系统，它赋予了机器人在动态变化的3D环境中，通过自然语言进行理解、交互和导航的能力。

该系统的设计具备高度的灵活性和可扩展性，支持多种输入源，包括：
- **离线数据集模式 (Dataset Mode)**：支持 Replica, ScanNet, TUM RGB-D 等标准数据集。
- **ROS 模式 (ROS Mode)**：兼容 ROS1 和 ROS2，可处理离线的 rosbag 文件或实时的机器人传感器数据流。
- **移动设备模式 (Record3d Mode)**：支持通过 iPhone 的 Record3D 应用进行实时数据流建图。

## 核心功能

- **在线与开放词汇 (Online & Open-Vocabulary)**: 系统能够实时构建环境的3D语义地图，并且不局限于预定义的物体类别，可以理解任意自然语言描述。
- **动态场景理解 (Dynamic Scene Understanding)**: DualMap 能够跟踪场景中物体的变化，并更新地图，使其适用于人机共存的动态环境。
- **多模态输入支持 (Multi-Modal Inputs)**: 无缝集成多种数据源，无论是学术研究常用的数据集，还是真实机器人使用的ROS系统，都能轻松接入。
- **自然语言交互与导航 (Natural Language Interaction & Navigation)**: 用户可以通过自然语言查询地图中的物体（例如，“我的背包在哪里？”），并指令机器人导航至目标位置。

## 项目结构解析

为了方便二次开发和使用，项目采用了清晰的模块化结构：

```
DualMap/
├── 3rdparty/           # 第三方依赖库 (如 MobileCLIP)
├── applications/       # 各种应用场景的启动器 (Runner)
├── config/             # 系统配置文件 (YAML, TXT, JSON)
├── dualmap/            # 项目核心算法与逻辑
├── evaluation/         # 评估代码与脚本
├── resources/          # 文档、图片、视频等静态资源
├── scripts/            # 便捷的运行脚本 (Shell)
└── utils/              # 通用工具模块 (地图管理、数据处理等)
```

- **`applications/`**: 这是项目的主要入口。不同的 `runner_*.py` 文件对应不同的应用模式（如数据集、ROS等），负责初始化系统并处理相应的数据流。
- **`dualmap/`**: 包含系统的核心实现。`core.py` 文件很可能是 DualMap 系统的主要逻辑所在，整合了地图构建、语义理解和导航等功能。
- **`config/`**: 所有的可调参数都在这里。你可以修改 `.yaml` 文件来调整模型、数据路径、可视化选项等。`class_list/` 目录定义了不同场景下的物体类别。
- **`utils/`**: 提供了一系列辅助工具，例如点云处理 (`pcd_utils.py`)、地图管理 (`global_map_manager.py`, `local_map_manager.py`)、ROS 发布器 (`ros_publisher.py`) 等。
- **`scripts/`**: 提供了一些 `.sh` 脚本，用于批量运行测试或复现论文中的实验结果。

## 安装与配置

> ✅ **环境测试报告**: 已在 **Ubuntu 22.04** + **ROS 2 Humble** + **Python 3.10** 环境下成功测试。

### 1. 克隆仓库 (包含子模块)

```bash
# 推荐使用 SSH 方式克隆
git clone --branch main --single-branch --recurse-submodules git@github.com:Eku127/DualMap.git
cd DualMap
```
> **注意**: 必须使用 `--recurse-submodules` 参数，以确保 `mobileclip` 等子模块被正确下载。

### 2. 创建并激活 Conda 环境
```bash
conda env create -f environment.yml
conda activate dualmap
```

### 3. 安装 MobileCLIP
```bash
cd 3rdparty/mobileclip
pip install -e . --no-deps
cd ../..
```
> 系统默认使用 `MobileCLIP-v1`。你也可以按照 [Apple MobileCLIP 官方仓库](https://github.com/apple/ml-mobileclip) 的指引配置 `v2` 版本。

### 4. (可选) 配置 ROS 环境
为了使用机器人进行真实世界的交互和导航，强烈建议安装 ROS。

- **ROS2 (推荐)**: 我们推荐安装 [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)。
- **ROS1 (也支持)**: 如果你需要使用 ROS1, 可以参考 [此指南](resources/doc/ros_communication.md) 在 Ubuntu 22.04 上安装 ROS1 Noetic。

安装后，记得激活 ROS 环境：
```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# ROS1 Noetic
source /opt/ros/noetic/setup.bash
```

### 5. (可选) 配置 Habitat 仿真环境
为了体验完整的在线交互式建图和导航功能，建议安装 [Habitat Data Collector](https://github.com/Eku127/habitat-data-collector)。这是一个基于 Habitat-sim 的仿真工具，可以与 DualMap 无缝集成。

## 核心算法解析：并行处理流水线

为了实现实时建图与导航，DualMap 采用了一种高效的并行处理流水线，将计算密集型任务与需要快速响应的任务解耦。核心逻辑位于 `dualmap/core.py` 的 `Dualmap` 类中，主要由 `parallel_process` 方法（主线程）和 `run_mapping_thread` 方法（后台建图线程）协同工作。

<p align="center">
  <img src="https://raw.githubusercontent.com/realLiangxd/DualMap/main/resources/image/pipeline.png" width="90%">
</p>

### 数据处理流程

无论数据来自离线数据集、ROS还是移动设备，它们都会被统一封装成 `DataInput` 对象，然后进入以下处理流程：

#### 1. 主线程：感知与决策 (`parallel_process` 方法)

主线程负责需要快速响应的任务，确保系统的实时性。当一个新的数据帧 (`DataInput`) 到达时，它会执行：

- **物体检测与观察生成 (Observation Generation)**:
  - 调用 `self.detector.process_detections()` 运行 YOLO、SAM 等模型，识别当前帧中的物体。
  - 调用 `self.detector.calculate_observations()` 将2D检测结果（如边界框、掩码）转换为包含3D位置、尺寸和视觉特征的 `Observation` 对象列表。
  - 这是整个流程中计算最密集的部分。

- **数据移交**:
  - 将生成的观察结果 `curr_obs_list` 和帧ID `curr_frame_id` 推入一个线程安全的队列 `self.detection_results_queue`。
  - 这个队列是主线程和后台建图线程之间的“数据交接处”。主线程完成这一步后，即可处理下一帧数据，无需等待耗时的建图过程。

- **导航路径规划 (Navigation Path Planning)**:
  - 当接收到导航指令时 (`self.calculate_path` 为 `True`)，主线程会利用**当前已构建的地图**进行路径规划。
  - **全局路径规划**: 调用 `self.global_map_manager.calculate_global_path()`，在宏观的全局地图上计算出一条从当前位置到目标区域的大致路径。
  - **局部路径规划**: 调用 `self.local_map_manager.calculate_local_path()`，在全局路径的指引下，结合精细的局部地图信息（包含动态障碍物），计算出一条更精确、安全且可执行的局部路径。

#### 2. 后台建图线程：地图更新 (`run_mapping_thread` 方法)

这个独立的后台线程负责所有耗时的建图任务，它不断地从 `detection_results_queue` 队列中消费数据。

- **获取数据**:
  - 线程阻塞等待，直到队列中有新的 `curr_obs_list`。

- **局部地图更新 (Local Mapping)**:
  - 调用 `self.local_map_manager.process_observations(curr_obs_list)`。
  - `LocalMapManager` 在一个滑动时间窗口内（例如，最近几十帧）跟踪和融合物体，构建一个**精细但范围有限的局部地图**。这个地图主要关注动态物体和场景的最新变化，为实时避障和精细操作提供支持。

- **全局地图更新 (Global Mapping)**:
  - 当局部地图中的某些物体被确认为稳定（例如，连续多帧被看到且位置固定）后，它们会被提取为“全局观察” (`global_obs_list`)。
  - 调用 `self.global_map_manager.process_observations(global_obs_list)`。
  - `GlobalMapManager` 负责将这些稳定的物体融合到一个**长期、持久化、大范围的全局语义地图**中。这个地图代表了整个环境的静态结构和关键语义信息，为全局导航和长期记忆提供基础。

### 总结

这种主线程（感知+决策）与后台线程（建图）并行的设计，使得 DualMap 能够：
- **实时响应**: 快速处理传感器数据和用户指令，避免系统卡顿。
- **高效建图**: 将耗时的地图更新任务异步处理，不阻塞主流程。
- **双层地图结构**:
  - **局部地图**保证了对动态环境的快速适应和精细导航。
  - **全局地图**实现了对环境的长期记忆和稳健的全局路径规划。

## 应用指南

下表总结了不同应用所需的环境配置：

| 应用场景 | Conda 环境 | ROS1 | ROS2 | Habitat 仿真器 |
| :--- | :---: | :---: | :---: | :---: |
| 数据集 / 查询 / iPhone | ✓ | | | |
| ROS (离线/在线) | ✓ | ✓ | ✓ | |
| 在线仿真 (建图+导航) | ✓ | | ✓ | ✓ |

### 💾 使用数据集运行

DualMap 支持多种离线数据集。请参考 [数据集运行指南](resources/doc/app_runner_dataset.md) 来配置数据并复现论文中的离线建图结果。

**启动示例**:
```bash
python applications/runner_dataset.py --config config/runner_dataset.yaml
```

### 🤖 使用 ROS 运行

系统支持通过 ROS1 或 ROS2 接口处理 rosbag 文件或实时传感器数据。这使得 DualMap 可以轻松部署在真实机器人上。详细步骤请参考 [ROS 运行指南](resources/doc/app_runner_ros.md)。

**启动示例**:
```bash
# 需要先根据你的 ROS 版本选择
# 1. ROS1
# 2. ROS2
# 3. 无 ROS (默认)

python applications/runner_ros.py --config config/runner_ros.yaml
```

### 🕹️ 在仿真环境中进行在线建图与导航

结合 [Habitat Data Collector](https://github.com/Eku127/habitat-data-collector)，你可以在逼真的仿真环境中进行实时的交互式建图和导航测试。这对于算法验证和开发非常有用。详情请见 [在线仿真与导航指南](resources/doc/app_simulation.md)。

### 📱 使用 iPhone 实时建图

通过 Record3D 应用，你可以将 iPhone 变成一个实时 3D 扫描设备，并将数据流直接传输给 DualMap 进行建图。配置方法请参考 [iPhone 运行指南](resources/doc/app_runner_record_3d.md)。

### 🔍 离线地图查询

我们提供了预先构建好的地图示例，让你无需运行完整的建图流程即可体验自然语言查询功能。请参考 [离线查询指南](resources/doc/app_offline_query.md) 来启动查询应用。

## 可视化

系统支持 [Rerun](https://rerun.io) 和 [Rviz](http://wiki.ros.org/rviz) 两种可视化工具。在 `config/runner_ros.yaml` 文件中，你可以通过 `use_rerun` 和 `use_rviz` 选项来切换。

<p align="center">
    <img src="resources/image/app_visual.jpg" width="100%">
</p>

## 引用

如果这项工作对您有帮助，请考虑给这个项目一个 Star 🌟 并引用我们的论文：

```bibtex
@ARTICLE{jiang2025dualmap,
  author={Jiang, Jiajun and Zhu, Yiming and Wu, Zirui and Song, Jie},
  journal={IEEE Robotics and Automation Letters},
  title={DualMap: Online Open-Vocabulary Semantic Mapping for Natural Language Navigation in Dynamic Changing Scenes},
  year={2025},
  volume={10},
  number={12},
  pages={12612--12619},
  doi={10.1109/LRA.2025.3621942}
}
```

## 联系方式
如有技术问题，请在 GitHub 上创建 Issue。其他问题请联系第一作者: jjiang127 [at] connect.hkust-gz.edu.cn
