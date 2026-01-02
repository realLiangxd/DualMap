## runner_ros_base.py中

问题1 frame idx 是否有自增，没发现

问题2 fast-sam、与yolo、sam、之间的关系

# 运行一次处理逻辑,注意；dualMap核心处理函数包括： sequential_process、parallel_process、check_keyframe；另外还有部分辅助函数包括：get_keyframe_idx、end_process

## core.py中
## 核心处理：顺序处理（sequential_process）

说明  
`sequential_process` 是 Dualmap 在不启用并行映射线程时的处理入口，主要用于调试与小规模测试。其特点是：所有步骤在同一线程内按序执行，处理延迟较大，不支持导航功能（navigation）。

输入  
- 接受一个 DataInput 对象，包含：idx、time_stamp、color、depth、intrinsics、pose 等。

执行流程（按顺序）
1. 更新帧与位姿信息
   - 设置当前帧 id：`self.curr_frame_id = data_input.idx`
   - 设置当前位姿：`self.curr_pose = data_input.pose`

2. 更新可视化状态
   - `visualizer.set_time_sequence("frame", ...)`
   - `visualizer.set_camera_info(...)`
   - `visualizer.set_image(...)`

3. 检测与观察生成（Detection）
   - `detector.set_data_input(data_input)`
   - 若 `cfg.run_detection`：`detector.process_detections()`（并可选择保存）
   - 否则：`detector.load_detection_results()`
   - `detector.calculate_observations()` 将检测结果转换为 Observation 列表
   - 可视化检测结果（若开启）

4. 局部建图（Local Mapping）
   - 获取当前观察：`curr_obs_list = detector.get_curr_observations()`
   - 更新检测器内部状态与数据：`detector.update_state()`、`detector.update_data()`
   - 告知局部管理器当前帧索引：`local_map_manager.set_curr_idx(self.curr_frame_id)`
   - 处理观察并更新局部地图：`local_map_manager.process_observations(curr_obs_list)`

5. 全局建图（Global Mapping）
   - 从局部管理器获取应合并到全局的观察：`global_obs_list = local_map_manager.get_global_observations()`
   - 清空局部的全局候选缓存：`local_map_manager.clear_global_observations()`
   - 将稳定观察送入全局管理器：`global_map_manager.process_observations(global_obs_list)`

注意事项
- 该模式仅用于调试/小数据集；大数据集建议启用并行（`cfg.use_parallel = True`），使用 `parallel_process` 与后台建图线程提高实时性。
- 顺序模式不支持导航子流程（例如全局/局部路径规划通常在并行模式下触发）。
- 各子模块（detector、local_map_manager、global_map_manager）依赖于 DataInput 中 pose/intrinsics 的正确性与坐标系一致性。

调用位置  
- 由各 `applications/runner_*.py` 根据配置调用：当 `cfg.use_parallel == False` 时调用 `dualmap.sequential_process(data_input)`。

## 可视化模块 ReRunVisualizer

## Detector 模块

## LocalMapManager 模块

## GlobalMapManager 模块