# 长轨迹地图与长距离 Nav2 实施计划

**目标：** 使用真实 G1 whole-body policy 生成覆盖更大的地图，验证连续保存，并完成净空不低于 0.50 m、距离大于 1.0 m 的 Nav2 action。

**架构：** 复用现有 `sim_main.py`、ROS bridge、FAST-LIVO、保存服务和地图后处理脚本。轨迹由短时 `/cmd_vel` 分段组成，每段之间停车和检查；GT 只离线记录。所有新地图写入新目录。

**技术栈：** IsaacLab、Unitree DDS、ROS 2 Humble、FAST-LIVO、Python 3.10/3.11、PCL、Nav2。

## 实施状态（2026-08-07）

- 已发现原“短地图无 >1 m 安全路径”的前提来自静态栅格 `min_obstacle_z=-0.8 m` 与运行时代价地图 `-0.3 m` 不一致；修正后同一地图的 0.50 m 净空连通路径约 4.75 m。
- 已完成 FAST-LIVO 空关键帧保存防护，并修正文档遗漏的 mapping overlay；同一会话两次保存均成功，各 46 个关键帧，进程保持存活。
- 已在修正地图上提交距离 1.1468 m 的真实 Nav2 目标。首次复测定位到高 yaw 混合命令的策略弱响应，扩展曲线工作点适配后 action 返回 SUCCEEDED。
- 已完成 754 关键帧长轨迹地图的双份保存与后处理；LIO 轨迹长度 9.506 m，轨迹点全部位于同一自由空间连通域，最小净空 0.552 m。
- 已使用新地图重新初始化 `online_relo`（NDT score 0.00677），并完成距离 1.834 m 的真实 Nav2 action，结果为 SUCCEEDED。
- 证据汇总：`holoagent_bridge/validation/final_long_map_summary_20260807.json`。

## 全局约束

- 不使用 `write_root_pose_to_sim()` 或 GT 驱动机器人。
- 不缩小 `robot_radius`、`inflation_radius`，不扩大目标容差。
- 不覆盖 `holoagent_bridge/maps/mid360_sim_20260806_102736/`。
- 每个运动段后必须发布零速度并检查停车。
- 当前仓库不提交、不清理用户已有修改。

---

### 任务 1：同步状态文档

**文件：**

- 修改：`docs/holoagent/integration_status.md`

- [x] 把总体任务 8 从“阻塞”改为“基础闭环已完成，长距离待验证”。
- [x] 删除任务 9 中“任务 8 action 尚未成功”的过期前提。
- [x] 修复推荐推进顺序缺少编号 3。
- [x] 运行 `git diff --check -- docs/holoagent/integration_status.md`，预期退出码 0。

### 任务 2：启动真实建图基线并校验静止输入

**接口：**

- 消费：IsaacLab shared-memory MID360/IMU。
- 产生：`/mid360/points`、`/livox/imu`、`/aft_mapped_to_init`。

- [x] 启动 `sim_main.py`，必须包含 `--action_source dds_wholebody --enable_wholebody_dds`。
- [x] 启动 MID360、IMU 和 `/cmd_vel`→DDS bridge。
- [x] 复位 category 2，并连续发送至少 5 s 零命令。
- [x] 运行 `validate_lidar_imu_sync.py --duration 5`，要求退出码 0。
- [x] 启动 FAST-LIVO，等待 IMU 初始化和连续 `/aft_mapped_to_init`。
- [x] 启动 GT 与 LIO 轨迹记录，GT 文件只用于离线评估。

### 任务 3：执行分段物理建图轨迹

**接口：**

- 使用：现有 `/cmd_vel` 和 `cmd_vel_to_unitree_dds.py`。
- 输出：真实 whole-body 运动、FAST-LIVO keyframe、GT/LIO 轨迹。

- [ ] 先发送一个短直行段，wall-time 2–3 s，随后停止至少 2 s。
- [ ] 计算该段 GT 位移和航向变化；确认方向正确、停车后速度接近零。
- [ ] 以 0.4–0.6 m 目标位移为单位继续分段，使用带平移曲线改变方向。
- [ ] 每段后检查 GT 位移、LIO 连续性、点云新鲜度和进程存活。
- [x] 达到至少 2.0 m XY 覆盖或出现安全/定位异常时停止。
- [x] 保存 GT、LIO、重定位与完整建图轨迹记录。

### 任务 4：同会话连续保存并验证地图结构

**输出目录：**

- `holoagent_bridge/maps/mid360_long_<timestamp>_a/`
- `holoagent_bridge/maps/mid360_long_<timestamp>_b/`

- [x] 连续调用 `/fast_livo/save_map` 两次，每次使用不同绝对目录。
- [x] 要求两次响应均为 `success=True`，FAST-LIVO 进程在第二次保存后仍存活。
- [x] 分别运行 `prepare_reloc_map.py <dir> --rebuild-global`，要求退出码 0。
- [x] 检查两份地图 pose/PCD/SCD 数量相等、索引连续、全局点云非空。
- [x] 正常停止 FAST-LIVO，确认没有保存子进程或 native crash。

### 任务 5：地图质量门禁与 Nav2 栅格

**使用文件：**

- `holoagent_bridge/prepare_reloc_map.py`
- `holoagent_bridge/generate_nav2_map.py`
- `holoagent_bridge/render_pcd_map_image.py`

- [x] 计算轨迹长度、XY 覆盖范围、最终回到重复区域时的漂移。
- [x] 为候选地图生成 `grid_map.pgm/yaml` 和俯视图。
- [x] 使用距离变换检查连通区域，要求存在长度至少 1.5 m、净空至少 0.50 m 的路径。
- [x] 检查已知障碍保留、轨迹区域自由，且没有明显双墙或断层。
- [x] 只把通过全部门禁的目录配置为下一阶段重定位地图。

### 任务 6：新地图重定位与长距离 Nav2 验证

- [x] 使用新地图启动 FAST-LIVO、`online_relo`、Nav2 和命令桥。
- [x] 静止初始化后记录 `/pose`，确认初始化稳定。
- [x] 在净空至少 0.50 m 的连通区域选择距离大于 1.0 m 的目标。
- [x] 提交真实 `NavigateToPose` action，同时记录 `/pose` 和命令日志。
- [x] 要求出现普通非零平移命令，action 返回 SUCCEEDED，重定位方向一致。
- [x] action 结束后确认 DDS 回零且 `/pose` 停车。
- [x] 更新 `docs/holoagent/integration_status.md` 和验证 JSON；如失败，明确归类为规划、定位、控制或地图问题。

### 任务 7：最终回归与清理

- [x] 运行 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/ubuntu/miniconda3/envs/unitree_sim_env/bin/python -m pytest -q tests/test_holoagent_bridge_static.py`。
- [x] 运行相关 Python 编译、地图 JSON/结构检查和 `git diff --check`。
- [x] 停止仿真、ROS、FAST-LIVO、重定位和 Nav2 进程。
- [x] 报告通过项、失败项、新地图路径和所有保留证据，不删除旧地图或用户文件。
