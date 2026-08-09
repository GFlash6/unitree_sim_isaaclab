# HoloAgent–IsaacLab 集成主线状态

更新日期：2026-08-08

## 目标与判定原则

本项目的主目标是把 HoloAgent 接入当前 IsaacLab/Unitree 仿真环境，使用 IsaacLab 中真实执行的 MID360 raycast 点云和机身 IMU 完成定位、建图、在线重定位、Nav2 导航以及 whole-body 控制闭环。

全链路遵守以下判定原则：

- 不使用 mock、占位数据、人工伪造位姿或重复发布旧帧来制造“链路可用”的结果。
- 收到 topic、服务返回成功、节点进入 active 或文件能够被读取，都不单独作为正确性结论。
- 传感器数据必须检查来源、坐标系、时间戳、有限性、更新率和动态响应。
- 定位与重定位必须与独立 IsaacLab ground truth 对比；ground truth 只用于离线验证，不进入算法输入。
- 控制必须验证持续命令、DDS 传输、机器人真实运动方向和幅度、定位反馈以及停车行为。
- “已完成”表示实现和运行时正确性均已有证据；缺少稳定性、长时运行或最终场景质量验证时标为“部分完成”。

## 总体进度

| 主线任务 | 当前状态 | 当前结论 |
| --- | --- | --- |
| 1. 仿真 MID360 真实性与数据链路 | 已完成 | 真实 MultiMeshRayCaster 点云已稳定进入 ROS2，坐标系与帧新鲜度已修正 |
| 2. IMU 与时间同步 | 已完成 | 真实机身 IMU 已进入 ROS2，与 LiDAR 使用同一仿真源时间并通过同步验证 |
| 3. FAST-LIVO 实时定位与建图前端 | 已完成 | XYZ-only LiDAR+IMU LIO 已跑通，并与独立 GT 完成动态精度验证 |
| 4. 地图保存、补全与地图质量 | 已完成目标区域长图，稳定性待加强 | 754-keyframe 长图已连续保存两份并通过结构、哈希、栅格和净空检查；全场景扩展与压力保存仍待完成 |
| 5. 在线重定位 | 已完成目标区域闭环，稳定性待加强 | 754-keyframe 地图可持续产生重定位位姿，已完成 GT 精度、初始 NDT 和速度输出验证 |
| 6. Nav2 地图与代价地图 | 已完成目标区域闭环 | 最终长图栅格已用于真实 Nav2；轨迹全部位于同一自由空间连通域，最小净空 0.552 m |
| 7. 定位输出接入 Nav2 | 已完成 | `/pose`、TF 和真实估计速度已接入 Nav2，所有生命周期节点可进入 active |
| 8. Nav2 到 Unitree 控制闭环 | 已完成目标区域闭环，策略边界待加强 | 1.834 m 新地图 action 与可视化四航点闭环均成功；低速死区、狭窄空间和原地转向能力仍有限制 |
| 9. HoloAgent 主 Agent 接入 | 部分完成 | AgentOS→HTTP→ROS→Nav2 调用路径已确认，Python 3.10 ABI 和 HTTP/topic 接线已通过；待全栈真实 action 复测 |
| 10. 稳定性、测试与版本管理 | 部分完成 | 定向测试通过；30 分钟 soak、原生保存压力测试和版本整理尚未完成 |

当前端到端数据流为：

```text
IsaacLab MID360 raycast ─┐
                        ├─ shared memory ─ ROS2 ─ FAST-LIVO ─ online_relo ─ /pose + TF ─ Nav2
IsaacLab torso IMU ─────┘                                                       │
                                                                                ▼
                                                                     /cmd_vel ─ DDS
                                                                                │
                                                                                ▼
                                                                  Unitree whole-body policy
```

# 主线任务 1：仿真 MID360 真实性与数据链路

## 目标

让 IsaacLab 中真实执行的 MID360 raycast 结果，以完整、新鲜、传感器坐标系下的点云进入 ROS2 `/mid360/points`。

## 当前状态

**已完成。** 当前没有 synthetic cloud 或固定平面点云路径。

## 已完成内容

- 确认 MID360 不是 IsaacLab 原生 Unitree 组件，而是当前工程后加的 `MultiMeshRayCaster`。
- raycast 目标包含真实任务场景 prim：
  - `Room`
  - `PackingTable_1`
  - `PackingTable_2`
  - `Object`
- 每帧约 11520 个真实 ray hit 点。
- `sim_main.py` 从 `sensor.data.ray_hits_w[0]` 读取有限点，并把 world-frame ray hit 转换到 MID360 sensor frame。
- 点云通过 shared memory 传递，使用序号锁和纳秒源时间戳，避免读取半写入帧。
- ROS2 桥只发布新的 shared-memory 帧；仿真停止后不会重复旧点伪造更新。
- 原始桥只发布 `/mid360/points`，不再在桥层伪造重定位使用的 `/reloc_body_cloud`。

## 已修复问题

- 修复早期点云近似 world frame、却被标记为传感器 frame 的坐标系错误。
- 修复仅依赖时间戳以及可能重复旧点云的问题。
- 移除 synthetic/fake cloud 调试路径作为运行时输入的可能性。

## 正确性证据

- 真实 smoke run 中传感器类型为 `MultiMeshRayCaster`。
- 实测单帧约 11520 点，点云 `z_span` 曾达到 2.072833 m，不是固定地板平面。
- 静态检查确认桥从 `PointCloudReader()` 读取真实 shared-memory 数据。
- 相关测试：
  - `tests/test_mid360_static.py`
  - `test/test_mid360_ros_bridge.py`
  - `tests/test_pointcloud_shared_memory_utils.py`

## 遗留问题

- 当前传感器模型提供瞬时 XYZ 几何点，没有真实 MID360 单点 firing time、reflectivity 或 RGB；这些字段不会被伪造。
- 尚未完成超长时间 shared-memory 压力运行的正式报告归档。

## 完成判据

本任务的基础接入判据已满足。后续只需在整链路 soak 中继续监控帧序号、时间戳、有限点比例和点数稳定性。

# 主线任务 2：IMU 与 LiDAR 时间同步

## 目标

把 IsaacLab 中真实机身 IMU 观测传给 FAST-LIVO，并确保 IMU、LiDAR 使用相同且单调的仿真源时间。

## 当前状态

**已完成。** 已通过静态和动态时序验证。

## 已完成内容

- 从 IsaacLab `imu_in_torso` 读取真实姿态、角速度和线加速度。
- 通过 shared memory 和 `/livox/imu` 发布到 ROS2。
- IMU 桥只接受新记录；时间戳重复、倒退或数据非有限时拒绝输出。
- LiDAR 和 IMU 都保留原始 simulator timestamp，不用 ROS wall time替代测量时间。
- FAST-LIVO 外参来自 G1 USD 固定关节关系，不使用单位矩阵占位或配准后手调值。

## 已修复问题

- 修复仅用 LiDAR、缺少真实 IMU 约束的问题。
- 修复定位评估时 root frame 与 IMU frame 杠杆臂不一致的问题；现在 LIO 状态和 GT 都以 `imu_in_torso` 为比较对象。
- 增加时间交叠、最近邻时间差、频率、重力模长和动态角速度检查。

## 正确性证据

`holoagent_bridge/validation/lidar_imu_sync.json`：

- LiDAR：503 帧，约 50 Hz。
- IMU：502 帧，约 50 Hz。
- 交叠时间：10.02 s。
- 最大最近邻时间差：0 s。
- 加速度模长中位数：9.8099 m/s²。

动态会话 `lidar_imu_sync_dynamic_session.json` 同样通过，IMU 保持约 50 Hz，且存在真实动态角速度。

## 遗留问题

- 尚未在最终 30 分钟整链路 soak 中单独统计 IMU 丢帧和抖动分布。

## 完成判据

基础同步判据已满足；最终交付时要求长时运行中时间戳持续单调、无旧帧重放、无明显频率退化。

# 主线任务 3：FAST-LIVO 实时定位与建图前端

## 目标

让 FAST-LIVO 直接消费真实 `/mid360/points` 和 `/livox/imu`，产生正确、连续的 LIO 位姿和真实 keyframe。

## 当前状态

**已完成基础 LIO。** 已有真实动态精度验证，不只是看到 keyframe 或 topic 输出。

## 已完成内容

- FAST-LIVO 输入已切换到 `/mid360/points` 和 `/livox/imu`。
- 新增 `XYZ32` 预处理类型，使用 `pcl::PointXYZ` 读取纯 XYZ PointCloud2。
- 不再要求不存在的 RGB、反射率或 per-point time 字段。
- LiDAR 到 IMU 的旋转和平移来自 G1 USD 固定关节。
- 可持续产生 `/undistort_cloud`、`/aft_mapped_to_init` 和真实 keyframe。
- 增加 XYZ-only 预处理 C++ 回归测试。

## 已修复问题

- 修复把仿真 XYZ 点云送入需要 RGB 字段的路径而产生 `Failed to find match for field 'rgb'` 的问题。
- 修复 LIO 与 GT 比较参考刚体不一致导致的隐藏误差。
- 禁用本场景不存在的 wheel odometry 输入，避免把空队列当有效约束。

## 正确性证据

动态 LIO 对 GT 报告：

| 报告 | 平移 RMSE | 航向 RMSE | 最终平移误差 | 最大误差跳变 |
| --- | ---: | ---: | ---: | ---: |
| `lio_dynamic_accuracy_20260806_2230.json` | 0.02498 m | 0.00570 rad | 0.04370 m | 0.02106 m |
| `lio_imu_dynamic_accuracy_20260806_2233.json` | 0.01224 m | 0.00376 rad | 0.00586 m | 0.01622 m |

FAST-LIVO 定向 CTest 当前 4/4 通过，其中包含：

- `test_preprocess_xyz32`
- `test_reloc_map_loading`
- `test_eigen_pcl_alignment`
- `test_ndt_target_validation`

## 遗留问题

- 已有 9.506 m 累计长轨迹，但 XY 覆盖约 1.11 m × 1.97 m，尚不能代表完整仓库或大范围闭环累计漂移。
- 最终目标区域地图仍未启用适合本场景的回环优化策略，也尚未从更大闭环量化重影和累计漂移。

## 完成判据

基础 LIO 与目标区域长轨迹建图已满足；全场景交付仍要求扩大覆盖并评估累计漂移、回环和地图重影。

# 主线任务 4：地图保存、补全与地图质量

## 目标

生成能被 HoloAgent `online_relo` 和 Nav2 正确使用的完整地图，并保证保存成功语义、文件一致性和场景覆盖真实可信。

## 当前状态

**已完成目标导航区域长图，稳定性待加强。** 地图结构、加载、后处理、同会话连续保存和目标区域质量门均已通过；全场景扩展、更多轮压力保存和正式 soak 尚未完成。

## 已完成内容

- 早期短链路验证地图：`holoagent_bridge/maps/mid360_sim_20260806_102736/`。
- 当前主验证地图：`holoagent_bridge/maps/mid360_final_long_20260807_a/`；同会话重复保存副本为对应的 `_b/` 目录。
- 当前长图每份包含：
  - 754 行 `mapping.txt`、`optimized_poses.txt` 和 `keyframe_pose.txt`
  - 754 个连续编号的 `keyframe_cloud/*.pcd`
  - 754 个连续编号的 `keyframe_scancontext/*.scd`
  - 1,432,144 点的 `cloudGlobal.pcd`
  - `singlesession_posegraph.g2o`、`cloudGlobal.ply`
  - 148 × 245、0.05 m/cell 的 `grid_map.pgm/yaml`
  - `map_topdown.png`
- 早期短地图包含：
  - 64 行 `mapping.txt`
  - 64 行 `optimized_poses.txt`
  - 64 行 `keyframe_pose.txt`
  - 64 个连续编号的 `keyframe_cloud/*.pcd`
  - 64 个连续编号的 `keyframe_scancontext/*.scd`
  - `singlesession_posegraph.g2o`
  - `cloudGlobal.pcd`、`cloudGlobal.ply`
  - `grid_map.pgm`、`grid_map.yaml`
  - `map_topdown.png`
- `prepare_reloc_map.py` 会检查索引连续性、pose/cloud/SCD 数量一致性和四元数有效性。
- 重建全局点云时，会把每个 keyframe cloud 按真实 keyframe pose 变换到 map frame 后再合并。
- FAST-LIVO 保存服务已改为同步调用 `saveKeyFrame()`，并在返回前检查关键文件和 keyframe 数量。
- 保存入口会在创建目标目录或调用 PCL writer 前拒绝零关键帧地图，避免空 PCD 异常终止进程。
- 建图启动命令现显式加载 `fast_livo_mid360_mapping_sim.yaml`；运行时已确认 `pcd_save=true`、`enable_gtsam=false` 和 `0.05 m / 0.01 rad` 关键帧阈值生效。
- 同一 FAST-LIVO 会话连续保存到 `mid360_save_retest_20260807_151740_a/b` 均返回 `success=True`；两份地图各含 46 个 pose/PCD/SCD，结构准备通过且第二次保存后进程仍存活。
- 最终长图连续保存到 `mid360_final_long_20260807_a/b` 均返回 `success=True`；两份地图各含 754 个 pose/PCD/SCD，`mapping.txt`、`cloudGlobal.pcd` 和生成栅格的 SHA-256 分别一致。
- `online_relo` 的 map loader 已修复为真正复制和检查 PCD 点，而不是文件可读但内存 keyframe 为空。

## 已修复问题

- 修复后处理直接拼接局部坐标 keyframe、导致 `cloudGlobal` 几何错误的问题。
- 修复 keyframe PCD 加载返回成功但目标点云为空的问题。
- 修复保存位姿文件追加写入、重复保存可能产生重复内容的问题。
- 修复服务过早返回成功而后台文件尚未完整写完的语义问题。
- 修复启动说明漏载 mapping overlay、定位虽正常但保存时关键帧恒为 0 的问题。
- 修复空关键帧保存继续调用 `pcl::PCDWriter::writeASCII`、抛出异常并 native abort 的问题。

## 正确性与局限证据

- 最终长图累计 XY 轨迹长度 9.506 m，轨迹边界约为 `[-0.153,-0.002]..[0.959,1.972]`。
- 754 个轨迹位姿全部位于同一自由空间连通域；轨迹最小障碍净空 0.552 m，5% 分位净空 0.743 m。
- 栅格包含 1725 个占用格、18797 个自由格和 15738 个未知格；人工俯视检查未发现轨迹区域明显双墙或断层。
- `map_topdown.png` 会投影所有高度且不做体素去重；其中约 65% 点的 `z < -0.5 m`，房间内部密集蓝点主要是累计地面回波，不代表 Nav2 障碍。Nav2 栅格仍使用 `min_obstacle_z=-0.3 m` 过滤地面。

- 早期短地图三类 keyframe 数据数量一致，索引从 0 到 63 连续。
- 连续保存复测的两份 `cloudGlobal.pcd` SHA-256 相同，文件大小均为 96203 bytes；`prepare_reloc_map.py --rebuild-global` 对两份目录均成功。
- `online_relo` 已真实加载 64 个 prior keyframe，加载过程不再出现空关键帧问题。
- 当前轨迹真实长度约 4.359 m，但覆盖范围仅约：
  - X：1.078 m
  - Y：0.615 m
  - Z：0.031 m
- 轨迹最终回到接近起点，净位移约 0.008 m。

以上 64-keyframe 数据描述的是早期短地图。当前 754-keyframe 地图已取代它成为目标区域主验证地图，但仍不是完整仓库全场景地图。

## 遗留问题

- 短图和 754-keyframe 长图的两次连续保存均已通过，但尚未完成更多轮压力保存和保存后的长时退出 soak。
- 目标导航区域长图已生成并通过基础质量门；完整仓库范围、更多闭环和多初始位姿覆盖仍未完成。
- 俯视渲染工具尚未提供地面高度过滤和体素去重参数，原始可视化会被重复地面点主导。
- 仓库中存在多个短测地图目录，尚未区分保留样本与可清理产物。

## 完成判据

- 同一运行中连续保存至少两次，服务结果与磁盘完整性一致，进程无 crash、死锁或遗留子进程。
- 生成覆盖目标导航区域的长轨迹地图。
- keyframe、pose、SCD、全局点云和 Nav2 栅格全部通过一致性检查。
- 人工图像检查与数值检查均无明显重影、断层或错误障碍。

# 主线任务 5：在线重定位

## 目标

使用实时真实 `/undistort_cloud` 和 `/aft_mapped_to_init` 对 prior map 做在线配准，持续输出可信的 map-frame 机器人位姿。

## 当前状态

**目标区域闭环已完成，长时和全场景稳定性待加强。** `online_relo` 已在早期短图与 754-keyframe 主验证地图上分别完成初始化和持续输出。

## 已完成内容

- `online_relo` 可加载早期 64-keyframe 地图和当前 754-keyframe 主验证地图。
- 使用真实点云做 PCL NDT 初始化和周期重配准。
- NDT 分辨率使用 1.0 m，避免把 0.2 m 预降采样分辨率错误当成协方差体素分辨率。
- 注册前检查 NDT target 是否存在足够可用体素，避免空 KD-tree/native crash。
- 发布：
  - `/pose`
  - `map -> base_link` TF
  - `/reloc_body_cloud`
  - `/relocalization/fitness_score`
  - `/relocalization/registration_success`
- `/reloc_body_cloud` 由真实 MID360 点经 USD 外参转换到 `base_link`，不是 raw bridge 的重复或伪造 topic。
- `/pose.twist` 由真实连续 LIO body pose 和仿真源时间计算，使用 1.0 s 位姿窗口抑制 FAST-LIVO burst update。

## 已修复问题

- 修复 prior keyframe 文件可读但点云未真正加载的问题。
- 修复 NDT 在无有效 target voxel 时进入不安全搜索的问题。
- 修复 body cloud 外参和 frame 标识不一致的问题。
- 修复 `/pose` twist 恒为零、Nav2 无真实速度反馈的问题。
- 避免使用 wall-time publication stamp 或周期性重定位校正计算速度，以免制造虚假速度尖峰。

## 正确性证据

动态重定位对 GT 的已有报告：

| 报告 | 平移 RMSE | 航向 RMSE | 最终平移误差 | 最大误差跳变 |
| --- | ---: | ---: | ---: | ---: |
| `relocalization_dynamic_accuracy_20260807_1000.json` | 0.01945 m | 0.00313 rad | 0.01532 m | 0.02037 m |
| `relocalization_extrinsic_dynamic_accuracy_20260807_1010.json` | 0.06021 m | 0.00565 rad | 0.04914 m | 0.07758 m |

本轮速度动态验证：

- 300 对 `/pose` 与 GT 同窗口速度样本。
- 速度相关系数：0.997516。
- 速度 MAE：0.009761 m/s。
- 速度 RMSE：0.015126 m/s。
- 输出速度均值：0.152306 m/s；GT 同窗口均值：0.147581 m/s。
- 停稳且 1 s 窗口耗尽后，速度 P95 为 0.000319 m/s。
- 真实运动中 GT 位移 1.192117 m，重定位位移 1.248834 m，差 0.056717 m。
- 运动阶段 NDT 配准 60/60 成功。
- 最终长图末位姿先验 `[0.3077,0.1749,-2.4955]` 初始化时 NDT score 为 0.006765；复位起点使用 `[0,0,0]` 先验时 score 约 0.00565，均显著低于 0.35 阈值。

## 遗留问题

- 尚未做 30 分钟以上重定位 soak，无法对内存增长、长期漂移和偶发 NDT 失败率做最终结论。
- 当前 prior map 已覆盖目标导航区域，但 XY 范围仍有限，不能外推到完整仓库或 kidnapped-robot recovery。
- 当前 `/pose` publication stamp 使用节点当前时间，而速度窗口使用传感器源时间；虽然运行时已验证，但时间策略仍需在最终全栈中持续检查 costmap message-filter 丢帧。

## 完成判据

- 在最终全场景地图上从多个不同初始位姿完成初始化。
- 长时运动中注册成功率、fitness、位姿跳变和 GT 误差持续满足阈值。
- 不出现 stale pose、非有限位姿、TF 断裂或持续 message-filter 丢帧。

# 主线任务 6：Nav2 地图与代价地图

## 目标

把真实建图结果转换为 Nav2 可用的静态栅格，并使用实时 MID360 body cloud 构建正确的局部和全局障碍代价地图。

## 当前状态

**目标区域闭环已完成，全场景和窄通道调参待补。** Nav2 已加载 754-keyframe 最终长图生成的栅格并完成真实长距离及多航点导航；静态栅格与实时代价地图均已排除地面误标。

## 已完成内容

- 已生成：
  - `grid_map.pgm`
  - `grid_map.yaml`
- 早期过滤地图栅格参数：
  - 分辨率：0.05 m/cell
  - 尺寸：204 × 194
  - origin：约 `[-3.899, -3.057, 0]`
- local/global obstacle layer 使用真实 `/reloc_body_cloud`。
- local/global costmap 的障碍最低高度从 -0.8 m 调整到 -0.3 m，以排除 base_link 下方约 1.1 m 的地面回波。
- `generate_nav2_map.py` 的静态障碍最低高度也从 -0.8 m 同步为 -0.3 m；此前静态/实时阈值不一致是“短地图无 >1 m 安全路径”的直接原因。
- 修正后的诊断地图保存在 `holoagent_bridge/maps/mid360_sim_20260806_102736_ground_filtered_20260807/`，原地图未被覆盖。
- 以 0.50 m 净空做距离变换后，复位点所在连通区域最长路径由约 0.71 m 增长到约 4.75 m；已验证目标路径的最小静态净空约 0.68 m。
- 复位后的初始位置上，机器人所在 local-costmap 栅格代价实测为 0；附近桌体和物体障碍仍保留。
- 当前主验证栅格位于 `holoagent_bridge/maps/mid360_final_long_20260807_a/`：148 × 245、0.05 m/cell，轨迹最小净空 0.552 m，所有 754 个轨迹点属于同一自由空间连通域。
- 当前主栅格已用于距离 1.834 m 的真实 action 和四航点可视化闭环，不再只是离线质量检查产物。

## 已修复问题

- 修复大量地面斜射点被 obstacle layer 标成障碍的问题。
- 修复同类地面点仍被静态栅格生成器保留、导致 Navfn 误判不可达的问题。
- 避免仅通过 costmap topic 存在判断其正确性，增加机器人所在栅格和邻域代价的运行时检查。
- 区分真实近桌体高代价与地面误标：机器人运动到桌体附近时，高代价属于合理结果。

## 遗留问题

- 当前静态地图已来自目标区域长轨迹，但覆盖仍不足以代表完整仓库全场景。
- 尚未对主验证地图做更严格的膨胀半径、机器人 footprint、约 0.35 m 净空窄通道和动态障碍清除系统调参。
- 运行中偶发出现 point cloud 时间早于 TF cache 的 message-filter drop，需要在最终时间策略下量化比例。

## 完成判据

- 使用最终全场景地图生成栅格。
- 机器人初始位置和已知自由区域代价正确，真实墙体/桌体/任务物体均被保留。
- 行走时 clearing/marking 工作稳定，无大面积地面假障碍。
- costmap 与 TF 长时更新，无持续丢帧。

# 主线任务 7：重定位输出接入 Nav2

## 目标

让 Nav2 使用真实重定位位姿、TF 和速度反馈，而不是不存在的 `/odom` 或零速度占位。

## 当前状态

**已完成。** Nav2 定位输入和生命周期启动已真实验证。

## 已完成内容

- `/pose` 使用：
  - `header.frame_id = map`
  - `child_frame_id = base_link`
- 发布对应的 `map -> base_link` TF。
- `bt_navigator.odom_topic` 设置为 `/pose`。
- `controller_server.odom_topic` 设置为 `/pose`。
- controller 和 navigator 的实际 ROS graph endpoint 均确认订阅 `/pose`。
- `/odom` 当前没有发布者，也不再被 Nav2 控制器误用。
- map server、controller server、planner server、behavior server、BT navigator、waypoint follower 和 velocity smoother 均已进入 `active`。

## 已修复问题

- 修复仅修改 `bt_navigator`、但 `controller_server` 仍等待 `/odom` 的配置遗漏。
- 修复 `/pose.twist` 全零导致 Nav2 OdomSmoother 没有真实速度反馈的问题。
- 增加静态回归测试，分别约束 navigator 和 controller 的 odom topic。

## 正确性证据

- ROS graph 实测 `/pose` 有一个真实 `pose_estimator_node` 发布者。
- `/pose` 同时有 `bt_navigator` 与 `controller_server` 两个订阅者。
- 两个节点运行时参数 `odom_topic` 均为 `/pose`。
- Nav2 所有关键 lifecycle node 均达到 `active [3]`。
- `/pose.twist` 与 GT 动态速度的一致性证据见主线任务 5。

## 遗留问题

- 需要在最终导航运行中继续观察 TF cache 和点云时间戳的偶发边界丢帧。

## 完成判据

本任务的接入判据已满足，并已在主验证长图的真实 Nav2 action 和四航点闭环中重复使用；后续只需继续量化 TF/message-filter 边界丢帧。

# 主线任务 8：Nav2 到 Unitree whole-body 控制闭环

## 目标

让真实 Nav2 goal 产生持续 `/cmd_vel`，经 DDS 进入 IsaacLab whole-body policy，使机器人按目标运动，并由重定位反馈闭环完成 action。

## 当前状态

**目标区域闭环已完成，策略边界待加强。** 命令保持与失联超时语义已经修复；A/B 动态实验确认 whole-body 策略只在特定曲线工作点具有可靠转向响应。ROS→DDS 桥现在对 `|yaw|>=0.75 rad/s` 的纯角速度或混合命令统一选择实测有效的方向性曲线工作点，较低 yaw 的普通平移命令保持不变。正负同位置 action、0.65 m 和 1.1468 m action、最终长图 1.834 m action，以及四航点可视化闭环均已返回 SUCCEEDED。

## 已完成内容

- `cmd_vel_to_unitree_dds.py`：
  - 订阅 `/cmd_vel`。
  - 以 20 Hz 持续发布到 `rt/run_command/cmd`。
  - 限制线速度、横向速度和角速度。
  - 0.5 s 未收到新命令后发送零速度。
- 仿真启动使用 `--action_source dds_wholebody`，确保 DDS 命令由真实 locomotion policy 消费。
- 修复 whole-body category 2 复位错误，现在触发 `reset_all_self`，便于重复做干净动态实验。
- 新增最小 C++ `NavigateToPose` 客户端，用真实 Nav2 action 做闭环验证。
- DWB 最大角速度由 0.35 提高到实测能触发策略响应的 0.8 rad/s。
- 高 yaw 适配覆盖纯 yaw 和 DWB 的混合命令：正 yaw 使用 `[0, +0.3, yaw, 0.8]`，负 yaw 使用 `[-0.3, 0, yaw, 0.8]`；默认阈值为 0.75 rad/s，可显式关闭。
- 修复 DDS 命令消费语义：接收端为每条命令记录单调时钟，策略在可配置的 0.5 s 新鲜期内持续使用最新命令，不再读取一次后立即写零。
- 即使 ROS→DDS 桥或发布者退出，策略侧也会在命令超时后独立回到 `[0, 0, 0, 0.8]`，保留失联停车保护。

## 已修复问题

- 修复只发送一次 `/cmd_vel`、被 stale timeout 清零而机器人不持续运动的使用问题；文档现在要求持续发布。
- 修复 whole-body DDS endpoint 已创建、但 action provider 未选择导致命令被忽略的问题。
- 修复 Nav2 controller 没有真实 odometry 速度反馈的问题。
- 对 yaw 符号做了正负短窗口 GT 对照；最终保留 ROS 与策略同号映射，没有保留错误的反号补丁。

## 已验证部分

### DDS 传输

- 独立 DDS subscriber 在 4 s 内收到 99 条消息，其中 80 条非零。
- 仿真 command shared memory 采样 380 次，其中 263 次为真实非零命令 `[0.3, 0, 0, 0.8]`。

### 线性运动因果性

- 持续发送 `linear.x = 0.6` 约 8 s 仿真源时间：
  - 运动阶段 GT 位移约 0.842 m。
  - 全过程位移约 1.027 m。
  - 停车阶段平均速度约 0.021 m/s。
- 同一初始状态下发送零命令约 10 s，GT 位移仅约 0.017 m。
- 因此线性位移来自真实控制命令，不是初始化漂移。

### 当前 Nav2 action 失败证据

- Nav2 连续输出最大 0.8 rad/s 的纯角速度。
- 记录到 783 帧非零 `/cmd_vel`，不是 one-shot 或传输中断。
- 40 s action 内：
  - GT 航向变化约 0.1107 rad。
  - 重定位航向变化约 0.1178 rad。
  - 定位反馈与 GT 仅差约 0.0071 rad。
- 机器人未在超时前达到约 0.5 rad 的目标航向，action 被取消。

这组数据能排除定位方向错误和 DDS 完全未送达。命令保持修复后的同类复测结果见下一节；复测没有显著改善航向响应，因此当前证据再次指向策略运动学限制。

## 本轮命令语义修复与复测

- `sim_main.py` 默认控制频率为 100 Hz，ROS→DDS 桥默认发布频率为 20 Hz。
- `DDSRLActionProvider.compute_current_observations()` 原先每读取一次命令，就主动把共享内存写成零命令。
- 该消费语义存在控制连续性风险，但由于当前仿真慢于实时，不能只按配置频率推断实际策略输入占空比。
- 当前修复使用 DDS 回调的 `time.monotonic_ns()` 作为新鲜度依据，不依赖共享内存原有的秒级时间戳；新鲜命令持续生效，超过 0.5 s 后归零。
- 修复后提交同位置、约 `+0.5 rad` 目标航向的真实 Nav2 goal，action 仍在 40 s wall-time 后超时。
- 本轮记录：
  - GT：492 帧、9.82 s 仿真源时间、航向变化 0.11082 rad、位移 0.05556 m。
  - 重定位：423 帧、航向变化 0.11296 rad、位移 0.06126 m。
  - GT 与重定位航向变化相差约 0.00214 rad，定位反馈方向和幅度仍一致。
- 修复前同类实验的 GT 航向变化约 0.1107 rad；本轮没有可观测改善。因此命令清零不是转向不足的主要根因，下一步应转向策略响应矩阵和曲线转向验证。
- 证据文件：
  - `holoagent_bridge/validation/ground_truth_yaw_hold_fix_20260807_retry.txt`
  - `holoagent_bridge/validation/relocalization_yaw_hold_fix_20260807_retry.txt`
  - `holoagent_bridge/validation/yaw_hold_fix_summary_20260807.json`

## 原地转向根因诊断

### 已排除

- ONNX 输入为 910 维，等于当前实现的 10 帧 × 91 维观测，未发现输入宽度错位。
- 受控 ONNX 推理中，把 yaw 命令从 0 改为 `+0.8` 会显著改变 12 维关节动作输出，模型并非忽略 yaw 字段。
- 正 yaw 命令产生正 GT yaw，命令符号没有反转。
- DDS 命令到达、保持、超时停车和重定位反馈方向均已分别验证。

### A/B 动态证据

保持 `angular.z = 0.8 rad/s`，只改变前向速度：

| 策略命令 `[x, y, yaw, height]` | GT 源时间 | GT 位移 | GT 航向变化 | 平均航向速率 |
| --- | ---: | ---: | ---: | ---: |
| `[0.0, 0.0, 0.8, 0.8]` | 7.88 s | 0.05556 m | 0.11081 rad | 0.01406 rad/s |
| `[0.3, 0.0, 0.8, 0.8]` | 5.18 s | 0.38877 m | 1.01728 rad | 0.19639 rad/s |

带前向速度后的平均航向速率约为纯原地命令的 13.97 倍。这证明策略具备正确方向的 yaw 控制能力，但工作点位于曲线行走而非原地旋转。

命令窗口按记录器启动延迟和 wall-time 发布时长近似截取；各窗口内的持续时间与速率均使用 simulator source time。另行记录零命令稳态 2.64 s，GT 位移仅 0.000015 m、航向变化 0.000020 rad，说明机器人最终能够停车；曲线控制仍需专门量化命令结束后的制动距离和停止时间。

### Nav2 侧触发机制

- 当前 `xy_goal_tolerance = 0.5 m`、`trans_stopped_velocity = 0.25 m/s`，并启用了 `RotateToGoal` critic。
- DWB 源码在进入 XY 容差且判定已停止后，会把所有 `linear.x != 0` 或 `linear.y != 0` 的轨迹标为非法，仅保留纯角速度轨迹。
- 因而 Nav2 正好在终点阶段强制选择策略最弱的输入模式；不断提高 `max_vel_theta` 不会解决运动学不兼容。

根因不是定位、DDS、yaw 字段或命令符号错误，而是 **DWB 终点原地旋转约束与 locomotion policy 的曲线转向能力不匹配**。完整数值摘要见 `holoagent_bridge/validation/turning_response_debug_summary_20260807.json`。

### 横移转向辅助验证与实现

- 真实 DDS 接收回调曾遗漏 `import time`，导致新增的 `time.monotonic_ns()` 在回调中抛出 `NameError` 并丢弃命令；直接写共享内存的实验不会暴露该问题。现已补齐导入，并增加动态回调测试。
- 长窗口响应：
  - `[-0.3, 0.0, 0.8, 0.8]`：6.22 s 内航向变化 3.3683 rad，平均 0.5415 rad/s。
  - `[0.0, 0.3, 0.8, 0.8]`：6.24 s 内航向变化 3.2926 rad，平均 0.5277 rad/s。
  - `[0.0, -0.3, 0.8, 0.8]`：6.04 s 内航向变化 3.2848 rad，平均 0.5438 rad/s。
- 贴近失败目标的短闭环：
  - `y=+0.3, yaw=+0.8` 达到 `+0.506 rad` 后停车，稳定位置漂移 0.212 m。
  - `y=-0.3, yaw=-0.8` 达到 `-0.506 rad` 后停车，稳定位置漂移 0.146 m。
  - 两个方向的漂移均低于当前 `xy_goal_tolerance=0.5 m`。
- `cmd_vel_to_unitree_dds.py` 现仅在新鲜命令满足零平移、`|yaw|>=1e-3` 时注入方向性辅助：正 yaw 使用 `linear.y=+0.3`，负 yaw 使用 `linear.x=-0.3`；可用 `--turn-assist-speed 0` 禁用。
- stale timeout 仍优先返回全零速度；普通平移命令不变。
- 数据摘要：`holoagent_bridge/validation/lateral_turn_assist_debug_summary_20260807.json`。

### 真实 Nav2 action 复测

- 完整启动 IsaacLab、MID360/IMU bridge、FAST-LIVO、online_relo、Nav2 和修改后的命令桥。
- 首次提交当前位置、绝对目标航向 `+0.5 rad`：
  - action 在约 9.7 s wall-time 内返回 `navigation_result_code=4`，客户端退出码 0，即 SUCCEEDED。
  - 命令桥实际输出 `[0.0, 0.3, 0.8, 0.8]`，完成后回到全零速度。
  - GT 稳定航向增加约 0.318 rad、位移约 0.259 m。
  - 重定位稳定航向增加约 0.326 rad、位移约 0.256 m，最大位移约 0.282 m。
  - GT 与重定位航向增量相差约 0.009 rad；位置漂移低于 0.5 m 容差。
- 首次成功日志发现 Nav2 停止前可能输出约 `-5e-16 rad/s` 的浮点 yaw 噪声；原非零判断会误触发一次反向辅助。已增加 `1e-3 rad/s` 死区及回归测试。
- 重启命令桥加载最终死区代码后，从新位置再次提交相对约 `+0.5 rad` 的目标，action 再次返回 SUCCEEDED。
- 第二次日志确认有效 yaw 阶段注入辅助，停止后持续输出全零，不再由浮点噪声触发横移。
- 第二次运行后的静止窗口：
  - GT 2.40 s 源时间内位移 0.000047 m、航向变化 0.000089 rad。
  - 重定位 9.13 s wall-time 内位移 0.00386 m、航向变化 0.00097 rad。
  - 按初始 map/world 航向偏置校正后，静止时 GT 与重定位航向约差 0.016 rad。
- 闭环摘要：`holoagent_bridge/validation/nav2_turn_assist_summary_20260807.json`。

### 方向性辅助负向与远位姿复测

- 原同号横移实现下，负向 action 虽返回 SUCCEEDED，但 GT 仅转动约 `-0.007 rad`，重定位却报告约 `-0.303 rad`，属于定位抢跑造成的假成功。
- 停止竞争 DDS 发布者后，直接命令与独立 IMU/GT 对照确认 `y=-0.3, yaw=-0.8` 最终能负向转动，但达到 `0.1 rad` 约需 1.56 s 源时间，慢于该 Nav2 短命令窗口。
- `x=-0.3, yaw=-0.8` 的短测在 2.36 s 源时间内达到 `-0.728 rad`、位移 `0.304 m`，因此负向纯 yaw 改用后退曲线工作点。
- 最终方向性代码下，同位置 `-0.5 rad` action 在 9.35 s wall-time 内返回 SUCCEEDED：
  - GT 航向变化 `-0.543 rad`、位移 `0.321 m`。
  - 重定位航向变化 `-0.536 rad`、位移 `0.290 m`。
  - 两者航向增量仅差约 `0.0062 rad`，排除了再次出现定位假成功。
- 0.65 m 远位姿 action 返回 SUCCEEDED；GT/重定位稳定位移分别为 `0.482/0.504 m`，航向变化分别为 `0.378/0.380 rad`。
- 约 1.0 m 的低净空回程目标未在 40 s 内完成，但桥日志确认普通 DWB 平移命令（例如 `x=0.35~0.38 m/s`）未经辅助改写；失败期间 Navfn 报告一次合法势场无法生成路径，随后在容差边缘反复修正航向。该结果记录为地图可达性与终端耦合边界，不据此增加桥侧状态机。
- 高负载下验证客户端的 10 s goal-response 等待曾先于服务器响应结束，现放宽为 20 s；这只修改验证工具，不改变 Nav2 action 的 40 s 执行窗口。
- 汇总：`holoagent_bridge/validation/nav2_directional_turn_assist_summary_20260807.json`。

### 开阔区大于 1 m action 复测

- 修正静态地图地面过滤后，从初始重定位位姿约 `[0.12, 0.07]` 提交目标 `[0.264, 1.212, yaw=1.45]`，目标直线距离 `1.1468 m`，候选直线路径最小静态净空约 `0.68 m`。
- 首次复测中 Nav2 持续输出普通混合命令 `linear.x≈0.158, angular.z=0.8`，但 40 s 内 GT 最大位移仅 `0.067 m`。这证明策略弱响应不只发生在纯 yaw，现有“仅零平移时辅助”的条件仍过窄。
- 将辅助触发条件改为 `|yaw|>=0.75 rad/s` 后，同一目标返回 `navigation_result_code=4`，客户端退出码 0，即 SUCCEEDED。
- 桥日志显示高 yaw 阶段输出 `[0, 0.3, 0.8, 0.8]`；航向进入有效区间后自动恢复普通 DWB 前进命令，例如 `x=0.50, yaw=0.718` 以及随后较低 yaw 的前进命令。
- GT 净位移 `0.7170 m`、XY 轨迹长度 `1.1898 m`、航向变化 `1.2512 rad`；重定位净位移 `0.7922 m`、航向变化 `1.2618 rad`，方向和幅度一致。净位移小于目标距离是现有 `xy_goal_tolerance=0.5 m` 的预期结果，本轮没有扩大容差。
- action 完成后 `/cmd_vel` 和 DDS 命令回零。摘要保存在 `holoagent_bridge/validation/nav2_filtered_over1m_mixed_assist_summary_20260807.json`。
- GT 使用 simulator source time，而 `/pose` 为满足 Nav2 使用 ROS wall time；两者无法直接按绝对时间戳做 RMSE，因此本轮只报告各自位移/航向不变量，不伪造时间对齐。

### 最终长轨迹地图与新地图闭环

- 使用真实 G1-12DOF whole-body policy 完成多航点与返程，FAST-LIVO 保存轨迹包含 754 个位姿，累计 XY 路径长度 9.506 m，覆盖范围约 1.11 m × 1.97 m。
- 同一内存状态连续保存到 `mid360_final_long_20260807_a/` 和 `_b/`，两次服务均返回 `success=True`；两份地图各含 754 个 keyframe PCD/SCD，`mapping.txt`、`cloudGlobal.pcd` 和生成栅格的 SHA-256 分别一致。
- 新栅格为 148 × 245、0.05 m/cell；754 个轨迹点全部落在同一自由空间连通域，轨迹最小净空 0.552 m，5% 分位净空 0.743 m。俯视图未见轨迹区域双墙或断裂。
- 用最终地图末位姿 `[0.3077, 0.1749, -2.4955]` 作为重定位先验，首次 NDT score 为 0.006765，显著低于 0.35 阈值；默认 `[0,0,0]` 先验在机器人未回到地图起点时会失败，这属于先验位置错误，不是地图加载失败。
- 在新地图上从约 `[0.281,0.139]` 向 `[0.237,1.972,yaw=1.026]` 提交距离 1.834 m 的真实 action，约 29 s 返回 SUCCEEDED；结束位姿约 `[0.059,1.953]`，DDS 随后回零。
- 针对中途低速停滞曾试验把非零平移抬升至 0.3 m/s，但持续 `[0.3,0,0.185,0.8]` 与 0.4 m/s 探针仍几乎无 GT 位移，因此该补丁已撤回。短时进入已验证的高 yaw 方向性曲线工作点后，同一目标以及后续 P2、返程和新地图长目标均成功。
- 汇总证据：`holoagent_bridge/validation/final_long_map_summary_20260807.json`。

### WebRTC 可视化四航点闭环

- 使用 Isaac Sim 5.1 WebRTC livestream 启动同一 G1-12DOF whole-body 任务，信令服务实际监听 TCP 49100；MID360、IMU、FAST-LIVO、`online_relo` 和 Nav2 同时保持真实运行。
- 复位起点重定位初始化 NDT score 约 0.00565。环线路径依次经过上侧 `[0.237,1.972]`、右上 `[0.842,1.874]`、右下 `[0.90,0.80]`，最后返回起点 `[0.044,-0.018]`。
- 可视化渲染降低了仿真推进速度，第一长段两次触发 40 s wall-time 客户端超时；第一次已真实前进约 1.2 m，第二次暴露低速组合命令死区。执行一次已验证的短时高 yaw 曲线脱困后，同一航点和其余三个航点均返回 SUCCEEDED。
- 返回起点后的重定位位置约 `[0.404,0.125]`，落在当前 0.5 m XY 目标容差内；action 完成后 `/cmd_vel` 和 DDS 命令保持为零。
- 该运行证明多方向闭环在可视化负载下仍可完成，同时再次确认 wall-time action 窗口和低速策略死区尚未消除，不能把超时简单归因于 Nav2 无路径或命令丢失。

## 当前待验证问题

- 大于 1 m 的开阔目标、1.834 m 主图目标和多方向闭环均已成功，但当前 `xy_goal_tolerance=0.5 m` 较宽，仍需收紧容差后复测，避免容差掩盖路径跟随问题。
- 约 1.0 m、最小障碍净空约 0.35 m 的目标触发过 Navfn 路径生成失败，并在位置容差边缘出现正负终端辅助切换；该低净空目标 40 s 内未完成。
- 方向性曲线辅助是当前 12DoF 策略的适配层；狭窄环境若不允许约 0.3 m 的终点曲线位移，仍需支持原地旋转的 locomotion policy。

## 下一步

1. 在主验证长图上收紧目标容差并补测狭窄目标，量化普通路径跟随、可视化慢实时和 40 s wall-time 窗口的影响。
2. 对低净空目标继续优先改善地图/目标选择与 Navfn 可达性，不增加桥侧状态机。
3. 若场景空间不允许终点曲线位移，替换或重新训练支持可靠原地旋转的 locomotion policy。

## 完成判据

- Nav2 action 返回真实 succeeded，而不是客户端超时或人为中止。
- GT 位移/航向与目标方向一致，重定位反馈误差满足阈值。
- 命令全程新鲜、有界，完成或失败后机器人可靠停车。
- 连续多个不同方向目标可重复成功。

# 主线任务 9：HoloAgent 主 Agent 接入

## 目标

确认 HoloAgent 主 Agent 如何读取定位、下发导航目标、获取 action 状态，并把导航与上层任务规划连接起来。

## 当前状态

**部分完成。** 主 Agent 的调用路径、Python ABI、HTTP 入口和 ROS topic 转发已打通；尚未在全量 IsaacLab/FAST-LIVO/Nav2 运行栈上由 AgentOS skill 重复完成真实 action。

## 已确认内容

- HoloAgent/Nav2 的预期控制入口是 `/navigate_to_pose` action，底层速度出口为 `/cmd_vel`。
- `online_relo` 已能提供 `/pose` 和 TF。
- C++ `NavigateToPose` 客户端可以连接真实 action server，用于排除 Python 客户端问题。
- 实际上层路径为：AgentOS `sem-nav-skill` / `rel-move-skill` → `robot_bridge` HTTP → `/chat_loc_pub` / `/relative_nav` → `/object_pose` → `nav_executor` 的 `BasicNavigator.goToPose()` → `/navigate_to_pose`。
- thirdparty `nav2_msgs` 已用 `/usr/bin/python3` 重建，Python 3.10 下 `NavigateToPose` typesupport 和 `BasicNavigator` 导入成功。
- AgentOS 两个导航脚本已与 bridge 的 `{"cmd": ...}` 协议对齐；真实 HTTP 请求均返回 200，ROS 端分别收到 `1.0,0.0,90` 和 `1F,lab,charger`。
- `nav_executor` 已能在 Python 3.10 下启动并加载 `unitree` signal registry，不再因默认 `robot_name=g1` 落入空 registry。
- 成功仍保持 `nav_finish`；取消和失败现分别上报 `nav_canceled` / `nav_failed`。
- `robot_bridge` 的 `/health` 已真实返回 200，关闭时 ROS spin 线程可干净退出，不再 native abort。

## 当前问题

- Python 3.10 `nav2_msgs` 是当前工作区生成产物，干净环境需按运行说明重建；`robot_bridge` 同时需在 Python 3.10 中安装 FastAPI/uvicorn。
- 底层真实 action 已成功，但本轮没有启动完整仿真、定位与 Nav2 运行栈，因此还缺 AgentOS skill 发起真实目标后的 feedback/result 证据。
- 尚未验证重定位丢失和 Agent 超时语义；当前已覆盖成功、失败和取消的终态上报代码路径。

## 下一步

1. 启动完整仿真、定位和 Nav2 栈，用 AgentOS skill 发送真实导航目标并归档 feedback/result。
2. 真实验证任务取消、失败、重定位丢失和停车安全路径。
3. 为 Agent 超时补充明确的取消与终态上报语义。

## 完成判据

- 主 Agent 能在正确 ABI 环境中调用真实 Nav2 action。
- 能收到并处理 succeeded、failed、canceled 和 timeout 状态。
- 不直接注入 GT，不绕过 online relocalization 和 Nav2 控制器。

# 主线任务 10：工程稳定性、测试与版本管理

## 目标

把当前实验链路整理为可重复运行、可回归验证、可安全纳入版本管理的工程实现。

## 当前状态

**部分完成。** 已有较完整的定向测试和验证脚本，但长时稳定性与仓库整理未完成。

## 已完成内容

- 关键工具脚本：
  - `holoagent_bridge/real_isaaclab_smoke.py`
  - `holoagent_bridge/validate_lidar_imu_sync.py`
  - `holoagent_bridge/record_ground_truth.py`
  - `holoagent_bridge/evaluate_localization_accuracy.py`
  - `holoagent_bridge/validate_relocalization.py`
  - `holoagent_bridge/prepare_reloc_map.py`
  - `holoagent_bridge/generate_nav2_map.py`
  - `holoagent_bridge/render_pcd_map_image.py`
  - `holoagent_bridge/validation/nav2_action_client/`
- 当前定向验证结果：
  - 父仓 `tests/`：81 passed。
  - HoloAgent bridge 静态集成测试：30 passed。
  - Nav2 静态地图生成测试：3 passed。
  - FAST-LIVO 定向 CTest：4/4 passed。
  - 相关 Python 文件编译检查通过。
- 根目录无范围 `pytest` 会误收集 HoloAgent thirdparty Nav2、sandbox 在线模型测试和不同 Python ABI 的 ROS 测试；这些收集错误已与本次定向回归分开记录。

## 已修复问题

- 为 shared memory、XYZ preprocess、地图加载、NDT target、GT 评估、Nav2 odom topic、地面过滤和复位逻辑增加回归覆盖。
- 明确 ROS bridge 使用 `/usr/bin/python3`，避免 conda Python 与 ROS Humble ABI 不兼容。
- 验证脚本使用 simulator source time 计算持续时间，避免慢于实时的仿真导致错误结论。

## 遗留问题

- 尚未完成全链路至少 30 分钟 soak。
- FAST-LIVO 两次连续保存已通过；尚未执行更多轮压力保存和保存后的长时退出稳定性测试。
- 父仓和 nested `HoloAgent` 仓库均存在大量已有修改与 untracked 文件。
- `holoagent_bridge/`、部分 `test/`、地图和验证产物尚未制定完整纳管/忽略策略。
- 多个短测地图和日志尚未分类清理；在确认保留策略前不能直接删除。

## 完成判据

- 有一条文档化的一键或分阶段启动流程，可在干净终端重复运行。
- 全链路 soak 中无进程泄漏、native crash、时间戳倒退、旧帧重放、非有限数据或失控运动。
- 明确区分源码、配置、必要验证样本、大体积地图产物和临时日志。
- 父仓与 nested 仓库的相关改动经过逐文件审查后再提交，不覆盖用户已有修改。

# 当前推荐推进顺序

1. **用主 Agent 完成真实 action 复测**：ABI 与 AgentOS→HTTP→ROS 调用路径已通，下一步在全栈运行时归档 feedback、result、取消和失败。
2. **收紧容差并补测控制边界**：量化 0.5 m 目标容差、低速组合命令死区、40 s wall-time 窗口和高 yaw 曲线适配边界。
3. **补测狭窄空间安全边界**：确认约 0.3 m 的终点曲线位移是否可接受；如不可接受，替换或重新训练支持原地旋转的 locomotion policy。
4. **扩大地图与重定位覆盖**：从当前约 1.11 m × 1.97 m 目标区域扩展到完整仓库，并验证多初始位姿、回环漂移和 kidnapped-robot recovery。
5. **执行连续保存压力测试和 30 分钟以上整链路 soak**：正式采集丢帧、NDT、TF、内存和退出稳定性指标，不能仅以进程持续存活代替报告。
6. **整理版本管理和地图/日志保留策略**：区分源码、主验证证据、大体积地图和可清理临时产物。

# 关键证据与入口

- 集成设计：`docs/holoagent/integration_design.md`
- 实施计划：`docs/holoagent/integration_plan.md`
- 运行说明：`holoagent_bridge/README.md`
- 当前主验证地图：`holoagent_bridge/maps/mid360_final_long_20260807_a/`
- 同会话重复保存地图：`holoagent_bridge/maps/mid360_final_long_20260807_b/`
- 早期短链路地图：`holoagent_bridge/maps/mid360_sim_20260806_102736/`
- 最终长图与 Nav2 汇总：`holoagent_bridge/validation/final_long_map_summary_20260807.json`
- LiDAR/IMU 同步报告：`holoagent_bridge/validation/lidar_imu_sync.json`
- LIO 动态精度：`holoagent_bridge/validation/lio_imu_dynamic_accuracy_20260806_2233.json`
- 重定位动态精度：`holoagent_bridge/validation/relocalization_extrinsic_dynamic_accuracy_20260807_1010.json`
- Nav2 配置：`HoloAgent/agentic_robot/core/src/nav_bringup/param/g1.yaml`
- FAST-LIVO 仿真配置：`holoagent_bridge/fast_livo_mid360_sim.yaml`
- online_relo 配置：`holoagent_bridge/fast_livo_mid360_reloc_sim.yaml`
