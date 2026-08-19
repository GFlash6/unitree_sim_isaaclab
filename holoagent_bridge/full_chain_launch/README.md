# HoloAgent 全真实链路启动脚本

此目录复用 2026-08-19 全链路实验中实际通过的入口、地图、模型和端口。`start_all.sh` 只启动服务，不自动发布任务或运动命令。

## IsaacLab 可视化启动

仅启动带窗口的 IsaacLab：

```bash
./01_isaaclab_visual.sh
```

让完整链路使用可视化 IsaacLab：

```bash
ISAAC_GUI=1 ./start_all.sh
```

默认 `ISAAC_GUI=0`，仍使用经过验证的无头模式。可视化模式要求当前终端的 `DISPLAY` 可连接。

## 一键启动

```bash
cd /home/ubuntu/agents/unitree_sim_isaaclab/holoagent_bridge/full_chain_launch
./00_preflight.sh
./start_all.sh
```

启动会等待真实 Isaac shared memory、ROS 传感器消息、定位 `TRACKING`、Nav2 lifecycle、HMSG、在线 OVO 真实查询、ROS Actions 和 robot bridge 依次就绪。后台日志写到 `runs/latest/logs/`。

另开终端检查状态：

```bash
cd /home/ubuntu/agents/unitree_sim_isaaclab/holoagent_bridge/full_chain_launch
./status.sh
```

下发一条真实 Agent 任务前，先确认机器人周围安全并配置 Qwen key：

```bash
export QWEN_API_KEY='你的 key'
./13_agent_task.sh "13号机器人导航到黄色塑料料箱附近"
```

停止本启动器启动的全部进程：

```bash
./stop_all.sh
```

停止脚本仅向 `.run/*.pid` 中经过进程启动时间校验的独立进程组发信号，不按模糊进程名批量杀进程。Isaac Kit 若不能在 15 秒内退出，最后会只强制停止该启动器记录的 Isaac 进程组。

## 逐个启动

每个长驻脚本在一个独立终端前台运行，按下面顺序启动：

```text
01_isaaclab.sh
02_wait_sim_ready.sh        # 数据就绪后自动退出
03_mid360_bridge.sh
04_imu_clock_bridge.sh
05_rgbd_bridge.sh
06_cmd_vel_dds_bridge.sh
07_localization.sh
08_nav2.sh
09_hmsg_server.sh
10_online_ovo.sh
11_task_plane.sh
12_robot_bridge.sh
```

示例：

```bash
cd /home/ubuntu/agents/unitree_sim_isaaclab/holoagent_bridge/full_chain_launch
./01_isaaclab.sh
```

保持终端运行，再打开下一个终端执行下一项。前台进程用 `Ctrl+C` 停止。`08_nav2.sh` 的现有上游 launch 总会尝试启动 RViz；纯 headless 终端中 RViz 可能报显示错误并退出，但只要 `./status.sh` 显示 `Nav2: ACTIVE`，Nav2 容器不受影响。

## 默认配置与覆盖

默认值位于 `config.sh`：

- Isaac 使用物理 GPU 0；在线 OVO 使用物理 GPU 1。
- 定位和 Nav2 都使用 `mid360_filtered_long_20260810_a`。
- HMSG 使用全链路实验产生的 9-object 实际图与 `sim_to_map.json`。
- HMSG 的在线锚点更新写入 `runtime/semantic_anchors.json`，不会改写验证证据目录。
- HTTP 端口为 robot bridge `8000`、HMSG `8120`、在线 OVO `8121`。

所有配置都能用同名环境变量临时覆盖，例如：

```bash
export PRIOR_MAP=/absolute/path/to/prepared_relocation_map
export NAV2_MAP=/absolute/path/to/grid_map.yaml
export CONTROL_URL=http://192.168.124.103:8080
export OVO_GPU=1
./start_all.sh
```

`PRIOR_MAP` 与 `NAV2_MAP` 必须来自同一地图坐标系。若更换 HMSG 场景，还必须一起覆盖 `SEMANTIC_SCENE_ROOT`、`HMSG_GRAPH_PATH`，并提供与该 Nav2 地图配套的 `sim_to_map.json`。

## 安全说明

- 不要同时运行两个 `sim_main.py`；预检会拒绝这种情况。
- `13_agent_task.sh` 是唯一会主动提交任务的脚本；执行前确认真实运动路径安全。
- 当前 whole-body 场景没有机械臂设备 adapter。Agent 的机械臂任务会被快速拒绝，不能把拒绝结果当作动作成功。
- `CONTROL_URL` 默认指向本机 `8080`；未运行多机器人控制中心时，旧反向回调可能记录连接失败，但 Action 的主结果轮询不依赖它。
