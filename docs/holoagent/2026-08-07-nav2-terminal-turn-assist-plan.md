# Nav2 终端转向辅助实施计划

**目标：** 让现有 12DoF 腿部策略能够完成 Nav2 的终端航向调整，同时保持普通速度命令和失联停车语义不变。

**实现边界：** 只在 ROS→DDS 桥转换纯 yaw 命令；不修改 ONNX、action provider 或 Nav2 DWB 源码。使用 Python 标准库和现有测试，不增加依赖。

## 任务 1：测试纯 yaw 辅助语义

**文件：**

- 修改 `tests/test_holoagent_bridge_static.py`
- 测试 `holoagent_bridge/cmd_vel_to_unitree_dds.py::command_from_twist`

- [x] 添加正 yaw 测试：输入 `[x=0, y=0, yaw=0.8]`，期望 DDS 命令 `[0, 0.3, 0.8, height]`。
- [x] 添加负 yaw 测试：输入 `[x=0, y=0, yaw=-0.8]`，期望 DDS 命令 `[-0.3, 0, -0.8, height]`。
- [x] 添加普通平移测试：只要 `x` 或 `y` 非零，不注入辅助横移。
- [x] 添加禁用测试：`--turn-assist-speed 0` 时纯 yaw 保持原样。
- [x] 扩充 stale 测试：即使 yaw 非零，超时后仍为 `[0, 0, 0, height]`。
- [x] 运行目标测试，确认因参数或行为尚不存在而失败。

运行命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/ubuntu/miniconda3/envs/unitree_sim_env/bin/python -m pytest -q tests/test_holoagent_bridge_static.py -k 'turn_assist or goes_zero_when_stale'
```

## 任务 2：实现最小桥接转换

**文件：**

- 修改 `holoagent_bridge/cmd_vel_to_unitree_dds.py`

- [x] 添加允许零值的 `--turn-assist-speed` 参数，默认 `0.3`。
- [x] 在 stale 判断之后先完成 `x/y/yaw` 限幅。
- [x] 仅当 `x`、`y` 均近似为零且 yaw 超过 `1e-3 rad/s` 死区时，正 yaw 使用 `y=+speed`，负 yaw 使用 `x=-speed`。
- [x] 返回转换后的 `[x, y, yaw, height]`。
- [x] 运行任务 1 测试并确认通过。

## 任务 3：回归验证与文档

**文件：**

- 修改 `holoagent_bridge/README.md`
- 修改 `docs/holoagent/integration_status.md`

- [x] 记录辅助参数、触发条件和 `0` 禁用方式。
- [x] 记录 DDS 时间戳缺失导入的根因及动态回归测试。
- [x] 记录本轮转向响应矩阵和短时 ±0.5 rad 实验文件。
- [x] 运行完整桥接测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/ubuntu/miniconda3/envs/unitree_sim_env/bin/python -m pytest -q tests/test_holoagent_bridge_static.py
```

- [x] 对本轮文件运行 Python 编译检查和 `git diff --check`。

## 任务 4：真实 Nav2 闭环复测

- [x] 启动当前 G1 29DoF 资产与 12DoF whole-body policy。
- [x] 启动传感器桥、重定位、Nav2 与修改后的 ROS→DDS 桥。
- [x] 提交同位置、约 `+0.5 rad` 的 `NavigateToPose` goal。
- [x] 保存 GT、重定位和 action 结果。
- [x] 验收 action 成功；最终代码连续两次返回 SUCCEEDED。
- [x] 补测约 `-0.5 rad` 负向 action；GT 与重定位一致且返回 SUCCEEDED。
- [x] 补测 0.65 m 远位姿 action；GT/重定位位移约 0.48/0.50 m 且返回 SUCCEEDED。
