# 控制接口文档

本文整理 `unitree_sim_isaaclab` 中仿真控制相关的启动方式、DDS topic、消息类型和示例控制脚本。仿真端 DDS 使用 `unitree_sdk2py`，所有外部控制程序需要与仿真使用同一个 DDS channel。

完整 Python 调用模板见：[G1 调用接口文档](g1_call_interface_zh.md)。

## 1. 启动仿真

进入项目和环境：

```bash
conda activate unitree_sim_env
cd /home/ubuntu/agents/unitree_sim_isaaclab
export CYCLONEDDS_HOME=/home/ubuntu/agents/cyclonedds/install
```

普通夹爪任务示例：

```bash
python sim_main.py --device cuda --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint \
  --enable_dex1_dds --robot_type g129
```

<!-- 灵巧手 Dex3/Inspire 操作说明暂时隐藏。 -->

Wholebody 移动任务示例：

```bash
python sim_main.py --device cuda --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody \
  --enable_dex1_dds --robot_type g129
```

无窗口或远程运行可加：

```bash
--no_render --public_ip <本机IP>
```

## 2. DDS 初始化

外部控制程序必须使用 channel 1：

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(1)
```

仿真端在 `dds/dds_master.py` 中同样调用 `ChannelFactoryInitialize(1)`。如果同一网络内有真实机器人，请注意 topic 与真实设备可能一致，避免误控。

## 3. Topic 与消息类型

| 功能 | 方向 | Topic | 消息类型 | 启用条件 |
| --- | --- | --- | --- | --- |
| G1/H1 本体状态 | Sim 发布 | `rt/lowstate` | `unitree_hg.msg.dds_.LowState_` | `--robot_type g129` 或 `h1_2` |
| G1/H1 本体命令 | Sim 订阅 | `rt/lowcmd` | `unitree_hg.msg.dds_.LowCmd_` | `--robot_type g129` 或 `h1_2` |
| Dex1 左夹爪状态 | Sim 发布 | `rt/dex1/left/state` | `unitree_go.msg.dds_.MotorStates_` | `--enable_dex1_dds` |
| Dex1 右夹爪状态 | Sim 发布 | `rt/dex1/right/state` | `unitree_go.msg.dds_.MotorStates_` | `--enable_dex1_dds` |
| Dex1 左夹爪命令 | Sim 订阅 | `rt/dex1/left/cmd` | `unitree_go.msg.dds_.MotorCmds_` | `--enable_dex1_dds` |
| Dex1 右夹爪命令 | Sim 订阅 | `rt/dex1/right/cmd` | `unitree_go.msg.dds_.MotorCmds_` | `--enable_dex1_dds` |
<!-- 灵巧手 Dex3/Inspire topic 行暂时隐藏。 -->
| Wholebody 移动命令 | Sim 订阅 | `rt/run_command/cmd` | `std_msgs.msg.dds_.String_` | 任务名包含 `Wholebody` 或 `--enable_wholebody_dds` |
| 重置命令 | Sim 订阅 | `rt/reset_pose/cmd` | `std_msgs.msg.dds_.String_` | 默认创建 |
| 仿真状态 | Sim 发布 | `rt/sim_state` | `std_msgs.msg.dds_.String_` JSON | 默认创建 |
| 仿真状态命令 | Sim 订阅 | `rt/sim_state_cmd` | `std_msgs.msg.dds_.String_` JSON | 默认创建 |
| Reward 状态 | Sim 发布 | `rt/rewards_state` | `std_msgs.msg.dds_.String_` JSON | 默认创建 |
| Reward 命令 | Sim 订阅 | `rt/rewards_state_cmd` | `std_msgs.msg.dds_.String_` JSON | 默认创建 |

## 4. 机器人传感器接口

项目里的传感器/观测数据主要有三条出口：

1. 机器人本体 DDS 状态：关节、力矩、IMU，经 `rt/lowstate` 发布。
2. 相机图像：Isaac Sim 相机写入共享内存，再由 TeleImager 发布 ZMQ/WebRTC。
3. 仿真状态和奖励：经 `rt/sim_state`、`rt/rewards_state` 发布 JSON 字符串。

### 本体状态与 IMU

G1/H1 本体状态通过 `rt/lowstate` 发布，消息类型是 `LowState_`。数据来源在 `tasks/common_observations/g1_29dof_state.py` 和 `tasks/common_observations/h12_27dof_state.py`。

关节状态：

```text
LowState_.motor_state[i].q       关节位置
LowState_.motor_state[i].dq      关节速度
LowState_.motor_state[i].tau_est 估计/施加力矩
```

IMU 状态：

```text
LowState_.imu_state.quaternion     四元数
LowState_.imu_state.accelerometer  加速度
LowState_.imu_state.gyroscope      角速度
```

仿真内部 IMU 原始拼接格式是 13 维：

```text
[pos(3), quat_wxyz(4), acceleration_body(3), angular_velocity_body(3)]
```

写入 `LowState_` 时会使用其中的四元数、加速度和角速度。

### 相机图像

启用相机：

```bash
--enable_cameras
```

默认相机名称：

```text
front_camera        -> head
left_wrist_camera   -> left
right_wrist_camera  -> right
```

相机图像由 `tasks/common_observations/camera_state.py` 从 Isaac Lab sensor 读取 RGB 图像，并写入多图共享内存。TeleImager 的 IsaacSimCamera 再从共享内存读取 `head`、`left`、`right` 三路图像并发布。

当前 `teleimager/cam_config_server.yaml` 默认配置：

| 相机 | 图像源 | ZMQ 端口 | WebRTC 端口 | 分辨率 | FPS |
| --- | --- | --- | --- | --- | --- |
| `head_camera` | `head` | `55555` | `60001` | `480x640` | `30` |
| `left_wrist_camera` | `left` | `55556` | `60002` | `480x640` | `30` |
| `right_wrist_camera` | `right` | `55557` | `60003` | `480x640` | `30` |

相机配置关键字段：

```yaml
type: isaacsim
image_shape: [480, 640]
enable_zmq: true
enable_webrtc: true
webrtc_codec: h264
```

运行时可用参数：

```text
--camera_include front_camera,left_wrist_camera,right_wrist_camera
--camera_exclude world_camera
--camera_write_interval <步数>
--camera_jpeg
--camera_jpeg_quality 85
--skip_cvtcolor
```

### 仿真状态和奖励

仿真状态：

```text
rt/sim_state      String_ JSON
rt/sim_state_cmd  String_ JSON
```

奖励状态：

```text
rt/rewards_state      String_ JSON
rt/rewards_state_cmd  String_ JSON
```

这些接口主要用于外部监控、数据记录、回放和调试。具体 JSON 内容由运行中的任务和写入共享内存的数据决定。

## 5. G1/H1 本体控制接口

本体接口在 `dds/g1_robot_dds.py`。

状态发布 `rt/lowstate`：

- 关节位置：`LowState_.motor_state[i].q`
- 关节速度：`LowState_.motor_state[i].dq`
- 估计力矩：`LowState_.motor_state[i].tau_est`
- IMU：`LowState_.imu_state`

命令订阅 `rt/lowcmd`：

- `mode_pr`
- `mode_machine`
- `motor_cmd[i].q`
- `motor_cmd[i].dq`
- `motor_cmd[i].tau`
- `motor_cmd[i].kp`
- `motor_cmd[i].kd`
- `crc`

仿真端会检查 `LowCmd_` 的 CRC，外部发送 `rt/lowcmd` 时需要按 Unitree SDK 的方式计算并写入 CRC。

## 6. Wholebody 移动接口

Wholebody 移动使用 `rt/run_command/cmd`，消息类型是 `String_`。字符串内容是一个 Python list 风格的 4 元素数组：

```text
[x_vel, y_vel, yaw_vel, height]
```

含义：

| 字段 | 含义 | 示例范围 |
| --- | --- | --- |
| `x_vel` | 前后速度，正数前进 | `[-0.6, 1.0]` |
| `y_vel` | 左右速度 | `[-0.5, 0.5]` |
| `yaw_vel` | 偏航角速度 | `[-1.57, 1.57]` |
| `height` | 站高目标值 | 默认约 `0.8`，下蹲时降低 |

示例发送：

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

ChannelFactoryInitialize(1)
pub = ChannelPublisher("rt/run_command/cmd", String_)
pub.Init()
pub.Write(String_(data="[0.2, 0.0, 0.0, 0.8]"))
```

注意：只有任务名带 `Wholebody` 的任务才会把移动命令接入控制，例如 `Isaac-Move-Cylinder-G129-Dex1-Wholebody`。

## 7. 键盘和手柄控制

Linux 终端推荐使用命令模式：

```bash
conda activate unitree_sim_env
cd /home/ubuntu/agents/unitree_sim_isaaclab
python agent/control_base/keyboard_control.py --mode command --no-grippers
```

实测移动命令：

```text
raw 0.8 0 0 0.8
hold 5
stop
```

常用命令：

| 命令 | 功能 |
| --- | --- |
| `forward` / `back` | 前进 / 后退 |
| `left` / `right` | 左移 / 右移 |
| `turn_left` / `turn_right` | 左转 / 右转 |
| `raw <x> <y> <yaw> [height]` | 直接发送 `[x_vel, y_vel, yaw_vel, height]` |
| `hold <seconds>` | 保持当前命令持续发布 |
| `stop` | 停止 |
| `reset` | 重置姿态 |
| `quit` | 退出 |

脚本发布到 `rt/run_command/cmd`，发布内容同样是 `[x_vel, y_vel, yaw_vel, height]`。默认 `keyboard` 模式只支持 Windows 终端；Linux 下用 `--mode command`。

8BitDo 手柄控制脚本：

```bash
conda activate unitree_sim_env
cd /home/ubuntu/agents/unitree_sim_isaaclab
python send_commands_8bit.py
```

手柄映射：

| 输入 | 功能 |
| --- | --- |
| 左摇杆上下 `ABS_Y` | 前进 / 后退 |
| 左摇杆左右 `ABS_X` | 左右平移 |
| 右摇杆左右 `ABS_RX` | 左右旋转 |
| 右摇杆上下 `ABS_RY` | 高度 / 下蹲 |

脚本会查找名称包含 `8BitDo` 的输入设备。

## 8. 夹爪接口

### Dex1 二指夹爪

启用：

```bash
--enable_dex1_dds
```

Topic：

- `rt/dex1/left/cmd`
- `rt/dex1/right/cmd`
- `rt/dex1/left/state`
- `rt/dex1/right/state`

消息类型：

- 命令：`MotorCmds_`
- 状态：`MotorStates_`

每侧当前只处理 1 个 `cmds/states` 元素。命令字段：

```text
cmds[0].q
cmds[0].dq
cmds[0].tau
cmds[0].kp
cmds[0].kd
```

`q` 在 DDS 侧是夹爪归一化控制值，仿真端会通过 `tools.data_convert.convert_to_joint_range()` 转换为 Isaac Lab 关节角。

<!-- 灵巧手 Dex3/Inspire 接口说明暂时隐藏。 -->

## 9. 重置接口

重置命令：

```text
Topic: rt/reset_pose/cmd
Type:  std_msgs.msg.dds_.String_
```

`sim_main.py` 中会读取 `reset_category`。当前逻辑里常见值：

- `"1"`：普通 reset
- `"2"`：Wholebody 相关 reset 分支

## 10. 常见注意事项

- 当前文档只保留 Dex1 二指夹爪操作，Dex3/Inspire 灵巧手操作说明暂时隐藏。
- Wholebody 移动必须使用带 `Wholebody` 的任务，普通 PickPlace/Stack 任务不会按移动命令行走。
- 外部 DDS 程序必须调用 `ChannelFactoryInitialize(1)`。
- G1/H1 本体命令 `rt/lowcmd` 需要正确 CRC。
- 相机不走 DDS topic，而是 Isaac Sim sensor -> 共享内存 -> TeleImager ZMQ/WebRTC。
- 如果同一局域网内有真实机器人，仿真 topic 与真实机器人 topic 可能重名，先隔离网络或确认 DDS 域/通道，避免误发命令。
