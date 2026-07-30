# G1 调用接口文档

本文面向外部 Python 程序调用 `unitree_sim_isaaclab` 的 G1 仿真接口。当前只整理 G1 本体、Dex1 夹爪、Wholebody 移动、仿真状态和相机接口；Dex3/Inspire 灵巧手操作暂不展开。

## 1. 启动仿真

进入环境：

```bash
conda activate unitree_sim_env
cd /home/ubuntu/agents/unitree_sim_isaaclab
export CYCLONEDDS_HOME=/home/ubuntu/agents/cyclonedds/install
```

PickPlace + Dex1 夹爪：

```bash
python sim_main.py --device cuda --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint \
  --enable_dex1_dds --robot_type g129
```

Wholebody 移动 + Dex1 夹爪：

```bash
python sim_main.py --device cuda --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody \
  --enable_dex1_dds --robot_type g129
```

无窗口远程运行可加：

```bash
--no_render --public_ip <本机IP>
```

第一次启动 Isaac Sim 如出现 EULA 提示，需要输入 `Yes`。

## 2. DDS 基础模板

所有外部脚本都要使用 channel `1`，否则收不到仿真端 topic。

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(1)
```

建议把外部脚本放在项目根目录运行：

```bash
conda activate unitree_sim_env
cd /home/ubuntu/agents/unitree_sim_isaaclab
python your_control_script.py
```

如果同一局域网内有真实 Unitree 机器人，DDS topic 名可能与真机一致。调试仿真时建议断开真机网络或确认 CycloneDDS 网卡配置，避免误控。

## 3. 接口总表

| 功能 | 方向 | Topic | 消息类型 | 备注 |
| --- | --- | --- | --- | --- |
| G1 本体状态 | Sim -> 外部 | `rt/lowstate` | `unitree_hg.msg.dds_.LowState_` | 关节、IMU、模式 |
| G1 本体底层命令 | 外部 -> Sim | `rt/lowcmd` | `unitree_hg.msg.dds_.LowCmd_` | 35 路 `motor_cmd`，需要 CRC |
| Dex1 左夹爪状态 | Sim -> 外部 | `rt/dex1/left/state` | `unitree_go.msg.dds_.MotorStates_` | 夹爪电机状态 |
| Dex1 右夹爪状态 | Sim -> 外部 | `rt/dex1/right/state` | `unitree_go.msg.dds_.MotorStates_` | 夹爪电机状态 |
| Dex1 左夹爪命令 | 外部 -> Sim | `rt/dex1/left/cmd` | `unitree_go.msg.dds_.MotorCmds_` | `q=0.0` 关，`q=5.4` 开 |
| Dex1 右夹爪命令 | 外部 -> Sim | `rt/dex1/right/cmd` | `unitree_go.msg.dds_.MotorCmds_` | `q=0.0` 关，`q=5.4` 开 |
| Wholebody 移动 | 外部 -> Sim | `rt/run_command/cmd` | `std_msgs.msg.dds_.String_` | JSON/list 字符串 |
| 重置姿态 | 外部 -> Sim | `rt/reset_pose/cmd` | `std_msgs.msg.dds_.String_` | 内容可用 `"1"` |
| 仿真状态 | Sim -> 外部 | `rt/sim_state` | `std_msgs.msg.dds_.String_` | JSON 字符串 |
| 仿真状态命令 | 外部 -> Sim | `rt/sim_state_cmd` | `std_msgs.msg.dds_.String_` | JSON 字符串 |
| Reward 状态 | Sim -> 外部 | `rt/rewards_state` | `std_msgs.msg.dds_.String_` | JSON 字符串 |
| Reward 命令 | 外部 -> Sim | `rt/rewards_state_cmd` | `std_msgs.msg.dds_.String_` | JSON 字符串 |
| 头部相机 | Sim -> 外部 | ZMQ `55555` / WebRTC `60001` | JPEG/BGR 视频流 | 不走 DDS |
| 左腕相机 | Sim -> 外部 | ZMQ `55556` / WebRTC `60002` | JPEG/BGR 视频流 | 不走 DDS |
| 右腕相机 | Sim -> 外部 | ZMQ `55557` / WebRTC `60003` | JPEG/BGR 视频流 | 不走 DDS |

## 4. 订阅 G1 状态

`LowState_` 主要字段：

| 字段 | 含义 |
| --- | --- |
| `motor_state[i].q` | 第 `i` 个关节位置 |
| `motor_state[i].dq` | 第 `i` 个关节速度 |
| `motor_state[i].tau_est` | 第 `i` 个关节估计力矩 |
| `imu_state.quaternion` | IMU 四元数 |
| `imu_state.rpy` | roll/pitch/yaw |
| `imu_state.gyroscope` | 角速度 |
| `imu_state.accelerometer` | 加速度 |
| `mode_machine` | 当前机器模式 |

最小订阅脚本：

```python
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


def on_lowstate(msg: LowState_):
    print("joint0 q:", msg.motor_state[0].q)
    print("imu rpy:", msg.imu_state.rpy)


ChannelFactoryInitialize(1)

sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(on_lowstate, 10)

while True:
    time.sleep(1)
```

## 5. 发布 Wholebody 移动命令

Wholebody 命令用于高层移动控制，适合先做键盘、手柄、上层策略控制。消息内容是字符串形式的数组：

```text
[x_vel, y_vel, yaw_vel, height]
```

字段含义：

| 下标 | 字段 | 含义 |
| --- | --- | --- |
| `0` | `x_vel` | 前后速度 |
| `1` | `y_vel` | 左右速度 |
| `2` | `yaw_vel` | 偏航角速度 |
| `3` | `height` | 高度目标或高度偏置 |

示例：

```python
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


ChannelFactoryInitialize(1)

pub = ChannelPublisher("rt/run_command/cmd", String_)
pub.Init()

cmd = String_()

# 前进，height 保持 0.8
cmd.data = "[0.3, 0.0, 0.0, 0.8]"
pub.Write(cmd)
time.sleep(1.0)

# 停止
cmd.data = "[0.0, 0.0, 0.0, 0.8]"
pub.Write(cmd)
```

对应现成脚本：

```bash
python agent/control_base/keyboard_control.py --mode command --no-grippers
```

Linux 终端使用 `command` 模式；默认 `keyboard` 模式依赖 Windows `msvcrt`。实测可用的移动测试：

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
| `raw <x> <y> <yaw> [height]` | 直接发送速度数组 |
| `hold <seconds>` | 保持当前命令持续发布 |
| `stop` | 停止 |
| `reset` | 重置 |
| `quit` | 退出 |

## 6. 发布 G1 底层关节命令

`rt/lowcmd` 会直接控制本体关节。推荐普通移动先用 `rt/run_command/cmd`，只有需要关节级控制时再使用 `lowcmd`。

G1 使用 `unitree_hg.msg.dds_.LowCmd_`。项目和 SDK 默认消息有 35 路 `motor_cmd`，G1 29DoF 常用前 29 路。

`LowCmd_` 主要字段：

| 字段 | 含义 |
| --- | --- |
| `mode_pr` | PR/AB 控制模式 |
| `mode_machine` | 机器模式，通常从 `lowstate.mode_machine` 复制 |
| `motor_cmd[i].mode` | `1` 启用，`0` 禁用 |
| `motor_cmd[i].q` | 目标位置 |
| `motor_cmd[i].dq` | 目标速度 |
| `motor_cmd[i].tau` | 前馈力矩 |
| `motor_cmd[i].kp` | 位置增益 |
| `motor_cmd[i].kd` | 速度增益 |
| `crc` | 写入前必须计算 |

安全保持当前位置模板：

```python
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


G1_NUM_MOTOR = 29
KP = [60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40,
      40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]
KD = [1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

latest_state = None


def on_lowstate(msg: LowState_):
    global latest_state
    latest_state = msg


ChannelFactoryInitialize(1)

sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(on_lowstate, 10)

pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()

crc = CRC()
cmd = unitree_hg_msg_dds__LowCmd_()

while latest_state is None:
    time.sleep(0.01)

while True:
    cmd.mode_pr = 0
    cmd.mode_machine = latest_state.mode_machine

    for i in range(G1_NUM_MOTOR):
        cmd.motor_cmd[i].mode = 1
        cmd.motor_cmd[i].q = latest_state.motor_state[i].q
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = KP[i]
        cmd.motor_cmd[i].kd = KD[i]

    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    time.sleep(0.002)
```

G1 常用关节下标：

| 下标 | 关节 |
| --- | --- |
| `0..5` | 左腿 |
| `6..11` | 右腿 |
| `12..14` | 腰部 |
| `15..21` | 左臂 |
| `22..28` | 右臂 |

## 7. 调用 Dex1 夹爪

Dex1 夹爪命令使用 `MotorCmds_`，每侧只需要写入 `cmds[0]`。

夹爪位置约定：

| `q` | 含义 |
| --- | --- |
| `0.0` | 闭合 |
| `5.4` | 张开 |

示例：

```python
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_


def make_gripper_cmd(q: float) -> MotorCmds_:
    msg = MotorCmds_()
    cmd = unitree_go_msg_dds__MotorCmd_()
    cmd.mode = 1
    cmd.q = q
    cmd.dq = 0.0
    cmd.tau = 0.0
    cmd.kp = 1.0
    cmd.kd = 0.0
    msg.cmds.append(cmd)
    return msg


ChannelFactoryInitialize(1)

left_pub = ChannelPublisher("rt/dex1/left/cmd", MotorCmds_)
right_pub = ChannelPublisher("rt/dex1/right/cmd", MotorCmds_)
left_pub.Init()
right_pub.Init()

# 张开
left_pub.Write(make_gripper_cmd(5.4))
right_pub.Write(make_gripper_cmd(5.4))
time.sleep(1.0)

# 闭合
left_pub.Write(make_gripper_cmd(0.0))
right_pub.Write(make_gripper_cmd(0.0))
```

订阅夹爪状态：

```python
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_


def on_state(msg: MotorStates_):
    if msg.states:
        print("gripper q:", msg.states[0].q)


ChannelFactoryInitialize(1)

sub = ChannelSubscriber("rt/dex1/left/state", MotorStates_)
sub.Init(on_state, 10)

while True:
    time.sleep(1)
```

## 8. 重置仿真姿态

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


ChannelFactoryInitialize(1)

pub = ChannelPublisher("rt/reset_pose/cmd", String_)
pub.Init()

msg = String_()
msg.data = "1"
pub.Write(msg)
```

## 9. 订阅仿真状态和 Reward

`rt/sim_state` 和 `rt/rewards_state` 发布的是 JSON 字符串，外部程序可以直接 `json.loads`。

```python
import json
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


def on_sim_state(msg: String_):
    print("sim_state:", json.loads(msg.data))


def on_reward_state(msg: String_):
    print("reward:", json.loads(msg.data))


ChannelFactoryInitialize(1)

sim_sub = ChannelSubscriber("rt/sim_state", String_)
reward_sub = ChannelSubscriber("rt/rewards_state", String_)
sim_sub.Init(on_sim_state, 10)
reward_sub.Init(on_reward_state, 10)

while True:
    time.sleep(1)
```

## 10. 读取相机图像

相机不走 DDS。仿真相机先写共享内存，再由 TeleImager 发布 ZMQ/WebRTC。

| 相机 | ZMQ | WebRTC |
| --- | --- | --- |
| 头部相机 | `tcp://<host>:55555` | `https://<host>:60001` |
| 左腕相机 | `tcp://<host>:55556` | `https://<host>:60002` |
| 右腕相机 | `tcp://<host>:55557` | `https://<host>:60003` |

使用 TeleImager 客户端：

```bash
python -m teleimager.image_client
```

或者直接订阅 ZMQ JPEG：

```python
import cv2
import numpy as np
import zmq


ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.setsockopt(zmq.RCVHWM, 1)
sock.connect("tcp://127.0.0.1:55555")
sock.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    jpg = sock.recv()
    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        cv2.imshow("head_camera", frame)
        cv2.waitKey(1)
```

WebRTC 预览可用浏览器访问对应端口，或使用 Isaac Sim WebRTC Streaming Client 查看渲染窗口。

## 11. 推荐调用顺序

1. 启动仿真，确认 EULA 已接受。
2. 外部脚本调用 `ChannelFactoryInitialize(1)`。
3. 先订阅 `rt/lowstate`，确认能收到关节和 IMU。
4. 需要移动时优先发布 `rt/run_command/cmd`。
5. 需要夹取时发布 `rt/dex1/left/cmd`、`rt/dex1/right/cmd`。
6. 需要关节级控制时再发布 `rt/lowcmd`，每次写入前计算 CRC。
7. 需要状态监控时订阅 `rt/sim_state`、`rt/rewards_state`。
8. 需要视觉时读取 ZMQ/WebRTC 相机流。

## 12. 常见问题

收不到 DDS 消息：

- 确认外部脚本是 `ChannelFactoryInitialize(1)`。
- 确认仿真已经启动并进入任务。
- 确认在同一个网络命名空间内运行；Docker 与宿主机混用时尤其要检查网络。

夹爪没有反应：

- 启动仿真时必须带 `--enable_dex1_dds`。
- 命令 topic 分左右手：`rt/dex1/left/cmd`、`rt/dex1/right/cmd`。
- `MotorCmds_` 需要至少包含一个 `MotorCmd_`。

低层关节命令没有生效：

- `LowCmd_` 写入前必须设置 `crc = CRC().Crc(cmd)`。
- `mode_machine` 建议从最新 `lowstate.mode_machine` 复制。
- `motor_cmd[i].mode` 需要设为 `1`。

相机没有图像：

- 启动仿真时需要 `--enable_cameras`。
- 确认读取的端口与相机对应。
- 无窗口远程运行时可使用 `--no_render --public_ip <本机IP>`。
