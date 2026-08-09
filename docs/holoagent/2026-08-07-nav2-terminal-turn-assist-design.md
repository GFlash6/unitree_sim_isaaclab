# Nav2 终端转向辅助设计

## 背景

当前仿真使用 G1 29DoF 机器人资产，但 locomotion ONNX 只输出 12 个腿部关节动作。动态实验确认该策略对纯 `angular.z` 响应极弱，对带平移速度的曲线转向响应正常。Nav2 DWB 的 `RotateToGoal` 在终点位置容差内强制输出零线速度，因此两者不兼容。

## 方案

只修改仿真 ROS→DDS 桥 `holoagent_bridge/cmd_vel_to_unitree_dds.py`：

- 正常平移命令保持原样。
- 当新鲜命令经限幅后 `linear.x`、`linear.y` 均近似为零，且 `angular.z` 为正时，注入 `linear.y=+0.3 m/s`。
- 同样条件下 `angular.z` 为负时，注入 `linear.x=-0.3 m/s`；短窗口 GT 实测表明该工作点的负向响应起步明显快于 `linear.y=-0.3 m/s`。
- 辅助速度通过统一 CLI 参数 `--turn-assist-speed` 配置，默认 `0.3 m/s`；设为 `0` 可关闭。
- stale timeout 优先级最高，超时命令始终输出 `[0, 0, 0, height]`，不得触发辅助。
- 不修改 action provider、ONNX 策略、DWB 源码或普通导航速度语义。

数据流为：

`Nav2 pure yaw -> ROS→DDS bridge selects a measured curve-turn working point -> existing 12DoF policy -> /pose feedback -> Nav2 closes yaw and position error`

## 依据与风险

- `y=+0.3, yaw=+0.8` 达到约 `+0.5 rad` 后，稳定位置漂移约 `0.212 m`。
- 原 `y=-0.3, yaw=-0.8` 在短 action 窗口内起步过慢；改用 `x=-0.3, yaw=-0.8` 后，2.36 s 源时间内航向变化 `-0.728 rad`、位移 `0.304 m`。
- 方向性实现下，负向真实 action 的 GT 航向变化 `-0.543 rad`、位移 `0.321 m`，与重定位的 `-0.536 rad`、`0.290 m` 一致。
- 风险是终点附近出现可见曲线位移；真实 `/pose` 反馈会继续进入 Nav2 闭环。狭窄或膨胀区边缘目标仍可能在位置修正与终端转向之间切换，应优先选择有足够净空的目标或使用支持原地旋转的策略。

## 验收

- 单元测试覆盖正负纯 yaw、普通平移、禁用辅助和 stale timeout。
- 原有桥接测试全部通过。
- 同位置约 `+0.5 rad` 与 `-0.5 rad` 的真实 `NavigateToPose` goal 均不再因纯 yaw 响应不足而超时。
- 保存 GT 与重定位结果并更新 `docs/holoagent/integration_status.md`。
