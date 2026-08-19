# HoloAgent ROS 2 module contract

This is the runtime contract for the Isaac G1 stack. Source directories may be
moved later; topic, service, action, frame, and time semantics are the stable
boundary.

## Runtime ownership

| Block | Owns | ROS input | ROS output |
|---|---|---|---|
| `robot_io` | Isaac shared memory, Unitree DDS, sensor calibration | `cmd_vel` | `sensors/lidar/points`, `sensors/imu/data`, `sensors/front/{rgb,depth,camera_info}`, sensor TF, `/clock` |
| `lio` | FAST-LIVO local odometry and deskew | `sensors/lidar/points`, `sensors/imu/data` | `lio/odom`, `lio/undistorted_points`, `lio/path`, `lio/save_map` |
| `localization` | Prior-map registration and `map -> base_link` | `lio/odom`, `lio/undistorted_points`, `initialpose` | `localization/odom`, `localization/status`, TF |
| `semantic_map` | HMSG graph, online OVO, object anchors | RGB-D, CameraInfo, TF | `semantic_map/query_object`, `semantic_map/update_anchor` |
| `navigation` | Nav2, relative/named/semantic goal execution | localization, obstacles, task Actions | `cmd_vel`, Nav2 Actions, `navigation/navigate_to_object` |
| `manipulation` | Robot-independent arm task lifecycle | `manipulation/execute_skill` | action feedback/result; robot adapter topics are internal |
| `agent_gateway` | HTTP compatibility and task routing | HTTP | ROS topics/services/Actions |

`generate_nav2_map.py`, relocation-map preparation, validation, and map export
are an eighth, offline tool block. They exchange versioned files, not live ROS
messages.

HMSG (`8120`) and online OVO (`8121`) HTTP servers are intentionally retained
inside `semantic_map`. Navigation never imports shared-memory utilities or calls
those ports; it calls `QueryObject` and `UpdateObjectAnchor` ROS services.
HMSG converts its offline Isaac-world object coordinates with a versioned
`sim_to_map.json` artifact, not a live simulator pose. Generate that artifact
from one simultaneous pose pair when a map is created:

```bash
/usr/bin/python3 holoagent_bridge/generate_sim_to_map.py OUT.json \
  --robot-sim X Y Z QW QX QY QZ \
  --robot-map X Y Z QW QX QY QZ \
  --source "map calibration run ID"
```

## Data flow

```text
Isaac sensors -> robot_io -> LIO -> localization -> Nav2 -> cmd_vel -> robot_io
                      |             |              ^
                      +-> RGB-D ----+-> TF -> semantic_map -> semantic action

HTTP Agent -> agent_gateway -> ROS Action -------------------^
                         +-> manipulation Action -> G1 adapter -> Unitree SDK
```

The obstacle cloud is currently produced by the tested online-relocalization
process and exposed as `perception/obstacles`. This is a compatibility adapter,
not an input to the localization algorithm. It may later move to a dedicated
point-cloud adapter without changing Nav2.

## Frame contract

- `map`: prior-map/global navigation frame.
- `base_link`: localized robot body frame.
- `mid360_link`, `imu_link`, `front_camera_optical_frame`: sensor frames.
- Localization is the only owner of `map -> base_link`.
- `robot_io` owns `base_link -> sensor` transforms. The front-camera transform
  is dynamic because the bridge exports the measured simulator relationship;
  it must not be replaced by a static TF unless the USD joint is verified fixed.
- Every `PointCloud2` describes points in its `header.frame_id`; remapping a
  topic never changes frame semantics.

For multiple robots, put relative topic names under a ROS namespace and give TF
frames a matching prefix. `/clock` remains global.

## Time and replay contract

The Isaac profile has one time domain:

1. LiDAR and IMU carry `env.sim.current_time` in nanoseconds.
2. RGB-D shared-memory records carry the same simulator time in milliseconds.
3. IMU bridge publishes that time on `/clock`.
4. LIO, localization, Nav2, semantic mapping, and task nodes use
   `use_sim_time:=true`.
5. Localization odom, TF, path, and obstacle cloud preserve the source-cloud
   stamp instead of replacing it with wall time.

Therefore a rosbag recorded from the canonical topics can be replayed with
`ros2 bag play BAG --clock` without regenerating timestamps. A real-robot
profile may use system time, but all nodes in one run must select the same
profile.

## QoS and task semantics

- LiDAR is reliable depth 10; IMU is reliable depth 2000 to match FAST-LIVO.
- RGB, depth, and CameraInfo use identical reliable queues and identical stamps.
- Long-running semantic navigation and manipulation use Actions, which provide
  goal IDs, feedback, cancellation, and one terminal result.
- `LocalizationStatus` distinguishes `INITIALIZING`, `TRACKING`, `DEGRADED`,
  `LOST`, and `STALE`; consumers do not infer freshness from an unstamped Bool.
- Legacy `chat_*`, `waypoint_reached`, `arm_signal_pub`, and HTTP routes remain
  compatibility entrances. New code should not depend on their String protocol.

## Launch and verification

Build the core layer, then launch the data plane and task plane separately:

```bash
bash HoloAgent/agentic_robot/build.sh -w core
source HoloAgent/agentic_robot/core/install/setup.bash

ros2 launch fast_livo isaac_localization.launch.py \
  prior_map:=/absolute/path/to/prepared_map/
ros2 launch nav_bringup holoagent_tasks.launch.py
ros2 launch nav_bringup g1_navigation2_launch.py \
  map:=/absolute/path/to/nav2_map.yaml use_sim_time:=true
```

Start the Isaac bridge processes and the two semantic HTTP model processes in
their required Python environments. The automated contract and regression suite
is:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q tests \
  --ignore=tests/test_hmsg_query_transform.py \
  --ignore=tests/test_fsr_model_loading.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/ubuntu/miniconda3/envs/holoagent_semantic_mapping/bin/python \
  -m pytest -q tests/test_hmsg_query_transform.py tests/test_fsr_model_loading.py
```
