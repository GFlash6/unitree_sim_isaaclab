# HoloAgent IsaacLab Bridge

This bridge connects the current Unitree IsaacLab simulation to HoloAgent through real ROS2 and Unitree DDS interfaces.

It does not publish synthetic data. The MID360 bridge reads the point cloud written by `sim_main.py` from the real IsaacLab ray-caster sensor that was added to this workspace. The velocity bridge subscribes to ROS2 `/cmd_vel` and publishes the same `rt/run_command/cmd` DDS command consumed by wholebody simulation tasks.

## Run

Terminal 1, start a wholebody sim task:

```bash
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
conda activate unitree_sim_env
env -u DISPLAY python sim_main.py --device cuda:0 --headless --livestream 0 --enable_cameras --task Isaac-Move-Cylinder-G129-Dex1-Wholebody --action_source dds_wholebody --robot_type g129 --enable_dex1_dds --enable_wholebody_dds
```

Keep `DISPLAY` unset for this headless launch. A reachable X server can still
make Vulkan initialization select the X/GLX path and fail with
`GLXBadFBConfig`; `env -u DISPLAY` forces the verified headless path.

Both `--action_source dds_wholebody` and `--enable_wholebody_dds` are required:
the first selects the locomotion policy action provider, while the second
creates its DDS run-command endpoint. With only the second flag, commands can
arrive on DDS but the joint-command provider will ignore them.

Terminal 2, source ROS2 and start the MID360 bridge:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 holoagent_bridge/mid360_to_ros2_topic.py
```

Terminal 3, publish the real IsaacLab body IMU with its original simulator timestamp:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 holoagent_bridge/imu_to_ros2_topic.py
```

The IMU bridge publishes only fresh shared-memory records. It exits non-zero
if source timestamps repeat or move backwards; it never substitutes ROS wall
time or fabricated measurements. It also publishes that simulator time on
global `/clock`.

Terminal 4, publish synchronized RGB-D, CameraInfo, and measured camera TF:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 holoagent_bridge/isaac_rgbd_pose_bridge.py
```

This bridge has no localization subscription. `robot_io` owns only the
`base_link -> front_camera_optical_frame` transform; localization separately
owns `map -> base_link`.

Terminal 5, source ROS2 and start the command bridge:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 holoagent_bridge/cmd_vel_to_unitree_dds.py
```

Terminal 6, continuously send a real ROS2 velocity command while motion is
required:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop the publisher to stop the robot. The bridge republishes the latest fresh
command to DDS at 20 Hz and switches to zero velocity when no new `/cmd_vel`
message has arrived for 0.5 seconds. A one-shot publication therefore is not a
valid sustained-motion command.

The current 12-DoF locomotion policy responds weakly to both pure yaw and the
high-yaw mixed commands produced by DWB. At `|yaw| >= 0.75 rad/s`, the bridge
therefore replaces the planar part with measured working points: positive yaw
uses a `linear.y=+0.3 m/s` curve and negative yaw uses a
`linear.x=-0.3 m/s` curve. Below that threshold, ordinary planar commands pass
through unchanged. Set `--turn-assist-speed 0` to disable the adapter, or tune
`--turn-assist-yaw-threshold` explicitly. Stale commands always become zero
velocity before this adjustment is considered.

Terminal 7, start HoloAgent LIO and prior-map localization with canonical
topics (omit `online_relo` only when building a new map):

```bash
source /opt/ros/humble/setup.bash
source HoloAgent/agentic_robot/core/install/setup.bash
ros2 launch fast_livo isaac_localization.launch.py \
  prior_map:=/absolute/path/to/prepared_relocation_map/
```

To build a new relocation map, run only FAST-LIVO with the complete base
configuration followed by the Isaac and mapping overlays:

```bash
ros2 run fast_livo fastlivo_mapping --ros-args \
  --params-file HoloAgent/agentic_robot/core/install/fast_livo/share/fast_livo/config/mid360_online_livo.yaml \
  --params-file HoloAgent/agentic_robot/core/install/fast_livo/share/fast_livo/config/camera_d435i.yaml \
  --params-file holoagent_bridge/fast_livo_mid360_sim.yaml \
  --params-file holoagent_bridge/fast_livo_mid360_mapping_sim.yaml \
  -p use_sim_time:=true \
  -r /undistort_cloud:=lio/undistorted_points \
  -r /aft_mapped_to_init:=lio/odom
```

Before starting FAST-LIVO, keep the robot stationary and require the live
sensor validator to pass:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 holoagent_bridge/validate_lidar_imu_sync.py --duration 10 \
  --output-json holoagent_bridge/validation/lidar_imu_sync.json
```

`--duration` is measured in source simulator time; the default wall timeout is
120 seconds so a slower-than-real-time rendered simulation is still measured
correctly. It exits with status 2 for missing/stale samples, non-finite or non-monotonic
data, insufficient rates or overlap, excessive timestamp separation, an
implausible gravity magnitude, or stationary angular motion. The FAST-LIVO
overlay contains the LiDAR-to-IMU transform derived from the G1 USD fixed-joint
poses: it is not an identity placeholder.

Record the independent IsaacLab `imu_in_torso` pose during an actual run, then compare a
timestamped eight-column localization trajectory such as `relo_pose.txt`:

```bash
/usr/bin/python3 holoagent_bridge/record_ground_truth.py \
  holoagent_bridge/validation/ground_truth.txt --duration 30
/usr/bin/python3 holoagent_bridge/evaluate_localization_accuracy.py \
  holoagent_bridge/validation/ground_truth.txt /absolute/map/relo_pose.txt \
  --output-json holoagent_bridge/validation/relocalization_accuracy.json
```

The FAST-LIVO state and ground truth both refer to the IMU rigid body, so the
comparison does not hide a root-to-IMU lever-arm error. The evaluator uses
ground truth only after the run. It removes only the initial
rigid map/world origin alignment, then checks translation/yaw RMSE, final error,
and discontinuities. Ground truth is never published into FAST-LIVO,
relocalization, Nav2, or the control path.

Save a native relocation map only after real keyframes have accumulated. The
call is synchronous: `success=True` means all required files were written and
validated before the response was sent.

```bash
source /opt/ros/humble/setup.bash
source HoloAgent/agentic_robot/core/install/setup.bash
ros2 service call /lio/save_map fast_livo/srv/SaveMap \
  "{resolution: 0.2, destination: '/absolute/path/to/new_map_directory'}"
python3 holoagent_bridge/prepare_reloc_map.py /absolute/path/to/new_map_directory
```

The validated 2026-08-07 long-run candidate is
`holoagent_bridge/maps/mid360_final_long_20260807_a/`. It contains 754
keyframes and 1,432,144 global points. Its independently repeated save is the
matching `_b` directory; their mapping, global-cloud, and generated-grid
hashes are identical. When relocalizing without returning the robot to the
map's first pose, provide `relo.initpose_prior` near the latest saved
`mapping.txt` pose instead of leaving the default `[0, 0, 0]` prior.

The canonical launch above starts online relocalization against the selected
map while FAST-LIVO publishes `lio/undistorted_points` and `lio/odom`:

```bash
/usr/bin/python3 holoagent_bridge/validate_relocalization.py --duration 30
```

The relocation configuration uses PCL NDT with a 1.0 m covariance-voxel
resolution. Do not reduce it to the 0.2 m pre-downsampling resolution: PCL
1.12 requires at least six points per covariance voxel. `online_relo` checks
the real target cloud before registration and rejects an empty covariance grid
instead of entering PCL's unsafe empty-KD-tree search path.

## Verification

Use these checks while the sim and bridges are running:

```bash
ros2 topic echo --once /sensors/lidar/points
ros2 topic echo --once /sensors/imu/data
ros2 topic echo --once /lio/odom
ros2 topic echo --once /localization/odom
ros2 topic hz /perception/obstacles
ros2 topic echo --once /localization/status
```

`perception/obstacles` is produced by the current online-relocalization adapter,
not by the raw MID360 bridge.
A passing result requires finite, strictly time-ordered poses, non-empty body
clouds, successful registrations below the configured fitness threshold, and
continued output over the validation interval.

## HoloAgent Topics

- HoloAgent/Nav2 command input: `/cmd_vel`
- IsaacLab wholebody DDS command output: `rt/run_command/cmd`
- IsaacLab MID360 point cloud: `sensors/lidar/points`
- IsaacLab torso IMU: `sensors/imu/data`
- IsaacLab RGB-D: `sensors/front/{rgb,depth,camera_info}`
- HoloAgent LIO output: `lio/{odom,undistorted_points,path}`
- HoloAgent localization output: `localization/{odom,status}`
- HoloAgent local costmap point cloud: `perception/obstacles`

## MID360 Authenticity

The simulator MID360 is an IsaacLab `MultiMeshRayCaster`, not a synthetic point generator. It reads `sensor.data.ray_hits_w[0]`, filters finite ray hits, transforms the hits from world coordinates into the MID360 sensor frame, and writes that sensor-frame cloud to shared memory with a sequence lock and nanosecond source timestamp.

The default raycast targets are the real task scene prims:

- `{ENV_REGEX_NS}/Room`
- `{ENV_REGEX_NS}/PackingTable_1`
- `{ENV_REGEX_NS}/PackingTable_2`
- `{ENV_REGEX_NS}/Object`

Real smoke check:

```bash
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
conda activate unitree_sim_env
env -u DISPLAY MID360_DEBUG=1 python holoagent_bridge/real_isaaclab_smoke.py --device cuda:0 --headless --livestream 0 --enable_cameras --steps 20 --print-stats
```

A real run should report `mid360_type=MultiMeshRayCaster`, non-empty `mid360_mesh_targets`, finite ray hits, and a non-zero z span. A previous real run produced `mid360_points=11520` and `z_span=2.072833`, which confirms the cloud was not only the floor plane.

Do not run HoloAgent `g1_move/pubvel` against the simulation. That executable targets the Unitree hardware SDK. Use `holoagent_bridge/cmd_vel_to_unitree_dds.py` for simulation.

Use `/usr/bin/python3` for ROS2 bridge processes. ROS Humble in this environment is built for Python 3.10; the default conda Python is not ABI-compatible with `rclpy`.

If `nav2_msgs` was previously built with conda Python, rebuild its Python type support for ROS Humble:

```bash
cd HoloAgent/agentic_robot/thirdparty
source /opt/ros/humble/setup.bash
colcon build --base-paths src/navigation2-humble --packages-select nav2_msgs \
  --build-base build --install-base install --symlink-install \
  --cmake-clean-cache --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Install the HTTP bridge dependencies into the same Python 3.10 environment:

```bash
/usr/bin/python3 -m pip install --user 'fastapi<1' 'uvicorn<1' 'anyio<4'
```
