# Real IMU LIO Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `executing-plans` inline. Do not use superpowers or subagents.

**Goal:** Feed timestamp-synchronized real IsaacLab IMU and MID360 data into FAST-LIVO, rebuild a map using real policy-driven motion, and reject localization unless it passes simulator-ground-truth accuracy gates.

**Architecture:** Add one fixed-layout seqlock shared-memory record for IMU data, parallel to the existing point-cloud record. IsaacLab writes real torso IMU values with simulator timestamps; a ROS 2 bridge publishes fresh records to `/livox/imu`. A separate offline evaluator compares FAST-LIVO output with timestamped simulator ground truth without feeding truth back into localization.

**Tech Stack:** Python 3.10/3.11, `ctypes`, `multiprocessing.shared_memory`, NumPy, ROS 2 Humble `rclpy`, IsaacLab, FAST-LIVO C++, pytest, gtest.

## Global Constraints

- No mock samples, generated sensor values, pose injection, or ground-truth input to FAST-LIVO.
- MID360 and IMU must retain the same `env.sim.current_time` clock domain.
- ROS 2 bridge processes use `/usr/bin/python3`; IsaacLab uses `unitree_sim_env`.
- Do not add dependencies or change the Unitree DDS schema.
- Do not enable Nav2 command forwarding until localization acceptance passes.
- Translation RMSE must be at most 0.15 m, yaw RMSE at most 5 degrees, and final translation error at most 0.20 m.

---

### Task 1: Fixed-layout IMU shared memory

**Files:**
- Create: `tools/imu_shared_memory_utils.py`
- Create: `tests/test_imu_shared_memory_utils.py`

**Interfaces:**
- Produces: `ImuWriter.write_sample(timestamp_ns, quaternion_wxyz, linear_acceleration, angular_velocity) -> bool`
- Produces: `ImuReader.read_sample() -> ImuSample | None`
- `ImuSample` contains `sequence`, `timestamp_ns`, `quaternion_wxyz`, `linear_acceleration`, and `angular_velocity`.

- [ ] Write tests proving a complete sample round-trips, repeated sequences return `None`, partial odd-sequence records return `None`, invalid vector sizes are rejected, and non-finite samples are rejected.
- [ ] Run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q tests/test_imu_shared_memory_utils.py` and verify the import fails because the module does not exist.
- [ ] Implement one `ctypes.LittleEndianStructure` header with magic, version, sequence, timestamp and ten `float32` values; use the existing MID360 odd/even seqlock pattern.
- [ ] Re-run the focused test and expect all cases to pass.
- [ ] Run `tests/test_pointcloud_shared_memory_utils.py` to ensure the parallel transport remains unchanged.

### Task 2: Publish real IsaacLab IMU records

**Files:**
- Modify: `tasks/common_observations/g1_29dof_state.py`
- Modify: `tasks/common_observations/h12_27dof_state.py`
- Modify: `tests/test_mid360_static.py`

**Interfaces:**
- Consumes: `ImuWriter.write_sample(...)` from Task 1.
- Produces: one IMU record whenever `get_robot_imu_data(env)` produces the real body measurement.

- [ ] Add a static test requiring `env.sim.current_time`, quaternion `wxyz`, body acceleration, and body gyro to be passed directly to `ImuWriter`; prohibit constants other than validation tolerances.
- [ ] Run the focused static test and verify it fails because no IMU writer exists in the observation path.
- [ ] Add a module-level `ImuWriter`; after calculating `imu_data`, write indices `[3:7]`, `[7:10]`, and `[10:13]` with `int(env.sim.current_time * 1e9)`.
- [ ] Keep DDS publication consuming the same `imu_data`; do not calculate IMU twice in one observation call.
- [ ] Run the focused test and existing MID360/static tests.

### Task 3: ROS 2 IMU bridge

**Files:**
- Create: `holoagent_bridge/imu_to_ros2_topic.py`
- Create: `tests/test_imu_ros_bridge.py`
- Modify: `holoagent_bridge/README.md`

**Interfaces:**
- Consumes: `ImuReader.read_sample()`.
- Produces: `/livox/imu` as `sensor_msgs/msg/Imu`, frame `imu_link`, using the source simulator timestamp.

- [ ] Write tests for quaternion order conversion (`wxyz` to ROS `xyzw`), header timestamp conversion, finite validation, and suppression of timestamp regressions.
- [ ] Run the focused test and verify failure because bridge functions are missing.
- [ ] Implement pure `sample_to_fields()` and `timestamp_msg()` helpers plus a timer-driven ROS node; publish only a fresh reader sample.
- [ ] On a timestamp regression, log an error and stop with a non-zero exit rather than rebasing time.
- [ ] Run focused and existing bridge tests.

### Task 4: Sensor-stream validator and FAST-LIVO configuration

**Files:**
- Create: `holoagent_bridge/validate_lidar_imu_sync.py`
- Create: `tests/test_validate_lidar_imu_sync.py`
- Modify: `holoagent_bridge/fast_livo_mid360_sim.yaml`

**Interfaces:**
- Consumes: `/mid360/points` and `/livox/imu` header timestamps and IMU vectors.
- Produces: a non-zero exit unless both streams are finite, monotonic, current, sufficiently populated, and overlap in simulator time.

- [ ] Write pure-statistics tests covering monotonic sequences, missing overlap, gravity-magnitude bounds, gyro bias, and stale streams.
- [ ] Run the focused test and verify failure because validator functions are absent.
- [ ] Implement a 10-second ROS validator reporting rates, time ranges, maximum nearest-timestamp offset, acceleration norm, and gyro norm.
- [ ] Set `common.imu_topic: /livox/imu`, `imu.imu_en: true`, image off, and wheel odometry off in the simulator FAST-LIVO overlay.
- [ ] Run focused tests and YAML static assertions.

### Task 5: Ground-truth recording and trajectory accuracy evaluation

**Files:**
- Create: `tools/ground_truth_shared_memory_utils.py`
- Modify: `sim_main.py`
- Modify: `holoagent_bridge/stream_mid360_mapping_sequence.py`
- Create: `holoagent_bridge/record_ground_truth.py`
- Create: `holoagent_bridge/evaluate_localization_accuracy.py`
- Create: `tests/test_localization_accuracy.py`

**Interfaces:**
- Produces: timestamped real IsaacLab root pose records and a JSON validation report.
- Evaluator consumes ground-truth rows and `relo_pose.txt`; it estimates only the initial rigid map/world alignment, then measures relative trajectory error.

- [ ] Write tests with exact trajectories proving interpolation, yaw wrapping, rigid initial alignment, RMSE, final error, and jump detection; include failing threshold cases.
- [ ] Run the focused test and verify failure because evaluator functions are absent.
- [ ] Reuse the Task 1 fixed-record/seqlock pattern for timestamp plus position/quaternion ground truth.
- [ ] Write ground truth from both normal `sim_main.py` and the diagnostic stream after the simulator state update.
- [ ] Implement a recorder that writes only fresh real records to text and an evaluator that exits non-zero on any acceptance failure.
- [ ] Run focused tests and all shared-memory tests.

### Task 6: Real stationary LiDAR/IMU integration

**Files:**
- Modify only if evidence requires it: `holoagent_bridge/fast_livo_mid360_sim.yaml`
- Save run evidence under: `holoagent_bridge/validation/` without committing large logs.

**Interfaces:**
- Consumes the real IsaacLab sensor and ROS bridges from Tasks 1-4.
- Produces measured sensor statistics and stationary FAST-LIVO drift.

- [ ] Start the real headless wholebody IsaacLab task with MID360 and DDS enabled.
- [ ] Start MID360 and IMU ROS bridges, then run the sensor synchronizer for at least 10 seconds.
- [ ] Reject the run if timestamps differ in domain, regress, or contain non-finite data.
- [ ] Start FAST-LIVO and verify IMU initialization from real stationary samples.
- [ ] Record at least 30 seconds and calculate translation/yaw drift; do not tune parameters unless logs isolate one violated model assumption.

### Task 7: Real policy-driven mapping and native map validation

**Files:**
- Create: `holoagent_bridge/run_real_mapping_validation.sh`
- Modify: `holoagent_bridge/README.md`
- Output: a new timestamped directory under `holoagent_bridge/maps/`.

**Interfaces:**
- Consumes: `/cmd_vel -> rt/run_command/cmd`, real sensor bridges, FAST-LIVO, ground-truth recorder.
- Produces: native keyframes/map files, ground-truth log, and mapping accuracy report.

- [ ] Start `sim_main.py` and move the robot only through the existing locomotion policy command path.
- [ ] Execute a route containing stationary initialization, straight motion, turns, and return motion while respecting command clamps and stale timeout.
- [ ] Save the native map synchronously and require service success plus file/cardinality validation.
- [ ] Run the trajectory evaluator against mapping odometry; reject the map if thresholds fail.
- [ ] Generate relocation files, the pose-transformed top-down image, and Nav2 occupancy map only for an accepted map.

### Task 8: Dynamic relocalization and Nav2/control acceptance

**Files:**
- Create: `holoagent_bridge/run_real_relocalization_validation.sh`
- Modify: `holoagent_bridge/README.md`
- Modify only when reproduced: Nav2 launch/config files involved in lifecycle failure.

**Interfaces:**
- Consumes: accepted map, real sensors, real ground truth, online_relo, Nav2, command bridge.
- Produces: relocation accuracy report and end-to-end navigation result.

- [ ] Restart the simulation and initialize online relocalization without ground-truth pose injection.
- [ ] Run a real policy-driven route and require translation RMSE <= 0.15 m, yaw RMSE <= 5 degrees, final translation error <= 0.20 m, and no unexplained jump > 0.30 m.
- [ ] Launch Nav2 repeatedly until lifecycle startup succeeds without manual activation; if it fails, reproduce and fix the lifecycle root cause with a focused test.
- [ ] Verify `map -> odom -> base_link`, current point-cloud transforms, both costmaps, and an actual planner result.
- [ ] Enable the command bridge only after all localization checks pass; send a bounded goal and verify DDS command, physical ground-truth displacement, and localized displacement agree.
- [ ] Stop every process cleanly and record exit codes.

### Task 9: Full regression and handoff

**Files:**
- Modify: `.gitignore` only for generated validation artifacts proven safe to ignore.
- Modify: `holoagent_bridge/README.md`

- [ ] Run all relevant Python tests with `/usr/bin/python3` and plugin autoload disabled.
- [ ] Run all FAST-LIVO C++ tests and a Release build/install.
- [ ] Check the worktree and stage only files belonging to this implementation.
- [ ] Document exact accepted map path, measured errors, launch commands, known limitations, and generated artifacts.
- [ ] Do not delete older maps or user files without explicit authorization.
