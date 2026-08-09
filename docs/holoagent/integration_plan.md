# HoloAgent–IsaacLab Integration Implementation Plan

**Goal:** Deliver a verified real-data mapping, relocalization, and navigation-control chain from IsaacLab MID360 raycasts.

**Architecture:** Repair each existing boundary in dependency order. Unit tests prove local invariants; real IsaacLab runs then prove coordinates, timing, numerical quality, and stability across component boundaries.

**Tech stack:** Python 3.11/3.10, NumPy, ROS2 Humble, IsaacLab/Isaac Sim, C++17, PCL, GTSAM, FAST-LIVO, Unitree DDS.

## Global constraints

- Do not introduce mock, fake, placeholder, or replayed-stale sensor results.
- Do not treat topic presence or message receipt as correctness.
- Preserve unrelated parent-repository and nested-HoloAgent changes.
- Generate new diagnostic maps in new directories; do not overwrite the reference map.
- Execute every behavioral change test-first and observe the expected failure before implementation.

## Task 1: Make prior-map keyframes load real points

**Files:**

- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/include/multi-session/Incremental_mapping.cpp`
- Create: `HoloAgent/agentic_robot/core/src/fast_livo/test/test_reloc_map_loading.cpp`
- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/CMakeLists.txt`

**Deliverable:** Loading `mid360_sim_20260806_102736` produces 64 non-empty keyframes and rejects missing, empty, duplicate, or non-contiguous PCD/SCD indices.

- [ ] Add a C++ regression test that loads a real temporary PCD through `Session` and asserts the resulting keyframe contains the same finite XYZ points.
- [ ] Run the focused CTest and observe failure because the target cloud is empty.
- [ ] Copy from the actual PCL source cloud, validate `loadPCDFile` status, and reject invalid file/index/count combinations.
- [ ] Rebuild `fast_livo`, run CTest, then launch `online_relo` against the reference map and verify non-zero per-keyframe point counts in diagnostics.

## Task 2: Replace false asynchronous map saving with truthful saving

**Files:**

- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/src/LIVMapper.cpp`
- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/include/LIVMapper.h`
- Create: `HoloAgent/agentic_robot/core/src/fast_livo/test/test_map_artifacts.cpp`

**Deliverable:** `fast_livo/save_map` returns only after a consistent map is complete, returns false on failure, and leaves no child processes.

- [ ] Add an artifact validator test covering required files, equal pose/PCD/SCD counts, continuous indices, non-empty clouds, and readable global PCD.
- [ ] Reproduce the current save with debug symbols and capture the failing/deadlocking location before changing the service.
- [ ] Remove `fork()`, serialize the save at a safe mapping boundary, write pose files with truncation rather than append, and return the validator result.
- [ ] Run two consecutive real saves into separate directories and verify service results, process trees, file hashes/counts, and clean shutdown.

## Task 3: Make shared-memory transfer atomic and freshness-aware

**Files:**

- Modify: `tools/pointcloud_shared_memory_utils.py`
- Modify: `test/mid360_to_ros2_topic.py`
- Modify: `tasks/common_observations/mid360_state.py`
- Modify: `test/test_mid360_ros_bridge.py`

**Deliverable:** Readers receive only complete new sensor frames; the ROS bridge never repeats a stale frame.

- [ ] Add tests for odd/in-progress sequence rejection, malformed sizes, monotonically increasing sequence, same-millisecond frames, and no output when no new frame exists.
- [ ] Run the tests and observe failures under the current timestamp-only/header-first implementation.
- [ ] Implement a seqlock-style header with sequence and nanosecond timestamp, payload-before-final-header publication, and bounded header validation.
- [ ] Remove `last_points` replay from the ROS bridge and fix the debug variable typo.
- [ ] Stress one real writer and reader for at least 10,000 frames and verify zero torn frames, strictly increasing sequences, finite XYZ, and clean process exit.

## Task 4: Use an honest XYZ FAST-LIVO preprocessing path

**Files:**

- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/include/common_lib.h`
- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/include/preprocess.h`
- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/src/preprocess.cpp`
- Modify: `holoagent_bridge/fast_livo_mid360_sim.yaml`
- Create: `HoloAgent/agentic_robot/core/src/fast_livo/test/test_xyz_preprocess.cpp`

**Deliverable:** XYZ PointCloud2 enters LIO without RGB warnings or invented measurement fields.

- [ ] Add a real PointCloud2 conversion test containing only XYZ and assert finite output geometry and zero instantaneous-scan curvature.
- [ ] Observe the existing L515 path warning/failure expectation.
- [ ] Add the smallest dedicated XYZ lidar type using `pcl::PointXYZ`; keep absent intensity/timing internal and explicitly non-measured.
- [ ] Build and run a real MID360 mapping sequence; require no missing-field warnings and compare input/output point bounds and counts.

## Task 5: Generate correct global maps and validate preparation

**Files:**

- Modify: `holoagent_bridge/prepare_reloc_map.py`
- Create: `tests/test_prepare_reloc_map.py`

**Deliverable:** Fallback global-map generation applies each keyframe pose and refuses inconsistent maps.

- [ ] Add a two-keyframe numerical test whose translated output bounds cannot pass under raw concatenation.
- [ ] Observe that the current output equals local-frame concatenation.
- [ ] Parse mapping quaternions, transform XYZ into map frame, preserve only genuine fields, and validate all keyframe indices/counts before writing.
- [ ] Regenerate a new copy of the short map and numerically compare its output with an independent NumPy transformation.

## Task 6: Prove online relocalization pose and cloud correctness

**Files:**

- Modify: `HoloAgent/agentic_robot/core/src/fast_livo/include/online-relo/pose_estimator.cpp`
- Modify: `holoagent_bridge/fast_livo_mid360_reloc_sim.yaml`
- Create: `holoagent_bridge/validate_relocalization.py`
- Modify: `holoagent_bridge/README.md`

**Deliverable:** Real `/undistort_cloud` and odometry produce synchronized, finite `/pose`, TF, and body-frame `/reloc_body_cloud`; no empty `relo_pose.txt` is used as an oracle.

- [ ] Add validator tests for timestamp skew, non-finite poses, discontinuities, stale output, frame IDs, and ground-truth relative trajectory error.
- [ ] Run against the pre-fix chain and observe rejection.
- [ ] Add explicit registration-status/fitness diagnostics and correct shutdown ownership; publish outputs only after valid initialization.
- [ ] Run a real mapped trajectory and require continuous output over the full run, bounded timestamp skew, stable stationary pose, and bounded aligned trajectory error.

## Task 7: Close and soak-test the Nav2 control loop

**Files:**

- Modify: `holoagent_bridge/cmd_vel_to_unitree_dds.py` only if dynamic evidence exposes a boundary defect.
- Create: `holoagent_bridge/validate_navigation_loop.py`
- Modify: `holoagent_bridge/README.md`

**Deliverable:** A real Nav2 goal moves the simulated robot using the real whole-body policy while localization and obstacle data remain correct.

- [ ] Add checks for command bounds, command freshness, pose freshness, goal-direction progress, stop timeout, and TF availability.
- [ ] Start IsaacLab, MID360 bridge, FAST-LIVO, online relocation, Nav2, and DDS bridge with no duplicate publishers.
- [ ] Submit a real navigation goal and compare IsaacLab root ground truth with localization and commanded direction.
- [ ] Hold the complete chain for at least 30 minutes; require no process growth, child leaks, stale publication, non-finite data, or unbounded localization drift.

## Final verification

- [ ] Run all focused Python tests with plugin autoload disabled.
- [ ] Run FAST-LIVO CTest and a clean `colcon build --packages-select fast_livo`.
- [ ] Run Python compilation checks.
- [ ] Run fresh real mapping, save, load, relocalization, navigation, and soak validations.
- [ ] Inspect parent and nested-repository diffs and report every changed file and any remaining limitation without deleting unrelated work.
