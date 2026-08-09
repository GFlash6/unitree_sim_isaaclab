# HoloAgent–IsaacLab Integration Design

## Objective

Provide a production-credible path from the IsaacLab MID360 ray caster through FAST-LIVO and online relocalization to Nav2 and the Unitree whole-body controller. No synthetic sensor inputs, placeholder poses, repeated stale frames, or success responses that are not backed by completed work are allowed.

## Canonical data flow

1. IsaacLab produces one instantaneous sensor-frame XYZ raycast cloud and a simulator timestamp.
2. Shared memory transfers only complete, monotonically sequenced frames.
3. The ROS bridge publishes only `/mid360/points` in `mid360_link`; it stops publishing when the simulator stops.
4. FAST-LIVO consumes XYZ without pretending RGB, reflectivity, or per-point timing exists, then publishes `/undistort_cloud` and `/aft_mapped_to_init` with matching timestamps.
5. `online_relo` consumes those two outputs, validates the prior map, performs registration, and publishes `/pose`, `map -> base_link`, and `/reloc_body_cloud` in `base_link`.
6. Nav2 generates `/cmd_vel`; the existing DDS bridge forwards bounded, freshness-checked commands to the real whole-body policy in IsaacLab.

## Correctness gates

- A shared-memory frame is accepted only when its header is stable before and after payload copying, dimensions fit the segment, and its sequence increases.
- A relocation map is accepted only when graph poses, PCD files, and SCD files have identical continuous indices and every keyframe cloud is non-empty.
- Map export transforms each keyframe cloud by its map-frame pose before aggregation.
- Saving reports success only after all required files are written, readable, non-empty, and mutually consistent.
- Relocalization acceptance requires finite continuous poses, synchronized cloud/odometry timestamps, successful registration, acceptable fitness, and bounded error against IsaacLab ground truth.
- Control acceptance requires a real navigation goal to produce bounded commands, measurable simulator motion in the correct direction, fresh pose feedback, and a zero command after timeout.

## Failure handling

- Reject malformed or torn shared-memory frames rather than reshaping unchecked bytes.
- Stop publishing when no new sensor sequence arrives within the configured freshness interval.
- Fail map loading with an explicit error instead of logging a misleading loaded count.
- Keep a failed save directory distinguishable from a validated map and return `success=false`.
- Do not publish a relocalized pose until initialization/registration has actually succeeded.

## Scope

The existing MID360 ray caster, whole-body policy, ROS2, DDS, PCL, GTSAM, and FAST-LIVO implementations remain in place. Changes are limited to their faulty boundaries and validation paths; no replacement SLAM stack or synthetic test publisher is introduced.
