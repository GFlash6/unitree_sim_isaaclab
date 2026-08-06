# HoloAgent IsaacLab Real-IMU LIO Validation Design

## Objective

Replace the current LiDAR-only FAST-LIVO validation path with a time-synchronized
LiDAR/IMU path using real IsaacLab state. Rebuild and relocalize against a map
created by actual locomotion-policy motion, then accept the chain only when its
pose agrees with simulator ground truth.

No mock samples, generated sensor values, pose injection, or ground-truth input
to FAST-LIVO are permitted.

## Selected architecture

### Sensor data path

IsaacLab already computes the torso IMU measurements used by the Unitree robot
state path:

- orientation from the simulated body pose;
- proper acceleration from simulated body velocity and gravity;
- angular velocity transformed into the body frame.

At the point where these values exist, write one fixed-layout IMU shared-memory
record containing:

- sequence number protected by the same odd/even seqlock convention as MID360;
- `env.sim.current_time` in nanoseconds;
- quaternion in an explicitly documented order;
- body-frame linear acceleration;
- body-frame angular velocity.

A ROS 2 bridge reads only fresh, complete records and publishes
`sensor_msgs/msg/Imu` on `/livox/imu`. MID360 and IMU retain the same simulator
clock domain for FAST-LIVO synchronization. Wall time remains limited to
external Nav2 outputs produced after relocalization.

### Motion path

Final validation must not move the robot by calling `write_root_pose_to_sim`.
The robot is moved through the existing path:

`/cmd_vel -> cmd_vel_to_unitree_dds.py -> rt/run_command/cmd -> Unitree
locomotion policy -> IsaacLab dynamics`.

The kinematic root-pose sequence remains a sensor/registration diagnostic only
and cannot be used as evidence that mapping or navigation control works.

### Ground-truth path

Record the IsaacLab root pose with simulator timestamps into a validation log.
This log is consumed only after a run to align and compare trajectories. It is
never published as an initial pose, odometry measurement, localization prior,
or control input.

The evaluator reports:

- matched sample count and timestamp coverage;
- translation RMSE and maximum translation error;
- yaw RMSE and maximum yaw error;
- final translation/yaw error;
- non-finite values, timestamp regressions, and discontinuous pose jumps.

## FAST-LIVO configuration

Enable the existing IMU path and point it at `/livox/imu`. Keep the simulated
LiDAR and IMU extrinsics explicit. Do not enable image processing or wheel
odometry as part of this change.

Before mapping starts, require an IMU initialization interval containing
stationary samples. Reject a run when either sensor stream is absent, stale,
non-monotonic, non-finite, or not in the same time domain.

The existing NDT covariance-voxel preflight, score threshold, and translation
prior gate remain secondary safety checks. They do not replace ground-truth
accuracy validation.

## Implementation boundaries

Use the existing point-cloud shared-memory implementation as the pattern, with
one small IMU-specific fixed record. Do not add a generic transport framework,
new dependency, DDS schema, or alternate localization implementation.

Expected components:

1. IMU shared-memory writer/reader and focused unit tests.
2. IsaacLab observation integration that writes the real computed IMU record.
3. ROS 2 IMU bridge with freshness and monotonicity checks.
4. FAST-LIVO simulator configuration enabling IMU.
5. Ground-truth recorder and offline trajectory evaluator.
6. A documented real-process launch and validation sequence.

## Failure handling

- A partial seqlock record is ignored.
- Repeated sequence numbers are not republished.
- Timestamp regression stops the validation run and is not silently rebased.
- Invalid quaternion norms or non-finite vectors are rejected.
- Missing LiDAR or IMU data prevents FAST-LIVO acceptance.
- A low NDT score with excessive prior or ground-truth error remains a failure.
- Nav2 and command forwarding remain disabled until localization acceptance.

## Verification sequence

1. Unit-test record layout, seqlock reads, quaternion order, finite validation,
   freshness, and timestamp regression.
2. Run a stationary real IsaacLab session and measure IMU frequency, gravity
   magnitude, gyro bias, and timestamp alignment with MID360.
3. Run FAST-LIVO with real stationary LiDAR/IMU and confirm initialization and
   bounded pose drift.
4. Drive a real locomotion-policy mapping route and record ground truth.
5. Save and validate a new native map; generate its relocation and Nav2 files.
6. Restart from a known but non-injected pose, run online relocalization, and
   compare the full output trajectory with ground truth.
7. Only after passing accuracy gates, launch Nav2 and verify map/costmap/TF,
   path generation, `/cmd_vel`, DDS command delivery, real robot displacement,
   and localization feedback consistency.

## Acceptance criteria

Sensor requirements:

- no non-finite IMU or MID360 samples;
- strictly increasing timestamps and sequence numbers;
- LiDAR/IMU timestamps in the same simulator time domain;
- no repeated publication of stale shared-memory records;
- stationary acceleration magnitude consistent with gravity and bounded gyro
  bias, with measured values recorded in the run report.

Localization requirements:

- translation RMSE at most 0.15 m;
- yaw RMSE at most 5 degrees;
- final translation error at most 0.20 m;
- no accepted registration outside the configured prior gate;
- no discontinuous pose jump greater than 0.30 m between adjacent matched
  validation samples unless ground truth contains the same displacement.

Operational requirements:

- mapper, relocalizer, bridges, and Nav2 processes terminate cleanly;
- Nav2 lifecycle activation succeeds without manual intervention;
- local/global costmaps consume current real point clouds without TF drops;
- a commanded route produces matching physical and localized motion.

Any failed criterion leaves the control connection disabled and produces an
explicit failure report rather than a success marker.
