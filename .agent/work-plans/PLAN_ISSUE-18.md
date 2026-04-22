# Plan: odom.twist.linear published in world frame, not body frame (REP-105)

## Issue

https://github.com/rolker/mru_transform/issues/18

## Context

`MRUTransform::updateVelocity` copies the incoming world-frame velocity
verbatim into `nav_msgs/Odometry.twist.twist.linear`, which per REP-105
must be expressed in `child_frame_id` (body frame). Same latent pattern
applies to `updateOrientation`'s angular-velocity copy and to the
covariance matrix.

Field evidence (BizzyBoat 2026-04-21, ~50k samples): correlation between
`/bizzy/odom.twist.linear` and `/bizzy/mavros/.../gps_vel.twist.linear`
is 1.0000 with median diff 0.0 mm/s — literally the same ENU signal.

Downstream consumers `asv_helm_node.cpp` and `ben_gazebo/gazebo_helm_node.cpp`
feed `odom.twist.linear.x` into a speed PID as body-forward feedback; when
the vessel heading ≠ due east, the PID feedback is silently rotated and the
controller misbehaves. BizzyBoat is not affected in the field because
`echo_helm` doesn't close a loop on `odom.twist`, but the sim graph is
affected and any future consumer trusting REP-105 will be too.

Design agreed on the issue (comment 4292978963):
- Rotate linear, angular, and the full 6×6 `twist.covariance` block-diagonal.
- Drive rotations by tf2 lookups keyed on each message's `header.frame_id`.
- TF lookup failure → drop message + `RCLCPP_WARN_THROTTLE` at 5 s.
- Emit `RCLCPP_WARN_ONCE` in `OrientationSensor`'s `QuaternionStamped` and
  `GeoPoseStamped` callbacks noting `angular_velocity` is unavailable there.

## Approach

1. **Add tf2 buffer/listener to `MRUTransform`** — new member fields in
   `mru_transform.hpp`, initialized in the constructor alongside the
   existing `TransformBroadcaster`. Construct the buffer with
   `node_->get_clock()` so it honors sim time (important because the
   simulation pipeline consumers `asv_helm` + `ben_gazebo` run under
   simulated time and are primary beneficiaries of this fix).
2. **Change `VelocitySensor::ValueType` to `TwistWithCovarianceStamped`**
   (`velocity_sensor.hpp`, `velocity_sensor.cpp`). Today the sensor
   discards covariance when unwrapping `TwistWithCovarianceStamped` →
   `TwistStamped`. Preserving covariance end-to-end is required for
   the rotation work below to have any effect on `TwistWithCovariance`
   inputs. The plain `TwistStamped` subscription path fills a zero
   covariance block.
3. **Refactor `updateVelocity`** (`mru_transform.cpp:135`):
   - Resolve TF `base_frame_ ← velocity.header.frame_id` at `velocity.header.stamp`.
   - On lookup failure, `RCLCPP_WARN_THROTTLE` (5 s) and return without publishing.
   - Rotate `velocity.twist.twist.linear` into body via the lookup's rotation component.
   - Rotate the 3×3 linear-covariance sub-block of `velocity.twist.covariance`
     as `R Σ Rᵀ` and write into `odom_.twist.covariance[0..2,0..2]`.
4. **Refactor `updateOrientation`** (`mru_transform.cpp:approx-130`, IMU path):
   - Same TF lookup keyed on `imu.header.frame_id`.
   - Rotate `angular_velocity` and the 3×3 angular-covariance block (indices 3..5).
   - No change to `odom_.pose.pose.orientation` — still the raw world-frame quaternion.
5. **Add warnings to `OrientationSensor` non-IMU callbacks**
   (`orientation_sensor.cpp:53,60`): `RCLCPP_WARN_ONCE` saying
   "angular_velocity not available from QuaternionStamped / GeoPoseStamped".
6. **Add gtest-based unit tests** under `mru_transform/test/`:
   - Synthetic ENU velocity + known orientation → assert rotated linear matches hand-computed.
   - Synthetic gyro in a rotated sensor frame → assert rotated angular matches.
   - Both with non-diagonal covariance → assert `R Σ Rᵀ` block-diagonal output.
   - TF-missing case → assert no publish + warn log observed.
7. **Update CMakeLists / package.xml** to enable `gtest` + declare
   `<test_depend>ament_cmake_gtest</test_depend>`.
8. **Manual bag regression** (not automated, documented in PR description):
   replay a bizzyboat bag, confirm magnitude preserved and
   `rotate(odom.twist.linear, +yaw_enu) ≈ gps_vel.twist.linear` with no
   heading-dependent residual.

## Files to Change

| File | Change |
|---|---|
| `mru_transform/include/mru_transform/mru_transform.hpp` | Add `tf2_ros::Buffer` + `TransformListener` members |
| `mru_transform/include/mru_transform/velocity_sensor.hpp` | Change `VelocitySensor` base type to `TwistWithCovarianceStamped` |
| `mru_transform/src/velocity_sensor.cpp` | Zero-fill covariance on `TwistStamped` input path; forward covariance on `TwistWithCovarianceStamped` path |
| `mru_transform/src/mru_transform.cpp` | Refactor `updateVelocity` + `updateOrientation`; initialize tf2 buffer/listener with `node_->get_clock()` |
| `mru_transform/src/orientation_sensor.cpp` | `RCLCPP_WARN_ONCE` in quaternion + geopose callbacks |
| `mru_transform/CMakeLists.txt` | Enable gtest in `BUILD_TESTING` block, link test target |
| `mru_transform/package.xml` | Add `ament_cmake_gtest` test dependency |
| `mru_transform/test/test_twist_rotation.cpp` *(new)* | Unit tests for all four rotation scenarios |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Correctness over convenience | Fix the frame bug properly (rotate everything) rather than papering over with a magnitude-only workaround |
| Fail loudly, not silently | TF failure logs via `WARN_THROTTLE`; non-IMU orientation sources now surface a `WARN_ONCE` instead of publishing zero/stale `angular_velocity` |
| Tests accompany fixes | New gtest coverage ships with the PR, not as a follow-up |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 — Follow ROS 2 Official Conventions | Yes (modifies `package.xml`, `CMakeLists.txt`, adds test files) | Complies: standard `ament_cmake_gtest` for tests, `<test_depend>` placement, REP-105 is precisely the convention being aligned to |
| ADR-0002 — Worktree isolation | Yes (any feature work) | Satisfied: work happens in `layers/worktrees/issue-mru_transform-18` |

No other workspace-level ADRs apply. No project-level governance file
exists in `rolker/mru_transform` (no `AGENTS.md` or `.agent/` at repo root
beyond the new work-plans directory).

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `odom.twist.linear` frame | Downstream speed-PID consumers (`asv_helm`, `ben_gazebo`) | No — those consumers become correct automatically once mru_transform is fixed; no code change needed in them. Note in PR description for verification. |
| `odom.twist.covariance` block | EKF/robot_localization consumers reading it | No downstream consumers identified in workspace grep; ships as-is |
| Added `tf2_ros::TransformListener` | Node QoS / thread model | Standard add, matches existing broadcaster pattern |
| TF lookup keyed on `velocity.header.frame_id` | **`rolker/posmv` users** — posmv publishes with `header.frame_id = posmv_frame` (user-configured). After this fix, a static TF from `posmv_frame` → world frame must exist in the TF tree, or velocity messages drop with `WARN_THROTTLE`. | Documented in PR description as deployment note — not a regression (this is correct REP-105 behavior), but an operational change for existing posmv users. |
| `VelocitySensor::ValueType` change from `TwistStamped` → `TwistWithCovarianceStamped` | Internal to mru_transform — changes signature of `MRUTransform::updateVelocity` | Included in plan step 2 |

## Open Questions

None — design settled in issue discussion. Proceed to implementation on
explicit go-ahead.

## Estimated Scope

Single PR. Mid-hundreds of lines (230-300) including the gtest suite with
hand-computed covariance rotations.
