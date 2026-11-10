# LWA4P IP teleop handoff — 2026-11-10

LATEST: offline implementation continued after the diagnostic window with explicit user
authorization. Read the appended "Offline repair update" before resuming. The earlier
implementation details below describe the intermediate 0.5.1 version, not the latest code.

## Commit and publication update

The user authorized committing the work and pushing only their own repository.
The IP driver/teleop implementation is committed as `602a79b` on
`fix/ip_teleop_smoothing` in `edisimo/schunk_vla` (client 0.5.2, bridge 0.3.2,
hardware 0.1.2). The handoff is committed separately on the same branch.

The independent upstream-driver checkout at `/home/edisimo/Temp/lwa4p_driver` has
local commit `2f671b7` on `fix/wrist_camera_calibration`, containing the earlier
camera-calibration change and description-package version 0.1.1. Do not push that
repository until the user confirms access. Its earlier `can_setup.sh`, original
Claude transcript and raw captures remain local. The primary project's existing
submodule modifications remain represented by its committed patch files; the gitlink
still refers to the original published upstream commit.

Raw diagnostic captures, temporary patches, downloaded vendor tools and mock-run
artifacts under `docs/diagnostics/` are ignored locally; only the verification JSON
is published. The test procedure and permanent regression tests are self-contained.
The rebuilt installed bridge was compared byte-for-byte with its latest source,
then all 57 offline tests passed again before committing. No robot access occurred.

Entries below are chronological session history and may describe earlier uncommitted
states. Use the commits and current Git status as the source of truth for publication.

## Current user instructions

The user is disconnecting the robot. A five-minute read-only diagnostic window began at
17:18:18 CET and ends at 17:23:18 CET. Do not access or move the robot after this window
unless the user explicitly authorizes another connected test. Report findings first; the
user expects a subsequent 30–50 minute offline repair session before a possible retest.
No resets, enables, motion goals, PDO remapping or configuration writes were performed
during this diagnostic window. Teleop had already exited and drives were disabled.

## Repositories and changes

Actual implementation: `/home/edisimo/Temp/schunk_vla`, branch
`fix/ip_teleop_smoothing`, based on Claude's `1c09ce5` on `feat/vla_pipeline`.
There are 14 modified tracked files, uncommitted; nothing was pushed by Codex.
Run `git diff` there for the exact changes. The original workspace is
`/home/edisimo/Temp/lwa4p_driver`, still on main; its camera calibration, can_setup.sh and
claude-handoff.txt predated Codex. Leave those changes intact.

The installed smoothing change:

- IP launch sets bridge `interpolated_position=true`.
- Ruckig's timestamped preview now includes time zero, positions, velocities and
  accelerations. The IP bridge preserves these through JTC quintic interpolation.
- The four existing future samples are at 0.1, 0.2, 0.3 and 0.4 seconds.
- JTC `open_loop_control=true` anchors replacement trajectories to commanded state.
  Measured state still participates in trajectory error checks for this position interface.
- The bridge keeps its protocol reference at the first future target, not time zero.
- Legacy Python-driver transport remains position-only. Safety clamps can still discard
  the smooth preview and take that legacy path, even in IP mode; investigate this.
- The client now raises RuntimeError when the bridge reports rejected arm goals. This
  makes a drive trip exit teleop instead of continuing indefinitely into rejected goals.
  It currently prints a traceback; replace that with clear, orderly fault handling.
- Versions: root project/uv.lock 0.5.1; hardware CMake/package 0.1.1;
  bridge package/setup 0.3.1 (setup had previously lagged at 0.1.0).
- README documents the IP path and voltage-fault limitation.

Validation before the second hardware run: 17 Python tests passed; 8 C++ gtests passed;
10 additional ROS-message/bridge tests passed from the diagnostic test artifact;
Black, isort and git diff --check passed. Both ROS packages rebuilt successfully.
The CMake warning is a pre-existing gtest minimum-version deprecation.

## Hardware results and limitations

First IP enable and stationary hold succeeded. All six statuswords were 0x9237 during
10 seconds of holding; largest position span was 0.001 degree and targets stayed fixed.

First teleop run used the old position-only transport and tripped with joint 5 (CAN node 7)
first reporting EMCY 0x3222. The user confirmed they did NOT press E-stop/cut power.

Second run used the smoothing change. The user reports better precision and less jerking,
but some poses still jerked and motion stopped again. At 17:15:58 CET the driver again
logged joint 5/node 7 EMCY 0x3222, then disabled all joints and deactivated arm_controller.
The teleop RuntimeError happened AFTER that trip, from the newly added rejection handling.
Do not describe this traceback as evidence that Ruckig or the control process crashed first.

The second teleop ran 2094 ticks / about 105.8 seconds, effective cadence 19.80 Hz.
52 ticks (2.5%) exceeded 50 ms. Mean command-send work was 15.63 ms; worst was 102.28 ms.
The whole loop reached 109.7 ms. Observe averaged 2.06 ms, teleop/IK 3.64 ms.
These delays are evidence of client/bridge latency, not proof of the voltage fault's cause.

At capture time, can0 was UP, 500 kbit/s, ERROR-ACTIVE (normal active error state), with
zero TX/RX bus errors, drops or bus-off events. A later 12-second passive capture contained
1499 SYNC intervals: median 8.0011 ms, p99 8.1520 ms, maximum 8.2359 ms. This capture was
AFTER the trip with motors disabled; it cannot establish timing during the jerky motion.

## Drive diagnostic snapshot

All nodes 3–8 identify as SCHUNK ERBo, hardware 6.11, firmware
`63 Build:2462 Date:2013-07-04 10:05:04`.

- 0x6041:0 = 0x9228: faulted/disabled.
- 0x1002:0 = 0x8874; Schunk SMP error 0x74 = motor voltage low.
- 0x1003 contains two entries 0x00743222 on every node. These are fault history entries;
  this alone does not establish that all six independently faulted simultaneously.
- Standard voltage 0x6079 and current 0x6078 return SDO abort 0x06020000 on all nodes.
  There is NO measured voltage in volts available from these reads. Do not infer 0 V,
  74 V, or supply sag magnitude from these codes.
- Vendor 0x200B:1 is a floating-point error-detail scalar: nodes 3–8 report
  350, 352, 358, 354, 396, 396. Units and scaling are UNKNOWN. Schunk's diagnostic utility
  labels this detailed error information. 0x200B:2 = main.c, 0x200B:3 = line 986.
  These scalars may be valuable for vendor interpretation; do NOT label them volts.
- Vendor 0x200D:1 is actual PCB temperature, confirmed by the bundled Schunk utility's
  embedded documentation (not by guessing). Joint temperatures were
  37.6875, 37.5625, 39.4375, 39.5, 50.3125, 48.375 Celsius.
- 0x200D:2 = 20000, units not established. Vendor 0x200C/0 and 0x200E/0 are scalar
  values; attempts to treat them as arrays returned subindex-not-found.
- Extended error history 0x200F entries were not readable at the normal user level.
  The vendor utility says root credentials are needed for extended history. No user-level
  changes or passwords were attempted.
- Profile acceleration 0x6083 = 40000 mdeg/s². This applies to profile-position startup;
  do not assume it limits the externally generated IP trajectory acceleration.
- 0x6065 following-error window = 9999 mdeg. No following-error fault was logged.
- Node 7/8 rated current 0x6075 = 1409 raw; nodes 3–6 = 7000 raw. Captured torque/current
  related values were read after disable and cannot reconstruct load during the fault.

Last measured joint pose (rad):
`[0.3642851215, 0.0019024089, 1.5871849685, -0.0368264472, 1.4748206712, -2.4821898089]`.
Gripper gap was about 0.0196973 m. Exact URDF and running ROS parameters are saved.

## Offline priorities — hypotheses, not confirmed diagnoses

1. Reproduce pose-dependent behavior at the captured pose using the exact URDF. Check
   joint 6 limits, IK conditioning and transitions into refusal/clamping paths.
2. Preserve p/v/a continuity through EVERY IP path. Collision refusal, rate clamping,
   holds and client smoother resets currently fall back to measured position with zero
   velocity; these can reintroduce abrupt changes during motion.
3. Reduce synchronous bridge latency. Each preview currently checks time-zero and all
   four future samples via blocking MoveIt service calls, in addition to the first-target
   check. Measure before removing any collision protection; cache/avoid duplicate checks
   or move validation off the streaming critical path with bounded freshness.
4. Replay controller output under realistic network jitter, delayed messages, dropout,
   goal replacement and pose limits. The existing standalone replay uses the installed
   real JTC interpolation library and a synthetic 16 ms feedback lag. It reduced peak
   commanded acceleration from 53.5238 to 0.496944 rad/s² and RMS from 20.8139 to 0.0825922.
   This is a single synthetic case, NOT a physical measurement or a safety bound.
5. Investigate 100 ms preview spacing: quintic reconstruction can exceed the planner's
   nominal 0.4 rad/s² limit (the replay peak was 0.497). Consider denser/adaptive samples
   at Ruckig jerk boundaries, without multiplying synchronous collision-check latency.
6. IP hardware write currently limits position rate only (30 deg/s), with no acceleration
   or jerk envelope. Evaluate a continuous bounded trajectory/stop design rather than
   clipping individual samples or silently changing the path.
7. Replace rejection traceback with an explicit stopped/faulted teleop state and useful
   driver fault reason. Keep motors disabled after faults; no automatic re-enable loops.
8. For the next connected test, record raw CAN RPDO/TPDO/EMCY and timestamped planner/JTC
   desired/measured states before moving. Neither failed run had a motion CAN trace, so
   the remaining physical jerk cannot be quantified retrospectively.
9. Voltage remains unresolved. Software smoothing may reduce current transients but does
   not prove or repair a power-supply/cable problem. Decode vendor error-detail scaling
   from authoritative documentation, or measure motor supply at the arm under load on a
   future authorized test. Do not disable undervoltage protection or raise fault thresholds.

## Durable artifacts and commands

Everything captured in the window is under
`/home/edisimo/Temp/lwa4p_driver/diagnostics/2026-11-10-ip-jerk/`:
robot-stack.log, teleop.log, drive-registers.json, vendor-arrays.json,
ros-parameters.json, last-state.json, robot.urdf, passive-can.log and its summary,
first-hold.json, project.patch, smoothing.patch, test_ip_bridge.py, replay.cpp,
CMakeLists.txt, plans.txt and vendor diagnostic binaries/archive (inspect only).
No vendor executable has been run.

The vendor software was obtained read-only from
https://github.com/JustZeus/Lwa4p_guide/tree/main/LWA_Software/CANopenToolsExt .
Schunk manual for SMP error 0x74:
https://cmp.felk.cvut.cz/cmp/hardware/chapadloSchunk/MotionControl_Eng.pdf .
ROS replacement semantics were checked against the installed headers and official
https://github.com/ros-controls/ros2_controllers/tree/humble/joint_trajectory_controller .

Use `. scripts/env.sh` in the actual project for ROS (system Python 3.10).
Use `scripts/vla.sh` for LeRobot/Python 3.12, stripping the ROS PYTHONPATH.
Offline pytest may need `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` to avoid unrelated ROS plugins.
The fake-bridge tests need localhost sockets, which the filesystem sandbox blocks;
request normal sandbox escalation for those tests, explicitly identifying the fake server.
Build: `colcon build --symlink-install --base-paths ros --packages-select lwa4p_hardware
lwa4p_vla_bridge`. C++ tests: `colcon test --packages-select lwa4p_hardware`.
Formatting: Black/isort, 100 columns; existing MIT project style. Read the user's Airvolute
Python/C++ rules before editing. Stay on a feature/fix branch and never push without permission.

The last live stack was session 20753, launch PID 423086, log /tmp/codex-ip-stack.log.
Teleop session 29752 exited with RuntimeError; there is no remaining teleop process.
The trajectory controller was confirmed inactive and joint_state_broadcaster active.
Do not infer that the robot remains connected from these historical process IDs.

## Offline repair update — current implementation

The user explicitly authorized offline fixes/tests after disconnecting, then said Continue.
All subsequent verification used local fake sockets, analytic trajectories or an isolated
localhost-only ROS domain with memory-only `mock_components/GenericSystem`. There has been
no further real robot access, CAN access, enable, reset or launch since the diagnostic window.
The old launch may still exist; it was not queried or restarted. A rebuild does not update
an already running process. Stop the previous launch normally before a future connected test.

Actual project remains `/home/edisimo/Temp/schunk_vla`, branch `fix/ip_teleop_smoothing`.
Changes are uncommitted and unpushed. Versions are now root/uv.lock 0.5.2, hardware 0.1.2,
and bridge 0.3.2. Existing earlier changes and raw diagnostic captures remain intact.

### Implemented behavior

- `JointSmoother.preview()` exports every Ruckig jerk boundary plus 20 ms sample spacing,
  continuing to the true terminal position with zero velocity and acceleration. The old
  0.4-second truncation could stop while the planner still had nonzero velocity.
- The IP bridge verifies constant-jerk consistency, finite states, velocity/acceleration/jerk
  bounds, terminal rest, timestamps, actual drive bounds and position extrema between points.
  It also verifies continuity with its previously accepted command timeline.
- Full stop trajectories can take up to 2 seconds; this is an intentional behavior change.
  The prior maximum position lead remains 0.225 rad/joint (0.45 rad/s times 0.5 s).
  Client raw targets are bounded to 0.2 rad from planned position. Refused plans keep the
  previous complete validated stop, and the client restores its previous Ruckig timeline.
  There is no position-only IP fallback or reset to measured position at zero velocity.
- Discrete collision checks remain at 100 ms spacing, independent of dense exported samples.
  Unique states are submitted concurrently with one 35 ms deadline; unreturned requests are
  removed/cancelled. No cache is reused across requests/scenes. Missing/late results refuse
  the new command. This is NOT continuous collision checking or certified safety control.
- C++ publishes latched `/lwa4p_driver/limits` (Float64MultiArray, lower/upper pairs after
  sign/offset/margin) and `/lwa4p_driver/fault` (String). Explicit successful enable clears
  the fault. The bridge requires actual limits before accepting IP commands.
- IK now uses those same actual bounds with a small additional 0.005 rad buffer, rather
  than applying a second large margin. Small moves away from a nearby limit remain allowed.
  The captured final pose had manipulability 0.03233, above the 0.002 refusal threshold;
  that snapshot does not establish singularity as the cause of the failed motion.
- New client defaults: acceleration 0.3 rad/s², jerk 2 rad/s³. Dedicated gamepad CLI also
  defaults to 0.3 rad/s joint speed and 0.03 m/s TCP speed. Voltage protection is unchanged.
- JTC now rejects nonzero terminal velocity. Stale action callbacks cannot clear new fault
  information. Drive fault text reports undervoltage without asserting a measured sag magnitude.
- Legacy Python transport retains its original four short position-only future points and
  does not generate the full IP preview. Its existing clamping/reset behavior remains separate.

### Operator tools

Use the new documented entry point for clean fault reporting:
`scripts/vla.sh lwa4p-vla teleop`. The generic upstream `lerobot-teleoperate` still uses its
upstream exception display, so an ArmFault there can still print a traceback.

`scripts/vla.sh lwa4p-vla capture --output <new-file>.jsonl --duration 180` records passive
CAN with kernel receive timestamps and receive-loss counters, bridge/JTC demand/feedback,
and a bounded recent command trace on faults. It sends only state/trace TCP requests and
never transmits CAN frames. File creation is exclusive; it does not overwrite captures.
`scripts/vla.sh lwa4p-vla analyze <file>.jsonl` summarizes timing/faults offline and explicitly
reports voltage as unknown. Shutdown controlwords are excluded from command-step statistics.

The full manual retest procedure, conservative settings and build/test commands are in
`/home/edisimo/Temp/schunk_vla/docs/ip-motion-test.md`. It explains the longer smooth-stop
time and the need to restart the rebuilt stack before reconnecting/testing.

### Verification and remaining work

- Client/capture/IK pytest suite: 24 tests pass, including fake sockets, refusal continuity, limit
  escape, clean CLI fault exit and legacy transport preservation.
- ROS bridge suite: 22 pytest cases with actual message types and fake service clients;
  no ROS graph needed. Coverage includes collision timeout cleanup, duplicates, refusal,
  discontinuities, terminal rest, between-point limits, and stale callback handling.
- C++: 8 existing CANopen tests plus 3 new tests using the installed JTC interpolator at
  125 Hz. Delayed replacement (including 80/90 ms delays) and missing-update stops pass.
- 180 real six-joint Ruckig exports passed the bridge's full dynamics validator offline;
  largest export had 98 points and longest duration was about 1.424 seconds.
- A separate end-to-end smoke run used ONLY GenericSystem in ROS domain 169,
  `ROS_LOCALHOST_ONLY=1`, Fast DDS and an ephemeral TCP test port. All 50 streamed goals
  were accepted and controller demand/feedback were received. Test processes were stopped.
  Cyclone DDS loopback discovery initially failed; Fast DDS matched the project environment.
- Both ROS packages rebuilt successfully; formatting and whitespace checks passed. CMake
  emits third-party gtest minimum-version and FindBoost policy warnings, with no build failure.
- For colcon bridge testing, use `--python-testing pytest` explicitly. This machine's newer
  setuptools ignores the removed `tests_require` metadata; the default colcon Python runner
  otherwise silently runs zero bridge tests. No unsupported metadata was retained in setup.py.
- No robot retest has occurred. Electrical undervoltage remains unresolved and still requires
  an authorized connected test and possibly a calibrated supply measurement under load.
- The hardware write layer still has its original position-rate limiter, not an independent
  jerk limiter. The new dynamics validation covers the bridge IP path; direct MoveIt/other
  controller clients do not gain a separate hardware acceleration limiter from this change.
- Collision geometry and PSU/cabling were not modified; no undocumented diagnostic scaling
  was assumed, and no drive fault thresholds were raised.

The only added direct runtime package is ROS `std_msgs`, already installed from the official
ROS apt repository; JTC was already a runtime dependency and is now also used by offline tests.
No third-party package downloads were required. New source/test headers follow the provided
ADPL templates; existing project license files were not converted.

Final offline verification: 24 client tests + 22 ROS bridge tests + 11 C++ tests = 57 passed.
Both packages built successfully. Black, isort, clang-format and git diff --check pass.
The collision deadline includes request-submission time as well as reply waiting.
The positive batch-order test uses a generous fake deadline to avoid conflating lazy ROS
type-loading cost with concurrency; separate tests cover timeout and submission budgets.
