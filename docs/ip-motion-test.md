# IP motion test procedure

These changes were developed and checked offline after the robot was disconnected. They address
command discontinuities and provide diagnostics; they do not establish or repair the electrical
cause of the repeated undervoltage trips. No undervoltage thresholds or drive protections changed.

## What changed

- Ruckig exports every jerk-phase boundary and the complete trajectory to zero velocity and
  acceleration. Quintic interpolation reproduces each constant-jerk segment instead of joining
  sparse 100 ms samples across jerk changes.
- All IP requests pass the same path: no position-only fallback on a collision, limit, stale
  timestamp or rate refusal. The last accepted trajectory continues to its validated stop.
  The client retains that same timeline instead of resetting to lagging feedback at zero speed.
- Complete trajectories are capped at 2 seconds. The existing position lead remains capped at
  `max_velocity_rad_s * max_segment_s`, normally 0.225 rad. The client also limits each new raw
  target to 0.2 rad from its current planned position. Releasing LB is a smooth deceleration,
  not an immediate brake application; a previously accepted stop can take up to 2 seconds.
- Discrete collision checks retain at most 100 ms spacing, independent of dense spline samples.
  Unique states are checked concurrently against MoveIt with one 35 ms deadline. Missing,
  failed or late validation refuses a new trajectory. This is not continuous collision checking.
- The driver publishes margin-adjusted joint limits on `/lwa4p_driver/limits` and a latched
  reason on `/lwa4p_driver/fault`. The bridge and gamepad IK use the actual limits.
- New client defaults are 0.3 rad/s² acceleration and 2 rad/s³ jerk. The dedicated teleop CLI
  defaults to a 0.3 rad/s joint speed limit and 3 cm/s TCP speed.
- The recommended CLI stops with the reported fault instead of a traceback, and never retries
  enable automatically. Stale action callbacks cannot clear newer fault information.

## Offline verification

From the repository root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 scripts/vla.sh pytest -q tests
. scripts/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q ros/lwa4p_vla_bridge/test
colcon test --packages-select lwa4p_hardware
colcon test-result --test-result-base build/lwa4p_hardware --verbose
```

The Python socket tests create local fake bridges and Unix socket pairs; no CAN adapter is needed.
Bridge tests use real ROS message types with fake service/action clients, without starting ROS.
C++ tests exercise the installed JTC interpolator at 125 Hz, delayed trajectory replacements and
missing updates. None of these tests moves a robot or establishes a power-supply rating.

## Build before reconnecting

```bash
. scripts/env.sh
colcon build --symlink-install --base-paths ros \
  --packages-select lwa4p_hardware lwa4p_vla_bridge
```

Rebuild both packages together: the new IP bridge requires the driver's actual-limit publication.
The only added direct runtime dependency is the already installed standard ROS `std_msgs` package.
No package downloads or new third-party libraries are required.

## Next authorized hardware test

Use someone at the E-stop and a clear arm workspace, as in the previous test. Keep the capture
running before enabling. The commands below are for a future connected test, not an automatic
motion sequence. No tool moves the robot into the saved failed pose.

Stop the earlier launch with Ctrl+C, then source the rebuilt workspace and start one updated
stack. A process already running before the rebuild still uses the older driver code.

Terminal 1, from this repository:

```bash
. scripts/env.sh
ros2 launch lwa4p_vla_bridge robot.launch.py driver:=ip
```

Terminal 2:

```bash
mkdir -p outputs
scripts/vla.sh lwa4p-vla capture --output outputs/ip-test-01.jsonl --duration 180
```

The recorder opens CAN receive sockets and sends only `state`/`trace` bridge requests. It never
enables, resets, changes mapping or sends a CAN frame. It records kernel receive timestamps,
RPDO/TPDO/EMCY payloads, receive-loss counts, JTC demand/feedback and the recent command trace
when a fault appears. It refuses to overwrite an existing output file; choose a new filename.

Enable from a ROS terminal when the physical setup is ready:

```bash
ros2 service call /lwa4p_driver/enable std_srvs/srv/Trigger '{}'
```

This retains the existing ±0.5-degree commutation checks. Confirm enable succeeds before teleop.

Terminal 3, begin with conservative settings:

```bash
scripts/vla.sh lwa4p-vla teleop --speed 0.02 --joint-speed 0.2 \
  --acceleration 0.2 --jerk 1.0
```

Hold LB to move, add RB for quarter-speed precision, and release LB for a smooth stop.
A/B grasps/opens the gripper, with a 17 mm grasp target by default.

1. Hold still for 10 seconds, then make short, slow single-axis movements and release LB.
2. Test short vertical and sideways moves, including reversals. Confirm deceleration is smooth.
3. Approach the previously troublesome pose using the gamepad, slowly. Check that a rejected
   target leaves a stable stop and that moving back away remains possible.
4. If stable, restart teleop at `--speed 0.04 --joint-speed 0.3 --acceleration 0.3 --jerk 2.0`.
   Do not raise all limits at once if a jerk or fault reappears.
5. If the drive trips, retain the capture and stack log before recovery. The CLI must show the
   fault and stop without enabling again. Do not repeatedly reset and retry an unresolved trip.

After stopping the capture:

```bash
scripts/vla.sh lwa4p-vla analyze outputs/ip-test-01.jsonl
```

Retain the full JSON-lines file and the launch log, not only the summary. Check receive losses,
EMCY timestamps, command-step changes, JTC tracking error and validation delays. Sampled command
speed contains encoder quantization and bus timestamp effects; it is not an electrical measurement.

## Undervoltage evidence still needed

Both failures first logged joint 5/node 7 EMCY `0x3222`. Standard voltage object `0x6079` is
unsupported by this 2013 firmware. Vendor `0x200B:1` fault-detail scalars were 350–396; their
units remain unknown, so they must not be reported as volts. The PCB temperatures were 38–50 °C.

If an undervoltage trip remains after the command fixes, a future test needs a calibrated motor
supply measurement under load and checks of supply current limiting and voltage drop. The
passive recorder preserves the timing needed to correlate that measurement with the commands.
