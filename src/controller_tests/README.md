# controller_tests

Test harness for the two MACT controllers of `mact_controllers`.

It provides the *same* experiment for both, so that a comparison between them
measures the architecture and nothing else:

* a joint-space **Lissajous** reference, generated at 1 kHz by
  `trajectory_publisher_node` from `config/lissajous.yaml`;
* launch files that bring up either the Gazebo simulation or the real robot,
  spawn the controller under test, start the Robot Server only when it is
  needed, and record a bag.

## Running

```bash
# simulation
ros2 launch controller_tests gazebo_experiment.launch.py controller:=local
ros2 launch controller_tests gazebo_experiment.launch.py controller:=server

# real robot (FCI unlocked, robot in the start pose region)
ros2 launch controller_tests real_experiment.launch.py controller:=local robot_ip:=172.16.0.3
```

Arguments: `controller` (`local` | `server`), `arm_id` (`fr3` | `fer` | `fp3`),
`record`, `use_rviz`, `headless` (simulation only), and for the real robot
`robot_ip` and `use_fake_hardware`.

The simulation launch also starts a `ros_gz_bridge` for `/clock`. Gazebo drives
the controller manager with simulated time through the `gz_ros2_control`
plugin, but it does not publish `/clock` on its own; without the bridge every
other node running with `use_sim_time` sits at t = 0 and the 1 kHz trajectory
timer never fires.

Startup order is enforced with event handlers, not delays:

```
robot spawned -> joint_state_broadcaster -> MACT controller
              -> bag recorder -> (2 s) trajectory generator
```

so the reference never starts before the controller is active, and the
recorder has time to open the bag before the first sample.

## The trajectory

Three phases, each continuous in position, velocity and acceleration:

1. **approach**, a quintic from wherever the robot is to the fixed `centre`,
   starting and ending at rest — this is what makes two runs comparable;
2. **Lissajous**, `q_d,i = centre_i + s(t)·A_i·sin(2π f_i t + φ_i)`, with a
   quintic envelope `s(t)` rising over `ramp_duration` and falling again at the
   end, so the phase offsets can be chosen freely without causing a jump;
3. **hold** at the centre, at rest.

The frequencies are mutually incommensurate, which keeps the regressor
persistently exciting — without that, the parameter estimates converge to
something that merely fits the executed motion.

`position_min`/`position_max` are checked at start-up: the node refuses to run
if `centre ± amplitude` leaves the joint range.

## Recorded topics

`/mact/state` (the whole control cycle: q, dq, ddq, the reference, the error,
the torques, π̂ and the loop timing), `/mact/trajectory` and `/joint_states`;
plus the `/controller/*` traffic when the server is in the loop.

`Yr` and `Y` are **not** recorded by default: they are 7×100 matrices at 1 kHz,
about 5.6 MB/s, and recording them for the server run only would make the two
runs unequal in I/O load. Add them to `SERVER_BAG_TOPICS` in
`launch/mact_launch_utils.py` if you want to inspect them.

Bags land in `bags/<environment>_<controller>_<timestamp>/` relative to the
directory the launch was started from.

## Analysing a run

```bash
ros2 run controller_tests analyse_run.py bags/gazebo_local_20260918_150000
ros2 run controller_tests analyse_run.py bags/gazebo_local_* bags/gazebo_server_*   # side by side
```

It reports the tracking error (rms, max, per joint), the peak torque, the
`update()` duration, how fresh the model data was, how many cycles fell back to
the PD term, and how far π̂ travelled. `--csv` writes the per-sample series next
to the bag so the figures can be drawn with any plotting tool.
