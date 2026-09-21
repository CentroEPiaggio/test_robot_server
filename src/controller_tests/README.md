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

## Why the generator waits before it starts

The node captures the start pose, then **holds it on the reference topic**
until the controller reports (on `/mact/state`) that it is receiving, and only
then starts the motion clock.

Without that handshake the generator publishes into the void while DDS
discovery completes, and whatever part of the approach goes out in that window
never reaches the controller. Measured across two runs of the same launch
file, the gap was **2.3 s in one and 0.3 s in the other**. The damage is not
only a step in `q_d` when the controller finally connects: by the time the
Lissajous begins, two runs have adapted for different lengths of time, so they
sit at different points of the parameter convergence and no choice of time
origin can make them comparable again.

`wait_for_controller: false` disables it, `handshake_timeout_s` bounds it (the
run then starts anyway, with a warning). While the pose is held the motion
clock reads exactly 0, which is how the controller knows not to start adapting
yet and how the analysis tells the hold apart from the approach.

## Recorded topics

`/mact/state` (the whole control cycle: q, dq, ddq, the reference, the error,
the torques, π̂ and the loop timing), `/mact/trajectory` and `/joint_states`;
plus the `/controller/*` traffic when the server is in the loop.

`Yr`, `Y` and `reg_G` are **not** recorded by default: they are 7×100 matrices
at 1 kHz, about 2.8 MB/s each, and recording them for the server run only would
make the two runs unequal in I/O load. Add them to `SERVER_BAG_TOPICS` in
`launch/mact_launch_utils.py` if you want to inspect them.

Bags land in `bags/<environment>_<controller>_<timestamp>/` relative to the
directory the launch was started from.

## Figures for the paper

```bash
ros2 run controller_tests plot_abstract_figure.py \
    --server bags/gazebo_server_... --local bags/gazebo_local_... --output fig2
```

writes `fig2.pdf` with the two data panels of Fig. 2, plus `fig2_error.pdf`,
`fig2_residual_y.pdf` and `fig2_residual_yr.pdf` for including them
separately. The server run is solid in MATLAB blue (`#0072BD`), the in-process
run dotted; `--local-colour '#0072BD'` makes the figure single-colour.

The time axis is the trajectory generator's own clock, recorded with every
sample, so two runs line up exactly with nothing detected from the data: 0 is
the start of the approach, 5 s the start of the Lissajous.

Useful arguments: `--t0/--t1` for the window and `--rezero` to put t = 0 at
`--t0` (so `--t0 5 --t1 35 --rezero` plots the Lissajous alone), `--png`,
`--width/--height`.

### The parameter metric

Plotting entries of π̂ says little: the regressor has a null space, so many
parameter vectors reproduce the same dynamics. What the figure shows instead is
how well the estimate explains the torque the robot actually produced,

```
r_Y (t) = τ_meas − Y(q, q̇, q̈)·π̂
r_Yr(t) = τ_meas − Y_r(q, q̇, q̇_d, q̈_d)·π̂
```

with both regressors re-evaluated from the recorded state through the same
generated library the controller ran (see `thunder_ctypes.py` — the CasADi
entry points are called directly with `ctypes`, so no bindings or build step
are needed). `r_Yr` replaces the measured acceleration with the reference one
and is exact only in the limit of perfect tracking.

Two things to know before reading the plot:

* **It must use the measured torque.** The control law is
  `τ_model = Y_r·π̂ + PD`, so `τ_model − Y_r·π̂` is identically the PD term and
  says nothing about the estimate. `--torque model` exists only to demonstrate
  that; the default is `measured`.

* **The residual has a floor.** Joint friction is not in the model — the FR3
  URDF has 0.2 Nm of Coulomb friction per joint — so `r` cannot decay below
  roughly that, no matter how good π̂ becomes. In these runs it settles around
  0.2–0.25 Nm.

Adaptation is held until the motion actually starts, so every run begins
adapting at the same point regardless of how long the controller happened to
be active beforehand.

To make the convergence visible at all, start the estimate away from the
nominal parameters, otherwise there is nothing for the adaptation to do:

```bash
ros2 launch controller_tests gazebo_experiment.launch.py \
    controller:=server initial_scale:=0.6
```

The script prints a self-check, `max |τ_model − (Y_r·π̂ + PD)|` over the
in-process run. It should be of the order of one adaptation step
(10⁻⁶–10⁻³ Nm) — the recorded π̂ is the value *after* that cycle's update,
while `τ_model` was computed with the previous one. Whole newton-metres would
mean the library, the column ordering or the parameter file is wrong, and the
residual curves could not be trusted.

## Analysing a run

```bash
ros2 run controller_tests analyse_run.py bags/gazebo_local_20260918_150000
ros2 run controller_tests analyse_run.py bags/gazebo_local_* bags/gazebo_server_*   # side by side
```

It reports the tracking error (rms, max, per joint), the peak torque, the
`update()` duration, how fresh the model data was, how many cycles fell back to
the PD term, and how far π̂ travelled. `--csv` writes the per-sample series next
to the bag so the figures can be drawn with any plotting tool.
