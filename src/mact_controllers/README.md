# mact_controllers

Two `ros2_control` implementations of the **Modified Adaptive Computed Torque**
(MACT) law for the 7-DoF Franka manipulator:

| plugin | where the model lives |
|---|---|
| `mact_controllers/MactLocalController` | Thunder library **linked in-process** |
| `mact_controllers/MactServerController` | Thunder **Robot Server**, over topics |

Everything that is not the source of the model is shared: the same base class,
the same gains, the same filters, the same reference handling, the same update
law, the same saturation, the same diagnostics, and — literally — the same
parameter file (`config/mact_common.yaml`). Comparing the two therefore
measures the architecture, not the tuning.

## The control law

With `e = q_d − q` and `π̂` the estimate of the dynamic parameters:

```
τ_model = Y_r(q, q̇, q̇_d, q̈_d) · π̂  +  k_v ė  +  k_p e            (1) via (3)
π̂̇      = R_p ( Y_r(q, q̇, q̇_d, q̈_d)ᵀ ė  +  γ Yᵀ R_t (τ − Y π̂) )   (2)
```

Both controllers assemble the model term from the Slotine-Li regressor and
their own copy of `π̂` — eq. (3) of the extended abstract — rather than from
`M̂, Ĉ, D̂, Ĝ`. That is the form the server exposes, and using it on both sides
keeps the two mathematically identical.

Friction is **not** modelled: the generated `Y_r` is 7×100 and covers inertia
only (`reg_M + reg_C + reg_G`); the friction regressor `reg_dl` is a separate
7×14 with its own 14 parameters. The `D̂` of eq. (1) is therefore absent from
both controllers, equally.

## What is actually commanded

`τ_model` is the torque the robot applies, and it is the `τ` that eq. (2)
refers to. What reaches the command interface is

```
τ_cmd = τ_model − G
```

**in simulation and on the real robot alike**, because both add their own
gravity compensation to whatever torque is commanded:

* on hardware, `franka_hardware` passes the effort command straight to
  `franka::Torques` (libfranka 0.15.0), which the robot adds on top of its
  internal gravity compensation;
* in simulation, `franka_ign_ros2_control/IgnitionSystem::write()` computes
  `kdl_model_.gravity(q)` and **adds it to every effort command** before
  handing it to Gazebo.

### Which gravity is subtracted

`gravity.source` selects `G`, and the choice is not cosmetic, because
`Y_r·π̂` already contains the controller's own gravity estimate `reg_G·π̂`.
Writing `G_hw` for what the hardware adds back:

| `gravity.source` | `G` | resulting applied torque |
|---|---|---|
| `estimate` | `reg_G·π̂` | `M̂q̈_d + Ĉq̇_d + PD + G_hw` |
| `urdf_kdl` | KDL on the URDF, as the Gazebo plugin computes it | `M̂q̈_d + Ĉq̇_d + PD + Ĝ` |
| `franka_model` | the robot's own vector, via `FrankaRobotModel` | `M̂q̈_d + Ĉq̇_d + PD + Ĝ` |
| `none` | 0 | for a simulator that does not compensate gravity |

With `estimate` the two gravity terms cancel and the robot is handed the
hardware's *exact* gravity compensation — which the controller never computed.
That flatters the tracking, and it biases the prediction error of eq. (2),
because the applied torque is then no longer the `τ_model` the update law
assumes. With an exact source the loop really runs on the controller's own
model, which is the honest setup for an adaptive controller: the gravity error
becomes something the adaptation has to learn rather than something the
simulator hides.

`urdf_kdl` reproduces the plugin's computation exactly — same URDF, same
`KDL::ChainDynParam::JntToGravity`, same chain (URDF root to the last leaf of a
breadth-first walk, here `world → fr3_link8`), same hard-coded `g = -9.8`.
`franka_model` reads the vector libfranka itself compensates, through the
`<arm_id>/robot_model` and `<arm_id>/robot_state` state interfaces; those exist
only with `franka_hardware`, so in Gazebo the controller would fail to activate
on the missing interfaces. The launch files select `urdf_kdl` in simulation and
`franka_model` on the robot, the same for both controllers.

Measured here: Thunder's gravity (identified Panda parameters) differs from the
FR3 URDF gravity Gazebo applies by **2.2 % rms, at most 0.44 Nm** on joint 4.
Small, but it moves the steady tracking error of the wrist joints by the
expected `G_error / k_p`.

The gravity is subtracted only when the model term is actually commanded: with
the PD term alone there is no gravity in the command to remove, and taking it
out would make the arm drop. The value used each cycle is recorded as
`tau_gravity` in the diagnostics.

## When the estimate starts moving

Adaptation is held until the motion starts, and from then on it runs for the
rest of the activation — including if the reference later goes stale. There is
nothing wrong with estimating before a trajectory is being tracked; the reason
for the latch is purely that a run has to be reproducible. The controller is
activated a second or two before the generator starts, and that interval is
not the same from one launch to the next; since the prediction error term of
eq. (2) is active even with zero tracking error, without the latch each run
would begin with an estimate that had already drifted by an arbitrary amount —
measured at 0.30 against 0.45 in two otherwise identical runs.

"Motion starts" means the reference's `time_from_start` becomes non-zero, or
failing that its velocity does. The generator holds the start pose with a zero
clock while it waits for this controller to be connected, and that hold must
not count as tracking.

With the latch the two controllers start from the same π̂ and their parameter
trajectories agree to 0.5 % instead of 18 %.

## How τ and Y line up in the update law

Eq. (2) compares the torque the robot applied with `Y π̂`, and the two should
refer to the same instant. They do not, quite, and the offset is different in
the two controllers. Writing the cycle index `k`:

* `ddq_k` is a backward difference, `(q̇_k − q̇_{k−1})/T`, so it is the mean
  acceleration over the interval `[t_{k−1}, t_k)`;
* `τ_{k−1}` is the torque that was applied over exactly that interval, and it
  is what the controller feeds the update law (`tau_model_previous_`);
* the **in-process** controller evaluates `Y(q_k, q̇_k, q̈_k)` in the same
  cycle, so its acceleration matches the interval of `τ_{k−1}` — aligned;
* the **server-based** controller receives `Y(q_{k−1}, q̇_{k−1}, q̈_{k−1})`,
  because the state it publishes at the end of cycle `k−1` comes back one
  round trip later. Its acceleration belongs to `[t_{k−2}, t_{k−1})` while
  `τ_{k−1}` was applied over `[t_{k−1}, t_k)` — one cycle out.

So the served `Y` is not stale with respect to the *controller's* clock in any
loose sense: it is exactly one control period behind the state it was asked
about, which `model_age_ms` measures directly (mean 0.78 ms at 1 kHz). What it
is out of step with is the torque it gets paired with.

This is fixable without touching the server: keeping a two-deep history of
`τ_model` and pairing the served `Y` with `τ_{k−2}` restores the alignment,
at the cost of feeding the update law a torque one extra cycle old. It is not
done here, because the difference between the two controllers is precisely the
quantity the comparison is meant to expose — with `gamma = 0` the two parameter
trajectories agree to 0.1 %, with `gamma = 1` they diverge by a few per cent,
and that gap *is* the measurement. Align them only if the goal changes from
measuring the architecture to hiding it.

## Degraded operation

The loop never waits for the model. When no usable `Y_r` is available — the
server has not answered yet, or its samples are older than
`server.timeout_ms` — the model term is dropped, the controller commands the PD
part alone, and the estimate is frozen until fresh data arrives. This is logged
(throttled) and reported in `mact_msgs/MactState` as `model_valid`,
`model_age_ms` and `degraded_cycles`, so a degraded interval is visible in the
bag rather than silently changing the control law.

## What the server has to expose

The server-based controller is written against the interface declared in
`franka_server/config/franka_conf.yaml`, which now reads

```yaml
inputs:  q, dq, ddq, dqr, ddqr, par_REG, par_DYN
topics:  [Yr, Y, reg_G, reg2dyn]
services: [par_KIN, par_REG, par_DYN]
```

Two properties of that interface are what make the controller possible, and
both arrived with the regeneration:

1. **`Y` is a function of the actual acceleration**, `Y(q, q̇, q̈)`, and no
   longer shares its inputs with `Y_r(q, q̇, q̇ᵣ, q̈ᵣ)`. A single server can
   therefore evaluate the Slotine-Li regressor on the desired motion and the
   standard regressor on the actual one in the same cycle, which is exactly
   what eq. (2) needs. `adaptation.gamma` may now be non-zero for the
   server-based controller as well; it is 1.0 by default.

2. **`reg_G` is published**, so the command can be made gravity-free on both
   sides (see above).

One issue remains open on the generator: **`reg2dyn` returns NaN for any
zero-mass block**, because it divides the first moments by the mass. With the
shipped parameters the `base` and `EE` blocks have zero mass, and the `EE`
block falls inside the 80 values that make up `par_DYN`. The controller filters
those NaNs out before publishing `par_DYN` (and warns once), but the generated
function should guard the division.

## Parameter feedback

The server-based controller publishes `par_REG` every cycle, so the server's
regressors and the controller's estimate never drift apart. `par_DYN` is
published too — the server's `M̂, Ĉ, Ĝ` are then meaningful as well — and is
**not** recomputed locally: it is taken from the server's own `reg2dyn` topic,
of which `par_DYN` is the trailing 80 values (the symbolic blocks `link1..EE`).
It is therefore one round trip behind `par_REG`, which is harmless because
nothing in the control law uses it.

Note that `R_p_link` leaves the `EE` block frozen by default (no payload) and
the `base` block too (it is fixed, so its regressor columns are zero anyway).
`link0` *is* adapted, but `par_DYN` starts at `link1`, so an adapted `link0`
is not reflected in the server's `par_DYN`. Again harmless: `Y_r`, `Y` and
`reg_G` depend on `par_REG`, never on `par_DYN`.

## Measured cost of the model

Standalone microbenchmark, `-O2`, i9-12900H, 20 000 back-to-back calls:

| call | time |
|---|---|
| `get_Yr()` | 37.9 µs |
| `get_Y()` | 38.4 µs |
| `get_reg_G()` | 0.46 µs |
| `get_M()` | 0.89 µs |
| `get_C()` | 7.5 µs |

**Do not quote those numbers for a control loop.** Measured inside the 1 kHz
Gazebo loop with `gamma = 1` (so both regressors are needed every cycle),
`update()` takes about **254 µs** for the in-process controller against
**16 µs** for the server-based one — in the same process and on the same
machine.

The reason is code size, not arithmetic. `get_Yr()` and `get_Y()` are 9-byte
thunks into CasADi blobs of **2.12 MiB and 2.13 MiB of straight-line code**
(`reg_G` is only 15 KiB; the `.so` is 15.4 MB of text in total). Called
back-to-back they stay resident in L2/L3 and run at benchmark speed; called
once per millisecond with a whole simulator running in between, 4.25 MiB of
instructions are re-fetched from L3 or DRAM every single cycle.

This cuts both ways for the architecture discussion in the abstract:

* the in-process option is more expensive in a real loop than a microbenchmark
  implies, and the gap grows with the size of the generated model;
* the server option moves that working set into another process, where it *is*
  executed back-to-back and where its cost overlaps with the control loop
  instead of adding to it; what the controller pays instead is serialisation
  and one round trip of staleness.

Measured staleness of the served regressors in Gazebo at 1 kHz: **mean 0.78 ms,
max 2.0 ms**, reported per sample as `model_age_ms`.

## Configuration

* `config/mact_common.yaml` — everything shared; loaded by both controllers.
* `config/mact_local.yaml` — in-process specifics.
* `config/mact_server.yaml` — topic names, staleness limits, service names.

The files are node-agnostic (`/**`), so they are loaded through the spawner's
`--param-file` and nothing depends on the name the controller is spawned under.

## Running

Through `controller_tests`, which brings up the robot, spawns the controller
and records a bag:

```bash
ros2 launch controller_tests gazebo_experiment.launch.py controller:=local
ros2 launch controller_tests gazebo_experiment.launch.py controller:=server
ros2 launch controller_tests real_experiment.launch.py  controller:=local robot_ip:=172.16.0.3
```

Standalone, against an already running controller manager:

```bash
ros2 run controller_manager spawner mact_controller \
  --controller-type mact_controllers/MactLocalController \
  --param-file $(ros2 pkg prefix mact_controllers)/share/mact_controllers/config/mact_common.yaml \
  --param-file $(ros2 pkg prefix mact_controllers)/share/mact_controllers/config/mact_local.yaml
```

Both controllers are spawned under the same name, `mact_controller`, so the
recorded topics and the analysis do not depend on which one was running.

Note that an **empty YAML list cannot be typed** by the ROS 2 parameter file
parser (`joints: []` makes the controller node throw while it is being
constructed, which in Gazebo takes the whole simulator down with it). Omit such
keys rather than leaving them empty.
