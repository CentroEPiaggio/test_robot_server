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
τ_cmd = τ_model − reg_G · π̂        (gravity.subtract: true, the default)
```

**in simulation and on the real robot alike**, because both add their own
gravity compensation to whatever torque is commanded:

* on hardware, `franka_hardware` passes the effort command straight to
  `franka::Torques` (libfranka 0.15.0), which the robot adds on top of its
  internal gravity compensation;
* in simulation, `franka_ign_ros2_control/IgnitionSystem::write()` computes
  `kdl_model_.gravity(q)` and **adds it to every effort command** before
  handing it to Gazebo — the simulated Franka behaves like the real one in
  this respect.

Commanding gravity would therefore apply it twice in both environments. That
the setting is the same on both sides is convenient: nothing about the control
law changes between a simulation and an experiment.

Note that the simulator's compensation uses the URDF inertias, while `reg_G·π̂`
uses the estimate, so what the joints actually see is `τ_model + (G_true − Ĝ)`
— exactly the situation on the real robot.

**The model term and the gravity subtraction are coupled.** If
`gravity.subtract` is set but no gravity regressor is available — which is the
case for the server-based controller until `reg_G` is added to the server's
published topics — the controller drops the model term and commands the PD
part alone, rather than commanding gravity twice. This is logged and shows up
as `model_valid = false` in the diagnostics.

## Degraded operation

The loop never waits for the model. When no usable `Y_r` is available — the
server has not answered yet, or its samples are older than
`server.timeout_ms` — the model term is dropped, the controller commands the PD
part alone, and the estimate is frozen until fresh data arrives. This is logged
(throttled) and reported in `mact_msgs/MactState` as `model_valid`,
`model_age_ms` and `degraded_cycles`, so a degraded interval is visible in the
bag rather than silently changing the control law.

## Server-side prerequisites

The server-based controller is written against the interface declared in
`franka_server/config/franka_conf.yaml`, and two things have to be regenerated
before it is fully usable:

1. **`Y` must be a function of the actual acceleration.** The generated
   `get_Y()` and `get_Yr()` share their inputs `{q, dq, dqr, ddqr}`, so a
   single server instance — which holds one `(dqr, ddqr)` pair — cannot
   evaluate `Y_r` on the desired motion and `Y` on the actual one at the same
   time. Until `Y(q, q̇, q̈)` is generated, keep `adaptation.gamma: 0.0` for the
   server controller: the prediction error term of eq. (2) would otherwise be
   computed on the wrong motion. The in-process controller has no such
   limitation — it simply re-evaluates `get_Y()` with the reference motion set
   equal to the measured one.

2. **`reg_G` must be published** (`topics: [Yr, Y, reg2dyn, reg_G]`) before
   `gravity.subtract` can be used — and it is needed in simulation as well, see
   above.

   Until then there is an exact workaround that needs no regeneration, and the
   launch files use it by default (`gravity_server:=true`). Since

   ```
   Yr = reg_M(ddqr) + reg_C(q̇, q̇r) + reg_G
   ```

   is linear in the reference motion, a **second `franka_server_node` whose
   `dqr`/`ddqr` inputs are remapped to topics nobody publishes** keeps them at
   the zeros of the generated constructor, and its `Yr` output is exactly
   `reg_G` — verified bit-for-bit against `get_reg_G()`. Its other outputs and
   all of its services are remapped aside so they do not collide with the real
   server. Pass `gravity_server:=false` once the server has been regenerated.

A third, independent issue: **`reg2dyn` returns NaN for any zero-mass block**,
because it divides the first moments by the mass. With the shipped parameters
the `base` and `EE` blocks have zero mass, and the `EE` block falls inside the
80 values that make up `par_DYN`. The controller filters those NaNs out before
publishing `par_DYN` (and warns once), but the generator should guard the
division.

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
| `get_Yr()` | 37 µs |
| `get_Y()` | 36 µs |
| `get_reg_G()` | 0.45 µs |
| `get_M()` | 0.93 µs |
| `get_C()` | 7.5 µs |

**Do not quote those numbers for a control loop.** Measured inside the 1 kHz
Gazebo loop, `update()` takes about **186 µs** for the in-process controller
against **17 µs** for the server-based one — five times what the benchmark
suggests, in the same process and the same machine.

The reason is code size, not arithmetic. `get_Yr()` is a 9-byte thunk into
`franka_gen_f60`, which is **2.12 MiB of straight-line CasADi code**. Called
back-to-back it stays resident in L2/L3 and runs at benchmark speed; called
once per millisecond with a whole simulator running in between, it is re-fetched
from L3 or DRAM every single cycle. The `.so` is 13 MB of text in total.

This cuts both ways for the architecture discussion in the abstract:

* the in-process option is more expensive in a real loop than a microbenchmark
  implies, and the gap grows with the size of the generated model;
* the server option moves that 2 MiB working set into another process, where it
  *is* executed back-to-back and where its cost overlaps with the control loop
  instead of adding to it; what the controller pays instead is serialisation
  and one round trip of staleness.

Measured staleness of the served regressor in Gazebo at 1 kHz: **mean 0.83 ms,
max 3.0 ms**, which is the `model_age_ms` field of the diagnostics.

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
