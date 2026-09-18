# thunder_franka

Ament wrapper around the [Thunder Dynamics](https://github.com/CentroEPiaggio/thunder_dynamics)
generated library for the Franka manipulator.

Everything under `src/` and `include/thunder_franka/` is **generated** by

```
thunder gen franka_conf.yaml
```

and is copied here verbatim from `franka_generatedFiles/`. Do not edit it:
change `config/franka_conf.yaml` and regenerate.

The package exists so that the model can be *linked* into a node — the
"in-process" alternative to the Robot Server discussed in the extended
abstract — without every consumer carrying its own copy of the 70 MB
generated translation unit.

## Use

```cmake
find_package(thunder_franka REQUIRED)
target_link_libraries(my_target thunder_franka::thunder_franka)
```

```cpp
#include <thunder_franka/thunder_franka.h>

thunder_franka robot;
robot.set_q(q);
robot.set_dq(dq);
robot.set_dqr(dq_d);    // Slotine-Li reference = desired motion (MACT)
robot.set_ddqr(ddq_d);
const Eigen::Matrix<double, 7, 100> Yr = robot.get_Yr();
```

Note that `get_Y()` and `get_Yr()` share the same inputs `{q, dq, dqr, ddqr}`:
to obtain the *standard* regressor evaluated on the actual motion,
`Y(q, q̇, q̈)`, set `dqr = dq` and `ddqr = ddq` before calling `get_Y()`.
