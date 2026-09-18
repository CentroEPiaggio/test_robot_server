# thunder_franka

Ament wrapper around the [Thunder Dynamics](https://github.com/CentroEPiaggio/thunder_dynamics)
generated library for the Franka manipulator.

Everything under `src/` and `include/thunder_franka/` is **generated** by

```
thunder gen franka_conf.yaml
```

and copied here verbatim, together with the identical copy that
`franka_server` carries. Do not edit it: change `config/franka_conf.yaml` and
regenerate, then refresh both packages.

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

The two regressors take different inputs, so both can be evaluated in one
pass over the state:

| | inputs | meaning |
|---|---|---|
| `get_Yr()` | `q, dq, dqr, ddqr` | Slotine-Li regressor on the reference motion |
| `get_Y()` | `q, dq, ddq` | standard regressor on the actual motion, `τ = Y·π` |
| `get_reg_G()` | `q` | gravity part alone |

`get_reg2dyn()` converts `par_REG` (100 values, 10 blocks of 10) into the
dynamic parametrisation. It divides the first moments by the mass, so a
**zero-mass block comes back as NaN** — with the shipped parameters both
`base` and `EE` are zero-mass, and the `EE` block falls inside the trailing 80
values that make up `par_DYN`. Filter them before feeding the result to
`set_par_DYN()`.
