# ERF 2027 experiments (not submitted)

Scripts and summary results of the real-robot comparison and the interface benchmarks that were prepared for an ERF 2027 paper. The paper was never submitted. This branch keeps the material reproducible. The raw data (robot bags, benchmark CSVs) is **not** in git; see *Data* below.

## Setup actually used
These values were recovered from the recorded data on 2026-11-10, because the configuration was not saved with the runs and the controller YAML in this branch still holds different gains.

| item | value | how it is known |
|---|---|---|
| robot | Franka Research 3, torque control at 1 kHz via `ros2_control` | launch files |
| model library | `thunder_franka`, generated 2026-09-18, Lagrange regressors (Thunder before v1.2.3) | build timestamps |
| bags | `bags/real_local_p06_new`, `bags/real_server_p06_new` (2026-10-12) | |
| K_p | diag(50, 50, 50, 50, 15, 15, 4) Nm/rad | fitted from τ_model − Y_r π̂ (`debug_gains.py`), residual ≤ 0.013 Nm |
| K_v | diag(5, 5, 5, 5, 1.5, 1.5, 0.4) Nm s/rad | same |
| π̂(0) | 0.6 · nominal `par_REG` (`initial_scale:=0.6`) | first `/mact/state` sample, all 100 entries |
| Lissajous | `run_duration` 30 s (adaptation on trajectory time 5 → 35 s) | `adaptation_enabled` in the bags |
| gravity | `franka_model` (libfranka adds it back) | launch default for the real robot |
| γ, R_p, R_t, torque_source, filter cut-offs | **not recorded** | — |
| PC, kernel, CPU affinity | **not recorded**; the scripts' CPU layout refers to an i9-12900H | — |

## Scripts
| script | input | output |
|---|---|---|
| `compare_runs.py` | the two p06 bags | `out_p06/summary.json` and `*.npy`/`*.npz` (trajectory difference, torque deviation) |
| `plot_paper_figures.py` | the p06 bags and `out_p06/trajectory_difference.npz` | `figures/fig_trajectory.pdf`, `figures/fig_timing.pdf` |
| `plot_benchmarks.py` | `bench_pinned/` | `figures/fig_benchmarks.pdf` |
| `debug_gains.py` | one bag | prints K_p and K_v fitted per joint |
| `check_initial.py` | bags | prints ‖π̂(0)‖ (writes `/tmp/pi0.npy`) |

Timing and tracking tables:

```bash
ros2 run controller_tests analyse_run.py    bags/real_local_p06_new bags/real_server_p06_new
ros2 run controller_tests analyse_timing.py bags/real_local_p06_new bags/real_server_p06_new
```

Interface benchmarks, run on the i9-12900H with every process pinned to a P-core, 30 s × 3 repetitions per case:

```bash
ros2 run server_benchmarks run_benchmarks.bash paper_erf/bench_pinned 30 3
python3 src/server_benchmarks/scripts/summarise_benchmarks.py paper_erf/bench_pinned
```

## Results kept in git
- `out_p06/summary.json`: run comparison.
- `bench_pinned/summary.json` and `bench_pinned/eval.csv`: benchmark summary.

## Data (not in git)
- `bags/real_*_p06_new`: 17 MB and 65 MB. These **cannot be recreated** without the robot; they are archived at: *(to be filled)*.
- `bench_pinned/*.csv|*.log`: 33 MB; recreated by `run_benchmarks.bash`.
- `out_p06/*.npy|*.npz` and `figures/`: recreated by the scripts above.

## Known caveat
The robot runs did not pin the controller thread. On the i9-12900H, the old library takes about 40 µs per regressor on a P-core and about 140 µs on an E-core. In the control loop it took 192 µs, so the in-process timing in these runs may include E-core scheduling.
