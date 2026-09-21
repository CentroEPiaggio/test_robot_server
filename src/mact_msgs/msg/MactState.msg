# Diagnostic snapshot of one MACT control cycle.
#
# Published by both MactLocalController (model linked in-process) and
# MactServerController (model served over topics by franka_server), with
# identical semantics, so that the two architectures can be compared on
# exactly the same quantities.
#
# Notation follows the extended abstract "Thunder Dynamics: A Robot Server for
# ROS": e = q_d - q, tau = M q_dd_d + C q_d_d + D + G + k_v e_dot + k_p e.

std_msgs/Header header

# ---------------------------------------------------------------- measured --
float64[7] q            # joint position, from the state interface
float64[7] dq           # joint velocity, low-pass filtered
float64[7] ddq          # joint acceleration, differentiated then filtered

# ----------------------------------------------------------------- desired --
float64[7] q_d
float64[7] dq_d
float64[7] ddq_d

# ------------------------------------------------------------------- error --
float64[7] e            # q_d - q
float64[7] de           # dq_d - dq

# ----------------------------------------------------------------- torques --
# tau_model is the full control law of eq. (1)/(3), gravity included; it is the
# torque the robot actually applies and therefore the tau used in the parameter
# update law. tau_cmd is what was written to the effort command interface: it
# equals tau_model in simulation, and tau_model - G_hat on the real robot,
# where libfranka compensates gravity internally.
float64[7] tau_model
float64[7] tau_cmd
float64[7] tau_meas     # effort state interface

# Gravity torque removed from the command, from whichever source
# 'gravity.source' selects. Recorded so that the cancellation can be checked
# against the alternatives offline.
float64[7] tau_gravity

# -------------------------------------------------------------- adaptation --
float64[] pi_hat        # current estimate of the regressor parameters (par_REG)

# ------------------------------------------------------------ health/timing --
# Wall-clock duration of update(), aggregated over the decimation window so
# that no outlier is lost when the message is published below the loop rate.
float64 update_duration_min_us
float64 update_duration_mean_us
float64 update_duration_max_us
uint32  cycles_in_window

bool    model_valid       # a usable regressor was available this cycle
float64 model_age_ms      # age of the newest Yr; always 0.0 in-process
uint32  degraded_cycles   # cycles spent in PD-only fallback since activation
bool    trajectory_valid  # a fresh desired trajectory was available
