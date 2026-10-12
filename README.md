# Safety-Governed Controller Switching for Disturbance-Resilient Delivery Quadrotors

MATLAB simulation of a **supervised switching flight controller** for a delivery
quadrotor flying under wind, gusts, and a mid-mission payload drop. The supervisor
switches the outer position loop between a lightweight **PID** baseline and a robust
**INDI** (incremental nonlinear dynamic inversion) controller, engaging the robust
controller only when a disturbance indicator fires — with hysteresis, a minimum
dwell time, and a bumpless, authority-limited blend.

> **Headline result (30 random wind seeds):** the supervisor matches the always-on
> INDI controller's tracking accuracy while using **~37–38% less control effort**
> (p < 10⁻²⁵), and both far outperform the fixed PID baseline.

---

## Requirements

- MATLAB **R2016b or newer**. **No toolboxes required.**
  (The Statistics Toolbox is used *only* for the exact `ttest` p-value; without it,
  the code automatically falls back to a t-statistic — it will not error.)

## How to run

Put all `.m` files in one folder, make it the MATLAB **Current Folder**, then:

```matlab
main
```

This produces the figures and prints the single-seed table plus the Monte-Carlo
study (mean ± 95% confidence interval and paired significance tests).

Single-file alternative (nothing else needed): run `run_quad_singlefile.m`.

### Other entry points

```matlab
make_plots('severe', 1);          % tracking / error / switching time-series
plot_mission_3d('severe', 1);     % 3-D flight path
animate_mission('severe', 1, true); % animation -> mission.gif
draw_schematic;                   % labelled quadrotor diagram
R = run_experiment(30);           % full Monte-Carlo (returns raw results)
log = run_sim('moderate','supervisor',7, quad_params());  % one closed-loop run
```

## Outputs (saved to the working folder)

| File | What it is |
|------|------------|
| `fig_run.png` | tracking, error, and supervisor switching time-series |
| `mission_3d.png` | 3-D delivery flight path with the drone drawn along it |
| `mission.gif` | animation of the mission |
| `quad_schematic.png` | labelled X-configuration quadrotor diagram |

## File overview

| File | Role |
|------|------|
| `main.m` | entry point: visuals + single-seed table + Monte-Carlo study |
| `quad_params.m` | all parameters, gains, and `robust_type` (`'indi'` or `'dobc'`) |
| `quad_deriv.m` | 6-DOF rigid-body dynamics (state derivative) |
| `Rzyx.m`, `euler_rate_matrix.m` | rotation / Euler-rate kinematics |
| `reference_traj.m` | delivery-mission reference trajectory |
| `make_wind.m`, `wind_at.m` | wind realisation (turbulence + 1-cosine gust) |
| `force_to_cmd.m` | map a required world force to thrust + roll/pitch |
| `attitude_ctrl.m` | inner attitude PD (shared by both controllers) |
| `run_sim.m` | one closed-loop run (observer, supervisor, INDI/DOBC, RK4) |
| `compute_metrics.m` | RMSE, peak deviation, recovery, control effort, switches |
| `run_experiment.m` | Monte-Carlo + 95% CI + paired t-tests |
| `make_plots.m`, `plot_mission_3d.m`, `animate_mission.m`, `draw_quad.m`, `draw_schematic.m` | figures |

## Method (one paragraph)

A fixed inner attitude loop always stabilizes the vehicle. The outer loop is
switched between a cascaded PID controller and an INDI controller that rejects the
lumped disturbance (wind + payload-change mismatch). A disturbance observer runs
continuously and provides the supervisor's switching indicator; the robust
controller does the actual rejection. Switching safety follows average-dwell-time
switched-systems theory (hysteresis + dwell time) and bumpless-transfer practice
(a blended, authority-limited feed-forward).

## Citation

If you use this code, please cite the accompanying paper. A DOI (via Zenodo) will be
added on release.

## License

Released under the MIT License — see [`LICENSE`](LICENSE).
