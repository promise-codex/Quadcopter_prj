% MAIN  Entry point for the supervised switched-control quadrotor study.
% -------------------------------------------------------------------------
% Project: Safety-governed controller switching for a delivery quadrotor.
%   Controller A (baseline) : cascaded PID, no disturbance compensation.
%   Controller B (robust)   : INDI (sensor-based incremental) by default, or
%                             DOBC (observer feed-forward) -- set P.robust_type.
%   Supervisor              : engages B on demand via a disturbance indicator,
%                             with hysteresis, dwell-time and bumpless blending.
%   A disturbance observer runs continuously as the supervisor's indicator.
% Run this whole file (F5 in MATLAB) for a demo + the Monte-Carlo table.
% -------------------------------------------------------------------------
clear; clc; close all;
P = quad_params();

% 1) Presentation visuals (these are what you SHOW): schematic, 3D flight
%    path, and the time-series figure. Set the animation flag to also save a GIF.
fprintf('Generating presentation visuals...\n');
draw_schematic();                 % -> quad_schematic.png
plot_mission_3d('severe', 1);     % -> mission_3d.png
make_plots('severe', 1);          % -> fig_run.png (tracking / error / switching)
animate_mission('severe', 1, true);   % -> mission.gif (set last arg false to skip)

% 2) Quick single-seed comparison across scenarios and controllers
fprintf('\nSingle-seed comparison (seed 1):\n');
fprintf('%-10s %-11s %8s %9s %8s %7s\n','scenario','mode','RMSE(m)','maxdev','effort','sw');
for scn = {'calm','moderate','severe'}
    for mode = {'baseline','robust','supervisor'}
        m = compute_metrics(run_sim(scn{1}, mode{1}, 1, P));
        fprintf('%-10s %-11s %8.3f %9.3f %8.1f %7.0f\n', ...
                scn{1}, mode{1}, m.rmse, m.maxdev, m.effort, m.nsw);
    end
end

% 3) Full Monte-Carlo study with 95% CI and paired significance tests
%    (robust controller is set by P.robust_type in quad_params: 'indi' default)
R = run_experiment(30);   %#ok<NASGU>

fprintf('\nDone. See fig_run.png and the printed tables.\n');
