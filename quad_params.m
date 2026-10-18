function P = quad_params()
% QUAD_PARAMS  Vehicle, controller, observer and supervisor parameters.
%   All gains were validated in a reference simulation; see the handbook.

P.g        = 9.81;                 % gravity (m/s^2)
P.m0       = 1.2;                  % nominal mass used by the controller (kg)
P.m_payload= 0.4;                  % payload dropped at delivery (kg)
P.I        = diag([0.015 0.015 0.027]);   % inertia (kg m^2)
P.kd       = 0.10;                 % linear aero drag coefficient (N per m/s)
P.Tmax     = 2.2*P.m0*P.g;         % max total thrust (N)
P.taumax   = [2.0 2.0 1.0];        % max body torques (N m)
P.tau_act  = 0.025;                % actuator first-order lag (s)
P.tilt_max = deg2rad(35);          % max commanded tilt (rad)

% ---- cascaded PID: position (outer) loop, per world axis x,y,z ----
P.Kp_pos = [3.0 3.0 6.0];
P.Kd_pos = [3.2 3.2 5.0];

% ---- attitude (inner) loop, shared by both controllers ----
P.Kp_att = [70 70 28];
P.Kd_att = [14 14 10];

% ---- disturbance observer gain (per world axis) ----
P.L_dob  = [6 6 6];

% ---- supervisor ----
P.thr_hi    = 1.5;    % engage robust when |d_hat| (N) exceeds this
P.thr_lo    = 0.8;    % disengage when below this (hysteresis)
P.dwell     = 0.5;    % minimum dwell time between switches (s)
P.tau_blend = 0.12;   % bumpless blend time constant (s)
P.s_lpf     = 0.06;   % indicator low-pass time constant (s)
P.dff_max   = 8.0;    % DOBC feed-forward authority limit per axis (N)

% ---- robust controller selection ----
P.robust_type = 'indi';   % 'indi' (sensor-based incremental) or 'dobc' (observer)
P.indi_clip   = 12.0;     % INDI force increment authority limit per axis (N)

% ---- measurement noise fed to the observer ----
P.acc_noise = 0.15;   % velocity/accel measurement noise scale

% ---- mission timing ----
P.t_drop = 14.0;      % payload release time (s)
P.Tend   = 32.0;      % simulation length (s)
P.dt     = 0.004;     % fixed control/integration step (s) -> 250 Hz
end
