% =====================================================================
%  SUPERVISED SWITCHED CONTROL FOR A DELIVERY QUADROTOR  (single-file)
%  Self-contained MATLAB script -- no file upload needed.
%  HOW TO USE in MATLAB Online:
%    1. HOME > New Script.  2. Paste ALL of this.  3. Ctrl+S (name it
%       run_quad.m).  4. Press Run (green triangle).
%  It produces, as image files you can show in a presentation:
%     quad_schematic.png  - labelled drone diagram
%     mission_3d.png      - 3-D flight path with the drone drawn along it
%     mission.gif         - animation of the drone flying the mission
%     fig_run.png         - tracking / error / switching time-series
%  plus the printed results tables with 95%% CI and significance tests.
% =====================================================================

clear; clc; close all;
P = quad_params();

% ---- presentation visuals (what you SHOW the examiners) ----
fprintf('Generating presentation visuals...\n');
draw_schematic();
plot_mission_3d('severe', 1);
make_plots('severe', 1);
animate_mission('severe', 1, true);   % set last arg to false to skip the GIF

% ---- single-seed comparison across scenarios and controllers ----
fprintf('\nSingle-seed comparison (seed 1):\n');
fprintf('%-10s %-11s %8s %9s %8s %7s\n','scenario','mode','RMSE(m)','maxdev','effort','sw');
for scn = {'calm','moderate','severe'}
    for mode = {'baseline','robust','supervisor'}
        m = compute_metrics(run_sim(scn{1}, mode{1}, 1, P));
        fprintf('%-10s %-11s %8.3f %9.3f %8.1f %7.0f\n', ...
                scn{1}, mode{1}, m.rmse, m.maxdev, m.effort, m.nsw);
    end
end

% ---- Monte-Carlo study with 95%% CI + paired tests ----
NSEEDS = 15;   % raise to 30 for paper-grade numbers
R = run_experiment(NSEEDS);   %#ok<NASGU>
fprintf('\nDone. Image files saved to the Files panel; tables are above.\n');

% =====================================================================
%  LOCAL FUNCTIONS
% =====================================================================

% ------------------------- quad_params -------------------------
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


% ------------------------- Rzyx -------------------------
function R = Rzyx(phi, th, ps)
% RZYX  Body-to-world rotation matrix from ZYX Euler angles (roll phi, pitch th, yaw ps).
cphi=cos(phi); sphi=sin(phi); cth=cos(th); sth=sin(th); cps=cos(ps); sps=sin(ps);
R = [ cth*cps, sphi*sth*cps-cphi*sps, cphi*sth*cps+sphi*sps; ...
      cth*sps, sphi*sth*sps+cphi*cps, cphi*sth*sps-sphi*cps; ...
      -sth,    sphi*cth,              cphi*cth ];
end


% ------------------------- euler_rate_matrix -------------------------
function W = euler_rate_matrix(phi, th)
% EULER_RATE_MATRIX  Maps body rates [p;q;r] to Euler-angle rates for ZYX angles.
cphi=cos(phi); sphi=sin(phi); cth=cos(th); tth=tan(th);
W = [ 1, sphi*tth, cphi*tth; ...
      0, cphi,    -sphi; ...
      0, sphi/cth, cphi/cth ];
end


% ------------------------- quad_deriv -------------------------
function xdot = quad_deriv(x, u, mass, I, P, Fext)
% QUAD_DERIV  6-DOF quadrotor state derivative.
%   x    = [p(3); v(3); eul(3); om(3)]  (world pos/vel, ZYX Euler, body rates)
%   u    = [T; tau_x; tau_y; tau_z]     (applied total thrust and body torques)
%   mass = current mass (changes at payload drop)
%   I    = current inertia matrix
%   Fext = external force in WORLD frame (wind), 3x1
v   = x(4:6);
eul = x(7:9);
om  = x(10:12);
phi = eul(1); th = eul(2); ps = eul(3);

R      = Rzyx(phi, th, ps);
Fthr   = R*[0;0;u(1)];
grav   = [0;0;-mass*P.g];
drag   = -P.kd*v;
vdot   = (Fthr + grav + drag + Fext)/mass;
pdot   = v;
euldot = euler_rate_matrix(phi, th)*om;
omdot  = I \ (u(2:4) - cross(om, I*om));

xdot = [pdot; vdot; euldot; omdot];
end


% ------------------------- reference_traj -------------------------
function [p,v,a,psi] = reference_traj(t)
% REFERENCE_TRAJ  Delivery mission reference: take off, cruise to drop point,
%   hover, return. Smooth (quintic) blends between waypoints. Returns position,
%   velocity, acceleration (world) and yaw reference at time t.
wp = [ 0 0 0;  0 0 5;  12 0 5;  12 0 5;  0 0 5;  0 0 0.5 ];
tk = [ 0 4 12 16 24 30 ];

p = pos_at(t, wp, tk);
h = 1e-3;
v = (pos_at(t+h,wp,tk) - pos_at(t-h,wp,tk))/(2*h);
a = (pos_at(t+h,wp,tk) - 2*pos_at(t,wp,tk) + pos_at(t-h,wp,tk))/(h*h);
psi = 0;
end

function p = pos_at(t, wp, tk)
p = wp(end,:)';
for i = 1:numel(tk)-1
    if t < tk(i+1)
        s = smoothstep(t, tk(i), tk(i+1));
        p = (wp(i,:) + (wp(i+1,:)-wp(i,:))*s)';
        return;
    end
end
end

function s = smoothstep(t,t0,t1)
if t<=t0, s=0; return; end
if t>=t1, s=1; return; end
x = (t-t0)/(t1-t0);
s = 6*x^5 - 15*x^4 + 10*x^3;
end


% ------------------------- make_wind -------------------------
function W = make_wind(scn, seed)
% MAKE_WIND  Pre-generate a wind realisation (mean 0): first-order-filtered
%   turbulence ("Dryden-like") plus a discrete 1-cosine gust. Returns a struct
%   sampled by WIND_AT. scn = 'calm' | 'moderate' | 'severe'.
switch scn
    case 'calm',     sigma=0.25; gustA=0.0;
    case 'moderate', sigma=0.90; gustA=3.0;
    case 'severe',   sigma=1.70; gustA=5.5;
    otherwise, error('unknown scenario %s', scn);
end
rng(seed, 'twister');
Tend=32.0; dt=0.01; N=round(Tend/dt)+2;
w = randn(N,3)*sigma;
a = exp(-dt/1.0);                 % turbulence correlation time 1 s
f = zeros(N,3);
for k=2:N
    f(k,:) = a*f(k-1,:) + (1-a)*w(k,:);
end
W.f = f; W.dt = dt; W.N = N;
W.axis = [1 0.7 0.2];             % anisotropy
W.gustA = gustA;
W.gust_t0 = 13 + 7*rand();        % gust onset in [13,20] s
W.gust_dur = 2.0;
end


% ------------------------- wind_at -------------------------
function F = wind_at(W, t)
% WIND_AT  Sample the pre-generated wind force (world frame, N) at time t.
k = min(floor(t/W.dt)+1, W.N);
turb = W.f(k,:) .* W.axis;
g = 0;
if W.gustA>0 && t>=W.gust_t0 && t<=W.gust_t0+W.gust_dur
    g = W.gustA*(1-cos(2*pi*(t-W.gust_t0)/W.gust_dur))/2;
end
F = (turb + [g, 0.3*g, 0])';
end


% ------------------------- force_to_cmd -------------------------
function [T, phi_des, theta_des] = force_to_cmd(F_req, psi_ref, P)
% FORCE_TO_CMD  Map a required world thrust-force vector to a thrust magnitude
%   and desired roll/pitch (given the yaw reference). Shared by all controllers.
Fn = norm(F_req); if Fn < 1e-6, Fn = 1e-6; end
zb = F_req/Fn;                                  % desired body-z axis
phi_des   = asin(max(min(zb(1)*sin(psi_ref) - zb(2)*cos(psi_ref), 1), -1));
theta_des = atan2(zb(1)*cos(psi_ref) + zb(2)*sin(psi_ref), zb(3));
phi_des   = max(min(phi_des,   P.tilt_max), -P.tilt_max);
theta_des = max(min(theta_des, P.tilt_max), -P.tilt_max);
T = F_req' * zb;                                % thrust along body z
end


% ------------------------- attitude_ctrl -------------------------
function tau = attitude_ctrl(eul, om, phi_des, theta_des, psi_ref, P)
% ATTITUDE_CTRL  Inner-loop PD with gyroscopic feed-forward. Identical in both
%   controllers (only the outer loop / feed-forward is switched).
e = [ phi_des   - eul(1); ...
      theta_des - eul(2); ...
      wrap_angle(psi_ref - eul(3)) ];
tau = P.I*(P.Kp_att(:).*e - P.Kd_att(:).*om) + cross(om, P.I*om);
end

function a = wrap_angle(a)
a = mod(a+pi, 2*pi) - pi;
end


% ------------------------- run_sim -------------------------
function log = run_sim(scn, mode, seed, P)
% RUN_SIM  One closed-loop simulation of the delivery mission.
%   scn  : 'calm' | 'moderate' | 'severe'
%   mode : 'baseline' (never engage robust) | 'robust' (always engage) |
%          'supervisor' (engage the robust controller on demand)
%   The robust controller is P.robust_type: 'indi' (sensor-based incremental,
%   default) or 'dobc' (disturbance-observer feed-forward). A disturbance
%   observer runs continuously as the supervisor's indicator in both cases.
%   The outer loop is inlined here so the two controllers can be blended
%   bumplessly; the inner attitude loop (attitude_ctrl) is shared.
dt = P.dt; N = round(P.Tend/dt);

x      = zeros(12,1);
u_app  = [P.m0*P.g; 0; 0; 0];
p_obs  = zeros(3,1);  d_hat = zeros(3,1);
alpha  = 0; mstate = 0; t_last = -10; s_f = 0; nsw = 0;
F_thr_prev = [0;0;P.m0*P.g];           % world thrust force applied last step

W = make_wind(scn, seed);
rng(seed+777, 'twister');

log.t=zeros(N,1); log.p=zeros(N,3); log.pref=zeros(N,3);
log.dhat=zeros(N,3); log.alpha=zeros(N,1); log.mode=zeros(N,1);
log.T=zeros(N,1); log.sat=zeros(N,1); log.eul=zeros(N,3);

for i = 1:N
    t = (i-1)*dt;
    if t < P.t_drop
        mass = P.m0;                 I = P.I;
    else
        mass = P.m0 - P.m_payload;   I = P.I*((P.m0-P.m_payload)/P.m0);
    end
    [pref,vref,aref,psi_ref] = reference_traj(t);
    pref=pref(:); vref=vref(:); aref=aref(:);
    p=x(1:3); v=x(4:6); eul=x(7:9); om=x(10:12);
    Fext = wind_at(W,t);

    % measured world acceleration (accelerometer) for INDI
    a_meas = quad_deriv(x,u_app,mass,I,P,Fext);
    a_meas = a_meas(4:6) + P.acc_noise*randn(3,1);

    % ---- disturbance observer: indicator only ----
    vmeas  = v + (P.acc_noise/10)*randn(3,1);
    R      = Rzyx(eul(1),eul(2),eul(3));
    F_known= R*[0;0;u_app(1)] + [0;0;-P.m0*P.g] - P.kd*vmeas;
    d_hat  = p_obs + P.L_dob(:).*(P.m0*vmeas);
    p_obs  = p_obs + dt*(-P.L_dob(:).*(F_known + d_hat));

    % ---- supervisor ----
    s = norm(d_hat); bet = exp(-dt/P.s_lpf); s_f = bet*s_f + (1-bet)*s;
    switch mode
        case 'baseline', target = 0;
        case 'robust',   target = 1;
        otherwise
            if mstate==0 && s_f>P.thr_hi && (t-t_last)>P.dwell
                mstate=1; t_last=t; nsw=nsw+1;
            elseif mstate==1 && s_f<P.thr_lo && (t-t_last)>P.dwell
                mstate=0; t_last=t; nsw=nsw+1;
            end
            target = mstate;
    end
    alpha = alpha + (target-alpha)*(dt/P.tau_blend);

    % ---- outer loop: baseline force, robust force, bumpless blend ----
    e = pref-p; ev = vref-v;
    a_des = aref + P.Kp_pos(:).*e + P.Kd_pos(:).*ev;        % desired world accel
    F_pid = P.m0*(a_des + [0;0;P.g]);                       % baseline required force
    if strcmp(P.robust_type,'indi')
        F_rob = F_thr_prev + P.m0*(a_des - a_meas);         % incremental (INDI)
        F_rob = max(min(F_rob, F_pid+P.indi_clip), F_pid-P.indi_clip);
    else                                                    % DOBC
        F_rob = F_pid - max(min(d_hat, P.dff_max), -P.dff_max);
    end
    F_req = (1-alpha)*F_pid + alpha*F_rob;

    [T,phi_des,theta_des] = force_to_cmd(F_req, psi_ref, P);
    tau = attitude_ctrl(eul,om,phi_des,theta_des,psi_ref,P);
    u_cmd = [T; tau];

    % ---- saturation ----
    sat = 0;
    if u_cmd(1)>P.Tmax, u_cmd(1)=P.Tmax; sat=1; end
    if u_cmd(1)<0,       u_cmd(1)=0;     sat=1; end
    for j=1:3
        if abs(u_cmd(1+j))>P.taumax(j)
            u_cmd(1+j)=sign(u_cmd(1+j))*P.taumax(j); sat=1;
        end
    end

    % ---- actuator lag, then record applied thrust force ----
    au = exp(-dt/P.tau_act);
    u_app = au*u_app + (1-au)*u_cmd;
    F_thr_prev = Rzyx(eul(1),eul(2),eul(3))*[0;0;u_app(1)];

    % ---- RK4 integration with true mass and continuous wind ----
    f1 = quad_deriv(x,           u_app, mass, I, P, wind_at(W,t));
    f2 = quad_deriv(x+0.5*dt*f1, u_app, mass, I, P, wind_at(W,t+0.5*dt));
    f3 = quad_deriv(x+0.5*dt*f2, u_app, mass, I, P, wind_at(W,t+0.5*dt));
    f4 = quad_deriv(x+dt*f3,     u_app, mass, I, P, wind_at(W,t+dt));
    x  = x + dt/6*(f1+2*f2+2*f3+f4);

    log.t(i)=t; log.p(i,:)=p'; log.pref(i,:)=pref';
    log.dhat(i,:)=d_hat'; log.alpha(i)=alpha;
    log.mode(i)= strcmp(mode,'robust') + strcmp(mode,'supervisor')*mstate;
    log.T(i)=u_app(1); log.sat(i)=sat; log.eul(i,:)=eul';
end
log.nsw = nsw;
end


% ------------------------- compute_metrics -------------------------
function m = compute_metrics(log)
% COMPUTE_METRICS  Performance metrics from one run's log.
err = sqrt(sum((log.p - log.pref).^2, 2));
m.rmse   = sqrt(mean(err.^2));
m.maxdev = max(err);

% disturbance-recovery time: within the 13-26 s window, time from the error
% peak until it falls back below 0.3 m.
w  = (log.t>=13) & (log.t<=26);
tt = log.t(w); ee = err(w);
[~,ipk] = max(ee);
m.recovery = NaN;
for j = ipk:numel(ee)
    if ee(j) < 0.3
        m.recovery = tt(j) - tt(ipk); break;
    end
end

m.effort = sum(abs(diff(log.T)));     % thrust-activity proxy (actuator wear)
m.sat    = mean(log.sat)*100;         % % of steps with any saturation
m.nsw    = log.nsw;                   % number of controller switches
end


% ------------------------- run_experiment -------------------------
function R = run_experiment(nseeds)
% RUN_EXPERIMENT  Monte-Carlo over scenario battery x controllers.
%   Prints a mean +/- 95% CI table and paired significance tests, returns raw
%   results in R. The robust controller is set by quad_params (P.robust_type).
%   Usage:  R = run_experiment(30);
if nargin<1, nseeds = 30; end
P = quad_params();
scn   = {'calm','moderate','severe'};
modes = {'baseline','robust','supervisor'};
tval  = 2.045;   % approx t_{0.975} for n~30 (95% CI half-width = tval*std/sqrt(n))

R = struct();
fprintf('\nMonte-Carlo, robust=%s, %d seeds (mean +/- 95%% CI)\n', P.robust_type, nseeds);
fprintf('%-9s %-11s %16s %16s %16s %6s\n','scenario','mode','RMSE(m)','maxdev(m)','effort','sw');
for a = 1:numel(scn)
    for b = 1:numel(modes)
        rr=zeros(nseeds,1); dd=rr; ee=rr; ss=rr;
        for s = 1:nseeds
            m = compute_metrics(run_sim(scn{a}, modes{b}, s, P));
            rr(s)=m.rmse; dd(s)=m.maxdev; ee(s)=m.effort; ss(s)=m.nsw;
        end
        key = sprintf('%s_%s', scn{a}, modes{b});
        R.(key).rmse=rr; R.(key).maxdev=dd; R.(key).effort=ee; R.(key).nsw=ss;
        ci = @(x) tval*std(x)/sqrt(numel(x));
        fprintf('%-9s %-11s %7.3f+/-%4.3f %7.3f+/-%4.3f %7.1f+/-%4.1f %6.1f\n', ...
            scn{a}, modes{b}, mean(rr),ci(rr), mean(dd),ci(dd), mean(ee),ci(ee), mean(ss));
    end
end

% ---- paired significance tests per scenario ----
fprintf('\nPaired tests per scenario:\n');
for a = 1:numel(scn)
    bl = R.(sprintf('%s_baseline',scn{a})).rmse;
    sv = R.(sprintf('%s_supervisor',scn{a})).rmse;
    rE = R.(sprintf('%s_robust',scn{a})).effort;
    sE = R.(sprintf('%s_supervisor',scn{a})).effort;
    effred = 100*(mean(rE)-mean(sE))/mean(rE);
    if exist('ttest','file')    % Statistics Toolbox
        [~,p1] = ttest(bl,sv); [~,p2] = ttest(rE,sE);
        fprintf('  %-9s RMSE supervisor<baseline p=%.1e ; effort supervisor<robust by %.0f%% p=%.1e\n',...
                scn{a}, p1, effred, p2);
    else                        % fall back to the t-statistic
        t1 = paired_t(bl,sv); t2 = paired_t(rE,sE);
        fprintf('  %-9s RMSE supervisor<baseline t=%.1f ; effort supervisor<robust by %.0f%% t=%.1f\n',...
                scn{a}, t1, effred, t2);
    end
end
end

function t = paired_t(a,b)
d=a(:)-b(:); n=numel(d); t=mean(d)/(std(d)/sqrt(n));
end


% ------------------------- draw_quad -------------------------
function draw_quad(pos, R, scale, color)
% DRAW_QUAD  Draw an X-config quadrotor with FILLED rotor discs at position pos
%   (3x1) and body-to-world rotation R, on the current 3-D axes. Used by
%   plot_mission_3d / animate_mission so the drone looks solid, not a stick.
if nargin<3, scale=1.0; end
if nargin<4, color=[0.12 0.23 0.39]; end
disk = [0.18 0.42 0.64];
L = 0.6*scale;
arms = [ L L 0; -L -L 0; -L L 0; L -L 0 ];    % 4 diagonal arm tips (X-config)
w = arms*R' + pos(:)';
plot3([w(1,1) w(2,1)],[w(1,2) w(2,2)],[w(1,3) w(2,3)],'Color',color,'LineWidth',3);
plot3([w(3,1) w(4,1)],[w(3,2) w(4,2)],[w(3,3) w(4,3)],'Color',color,'LineWidth',3);
th = linspace(0,2*pi,24)';
for a = 1:4
    ring = [0.30*scale*cos(th), 0.30*scale*sin(th), zeros(numel(th),1)]*R' + w(a,:);
    fill3(ring(:,1),ring(:,2),ring(:,3), disk, 'FaceAlpha',0.35, ...
          'EdgeColor',color,'LineWidth',1.1);
end
plot3(pos(1),pos(2),pos(3),'.','Color',color,'MarkerSize',14);
end


% ------------------------- plot_mission_3d -------------------------
function plot_mission_3d(scn, seed)
% PLOT_MISSION_3D  3-D flight path of the supervised controller flying the
%   delivery mission, with the quadrotor drawn at several instants and the
%   payload-drop point marked. The dashed reference path is drawn on top so it
%   remains visible where the actual path tracks it closely. Saves mission_3d.png.
%   Usage:  plot_mission_3d('severe', 1);
if nargin<1, scn='severe'; end
if nargin<2, seed=1; end
P  = quad_params();
lg = run_sim(scn, 'supervisor', seed, P);
p = lg.p; pref = lg.pref; eul = lg.eul; t = lg.t;

figure('Name','3D flight path','Position',[100 100 820 620]); hold on; grid on;
idx = round(linspace(1,numel(t),6));
for k = idx
    draw_quad(p(k,:), Rzyx(eul(k,1),eul(k,2),eul(k,3)), 1.5, [0.12 0.23 0.39]);
end
h_act = plot3(p(:,1),  p(:,2),  p(:,3),  'Color',[0.75 0.22 0.17],'LineWidth',1.8);
h_ref = plot3(pref(:,1),pref(:,2),pref(:,3),'k--','LineWidth',2.4);
[~,kd] = min(abs(t-P.t_drop));
plot3(p(kd,1),p(kd,2),p(kd,3),'r.','MarkerSize',26);
text(p(kd,1),p(kd,2),p(kd,3)+0.6,'payload drop','Color','r');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); view(-58,24); axis equal;
title(sprintf('Delivery mission: 3D flight path (%s wind, supervised)', scn));
legend([h_ref h_act],{'reference path','actual (supervised)'},'Location','northeast');
print(gcf,'mission_3d.png','-dpng','-r135');
fprintf('saved mission_3d.png\n');
end


% ------------------------- animate_mission -------------------------
function animate_mission(scn, seed, savegif)
% ANIMATE_MISSION  Animate the supervised quadrotor flying the delivery
%   mission, with a reference/actual legend. Set savegif=true to write
%   mission.gif (useful for slides).
%   Usage:  animate_mission('severe', 1, true);
if nargin<1, scn='severe'; end
if nargin<2, seed=1; end
if nargin<3, savegif=false; end
P  = quad_params();
lg = run_sim(scn, 'supervisor', seed, P);
p = lg.p; eul = lg.eul; t = lg.t; pref = lg.pref;

fig = figure('Name','Mission animation','Position',[100 100 720 560]);
step = max(1, round(numel(t)/120)); first = true;
for k = 1:step:numel(t)
    clf; hold on; grid on; view(-58,20);
    h_ref = plot3(pref(:,1),pref(:,2),pref(:,3),'k--','LineWidth',1.5);
    h_act = plot3(p(1:k,1),p(1:k,2),p(1:k,3),'Color',[0.75 0.22 0.17],'LineWidth',1.6);
    draw_quad(p(k,:), Rzyx(eul(k,1),eul(k,2),eul(k,3)), 1.6, [0.12 0.23 0.39]);
    if t(k) >= P.t_drop, text(0,0,9,'payload dropped','Color','r'); end
    xlim([-2 13]); ylim([-3 3]); zlim([0 7]);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    legend([h_ref h_act],{'reference path','actual path'},'Location','northeast');
    title(sprintf('Delivery quadrotor   t = %4.1f s', t(k))); drawnow;
    if savegif
        frame = getframe(fig); im = frame2im(frame); [A,map] = rgb2ind(im,256);
        if first
            imwrite(A,map,'mission.gif','gif','LoopCount',Inf,'DelayTime',0.06); first = false;
        else
            imwrite(A,map,'mission.gif','gif','WriteMode','append','DelayTime',0.06);
        end
    end
end
if savegif, fprintf('saved mission.gif\n'); end
end


% ------------------------- draw_schematic -------------------------
function draw_schematic()
% DRAW_SCHEMATIC  Clean, labelled top-view schematic of the X-config quadrotor:
%   filled rotor discs, motor hubs, colour-coded CW/CCW spin arrows, body axes
%   and a FRONT indicator. Saves quad_schematic.png. Self-contained.
figure('Name','Quadrotor schematic','Position',[100 100 720 720]); hold on; axis equal off;
navy=[0.12 0.23 0.39]; steel=[0.18 0.42 0.64]; disk=[0.86 0.91 0.95];
cwC=[0.70 0.28 0.12]; ccwC=[0.12 0.48 0.30];
armL=1.0; rotR=0.42; spins={'CCW','CW'};
% rotor layout: x, y, number, isCW
rot = [ armL  armL 1 1; -armL  armL 2 0; -armL -armL 3 1;  armL -armL 4 0];
cols = {cwC, ccwC, cwC, ccwC};
th = linspace(0,2*pi,60);

% arms
for i=1:4, plot([0 rot(i,1)],[0 rot(i,2)],'Color',navy,'LineWidth',7); end
% central body
rectangle('Position',[-0.3 -0.3 0.6 0.6],'Curvature',0.25, ...
          'FaceColor',navy,'EdgeColor','k','LineWidth',1.2);
text(0,-0.64,sprintf('flight controller\n(IMU, ESCs)'), ...
     'HorizontalAlignment','center','VerticalAlignment','top','FontSize',9);

% rotor discs, hubs, spin arrows, labels
for i=1:4
    x=rot(i,1); y=rot(i,2); isCW=rot(i,4); col=cols{i};
    fill(x+rotR*cos(th), y+rotR*sin(th), disk, 'EdgeColor',steel,'LineWidth',2);
    plot(x,y,'k.','MarkerSize',18);
    r=rotR*0.6; span=deg2rad(280);
    if isCW, a=linspace(deg2rad(250), deg2rad(250)-span, 40);
    else,    a=linspace(deg2rad(-70), deg2rad(-70)+span, 40); end
    plot(x+r*cos(a), y+r*sin(a), 'Color',col,'LineWidth',2);
    ae=a(end); tang=[-sin(ae) cos(ae)]; if isCW, tang=-tang; end
    quiver(x+r*cos(ae), y+r*sin(ae), tang(1)*0.16, tang(2)*0.16, 0, ...
           'Color',col,'LineWidth',2,'MaxHeadSize',5,'AutoScale','off');
    text(x*1.44, y*1.44, sprintf('Rotor %d\n%s',rot(i,3),spins{isCW+1}), ...
         'HorizontalAlignment','center','Color',col,'FontWeight','bold','FontSize',10);
end

% FRONT indicator
quiver(0,1.35,0,0.42,0,'k','LineWidth',2,'MaxHeadSize',2,'AutoScale','off');
text(0,1.9,'FRONT','HorizontalAlignment','center','FontWeight','bold','FontSize',11);
% body axes
quiver(0,0,0.95,0,0,'Color',[0.75 0.22 0.17],'LineWidth',2.4,'MaxHeadSize',1.2,'AutoScale','off');
quiver(0,0,0,0.95,0,'Color',[0.15 0.68 0.38],'LineWidth',2.4,'MaxHeadSize',1.2,'AutoScale','off');
text(1.02,0,'x_B (roll)','Color',[0.75 0.22 0.17],'FontSize',11,'VerticalAlignment','middle');
text(0.04,1.06,'y_B (pitch)','Color',[0.15 0.68 0.38],'FontSize',11);
plot(0,0,'o','MarkerEdgeColor',[0.17 0.24 0.62],'MarkerFaceColor','w','MarkerSize',10,'LineWidth',1.6);
plot(0,0,'.','Color',[0.17 0.24 0.62],'MarkerSize',9);
text(-0.12,0.34,'z_B (yaw, up)','Color',[0.17 0.24 0.62],'FontSize',9.5,'HorizontalAlignment','right');

hCW  = plot(nan,nan,'-','Color',cwC,'LineWidth',2.5);
hCCW = plot(nan,nan,'-','Color',ccwC,'LineWidth',2.5);
legend([hCW hCCW],{'CW rotor spin','CCW rotor spin'},'Location','southoutside','Orientation','horizontal','Box','off');
xlim([-2.1 2.1]); ylim([-2.3 2.1]);
title('Quadrotor model — X configuration (top view)','FontSize',13,'FontWeight','bold');
print(gcf,'quad_schematic.png','-dpng','-r140');
fprintf('saved quad_schematic.png\n');
end


% ------------------------- make_plots -------------------------
function make_plots(scn, seed)
% MAKE_PLOTS  Representative-run figures for the supervised controller.
%   Usage:  make_plots('severe', 1);
if nargin<1, scn='severe'; end
if nargin<2, seed=1; end
P  = quad_params();
lg = run_sim(scn, 'supervisor', seed, P);
t  = lg.t;
err= sqrt(sum((lg.p-lg.pref).^2,2));

figure('Name','Representative run','Position',[100 100 760 760]);
subplot(3,1,1); hold on; grid on;
plot(t, lg.pref(:,1),'k--','LineWidth',1.2);
plot(t, lg.p(:,1),'b','LineWidth',1.2);
plot(t, lg.pref(:,3),'--','Color',[.5 .5 .5]);
plot(t, lg.p(:,3),'g','LineWidth',1.0);
yl=ylim; plot([P.t_drop P.t_drop], yl, 'r:');   % payload-drop marker
ylabel('position (m)');
legend('x ref','x actual','z ref','z actual','Location','east');
title(sprintf('Representative run: %s wind, supervisor', scn));

subplot(3,1,2); hold on; grid on;
plot(t, err,'b','LineWidth',1.2); plot([t(1) t(end)],[0.3 0.3],':','Color',[.6 .6 .6]);
ylabel('tracking error (m)');

subplot(3,1,3); hold on; grid on;
plot(t, lg.alpha,'m','LineWidth',1.4);
plot(t, sqrt(sum(lg.dhat.^2,2))/10,'c','LineWidth',1.0);
plot([t(1) t(end)],[P.thr_hi/10 P.thr_hi/10],':','Color',[.6 .6 .6]);
ylabel('supervisor'); xlabel('time (s)');
legend('blend \alpha (0=PID, 1=robust)','|d\_hat|/10 (indicator, N)','Location','east');

% save
print(gcf,'fig_run.png','-dpng','-r120');
fprintf('saved fig_run.png\n');
end

