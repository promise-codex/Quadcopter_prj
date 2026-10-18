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
