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
