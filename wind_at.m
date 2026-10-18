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
