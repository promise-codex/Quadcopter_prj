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
