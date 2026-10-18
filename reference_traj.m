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
