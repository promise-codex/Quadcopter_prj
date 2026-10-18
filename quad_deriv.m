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
