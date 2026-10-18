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
