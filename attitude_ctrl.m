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
