function R = Rzyx(phi, th, ps)
% RZYX  Body-to-world rotation matrix from ZYX Euler angles (roll phi, pitch th, yaw ps).
cphi=cos(phi); sphi=sin(phi); cth=cos(th); sth=sin(th); cps=cos(ps); sps=sin(ps);
R = [ cth*cps, sphi*sth*cps-cphi*sps, cphi*sth*cps+sphi*sps; ...
      cth*sps, sphi*sth*sps+cphi*cps, cphi*sth*sps-sphi*cps; ...
      -sth,    sphi*cth,              cphi*cth ];
end
