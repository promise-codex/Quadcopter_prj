function W = euler_rate_matrix(phi, th)
% EULER_RATE_MATRIX  Maps body rates [p;q;r] to Euler-angle rates for ZYX angles.
cphi=cos(phi); sphi=sin(phi); cth=cos(th); tth=tan(th);
W = [ 1, sphi*tth, cphi*tth; ...
      0, cphi,    -sphi; ...
      0, sphi/cth, cphi/cth ];
end
