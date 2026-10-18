function draw_quad(pos, R, scale, color)
% DRAW_QUAD  Draw an X-config quadrotor with FILLED rotor discs at position pos
%   (3x1) and body-to-world rotation R, on the current 3-D axes. Used by
%   plot_mission_3d / animate_mission so the drone looks solid, not a stick.
if nargin<3, scale=1.0; end
if nargin<4, color=[0.12 0.23 0.39]; end
disk = [0.18 0.42 0.64];
L = 0.6*scale;
arms = [ L L 0; -L -L 0; -L L 0; L -L 0 ];    % 4 diagonal arm tips (X-config)
w = arms*R' + pos(:)';
plot3([w(1,1) w(2,1)],[w(1,2) w(2,2)],[w(1,3) w(2,3)],'Color',color,'LineWidth',3);
plot3([w(3,1) w(4,1)],[w(3,2) w(4,2)],[w(3,3) w(4,3)],'Color',color,'LineWidth',3);
th = linspace(0,2*pi,24)';
for a = 1:4
    ring = [0.30*scale*cos(th), 0.30*scale*sin(th), zeros(numel(th),1)]*R' + w(a,:);
    fill3(ring(:,1),ring(:,2),ring(:,3), disk, 'FaceAlpha',0.35, ...
          'EdgeColor',color,'LineWidth',1.1);
end
plot3(pos(1),pos(2),pos(3),'.','Color',color,'MarkerSize',14);
end
