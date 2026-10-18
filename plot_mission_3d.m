function plot_mission_3d(scn, seed)
% PLOT_MISSION_3D  3-D flight path of the supervised controller flying the
%   delivery mission, with the quadrotor drawn at several instants and the
%   payload-drop point marked. The dashed reference path is drawn on top so it
%   remains visible where the actual path tracks it closely. Saves mission_3d.png.
%   Usage:  plot_mission_3d('severe', 1);
if nargin<1, scn='severe'; end
if nargin<2, seed=1; end
P  = quad_params();
lg = run_sim(scn, 'supervisor', seed, P);
p = lg.p; pref = lg.pref; eul = lg.eul; t = lg.t;

figure('Name','3D flight path','Position',[100 100 820 620]); hold on; grid on;
idx = round(linspace(1,numel(t),6));
for k = idx
    draw_quad(p(k,:), Rzyx(eul(k,1),eul(k,2),eul(k,3)), 1.5, [0.12 0.23 0.39]);
end
h_act = plot3(p(:,1),  p(:,2),  p(:,3),  'Color',[0.75 0.22 0.17],'LineWidth',1.8);
h_ref = plot3(pref(:,1),pref(:,2),pref(:,3),'k--','LineWidth',2.4);
[~,kd] = min(abs(t-P.t_drop));
plot3(p(kd,1),p(kd,2),p(kd,3),'r.','MarkerSize',26);
text(p(kd,1),p(kd,2),p(kd,3)+0.6,'payload drop','Color','r');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); view(-58,24); axis equal;
title(sprintf('Delivery mission: 3D flight path (%s wind, supervised)', scn));
legend([h_ref h_act],{'reference path','actual (supervised)'},'Location','northeast');
print(gcf,'mission_3d.png','-dpng','-r135');
fprintf('saved mission_3d.png\n');
end
