function animate_mission(scn, seed, savegif)
% ANIMATE_MISSION  Animate the supervised quadrotor flying the delivery
%   mission, with a reference/actual legend. Set savegif=true to write
%   mission.gif (useful for slides).
%   Usage:  animate_mission('severe', 1, true);
if nargin<1, scn='severe'; end
if nargin<2, seed=1; end
if nargin<3, savegif=false; end
P  = quad_params();
lg = run_sim(scn, 'supervisor', seed, P);
p = lg.p; eul = lg.eul; t = lg.t; pref = lg.pref;

fig = figure('Name','Mission animation','Position',[100 100 720 560]);
step = max(1, round(numel(t)/120)); first = true;
for k = 1:step:numel(t)
    clf; hold on; grid on; view(-58,20);
    h_ref = plot3(pref(:,1),pref(:,2),pref(:,3),'k--','LineWidth',1.5);
    h_act = plot3(p(1:k,1),p(1:k,2),p(1:k,3),'Color',[0.75 0.22 0.17],'LineWidth',1.6);
    draw_quad(p(k,:), Rzyx(eul(k,1),eul(k,2),eul(k,3)), 1.6, [0.12 0.23 0.39]);
    if t(k) >= P.t_drop, text(0,0,9,'payload dropped','Color','r'); end
    xlim([-2 13]); ylim([-3 3]); zlim([0 7]);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    legend([h_ref h_act],{'reference path','actual path'},'Location','northeast');
    title(sprintf('Delivery quadrotor   t = %4.1f s', t(k))); drawnow;
    if savegif
        frame = getframe(fig); im = frame2im(frame); [A,map] = rgb2ind(im,256);
        if first
            imwrite(A,map,'mission.gif','gif','LoopCount',Inf,'DelayTime',0.06); first = false;
        else
            imwrite(A,map,'mission.gif','gif','WriteMode','append','DelayTime',0.06);
        end
    end
end
if savegif, fprintf('saved mission.gif\n'); end
end
