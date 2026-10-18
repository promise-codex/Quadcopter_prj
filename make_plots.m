function make_plots(scn, seed)
% MAKE_PLOTS  Representative-run figures for the supervised controller.
%   Usage:  make_plots('severe', 1);
if nargin<1, scn='severe'; end
if nargin<2, seed=1; end
P  = quad_params();
lg = run_sim(scn, 'supervisor', seed, P);
t  = lg.t;
err= sqrt(sum((lg.p-lg.pref).^2,2));

figure('Name','Representative run','Position',[100 100 760 760]);
subplot(3,1,1); hold on; grid on;
plot(t, lg.pref(:,1),'k--','LineWidth',1.2);
plot(t, lg.p(:,1),'b','LineWidth',1.2);
plot(t, lg.pref(:,3),'--','Color',[.5 .5 .5]);
plot(t, lg.p(:,3),'g','LineWidth',1.0);
yl=ylim; plot([P.t_drop P.t_drop], yl, 'r:');   % payload-drop marker
ylabel('position (m)');
legend('x ref','x actual','z ref','z actual','Location','east');
title(sprintf('Representative run: %s wind, supervisor', scn));

subplot(3,1,2); hold on; grid on;
plot(t, err,'b','LineWidth',1.2); plot([t(1) t(end)],[0.3 0.3],':','Color',[.6 .6 .6]);
ylabel('tracking error (m)');

subplot(3,1,3); hold on; grid on;
plot(t, lg.alpha,'m','LineWidth',1.4);
plot(t, sqrt(sum(lg.dhat.^2,2))/10,'c','LineWidth',1.0);
plot([t(1) t(end)],[P.thr_hi/10 P.thr_hi/10],':','Color',[.6 .6 .6]);
ylabel('supervisor'); xlabel('time (s)');
legend('blend \alpha (0=PID, 1=robust)','|d\_hat|/10 (indicator, N)','Location','east');

% save
print(gcf,'fig_run.png','-dpng','-r120');
fprintf('saved fig_run.png\n');
end
