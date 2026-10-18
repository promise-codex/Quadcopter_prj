function R = run_experiment(nseeds)
% RUN_EXPERIMENT  Monte-Carlo over scenario battery x controllers.
%   Prints a mean +/- 95% CI table and paired significance tests, returns raw
%   results in R. The robust controller is set by quad_params (P.robust_type).
%   Usage:  R = run_experiment(30);
if nargin<1, nseeds = 30; end
P = quad_params();
scn   = {'calm','moderate','severe'};
modes = {'baseline','robust','supervisor'};
tval  = 2.045;   % approx t_{0.975} for n~30 (95% CI half-width = tval*std/sqrt(n))

R = struct();
fprintf('\nMonte-Carlo, robust=%s, %d seeds (mean +/- 95%% CI)\n', P.robust_type, nseeds);
fprintf('%-9s %-11s %16s %16s %16s %6s\n','scenario','mode','RMSE(m)','maxdev(m)','effort','sw');
for a = 1:numel(scn)
    for b = 1:numel(modes)
        rr=zeros(nseeds,1); dd=rr; ee=rr; ss=rr;
        for s = 1:nseeds
            m = compute_metrics(run_sim(scn{a}, modes{b}, s, P));
            rr(s)=m.rmse; dd(s)=m.maxdev; ee(s)=m.effort; ss(s)=m.nsw;
        end
        key = sprintf('%s_%s', scn{a}, modes{b});
        R.(key).rmse=rr; R.(key).maxdev=dd; R.(key).effort=ee; R.(key).nsw=ss;
        ci = @(x) tval*std(x)/sqrt(numel(x));
        fprintf('%-9s %-11s %7.3f+/-%4.3f %7.3f+/-%4.3f %7.1f+/-%4.1f %6.1f\n', ...
            scn{a}, modes{b}, mean(rr),ci(rr), mean(dd),ci(dd), mean(ee),ci(ee), mean(ss));
    end
end

% ---- paired significance tests per scenario ----
fprintf('\nPaired tests per scenario:\n');
for a = 1:numel(scn)
    bl = R.(sprintf('%s_baseline',scn{a})).rmse;
    sv = R.(sprintf('%s_supervisor',scn{a})).rmse;
    rE = R.(sprintf('%s_robust',scn{a})).effort;
    sE = R.(sprintf('%s_supervisor',scn{a})).effort;
    effred = 100*(mean(rE)-mean(sE))/mean(rE);
    if exist('ttest','file')    % Statistics Toolbox
        [~,p1] = ttest(bl,sv); [~,p2] = ttest(rE,sE);
        fprintf('  %-9s RMSE supervisor<baseline p=%.1e ; effort supervisor<robust by %.0f%% p=%.1e\n',...
                scn{a}, p1, effred, p2);
    else                        % fall back to the t-statistic
        t1 = paired_t(bl,sv); t2 = paired_t(rE,sE);
        fprintf('  %-9s RMSE supervisor<baseline t=%.1f ; effort supervisor<robust by %.0f%% t=%.1f\n',...
                scn{a}, t1, effred, t2);
    end
end
end

function t = paired_t(a,b)
d=a(:)-b(:); n=numel(d); t=mean(d)/(std(d)/sqrt(n));
end
