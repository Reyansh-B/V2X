% platoon_twin_v3.m
% Monte Carlo comparison of gap-sensing schemes for a 6-vehicle CACC platoon.
%
% What changed vs platoon_twin_v2.m
%   * constant time-headway spacing (r + h v) and Ploeg-style CACC law
%   * GNSS error = common-mode (cancels in the gap) + receiver-specific part;
%     code-DGNSS (0.5 m) and RTK (2 cm) variants
%   * radar from a datasheet-class model (72 ms cycle, 0.1 m, 0.1 km/h)
%   * near-field ranging runs the ACTUAL estimator (nf_estimate.m) on simulated
%     phases at every received packet, incl. per-element calibration error
%   * ACC (no V2X) baseline; feedforward dropped after cfg.ff_timeout
%   * packet loss: none / burst during braking / Gilbert-Elliott
%   * stop-and-go manoeuvre so the gaps reach the 2-5 m near-field regime
% Needs: platoon_config, platoon_sim, leader_profile, nf_phase_meas,
%        nf_estimate, snr_linkbudget, nearfieldBounds.
clear; clc; close all;

Nruns = 20;
base = platoon_config();

% ---- schemes: {label, sensor, field-overrides} ----
schemes = {
  'ideal',          'ideal',  {};
  'ACC (radar)',    'acc',    {};
  'CACC radar',     'radar',  {};
  'CACC GNSS-DGNSS','gnss',   {'gnss.sig_rel', 0.5};
  'CACC GNSS-RTK',  'gnss',   {'gnss.sig_rel', 0.02};
  'CACC NF',        'nf',     {'nf.sig_cal', 0};
  'CACC NF cal3deg','nf',     {'nf.sig_cal', 3*pi/180};
};
% ---- packet-loss scenarios ----
scen = {
  'A: no loss',                 'none';
  'B: 1 s burst while braking', 'burst';
  'C: Gilbert-Elliott bursty',  'ge';
};

fid = fopen('results_v3.csv', 'w');
fprintf(fid, 'scenario,scheme,P_collision,min_gap_mean,min_gap_worst,rms_err_last,max_err,gamma_a,min_ttc\n');
for s = 1:size(scen,1)
  fprintf('\n%s  (h = %.2f s, %d runs, %s)\n', scen{s,1}, base.h, Nruns, base.scenario);
  fprintf('  %-16s %6s %8s %8s %8s %8s %7s\n', 'scheme', 'P(col)', 'mingap', 'worst', 'RMSe', 'max|e|', 'gamma');
  for m = 1:size(schemes,1)
    cfg = base; cfg.loss.type = scen{s,2}; cfg.loss.burst = [6 7];
    ov = schemes{m,3};
    for q = 1:2:numel(ov), pp = strsplit(ov{q}, '.'); cfg = setfield(cfg, pp{:}, ov{q+1}); end
    R = zeros(Nruns, 6);
    for r = 1:Nruns
      o = platoon_sim(cfg, schemes{m,2}, r);
      R(r,:) = [o.collision, o.min_gap, o.rms_err(end), max(o.max_err), o.gamma_a, max(o.min_ttc, 0)];
    end
    fprintf('  %-16s %6.2f %8.2f %8.2f %8.3f %8.2f %7.3f\n', schemes{m,1}, mean(R(:,1)), ...
      mean(R(:,2)), min(R(:,2)), mean(R(:,3)), mean(R(:,4)), mean(R(:,5)));
    fprintf(fid, '%s,%s,%.3f,%.3f,%.3f,%.4f,%.3f,%.4f,%.2f\n', scen{s,1}, schemes{m,1}, mean(R(:,1)), ...
      mean(R(:,2)), min(R(:,2)), mean(R(:,3)), mean(R(:,4)), mean(R(:,5)), min(R(:,6)));
  end
end
fclose(fid);

%% Figure: spacing error of the last follower, burst loss during braking
show = [3 4 6 7];
cols = lines(numel(show));
figure('Color', 'w', 'Position', [100 100 800 450]); hold on;
for j = 1:numel(show)
  m = show(j);
  cfg = base; cfg.loss.type = 'burst'; cfg.loss.burst = [6 7];
  ov = schemes{m,3};
  for q = 1:2:numel(ov), pp = strsplit(ov{q}, '.'); cfg = setfield(cfg, pp{:}, ov{q+1}); end
  o = platoon_sim(cfg, schemes{m,2}, 1);
  plot(o.t, o.err(end,:), 'Color', cols(j,:), 'LineWidth', 1.6, 'DisplayName', schemes{m,1});
end
yl = ylim; patch([6 7 7 6], [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, ...
  'EdgeColor', 'none', 'DisplayName', 'V2X packet loss');
grid on; box on; xlabel('Time [s]'); ylabel('Spacing error, last follower [m]');
title(sprintf('Stop-and-go, h = %.1f s, 1 s V2X loss during braking', base.h));
legend('Location', 'best');
print(gcf, 'platoon_v3.png', '-dpng', '-r150');

