% nf_calibration_study.m
% How much per-element phase calibration error can near-field ranging tolerate?
% Longitudinal (gap) RMSE of the estimator vs range, for several calibration
% error levels, compared with the CRLB (longitudinal bound, link-budget SNR).
% A NEW calibration error vector is drawn every trial (i.e. averaged over vehicles).
clear; clc; close all; rng(3);

cfg = platoon_config(); nf = cfg.nf;
d_list   = [2 3 5 7 10 15 20 30];
cal_deg  = [0 0.5 1 2 5];
Nt = 300;

rmse = zeros(numel(cal_deg), numel(d_list)); crlb = zeros(1, numel(d_list));
rmse_quad = zeros(1, numel(d_list));
for j = 1:numel(d_list)
  d = d_list(j);
  snr = snr_linkbudget(d, nf.lam, nf.margin_dB);
  o = nearfieldBounds(nf.theta, d, nf.x, nf.Mp, nf.Np, snr, nf.eta, nf.fc, 'paper');
  crlb(j) = o.longitudinal;
  for c = 1:numel(cal_deg)
    e = zeros(Nt,1); eq = e;
    for t = 1:Nt
      cal = cal_deg(c)*pi/180*randn(1, nf.Ka);
      ms = nf_phase_meas(nf.theta, d, nf.x, snr, nf.eta, nf.Mp, nf.Np, nf.lam, cal);
      [th, dh] = nf_estimate(ms, nf.x, nf.lam, nf.n_gn, nf.Apinv);
      e(t) = dh*cos(th) - d;
      if c == 1
        [th0, d0] = nf_estimate(ms, nf.x, nf.lam, 0, nf.Apinv);
        eq(t) = d0*cos(th0) - d;
      end
    end
    rmse(c,j) = sqrt(mean(e.^2));
    if c == 1, rmse_quad(j) = sqrt(mean(eq.^2)); end
  end
  fprintf('d = %4.1f m | SNR %5.1f dB | CRLB %.2e | quad-only %.2e | ', d, 10*log10(snr), crlb(j), rmse_quad(j));
  fprintf('%.2e ', rmse(:,j)); fprintf('(cal = %s deg)\n', mat2str(cal_deg));
end

figure('Color', 'w', 'Position', [100 100 700 480]);
semilogy(d_list, crlb, 'k--', 'LineWidth', 2, 'DisplayName', 'CRLB (longitudinal)'); hold on;
semilogy(d_list, rmse_quad, 'k:', 'LineWidth', 1.5, 'DisplayName', 'quadratic fit only, no cal. error');
cols = lines(numel(cal_deg));
for c = 1:numel(cal_deg)
  semilogy(d_list, rmse(c,:), 'o-', 'Color', cols(c,:), 'LineWidth', 1.5, ...
    'DisplayName', sprintf('estimator, cal. error %.1f deg', cal_deg(c)));
end
plot([d_list(1) d_list(end)], [0.1 0.1], 'r-.', 'LineWidth', 1, 'DisplayName', 'radar class (0.1 m)');
grid on; box on; xlabel('Gap d [m]'); ylabel('Gap RMSE [m]');
title('Near-field ranging vs per-element phase calibration error');
legend('Location', 'northwest');
print(gcf, 'nf_calibration.png', '-dpng', '-r150');

