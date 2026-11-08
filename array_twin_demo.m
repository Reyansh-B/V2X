% array_twin_demo.m
% Digital twin of ONE near-field receive array, on a simulated packet stream.
% Needs nearfieldBounds.m, nf_phase.m, nf_fit.m, nf_coarse.m, twin_init.m, twin_step.m.
%
% Physical system (simulated): a 41-element lambda/2 ULA tracks a neighbour vehicle that sends a
% packet every 100 ms (spacing 32..48 m, lane change at 60 s). The array has HIDDEN imperfections:
%   * static per-channel phase bias, 3 deg rms     * slow random-walk drift of that bias
%   * a fault at 70 s: element 11 jumps by +60 deg  * interference bursts on 8 % of packets
% A known-position reference transmitter (e.g. a roadside unit) sends a beacon every 10 s.
%
% Four receivers on the same data:
%   raw     : plain fit, no correction
%   selfcal : twin, learns the bias from the neighbour's packets only
%   refcal  : twin + the reference beacons
%   oracle  : true bias known (best case)
%
% Expected result: the twin flags and masks the faulty element within a few packets with no
% false alarms, but self-calibration does NOT reduce position error -- the bias component that
% looks like a change of (theta, d) is unobservable without a reference. Figure 2 shows exactly
% which part is left over.
clear; clc; close all; rng(1);

%% Parameters (same physical layer as the rest of the repo)
fc = 5.9e9; c0 = 299792458; lam = c0/fc; kw = 2*pi/lam;
Ka = 41; xa = (0:Ka-1)*lam/2; xa = xa - mean(xa); kc = (Ka+1)/2;
keep = true(1, Ka); keep(kc) = false; x = xa(keep).';
Mp = 2; Np = 120; eta = 10^(6/10);
T = 120; Tg = 0.1;
bias_deg = 3; drift_deg = 0.01;
ref_period = 10; ref_xy = [4.4 25];          % beacon position [x lateral, y ahead] in m
fault_t = 70; fault_el = 11; fault_deg = 60;

%% 1. Generate the stream (what a real receiver would log)
tp = (1:round(T/Tg))*Tg;
yv = 40 + 8*sin(2*pi*tp/40);
s  = min(max((tp - 60)/4, 0), 1);  xv = 3.5*(3*s.^2 - 2*s.^3);
tr = 0.05 + (0:ref_period:T-ref_period);
t  = [tp tr]; isref = [false(size(tp)) true(size(tr))];
X  = [xv ref_xy(1)*ones(size(tr))];  Y = [yv ref_xy(2)*ones(size(tr))];
[t, o] = sort(t); isref = isref(o); X = X(o); Y = Y(o);
n = numel(t); th_t = atan2(X, Y); d_t = hypot(X, Y);

snr = arrayfun(@(dd) snr_lin(dd, lam), d_t);
burst = (rand(1, n) < 0.08) & ~isref;
snr = snr ./ 10.^((burst.*(8 + 17*rand(1, n)))/10);
snr = snr .* 10.^(randn(1, n)/10);
s2  = eta./(2*Mp*Np*snr);                                % per-element pilot-averaged phase variance

b = (bias_deg*pi/180)*randn(1, Ka) + cumsum((drift_deg*pi/180)*randn(n, Ka), 1);
b(t >= fault_t, fault_el) = b(t >= fault_t, fault_el) + fault_deg*pi/180;
bd_true = b(:, keep) - b(:, kc);                         % n x (Ka-1) differential bias
e = sqrt(s2(:)).*randn(n, Ka);
obs = zeros(n, Ka-1);
for k = 1:n
  obs(k, :) = angle(exp(1j*(nf_phase(th_t(k), d_t(k), x.', kw) + bd_true(k, :) + e(k, keep) - e(k, kc))));
end
D = struct('t', t, 'isref', isref, 'X', X, 'Y', Y, 'obs', obs, 's2', s2, 'snr', snr, ...
           'bd_true', bd_true, 'x', x, 'xa', xa, 'kw', kw, 'kc', kc, 'lam', lam, 'Ka', Ka, ...
           'Mp', Mp, 'Np', Np, 'eta', eta, 'fc', fc, 'ref', ref_xy, 'drift', drift_deg);
ipk = find(~isref & t < 55, 1, 'last');                  % snapshot for figure 2

%% 2. Run the four receivers
modes = {'raw', 'selfcal', 'refcal', 'oracle'};
for q = 1:numel(modes)
  fprintf('running %s ...\n', modes{q});
  [ro, rtw, rsn] = run_mode(modes{q}, D, ipk);
  R.(modes{q}) = struct('out', ro, 'tw', rtw, 'snap', rsn);
end

%% 3. Results
tt = R.raw.out(:, 1);
win = {'pre-fault (5 s .. fault)', tt >= 5 & tt < fault_t; 'post-fault (+5 s ..)', tt >= fault_t + 5};
fprintf('\nRMS 2-D position error [m]\n%-26s %9s %9s %9s %9s\n', 'segment', modes{:});
for w = 1:2
  fprintf('%-26s', win{w, 1});
  for q = 1:numel(modes), fprintf(' %9.4f', sqrt(mean(R.(modes{q}).out(win{w, 2}, 2).^2))); end
  fprintf('\n');
end
pre = win{1, 2};
fprintf('median(error / predicted PEB) pre-fault:');
for q = 1:numel(modes), fprintf('  %s %.1f', modes{q}, median(R.(modes{q}).out(pre, 2)./R.(modes{q}).out(pre, 4))); end
fprintf('\n   (~1-2 = the twin''s accuracy prediction holds; >>1 = hidden hardware error)\n');
for q = 2:3
  dt_ = R.(modes{q}).tw.detect;
  fa  = sum(dt_(:, 1) < fault_t);
  hit = dt_(dt_(:, 1) >= fault_t, :);
  if isempty(hit)
    fprintf('[%s] false alarms before fault: %d, fault NOT detected\n', modes{q}, fa);
  else
    el = hit(1, 2) + (hit(1, 2) >= kc);                  % reduced index -> array element
    fprintf('[%s] false alarms before fault: %d, fault flagged after %.1f s, element %d (true %d)\n', ...
            modes{q}, fa, hit(1, 1) - fault_t, el, fault_el);
  end
end

% Unobservable part of the bias at the snapshot: projection onto the (theta, d) tangent space
bsn = bd_true(ipk, :).';
dk  = sqrt(d_t(ipk)^2 + x.^2 - 2*d_t(ipk)*x*sin(th_t(ipk)));
G   = [kw*d_t(ipk)*x*cos(th_t(ipk))./dk, -kw*((d_t(ipk) - x*sin(th_t(ipk)))./dk - 1)];
Wm  = eye(Ka-1) - ones(Ka-1)/Ka;
absorbed = G*((G.'*Wm*G)\(G.'*Wm*bsn));
fprintf('bias energy absorbed by the position fit (unobservable without a reference): %.1f %%\n', ...
        100*sum(absorbed.^2)/sum(bsn.^2));

%% 4. Figures
figure('Color', 'w', 'Position', [80 80 1000 720]);
subplot(2, 1, 1);
cols = {[0.84 0.15 0.16], [1 0.5 0.05], [0.17 0.63 0.17], [0.12 0.47 0.71]};
lw = [2.5 0.8 0.9 0.6];
for q = 1:numel(modes)
  semilogy(tt, R.(modes{q}).out(:, 2), 'Color', cols{q}, 'LineWidth', lw(q)); hold on;
end
semilogy(tt, R.refcal.out(:, 4), 'k--', 'LineWidth', 1);
xline(fault_t, 'r:', 'LineWidth', 1.5);
grid on; ylabel('2-D position error [m]');
legend('raw (no twin)', 'twin, self-calibration (== raw until the fault)', 'twin + reference beacons', ...
       'oracle (true bias known)', 'twin''s predicted bound (PEB)', 'fault', 'Location', 'northwest');
title('Digital twin of one array: error vs time');
subplot(2, 1, 2);
semilogy(tt, R.raw.out(:, 3), 'Color', cols{1}); hold on;
semilogy(tt, R.selfcal.out(:, 3), 'Color', cols{2});
semilogy(tt, R.refcal.out(:, 3), 'Color', cols{3});
yline(1, 'k:'); xline(fault_t, 'r:', 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('health: \chi^2 / dof (1 = healthy)');
legend('raw', 'twin selfcal', 'twin refcal', 'Location', 'northwest');
print(gcf, 'twin_error_health.png', '-dpng', '-r150');

figure('Color', 'w', 'Position', [100 100 1000 650]);
ant = find(keep);
subplot(2, 1, 1);
plot(ant, bsn*180/pi, 'k.-', 'MarkerSize', 10); hold on;
plot(ant, R.selfcal.snap*180/pi, '--', 'Color', cols{2}, 'LineWidth', 1.2);
plot(ant, R.refcal.snap*180/pi, ':', 'Color', cols{3}, 'LineWidth', 2);
grid on; ylabel('phase bias [deg]');
legend('true bias (vs centre element)', 'twin selfcal estimate', 'twin refcal estimate', 'Location', 'best');
title(sprintf('Bias recovery at t = %.1f s', t(ipk)));
subplot(2, 1, 2);
plot(ant, absorbed*180/pi, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 6); hold on;
plot(ant, (bsn - R.selfcal.snap)*180/pi, '.-', 'Color', cols{2}, 'MarkerSize', 10);
plot(ant, (bsn - R.refcal.snap)*180/pi, '.-', 'Color', cols{3}, 'MarkerSize', 10);
grid on; xlabel('antenna index'); ylabel('remaining bias [deg]');
legend('predicted unobservable part', 'left over after selfcal', 'left over after refcal', 'Location', 'best');
title('Self-calibration leaves exactly the smooth (linear + quadratic) part');
print(gcf, 'twin_bias_recovery.png', '-dpng', '-r150');

%% ---------------- local functions ----------------
function [out, tw, snap] = run_mode(mode, D, ipk)
  n = numel(D.t); out = zeros(0, 4); snap = zeros(D.Ka-1, 1);
  tw = twin_init(D.Ka, D.drift); prev = [];
  allm = true(D.Ka-1, 1);
  for k = 1:n
    if D.isref(k)
      if strcmp(mode, 'refcal')                          % known-position beacon: direct bias update
        tw.P = tw.P + tw.q;
        rr = angle(exp(1j*(D.obs(k, :).' - tw.bd - nf_phase(atan2(D.ref(1), D.ref(2)), hypot(D.ref(1), D.ref(2)), D.x, D.kw))));
        Kg = tw.P/(tw.P + 2*D.s2(k));
        tw.bd(tw.mask) = tw.bd(tw.mask) + Kg*rr(tw.mask);
        tw.P = tw.P*(1 - Kg);
      end
      continue;
    end
    if any(strcmp(mode, {'selfcal', 'refcal'}))
      [tw, th, d, qd] = twin_step(tw, D.t(k), D.obs(k, :), D.s2(k), D.x, D.kw, D.kc, D.lam/2);
    else
      oo = D.obs(k, :).';
      if strcmp(mode, 'oracle'), oo = angle(exp(1j*(oo - D.bd_true(k, :).'))); end
      if isempty(prev), prev = nf_coarse(oo, D.x, D.kw, D.kc, D.lam/2); end
      [th, d, ~, f] = nf_fit(oo, D.x, D.kw, allm, prev);
      prev = [th; d];
      qd = f/D.s2(k)/(D.Ka - 3);
    end
    if k == ipk, snap = tw.bd; end
    bnd = nearfieldBounds(th, d, D.xa, D.Mp, D.Np, D.snr(k), D.eta, D.fc, 'nuisance');
    err = hypot(d*sin(th) - D.X(k), d*cos(th) - D.Y(k));
    out(end+1, :) = [D.t(k), err, qd, bnd.PEB]; %#ok<AGROW>
  end
end

function s = snr_lin(d, lam)
  % LOS free-space link budget, same as platoon_twin_v2.m
  fspl = 20*log10(4*pi*d/lam);
  pr   = 23 + 2*3 - fspl - 10;
  nz   = -174 + 10*log10(10e6) + 9;
  s    = 10^((pr - nz)/10);
end
