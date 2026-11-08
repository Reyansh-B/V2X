% platoon_twin_v3.m
% CACC platoon with a DIGITAL TWIN of every follower's near-field array, closed loop, live view.
% Needs nearfieldBounds.m, nf_phase.m, nf_fit.m, nf_coarse.m, twin_init.m, twin_step.m on the path.
%
% What is new vs platoon_twin_v2.m (same controller, actuator lag, link budget, packet timing):
%   * the near-field gap is no longer "true gap + PEB noise". Each follower receives the real
%     phase stream of its predecessor's packet through a HIDDEN imperfect array
%     (3 deg rms phase bias per element + drift), and fits the gap from it
%   * every follower runs a digital twin of its array (twin_step.m): learns the bias, chi-square
%     health test, masks faulty elements, rejects bad packets (radar/GNSS fallback)
%   * radar-aided calibration: while the radar works, the twin uses the radar gap as a
%     reference, which fixes the bias component the twin cannot see on its own
%   * scenario stresses: radar outage (fog / blinded sensor) and an antenna fault during it
%   * all cases see the SAME random world per seed (common random numbers -> fair comparison)
%
% Scenario (60 s, leader + 5 followers, constant spacing 10 m)
%   10-13 s leader brakes -3 m/s^2, 18-22.5 s accelerates +2 m/s^2 (repeated at 40 s and 48 s)
%   30-55 s RADAR OUTAGE
%   45 s    element 11 of vehicle 3's array jumps by +60 deg (fault), during the outage
%
% Cases
%   radar          radar gap; in the outage only dead reckoning (speeds from CAM)
%   radar_gnss     radar gap; GNSS/CAM gap when the radar is out
%   nf_oracle      near-field gap, perfect array (reference; used to compute "dev")
%   nf_raw         near-field gap from the imperfect array, no twin
%   nf_twin        + twin (self-calibration, health test, fault masking, fallback)
%   nf_twin_radar  + radar-aided calibration (full system)
%   NF cases use radar until t_enable (twins warm up), then NF while the twin says healthy,
%   otherwise radar if available, else GNSS.
%
% PLACEHOLDERS (mine, not from the paper): radar 0.1 m noise + 3 cm bias at 20 Hz, 50 ms delay;
% speed sensors 0.2 % scale error; 5 % packet loss; 8 % of packets hit by 8-25 dB interference;
% GNSS model as in platoon_twin_v2.m.
clear; clc; close all;

%% Parameters
P.dt = 0.01; P.T = 60; P.Nv = 6; P.d0 = 10; P.tau = 0.4; P.kp = 1.5; P.kd = 2.0;   % as v2
P.Tg = 0.1; P.dly = 2; P.v0 = 25;                       % packet period [s], latency [steps]
P.sig_gnss = 1.5; P.corr = 2.0;                         % GNSS, as v2
P.sig_r = 0.1; P.Tr = 0.05; P.dly_r = 5; P.sig_rb = 0.03;   % radar noise, period, delay [steps], bias std
P.sig_vb = 0.002;                                       % speed-sensor scale error (0.2 %)
P.ploss = 0.05;                                         % random packet loss
P.outage = [30 55]; P.t_enable = 6;                     % radar outage [s]; NF used after t_enable
P.q_gap = 0.05; P.q_ref = 0.0025; P.R_floor = 0.02;     % gap-filter process noise, NF noise floor [m]
% near-field physical layer (same as the other scripts)
P.fc = 5.9e9; P.lam = 299792458/P.fc; P.kw = 2*pi/P.lam;
P.Ka = 41; P.kc = (P.Ka+1)/2; P.xa = (0:P.Ka-1)*P.lam/2; P.xa = P.xa - mean(P.xa);
P.keep = true(P.Ka, 1); P.keep(P.kc) = false; P.x = P.xa(P.keep).';
P.Mp = 2; P.Np = 120; P.eta = 10^(6/10);
% hidden hardware
P.bias_deg = 3; P.drift_deg = 0.01; P.burst_p = 0.08;
P.fault_t = 45; P.fault_veh = 3; P.fault_el = 11; P.fault_deg = 60;
% run control
Nseeds  = 5;
P.live  = true;        % live animation of one world after the Monte Carlo
P.video = false;       % also record the live view to a video file
P.realtime = true;     % pace the live view to (at most) real time

cases  = {'radar', 'radar_gnss', 'nf_oracle', 'nf_raw', 'nf_twin', 'nf_twin_radar'};
labels = {'Radar (dead reckoning in outage)', 'Radar + GNSS fallback', 'NF oracle (perfect array)', ...
          'NF raw, no twin', 'NF + twin', 'NF + twin + radar-aided cal.'};

%% Monte Carlo over random worlds
M = struct();
for sd = 1:Nseeds
  W = make_world(P, sd);
  res = struct();
  for c = 1:numel(cases)
    tic; res.(cases{c}) = run_platoon(cases{c}, P, W, []);
    fprintf('seed %d  %-14s %5.1f s\n', sd, cases{c}, toc);
  end
  for c = 1:numel(cases)
    M(sd).(cases{c}) = metrics(res.(cases{c}), res.nf_oracle, P);
  end
  if sd == 1, res1 = res; end
end

%% Results table
fprintf('\nMean over %d worlds. dev = RMS of (spacing error - oracle''s), i.e. the part caused by the gap sensor [m]\n', Nseeds);
fprintf('%-34s %8s %8s %8s %8s %8s %7s %6s\n', 'case', 'normal', 'outage', 'fault', 'others', 'min gap', 'delay', 'false');
for c = 1:numel(cases)
  A = [M.(cases{c})];
  if c >= 5, dly = sprintf('%.2fs', mean([A.delay], 'omitnan')); fa = sprintf('%d', sum([A.nfalse])); else, dly = '-'; fa = '-'; end
  fprintf('%-34s %8.3f %8.3f %8.3f %8.3f %8.2f %7s %6s\n', labels{c}, mean([A.normal]), mean([A.outage]), ...
          mean([A.fault]), mean([A.other]), mean([A.mingap]), dly, fa);
end
fprintf('normal = 6-30 s | outage = 30-55 s | fault = faulty link after 45 s | others = other links after 45 s\n');
for c = 5:6
  A = [M.(cases{c})];
  fprintf('%s: faulty element identified as %s (true %d)\n', labels{c}, mat2str([A.elem]), P.fault_el);
end
save('platoon_twin_v3_results.mat', 'M', 'P', 'cases', 'labels');

%% Figures
t = res1.radar.t; ref = res1.nf_oracle.err;
cols = lines(numel(cases));
figure('Color', 'w', 'Position', [60 60 1000 820]);
subplot(3, 1, 1);
plot(t, res1.radar.v(1, :), 'k', 'LineWidth', 2); grid on; ylabel('leader speed [m/s]');
title('Digital twin + platooning: radar outage 30-55 s (grey), antenna fault at 45 s (red)');
fi = P.fault_veh - 1;                                     % row of the faulty link in gap/err
for sp = 2:3
  subplot(3, 1, sp); hold on;
  row = Nfollow_row(sp, P, fi);
  for c = [1 2 4 5 6]
    plot(t, res1.(cases{c}).err(row, :) - ref(row, :), 'Color', cols(c, :), 'LineWidth', 1.2);
  end
  grid on; ylim([-2.5 2.5]); ylabel('sensor-induced spacing error [m]');
  if sp == 2, title('last follower'); legend(labels([1 2 4 5 6]), 'Location', 'southwest');
  else, title(sprintf('follower %d (gets the antenna fault)', fi)); xlabel('t [s]'); end
end
for sp = 1:3
  subplot(3, 1, sp);
  yl = ylim; patch([P.outage(1) P.outage(2) P.outage(2) P.outage(1)], [yl(1) yl(1) yl(2) yl(2)], ...
                   'k', 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility', 'off');
  xline(P.fault_t, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
print(gcf, 'platoon_v3_timeseries.png', '-dpng', '-r150');

figure('Color', 'w', 'Position', [80 80 1100 380]);
keys = {'outage', 'fault', 'mingap'};
ttl  = {'sensor-induced RMS error, radar outage [m]', 'sensor-induced RMS error, faulty link [m]', 'minimum gap [m]'};
show = [1 2 4 5 6];
for s = 1:3
  subplot(1, 3, s);
  vals = zeros(Nseeds, numel(show));
  for j = 1:numel(show), A = [M.(cases{show(j)})]; vals(:, j) = [A.(keys{s})]; end
  bh = bar(mean(vals, 1), 'FaceColor', 'flat'); bh.CData = cols(show, :); hold on;
  errorbar(1:numel(show), mean(vals, 1), std(vals, 0, 1), 'k.', 'LineWidth', 1);
  set(gca, 'XTick', 1:numel(show), 'XTickLabel', labels(show), 'XTickLabelRotation', 35, 'FontSize', 7);
  if s < 3, set(gca, 'YScale', 'log'); end
  title(ttl{s}, 'FontSize', 9); grid on;
end
sgtitle(sprintf('Mean \\pm std over %d worlds', Nseeds));
print(gcf, 'platoon_v3_summary.png', '-dpng', '-r150');

%% Live view: the full system simulated live, next to radar+GNSS (same world)
if P.live
  W = make_world(P, 1);
  viz.ghost = res1.radar_gnss; viz.ref = res1.nf_oracle;
  run_platoon('nf_twin_radar', P, W, viz);
end

%% ======================= local functions =======================
function row = Nfollow_row(sp, P, fi)
  if sp == 2, row = P.Nv - 1; else, row = fi; end
end

function s = snr_lin(d, lam)
  % LOS free-space link budget, same as platoon_twin_v2.m
  fspl = 20*log10(4*pi*d/lam);
  pr   = 23 + 2*3 - fspl - 10;
  nz   = -174 + 10*log10(10e6) + 9;
  s    = 10^((pr - nz)/10);
end

function a = u_lead(t)
  if (t >= 10 && t < 13) || (t >= 40 && t < 43), a = -3;
  elseif (t >= 18 && t < 22.5) || (t >= 48 && t < 52.5), a = 2;
  else, a = 0;
  end
end

function W = make_world(P, seed)
  % Every random draw of one world, made once, shared by all cases.
  rng(seed);
  Nv = P.Nv; K = P.Ka; N = round(P.T/P.dt) + 1; npk = ceil(P.T/P.Tg) + 2;
  W.loss  = rand(Nv, npk) < P.ploss;
  W.burst = (rand(Nv, npk) < P.burst_p).*(8 + 17*rand(Nv, npk));          % dB
  W.B = (P.bias_deg*pi/180)*randn(K, 1, Nv) + cumsum((P.drift_deg*pi/180)*randn(K, npk, Nv), 2);
  tp = (0:npk-1)*P.Tg;
  W.B(P.fault_el, tp >= P.fault_t, P.fault_veh) = W.B(P.fault_el, tp >= P.fault_t, P.fault_veh) + P.fault_deg*pi/180;
  W.E  = randn(K, npk, Nv);
  W.rn = randn(Nv, N);
  W.rb = P.sig_rb*randn(Nv, 1);
  al = exp(-P.dt/P.corr); W.g = zeros(Nv, N);
  for k = 2:N, W.g(:, k) = al*W.g(:, k-1) + sqrt(1 - al^2)*P.sig_gnss*randn(Nv, 1); end
  W.vb = P.sig_vb*randn(Nv, 1);
  W.ph = randi(round(P.Tg/P.dt), Nv, 1) - 1;
end

function res = run_platoon(scheme, P, W, viz)
  dt = P.dt; N = round(P.T/dt) + 1; Nv = P.Nv; t = (0:N-1)*dt;
  spp = round(P.Tg/dt); rpp = round(P.Tr/dt);
  wrapph = @(a) angle(exp(1j*a));
  x = zeros(Nv, N); v = x; a = x; u = x;
  x(:, 1) = -(0:Nv-1)'*P.d0; v(:, 1) = P.v0;
  ghat = P.d0*ones(Nv, 1); Pg = 1e-2*ones(Nv, 1);         % gap filter used by the controller
  gr   = ghat;            Pr = Pg;                         % radar-only filter (calibration reference)
  hv = P.v0*ones(Nv, 1); ha = zeros(Nv, 1);                % last CAM data: predecessor speed, accel
  src = zeros(Nv, N); last_src = zeros(Nv, 1);             % 0 none, 1 radar, 2 NF, 3 GNSS
  healthy = true(Nv, 1); hq = nan(Nv, N);
  use_nf  = strncmp(scheme, 'nf', 2);
  is_twin = any(strcmp(scheme, {'nf_twin', 'nf_twin_radar'}));
  tw = cell(Nv, 1); pprev = cell(Nv, 1);
  if is_twin, for i = 2:Nv, tw{i} = twin_init(P.Ka, P.drift_deg); end, end
  live = ~isempty(viz);
  if live, V = viz_init(P, viz, t); t0 = tic; end

  for k = 2:N
    tk = t(k); k0 = k - 1;                                 % 0-based step index
    radar_ok = ~(tk >= P.outage(1) && tk < P.outage(2));
    u(1, k-1) = u_lead(t(k-1));
    kd = max(1, k - P.dly);
    for i = 2:Nv
      p = i - 1;
      vo = v(i, k-1)*(1 + W.vb(i));                        % own speed sensor
      ghat(i) = ghat(i) + (hv(i) - vo)*dt; Pg(i) = Pg(i) + P.q_gap*dt;
      gr(i)   = gr(i)   + (hv(i) - vo)*dt; Pr(i) = Pr(i) + P.q_ref*dt;

      % ---- CAM packet from the predecessor (carries the NF pilots)
      if mod(k0 - W.ph(i), spp) == 0
        q = (k0 - W.ph(i))/spp + 1;
        if ~W.loss(i, q)
          hv(i) = v(p, kd)*(1 + W.vb(p)); ha(i) = u(p, kd);
          gt   = x(p, kd) - x(i, kd);
          corr = (hv(i) - vo)*P.dly*dt;
          used = false;
          if use_nf
            snr = snr_lin(gt, P.lam)/10^(W.burst(i, q)/10);
            s2  = P.eta/(2*P.Mp*P.Np*snr);
            if strcmp(scheme, 'nf_oracle'), bb = zeros(P.Ka, 1); else, bb = W.B(:, q, i); end
            e   = sqrt(s2)*W.E(:, q, i);
            obs = wrapph(nf_phase(0, gt, P.x, P.kw) + bb(P.keep) - bb(P.kc) + e(P.keep) - e(P.kc));
            if is_twin
              [tw{i}, th, dd, qd, ok] = twin_step(tw{i}, tk, obs, s2, P.x, P.kw, P.kc, P.lam/2, P.Tg);
              healthy(i) = ok; hq(i, k) = qd;
            else
              if isempty(pprev{i}), pprev{i} = nf_coarse(obs, P.x, P.kw, P.kc, P.lam/2); end
              [th, dd] = nf_fit(obs, P.x, P.kw, true(P.Ka-1, 1), pprev{i});
              pprev{i} = [th; dd]; healthy(i) = true;
            end
            z = dd*cos(th);
            if tk >= P.t_enable && healthy(i)
              bnd = nearfieldBounds(th, max(dd, 1), P.xa, P.Mp, P.Np, snr, P.eta, P.fc, 'nuisance');
              R = bnd.longitudinal^2 + P.R_floor^2;
              Kg = Pg(i)/(Pg(i) + R); ghat(i) = ghat(i) + Kg*(z + corr - ghat(i)); Pg(i) = Pg(i)*(1 - Kg);
              used = true; last_src(i) = 2;
            end
            if strcmp(scheme, 'nf_twin_radar') && radar_ok && healthy(i) && Pr(i) < 4e-3
              Kref = 0.01 + 0.15*exp(-tk/3);               % radar gap as a calibration reference
              rref = wrapph(obs - tw{i}.bd - nf_phase(th, gr(i), P.x, P.kw));
              m = tw{i}.mask;
              tw{i}.bd(m) = tw{i}.bd(m) + Kref*rref(m);
            end
          end
          if ~used && ~radar_ok && (strcmp(scheme, 'radar_gnss') || use_nf)
            z = (x(p, kd) + W.g(p, kd)) - (x(i, kd) + W.g(i, kd));
            R = 2*P.sig_gnss^2;
            Kg = Pg(i)/(Pg(i) + R); ghat(i) = ghat(i) + Kg*(z + corr - ghat(i)); Pg(i) = Pg(i)*(1 - Kg);
            last_src(i) = 3;
          end
        end
      end

      % ---- radar
      if radar_ok && mod(k0, rpp) == 0
        kr = max(1, k - P.dly_r);
        z = (x(p, kr) - x(i, kr)) + W.rb(i) + P.sig_r*W.rn(i, k) + (hv(i) - vo)*P.dly_r*dt;
        Kr = Pr(i)/(Pr(i) + P.sig_r^2); gr(i) = gr(i) + Kr*(z - gr(i)); Pr(i) = Pr(i)*(1 - Kr);
        if ~use_nf || tk < P.t_enable || ~healthy(i)
          R = P.sig_r^2 + P.sig_rb^2;
          Kg = Pg(i)/(Pg(i) + R); ghat(i) = ghat(i) + Kg*(z - ghat(i)); Pg(i) = Pg(i)*(1 - Kg);
          last_src(i) = 1;
        end
      end
      if strcmp(scheme, 'radar') && ~radar_ok, last_src(i) = 0; end
      src(i, k) = last_src(i);

      % ---- controller (platoon_twin_v2.m law, gap from the filter)
      u(i, k-1) = min(3, max(-8, P.kp*(ghat(i) - P.d0) + P.kd*(hv(i) - vo) + ha(i)));
    end
    a(:, k) = a(:, k-1) + dt*(u(:, k-1) - a(:, k-1))/P.tau;
    v(:, k) = max(0, v(:, k-1) + dt*a(:, k));
    x(:, k) = x(:, k-1) + dt*v(:, k);

    if live && mod(k0, 10) == 0
      viz_update(V, P, viz, t, k, x, src, hq, tw);
      if P.realtime, pause(max(0, tk - toc(t0))); end
    end
  end
  gap = x(1:end-1, :) - x(2:end, :);
  res.t = t; res.x = x; res.v = v; res.gap = gap; res.err = gap - P.d0;
  res.src = src(2:end, :); res.hq = hq(2:end, :);
  res.is_twin = is_twin;
  res.detect = repmat({zeros(0, 2)}, Nv-1, 1);
  if is_twin, for i = 2:Nv, res.detect{i-1} = tw{i}.detect; end, end
  if live && P.video, close(V.vw); fprintf('video written: %s\n', V.vfile); end
end

function m = metrics(res, ref, P)
  t = res.t; dev = res.err - ref.err;
  w1 = t >= P.t_enable & t < P.outage(1); w2 = t >= P.outage(1) & t < P.outage(2);
  post = t >= P.fault_t; fi = P.fault_veh - 1; oth = setdiff(1:P.Nv-1, fi);
  rmsv = @(A) sqrt(mean(A(:).^2));
  m.normal = rmsv(dev(:, w1)); m.outage = rmsv(dev(:, w2));
  m.fault  = rmsv(dev(fi, post)); m.other = rmsv(dev(oth, post));
  m.mingap = min(min(res.gap(:, t >= P.t_enable)));
  m.delay = NaN; m.elem = -1; m.nfalse = 0;
  if res.is_twin
    d = res.detect{fi};
    hit = d(d(:, 1) >= P.fault_t, :);
    if ~isempty(hit)
      m.delay = hit(1, 1) - P.fault_t;
      m.elem  = hit(1, 2) + (hit(1, 2) >= P.kc);           % masked index -> array element
    end
    for j = 1:P.Nv-1
      dj = res.detect{j};
      if j == fi, m.nfalse = m.nfalse + sum(dj(:, 1) < P.fault_t);
      else, m.nfalse = m.nfalse + size(dj, 1); end
    end
  end
end

function V = viz_init(P, viz, t)
  V.fig = figure('Color', 'w', 'Position', [60 60 1100 820], 'Name', 'platoon_twin_v3 live');
  V.cmap = [0.75 0.75 0.75; 0.95 0.55 0.17; 0.35 0.63 0.31; 0.69 0.48 0.63];   % none radar NF GNSS
  V.ax1 = subplot(4, 1, [1 2]); hold(V.ax1, 'on');
  lanes = [1.6 0];
  names = {'Radar + GNSS fallback', 'NF + twin + radar-aided cal. (simulated live)'};
  for L = 1:2
    text(V.ax1, -138, lanes(L) + 0.95, names{L}, 'FontWeight', 'bold', 'FontSize', 9);
    for i = 1:P.Nv
      V.rect(L, i) = rectangle(V.ax1, 'Position', [0 lanes(L) 4.5 0.8], 'FaceColor', 'k');
      V.lab(L, i)  = text(V.ax1, 0, lanes(L) + 0.85, '', 'FontSize', 7, 'HorizontalAlignment', 'center');
    end
  end
  V.lanes = lanes;
  xlim(V.ax1, [-140 6]); ylim(V.ax1, [-0.5 3.0]); set(V.ax1, 'YTick', []);
  xlabel(V.ax1, 'position relative to leader [m]');
  V.ttl = title(V.ax1, '', 'FontSize', 9);
  V.ax2 = subplot(4, 1, 3); hold(V.ax2, 'on'); grid(V.ax2, 'on');
  fi = P.fault_veh - 1;
  V.l(1) = plot(V.ax2, nan, nan, '--', 'Color', [0.69 0.48 0.63], 'LineWidth', 1.2);
  V.l(2) = plot(V.ax2, nan, nan, '-',  'Color', [0.35 0.63 0.31], 'LineWidth', 1.4);
  V.l(3) = plot(V.ax2, nan, nan, '-',  'Color', [0.84 0.15 0.16], 'LineWidth', 1.0);
  legend(V.ax2, {'radar+GNSS, last follower', 'full system, last follower', ...
                 sprintf('full system, follower %d (fault)', fi)}, 'Location', 'southwest', 'FontSize', 7);
  ylim(V.ax2, [-2 2]); xlim(V.ax2, [0 t(end)]); ylabel(V.ax2, 'sensor-induced error [m]');
  V.ax3 = subplot(4, 1, 4); hold(V.ax3, 'on'); grid(V.ax3, 'on');
  for j = 1:P.Nv-1
    if j == fi, c = [0.84 0.15 0.16]; lw = 1.4; else, c = [0.6 0.6 0.6]; lw = 0.6; end
    V.h(j) = plot(V.ax3, nan, nan, '-', 'Color', c, 'LineWidth', lw);
  end
  yline(V.ax3, 2.0, 'k:', 'HandleVisibility', 'off');
  set(V.ax3, 'YScale', 'log'); ylim(V.ax3, [0.3 1e6]); xlim(V.ax3, [0 t(end)]);
  xlabel(V.ax3, 't [s]'); ylabel(V.ax3, 'twin health \chi^2/dof');
  yr = {[-10 10], [1e-3 1e7]};                          % patch heights (log axis needs y > 0)
  axs = [V.ax2 V.ax3];
  for j = 1:2
    patch(axs(j), [P.outage(1) P.outage(2) P.outage(2) P.outage(1)], yr{j}([1 1 2 2]), 'k', ...
          'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    xline(axs(j), P.fault_t, 'r:', 'HandleVisibility', 'off');
  end
  if P.video
    try, V.vw = VideoWriter('platoon_twin_live.mp4', 'MPEG-4'); V.vfile = 'platoon_twin_live.mp4';
    catch, V.vw = VideoWriter('platoon_twin_live.avi'); V.vfile = 'platoon_twin_live.avi'; end
    V.vw.FrameRate = 10; open(V.vw);
  end
end

function viz_update(V, P, viz, t, k, x, src, hq, tw)
  if ~isvalid(V.fig), return; end
  fi = P.fault_veh - 1;
  pos = {viz.ghost.x(:, k), x(:, k)}; srcs = {viz.ghost.src(:, k), src(2:end, k)};
  gaps = {viz.ghost.gap(:, k), x(1:end-1, k) - x(2:end, k)};
  for L = 1:2
    xr = pos{L} - pos{L}(1);
    for i = 1:P.Nv
      if i == 1, fc = [0 0 0]; else, fc = V.cmap(srcs{L}(i-1) + 1, :); end
      ec = 'k'; lw = 0.5;
      if L == 2 && i == P.fault_veh && t(k) >= P.fault_t, ec = 'r'; lw = 2.5; end
      set(V.rect(L, i), 'Position', [xr(i) - 4.5, V.lanes(L), 4.5, 0.8], 'FaceColor', fc, ...
                        'EdgeColor', ec, 'LineWidth', lw);
      if i > 1
        set(V.lab(L, i), 'Position', [xr(i) - 2.25, V.lanes(L) + 0.85], ...
            'String', sprintf('%+.0f cm', 100*(gaps{L}(i-1) - P.d0)));
      end
    end
  end
  gl = x(1:end-1, 1:k) - x(2:end, 1:k) - P.d0;
  ref = viz.ref.err(:, 1:k);
  set(V.l(1), 'XData', t(1:k), 'YData', viz.ghost.err(end, 1:k) - ref(end, :));
  set(V.l(2), 'XData', t(1:k), 'YData', gl(end, :) - ref(end, :));
  set(V.l(3), 'XData', t(1:k), 'YData', gl(fi, :) - ref(fi, :));
  for j = 1:P.Nv-1
    ok = find(~isnan(hq(j+1, 1:k)));
    set(V.h(j), 'XData', t(ok), 'YData', min(max(hq(j+1, ok), 0.31), 9e5));
  end
  ev = '';
  if t(k) >= P.outage(1) && t(k) < P.outage(2), ev = [ev '   RADAR OUTAGE']; end
  if t(k) >= P.fault_t
    ev = [ev sprintf('   ANTENNA FAULT on vehicle %d', P.fault_veh)];
    d = tw{P.fault_veh}.detect;
    if ~isempty(d) && any(d(:, 1) >= P.fault_t)
      r = d(find(d(:, 1) >= P.fault_t, 1), :);
      ev = [ev sprintf('   -> twin masked element %d at %.1f s', r(2) + (r(2) >= P.kc), r(1))];
    end
  end
  set(V.ttl, 'String', {sprintf('t = %4.1f s,  leader %4.1f m/s%s', t(k), ...
      (x(1, k) - x(1, max(k-1, 1)))/P.dt, ev), ...
      'colour = gap source: orange radar | green near-field | purple GNSS | grey none (dead reckoning)'});
  drawnow limitrate;
  if P.video, writeVideo(V.vw, getframe(V.fig)); end
end
