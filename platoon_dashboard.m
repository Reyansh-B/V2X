function platoon_dashboard()
% PLATOON_DASHBOARD  Live digital-twin dashboard of a 6-car platoon.
%   Run:  platoon_dashboard
%   Needs nearfieldBounds.m, nf_phase.m, nf_fit.m, nf_coarse.m, twin_init.m, twin_step.m
%   in the same folder (the MATLAB Current Folder).
%
% WHAT YOU SEE (updated 10 times per second)
%   Road     : the 6 cars driving; colour = which sensor that car is trusting for the gap
%              orange radar | green near-field (V2X) | purple GNSS | grey none (dead reckoning)
%              a red outline = that car's antenna array has a broken element
%   Table    : per follower: speed, TRUE gap (simulation truth), the gap the car BELIEVES
%              (its estimate, +/- its own uncertainty), latest radar and near-field readings,
%              sensor in use, twin health, number of switched-off antennas
%   Log      : events as they happen (newest on top)
%   Plots    : gaps over the last 20 s, and estimate - truth in cm
%
% WHAT HAPPENS ON ITS OWN (scripted, P.scripted = true)
%   6 s   cars switch from radar to near-field once their twins have warmed up
%   10 s  leader brakes          30 s  radar outage starts (fog)
%   40 s  leader brakes again    45 s  antenna 11 on car 3 breaks (hidden from the twin)
%   55 s  radar comes back
%
% BUTTONS (any time)
%   Leader: brake hard | Blind / restore radar | Break an antenna (random car) | Pause | Stop
%
% Same physics, controller and twin as platoon_twin_v3.m (full system: near-field + twin +
% radar-aided calibration). PLACEHOLDER sensor numbers, see platoon_twin_v3.m.

  %% ---------------- parameters ----------------
  P.dt = 0.01; P.T = 90; P.Nv = 6; P.d0 = 10; P.tau = 0.4; P.kp = 1.5; P.kd = 2.0;
  P.Tg = 0.1; P.dly = 2; P.v0 = 25;
  P.sig_gnss = 1.5; P.corr = 2.0;
  P.sig_r = 0.1; P.Tr = 0.05; P.dly_r = 5; P.sig_rb = 0.03; P.sig_vb = 0.002;
  P.ploss = 0.05; P.t_enable = 6;
  P.q_gap = 0.05; P.q_ref = 0.0025; P.R_floor = 0.02;
  P.fc = 5.9e9; P.lam = 299792458/P.fc; P.kw = 2*pi/P.lam;
  P.Ka = 41; P.kc = (P.Ka+1)/2; P.xa = (0:P.Ka-1)*P.lam/2; P.xa = P.xa - mean(P.xa);
  P.keep = true(P.Ka, 1); P.keep(P.kc) = false; P.x = P.xa(P.keep).';
  P.Mp = 2; P.Np = 120; P.eta = 10^(6/10);
  P.bias_deg = 3; P.drift_deg = 0.01; P.burst_p = 0.08; P.fault_deg = 60;
  P.scripted = true;     % run the scripted scenario as well as the buttons
  P.speed = 1;           % 1 = real time, 2 = twice as fast, ...
  P.carlen = 4.5;        % drawing only
  rng('shuffle');

  %% ---------------- state ----------------
  dt = P.dt; N = round(P.T/dt) + 1; Nv = P.Nv; t = (0:N-1)*dt;
  spp = round(P.Tg/dt); rpp = round(P.Tr/dt);
  wrapph = @(a) angle(exp(1j*a));
  x = zeros(Nv, N); v = x; acc = x; u = x;
  x(:, 1) = -(0:Nv-1)'*P.d0; v(:, 1) = P.v0;
  ghat = P.d0*ones(Nv, 1); Pg = 1e-2*ones(Nv, 1);
  gr = ghat; Pr = Pg;
  hv = P.v0*ones(Nv, 1); ha = zeros(Nv, 1);
  last_src = ones(Nv, 1); healthy = true(Nv, 1); qlast = nan(Nv, 1);
  z_radar = nan(Nv, 1); z_nf = nan(Nv, 1);
  tw = cell(Nv, 1); for i = 2:Nv, tw{i} = twin_init(P.Ka, P.drift_deg); end
  ndet = zeros(Nv, 1);
  B = (P.bias_deg*pi/180)*randn(P.Ka, Nv);        % hidden phase bias of every array
  broken = zeros(Nv, 1);                           % time each array got a broken element (0 = never)
  vb = P.sig_vb*randn(Nv, 1); rb = P.sig_rb*randn(Nv, 1);
  g = zeros(Nv, N); al = exp(-dt/P.corr);
  ph = randi(spp, Nv, 1) - 1;
  gapT = nan(Nv, N); errE = nan(Nv, N);
  brake_until = -1; brake_acc = 0;
  % scripted events: time, kind, fired
  ev_t = [6 10 30 40 45 55]; ev_k = {'enable', 'brake', 'radar_off', 'brake', 'fault', 'radar_on'};
  ev_done = false(size(ev_t));
  script_radar_out = false;

  % button flags (changed by the callbacks below)
  k = 1;                                           % loop counter (read by the radar button)
  paused = false; stopReq = false; manualRadarOut = false; brakeReq = false; breakReq = false;
  logLines = {};

  %% ---------------- user interface ----------------
  fig = figure('Color', 'w', 'Position', [40 40 1380 860], 'Name', 'Platoon digital twin - live', ...
               'NumberTitle', 'off', 'CloseRequestFcn', @onClose);
  cmap = [0.75 0.75 0.75; 0.95 0.55 0.17; 0.35 0.63 0.31; 0.69 0.48 0.63];
  srcName = {'none (dead reckoning)', 'radar', 'near-field (V2X)', 'GNSS'};

  axR = axes(fig, 'Position', [0.04 0.70 0.92 0.22]); hold(axR, 'on');
  plot(axR, [-80 10], [-0.9 -0.9], 'Color', [0.5 0.5 0.5]); plot(axR, [-80 10], [1.9 1.9], 'Color', [0.5 0.5 0.5]);
  plot(axR, [-80 10], [0.5 0.5], '--', 'Color', [0.8 0.8 0.8]);
  for i = 1:Nv
    hCar(i)  = rectangle(axR, 'Position', [0 0 P.carlen 1], 'Curvature', 0.25, 'FaceColor', 'k'); %#ok<AGROW>
    hName(i) = text(axR, 0, 1.55, '', 'FontSize', 8, 'HorizontalAlignment', 'center'); %#ok<AGROW>
    if i > 1
      hGap(i) = text(axR, 0, -0.45, '', 'FontSize', 8, 'HorizontalAlignment', 'center', 'Color', [0.1 0.1 0.6]); %#ok<AGROW>
    end
  end
  xlim(axR, [-(Nv-1)*P.d0 - 12, 8]); ylim(axR, [-1.1 2.3]); set(axR, 'YTick', []);
  xlabel(axR, 'position relative to the leader [m]   (blue numbers = true gap to the car ahead)');
  hTitle = title(axR, '', 'FontSize', 11);

  cols = {'Car', 'Speed [km/h]', 'True gap [m]', 'Car believes [m]', '+/- [cm]', ...
          'Radar reads [m]', 'Near-field reads [m]', 'Sensor in use', 'Twin health', 'Antennas off'};
  hTab = uitable(fig, 'Units', 'normalized', 'Position', [0.04 0.43 0.62 0.22], ...
                 'ColumnName', cols, 'RowName', [], 'FontSize', 10, ...
                 'ColumnWidth', {40, 85, 85, 105, 60, 105, 125, 150, 110, 85});
  uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.68 0.635 0.28 0.025], ...
            'String', 'Event log (newest first)', 'FontWeight', 'bold', 'BackgroundColor', 'w', ...
            'HorizontalAlignment', 'left');
  hLog = uicontrol(fig, 'Style', 'listbox', 'Units', 'normalized', 'Position', [0.68 0.43 0.28 0.205], ...
                   'String', {}, 'FontName', 'monospaced', 'FontSize', 9);

  axG = axes(fig, 'Position', [0.05 0.08 0.40 0.27]); hold(axG, 'on'); grid(axG, 'on');
  axE = axes(fig, 'Position', [0.53 0.08 0.43 0.27]); hold(axE, 'on'); grid(axE, 'on');
  lc = lines(Nv-1);
  for j = 1:Nv-1
    hG(j) = plot(axG, nan, nan, 'Color', lc(j, :), 'LineWidth', 1.3); %#ok<AGROW>
    hE(j) = plot(axE, nan, nan, 'Color', lc(j, :), 'LineWidth', 1.1); %#ok<AGROW>
  end
  plot(axG, [0 P.T], [P.d0 P.d0], 'k:');
  legend(axG, arrayfun(@(j) sprintf('car %d', j+1), 1:Nv-1, 'UniformOutput', false), ...
         'Location', 'southwest', 'FontSize', 7, 'AutoUpdate', 'off');
  ylabel(axG, 'true gap to car ahead [m]'); xlabel(axG, 't [s]'); title(axG, 'Gaps (last 20 s)');
  ylabel(axE, 'belief - truth [cm]'); xlabel(axE, 't [s]');
  title(axE, 'How wrong each car''s gap estimate is (last 20 s)');

  bw = 0.13; by = 0.955; bh = 0.035;
  uicontrol(fig, 'Style', 'pushbutton', 'String', 'Leader: brake hard', 'Units', 'normalized', ...
            'Position', [0.04 by bw bh], 'Callback', @onBrake);
  hRadarBtn = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Blind the radars', 'Units', 'normalized', ...
            'Position', [0.04+bw+0.01 by bw bh], 'Callback', @onRadar);
  uicontrol(fig, 'Style', 'pushbutton', 'String', 'Break an antenna', 'Units', 'normalized', ...
            'Position', [0.04+2*(bw+0.01) by bw bh], 'Callback', @onBreak);
  hPauseBtn = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Pause', 'Units', 'normalized', ...
            'Position', [0.04+3*(bw+0.01) by 0.08 bh], 'Callback', @onPause);
  uicontrol(fig, 'Style', 'pushbutton', 'String', 'Stop', 'Units', 'normalized', ...
            'Position', [0.04+3*(bw+0.01)+0.09 by 0.08 bh], 'Callback', @onStopBtn);

  logEvent(0, 'Start: 6 cars at 25 m/s, 10 m apart. Twins warming up, cars on radar.');

  %% ---------------- main loop ----------------
  wall0 = tic; sim0 = 0; prev_src = last_src;
  for k = 2:N
    while paused && ~stopReq
      pause(0.05);
      if paused == false, wall0 = tic; sim0 = t(k); end
    end
    if stopReq || ~isvalid(fig), break; end
    tk = t(k); k0 = k - 1;

    % ---- scripted events
    if P.scripted
      for q = find(~ev_done & ev_t <= tk)
        ev_done(q) = true;
        switch ev_k{q}
          case 'enable', logEvent(tk, 'Twins warmed up -> cars may now use near-field.');
          case 'brake',  brakeReq = true;
          case 'radar_off', script_radar_out = true; logEvent(tk, 'FOG: all radars blinded (scripted).');
          case 'radar_on',  script_radar_out = false; logEvent(tk, 'Fog clears: radars back (scripted).');
          case 'fault', breakAntenna(3, 11, tk, '(scripted)');
        end
      end
    end
    % ---- button requests
    if brakeReq
      brakeReq = false; brake_until = tk + 3; brake_acc = -3.5;
      logEvent(tk, 'Leader brakes hard (-3.5 m/s^2 for 3 s).');
    end
    if breakReq
      breakReq = false;
      i = randi([2 Nv]); el = randi(P.Ka - 1); if el >= P.kc, el = el + 1; end
      breakAntenna(i, el, tk, '(button)');
    end
    radar_ok = ~(script_radar_out || manualRadarOut);

    % ---- leader
    if tk < brake_until, ul = brake_acc; else, ul = min(2, max(-2, 0.5*(P.v0 - v(1, k-1)))); end
    u(1, k-1) = ul;
    kd = max(1, k - P.dly);
    g(:, k) = al*g(:, k-1) + sqrt(1 - al^2)*P.sig_gnss*randn(Nv, 1);

    % ---- followers (same algorithm as platoon_twin_v3.m, case nf_twin_radar)
    for i = 2:Nv
      p = i - 1;
      vo = v(i, k-1)*(1 + vb(i));
      ghat(i) = ghat(i) + (hv(i) - vo)*dt; Pg(i) = Pg(i) + P.q_gap*dt;
      gr(i)   = gr(i)   + (hv(i) - vo)*dt; Pr(i) = Pr(i) + P.q_ref*dt;
      if mod(k0 - ph(i), spp) == 0 && rand >= P.ploss          % V2X packet received
        B(:, i) = B(:, i) + (P.drift_deg*pi/180)*randn(P.Ka, 1);
        hv(i) = v(p, kd)*(1 + vb(p)); ha(i) = u(p, kd);
        gt = x(p, kd) - x(i, kd);
        corr = (hv(i) - vo)*P.dly*dt;
        snr = snr_lin(gt, P.lam);
        if rand < P.burst_p, snr = snr/10^((8 + 17*rand)/10); end
        s2 = P.eta/(2*P.Mp*P.Np*snr);
        e = sqrt(s2)*randn(P.Ka, 1); bb = B(:, i);
        obs = wrapph(nf_phase(0, gt, P.x, P.kw) + bb(P.keep) - bb(P.kc) + e(P.keep) - e(P.kc));
        was = healthy(i);
        [tw{i}, th, dd, qd, ok] = twin_step(tw{i}, tk, obs, s2, P.x, P.kw, P.kc, P.lam/2, P.Tg);
        healthy(i) = ok; qlast(i) = qd;
        if size(tw{i}.detect, 1) > ndet(i)
          ndet(i) = size(tw{i}.detect, 1);
          jb = tw{i}.detect(end, 2); el = jb + (jb >= P.kc);
          if broken(i) > 0
            logEvent(tk, sprintf('Car %d twin: antenna %d faulty -> switched off (%.1f s after the break).', i, el, tk - broken(i)));
          else
            logEvent(tk, sprintf('Car %d twin: antenna %d switched off.', i, el));
          end
        end
        if tw{i}.warm < 0 && was && ~ok, logEvent(tk, sprintf('Car %d twin: ALARM, near-field reading rejected.', i)); end
        z = dd*cos(th);
        used = false;
        if tk >= P.t_enable && ok
          z_nf(i) = z;
          bnd = nearfieldBounds(th, max(dd, 1), P.xa, P.Mp, P.Np, snr, P.eta, P.fc, 'nuisance');
          R = bnd.longitudinal^2 + P.R_floor^2;
          Kg = Pg(i)/(Pg(i) + R); ghat(i) = ghat(i) + Kg*(z + corr - ghat(i)); Pg(i) = Pg(i)*(1 - Kg);
          used = true; last_src(i) = 2;
        end
        if radar_ok && ok && Pr(i) < 4e-3                       % radar-aided calibration
          Kref = 0.01 + 0.15*exp(-tk/3);
          rref = wrapph(obs - tw{i}.bd - nf_phase(th, gr(i), P.x, P.kw));
          m = tw{i}.mask;
          tw{i}.bd(m) = tw{i}.bd(m) + Kref*rref(m);
        end
        if ~used && ~radar_ok
          z = (x(p, kd) + g(p, kd)) - (x(i, kd) + g(i, kd));
          R = 2*P.sig_gnss^2;
          Kg = Pg(i)/(Pg(i) + R); ghat(i) = ghat(i) + Kg*(z + corr - ghat(i)); Pg(i) = Pg(i)*(1 - Kg);
          last_src(i) = 3;
        end
      end
      if radar_ok && mod(k0, rpp) == 0
        kr = max(1, k - P.dly_r);
        z = (x(p, kr) - x(i, kr)) + rb(i) + P.sig_r*randn + (hv(i) - vo)*P.dly_r*dt;
        z_radar(i) = z;
        Kr = Pr(i)/(Pr(i) + P.sig_r^2); gr(i) = gr(i) + Kr*(z - gr(i)); Pr(i) = Pr(i)*(1 - Kr);
        if tk < P.t_enable || ~healthy(i)
          R = P.sig_r^2 + P.sig_rb^2;
          Kg = Pg(i)/(Pg(i) + R); ghat(i) = ghat(i) + Kg*(z - ghat(i)); Pg(i) = Pg(i)*(1 - Kg);
          last_src(i) = 1;
        end
      end
      if ~radar_ok, z_radar(i) = nan; end
      u(i, k-1) = min(3, max(-8, P.kp*(ghat(i) - P.d0) + P.kd*(hv(i) - vo) + ha(i)));
    end
    acc(:, k) = acc(:, k-1) + dt*(u(:, k-1) - acc(:, k-1))/P.tau;
    v(:, k) = max(0, v(:, k-1) + dt*acc(:, k));
    x(:, k) = x(:, k-1) + dt*v(:, k);
    gapT(2:Nv, k) = x(1:Nv-1, k) - x(2:Nv, k);
    errE(2:Nv, k) = ghat(2:Nv) - gapT(2:Nv, k);

    % ---- sensor switches (one log line per tick)
    if any(last_src ~= prev_src) && tk >= 0.5
      chg = find(last_src ~= prev_src);
      msg = '';
      for s = unique(last_src(chg)).'
        cs = chg(last_src(chg) == s);
        msg = [msg sprintf('car%s %s -> %s; ', plural(cs), strjoin(arrayfun(@num2str, cs.', 'UniformOutput', false), ','), srcName{s+1})]; %#ok<AGROW>
      end
      logEvent(tk, ['Sensor switch: ' msg]);
      prev_src = last_src;
    end

    % ---- draw
    if mod(k0, 10) == 0
      if ~isvalid(fig), break; end
      drawNow(k);
      lag = (tk - sim0)/P.speed - toc(wall0);
      if lag > 0, pause(lag); end
    end
  end
  if isvalid(fig), logEvent(t(min(k, N)), 'Simulation finished.'); end

  %% ================= nested functions =================
  function breakAntenna(i, el, tk, how)
    B(el, i) = B(el, i) + P.fault_deg*pi/180;
    broken(i) = tk;
    logEvent(tk, sprintf('Antenna %d on car %d breaks %s. The twin is NOT told.', el, i, how));
  end

  function logEvent(tk, msg)
    logLines = [{sprintf('%5.1f s  %s', tk, msg)}, logLines];
    if exist('hLog', 'var') && isvalid(hLog), set(hLog, 'String', logLines, 'Value', 1); end
  end

  function drawNow(k)
    tk = t(k);
    xr = x(:, k) - x(1, k);
    for i = 1:Nv
      if i == 1, fc = [0 0 0]; else, fc = cmap(last_src(i) + 1, :); end
      if broken(i) > 0, ec = 'r'; lw = 2.5; else, ec = 'k'; lw = 0.5; end
      set(hCar(i), 'Position', [xr(i) - P.carlen, 0, P.carlen, 1], 'FaceColor', fc, 'EdgeColor', ec, 'LineWidth', lw);
      if i == 1, nm = 'Leader'; else, nm = sprintf('Car %d', i); end
      set(hName(i), 'Position', [xr(i) - P.carlen/2, 1.55], 'String', sprintf('%s\n%.0f km/h', nm, 3.6*v(i, k)));
      if i > 1
        set(hGap(i), 'Position', [(xr(i) + xr(i-1) - P.carlen)/2, -0.45], 'String', sprintf('%.2f m', gapT(i, k)));
      end
    end
    D = cell(Nv-1, numel(cols));
    for i = 2:Nv
      if tw{i}.warm > 0, hs = 'warming up';
      elseif healthy(i), hs = sprintf('OK (%.1f)', qlast(i));
      else, hs = 'ALARM';
      end
      D(i-1, :) = {i, sprintf('%.1f', 3.6*v(i, k)), sprintf('%.3f', gapT(i, k)), sprintf('%.3f', ghat(i)), ...
                   sprintf('%.1f', 100*sqrt(Pg(i))), num2s(z_radar(i)), num2s(z_nf(i)), srcName{last_src(i) + 1}, ...
                   hs, sum(~tw{i}.mask)};
    end
    set(hTab, 'Data', D);
    w = max(1, k - round(20/dt)):k;
    for j = 1:Nv-1
      set(hG(j), 'XData', t(w), 'YData', gapT(j+1, w));
      set(hE(j), 'XData', t(w), 'YData', 100*errE(j+1, w));
    end
    xlim(axG, [t(w(1)) max(t(w(end)), 20)]); xlim(axE, [t(w(1)) max(t(w(end)), 20)]);
    st = '';
    if ~radar_ok, st = [st '   |   RADARS BLIND']; end
    if tk < brake_until, st = [st '   |   LEADER BRAKING']; end
    if any(broken > 0), st = [st sprintf('   |   broken array on car %s', strjoin(arrayfun(@num2str, find(broken > 0).', 'UniformOutput', false), ','))]; end
    set(hTitle, 'String', sprintf('t = %5.1f s    leader %.0f km/h%s', tk, 3.6*v(1, k), st));
    drawnow limitrate;
  end

  function onBrake(~, ~), brakeReq = true; end
  function onBreak(~, ~), breakReq = true; end
  function onRadar(~, ~)
    manualRadarOut = ~manualRadarOut;
    if manualRadarOut
      set(hRadarBtn, 'String', 'Restore the radars'); logEvent(t(max(k, 1)), 'Radars blinded (button).');
    else
      set(hRadarBtn, 'String', 'Blind the radars');  logEvent(t(max(k, 1)), 'Radars restored (button).');
    end
  end
  function onPause(~, ~)
    paused = ~paused;
    if paused, set(hPauseBtn, 'String', 'Resume'); else, set(hPauseBtn, 'String', 'Pause'); end
  end
  function onStopBtn(~, ~), stopReq = true; end
  function onClose(src, ~), stopReq = true; delete(src); end
end

%% ================= local functions =================
function s = num2s(z)
  if isnan(z), s = '-'; else, s = sprintf('%.3f', z); end
end

function s = plural(c)
  if numel(c) > 1, s = 's'; else, s = ''; end
end

function s = snr_lin(d, lam)
  % LOS free-space link budget, same as platoon_twin_v2.m
  fspl = 20*log10(4*pi*d/lam);
  pr   = 23 + 2*3 - fspl - 10;
  nz   = -174 + 10*log10(10e6) + 9;
  s    = 10^((pr - nz)/10);
end
