function out = platoon_sim(cfg, sensor, seed)
% PLATOON_SIM  One run of a CACC platoon with a given gap-sensing scheme.
%   out = platoon_sim(cfg, sensor, seed)
%
%   sensor : 'ideal'    perfect, undelayed gap/velocity/feedforward (reference)
%            'acc'      radar only, NO V2X feedforward (ACC baseline)
%            'radar'    radar gap + range-rate, V2X CAM only for feedforward
%            'gnss'     gap from GNSS positions exchanged in the CAM
%            'nf_crlb'  near-field gap, error SAMPLED from the CRLB (old v2 approach)
%            'nf'       near-field gap from the ACTUAL estimator (nf_estimate.m)
%                       run on simulated phases at every received packet
%
%   GNSS and near-field ranging ride on the SAME CAM packet as the feedforward,
%   so a lost packet costs both. Radar ranging does not depend on the V2X link.
%
%   Between measurements, the follower dead-reckons the gap with its own
%   odometry and the predecessor's last reported speed/acceleration.

  if nargin < 3, seed = 1; end
  rng(seed);
  dt = cfg.dt; t = 0:dt:cfg.T; N = numel(t); Nv = cfg.Nv;
  h = cfg.h; r = cfg.r;
  cacc = ~strcmp(sensor, 'acc');
  use_radar = any(strcmp(sensor, {'acc', 'radar'}));

  %% States (row 1 = leader)
  x = zeros(Nv,N); v = x; a = x; u = x;
  gap0 = r + h*cfg.v0;
  x(:,1) = -(0:Nv-1)'*gap0;  v(:,1) = cfg.v0;
  ulead = leader_profile(t, cfg.scenario);

  %% GNSS errors: common-mode + receiver-specific (OU processes)
  ac = exp(-dt/cfg.gnss.corr_common); ar = exp(-dt/cfg.gnss.corr_rel);
  sr = cfg.gnss.sig_rel/sqrt(2);                 % per receiver -> gap std = sig_rel
  gc = zeros(1,N); gr = zeros(Nv,N);
  gc(1) = cfg.gnss.sig_common*randn; gr(:,1) = sr*randn(Nv,1);
  for k = 2:N
    gc(k)   = ac*gc(k-1)   + sqrt(1-ac^2)*cfg.gnss.sig_common*randn;
    gr(:,k) = ar*gr(:,k-1) + sqrt(1-ar^2)*sr*randn(Nv,1);
  end
  gnss = x*0;   % filled on the fly: gnss(i,k) = x(i,k) + gc(k) + gr(i,k)

  %% Near-field setup
  nf = cfg.nf;
  cal = nf.sig_cal*randn(Nv, nf.Ka);             % fixed per-vehicle calibration error
  if strcmp(sensor, 'nf_crlb')
    dg = linspace(0.5, 80, 160); sg = zeros(size(dg));
    for q = 1:numel(dg)
      o = nearfieldBounds(nf.theta, dg(q), nf.x, nf.Mp, nf.Np, ...
            snr_linkbudget(dg(q), nf.lam, nf.margin_dB), nf.eta, nf.fc, 'paper');
      sg(q) = o.longitudinal;
    end
  end

  %% Timing
  spp = round(cfg.Tg/dt); lc = round(cfg.lat_c/dt);
  nr  = round(cfg.radar.Tr/dt); lr = round(cfg.radar.lat/dt);
  ph  = randi(spp, Nv, 1) - 1;                   % CAM phase offsets

  %% Follower memory
  gm = gap0*ones(Nv,1); vpm = cfg.v0*ones(Nv,1); apm = zeros(Nv,1); km = ones(Nv,1);
  uff = zeros(Nv,1); kff = ones(Nv,1); uc = zeros(Nv,1);
  ge_bad = false(Nv,1);
  merr = nan(Nv*ceil(N/spp)+10, 2); nme = 0;     % [true gap, measured - true]
  nlost = 0; npk = 0;

  for k = 2:N
    ks = k-1;                                    % "now" (latest known state)
    gnss(:,ks) = x(:,ks) + gc(ks) + gr(:,ks);
    u(1,ks) = ulead(ks);
    for i = 2:Nv
      p = i-1;

      % ---------- V2X CAM reception ----------
      if strcmp(sensor, 'ideal')
        gm(i) = x(p,ks) - x(i,ks); vpm(i) = v(p,ks); apm(i) = a(p,ks); km(i) = ks;
        uff(i) = u(p,ks); kff(i) = ks;
      elseif mod(ks - ph(i), spp) == 0
        npk = npk + 1;
        lost = packet_lost(cfg.loss, t(ks), ge_bad(i));
        if strcmp(cfg.loss.type, 'ge'), ge_bad(i) = ge_next(cfg.loss, ge_bad(i)); end
        if lost
          nlost = nlost + 1;
        else
          kd = max(1, ks - lc);
          uff(i) = u(p,kd); kff(i) = kd;
          if ~use_radar
            gtrue = x(p,kd) - x(i,kd);
            switch sensor
              case 'gnss'
                gmeas = gnss(p,kd) - gnss(i,kd);
              case 'nf_crlb'
                gq = min(max(gtrue, dg(1)), dg(end));
                gmeas = gtrue + interp1(dg, sg, gq)*randn;
              case 'nf'
                dtrue = max(gtrue, 0.3);
                snr = snr_linkbudget(dtrue, nf.lam, nf.margin_dB);
                ms = nf_phase_meas(nf.theta, dtrue, nf.x, snr, nf.eta, nf.Mp, nf.Np, nf.lam, cal(i,:));
                [th_h, d_h] = nf_estimate(ms, nf.x, nf.lam, nf.n_gn, nf.Apinv);
                gmeas = d_h*cos(th_h);
              otherwise
                error('unknown sensor %s', sensor);
            end
            gm(i) = gmeas; vpm(i) = v(p,kd); apm(i) = a(p,kd); km(i) = kd;
            nme = nme + 1; merr(nme,:) = [gtrue, gmeas - gtrue];
          else
            apm(i) = a(p,kd);                     % radar: CAM refreshes accel only
          end
        end
      end

      % ---------- radar ----------
      if use_radar && mod(ks, nr) == 0
        kd = max(1, ks - lr);
        gtrue = x(p,kd) - x(i,kd);
        gm(i)  = gtrue + cfg.radar.sig_r*randn;
        vpm(i) = v(i,kd) + (v(p,kd) - v(i,kd)) + cfg.radar.sig_v*randn;
        if ~cacc, apm(i) = 0; end
        km(i) = kd;
        nme = nme + 1; merr(nme,:) = [gtrue, gm(i) - gtrue];
      end

      % ---------- dead-reckoned gap / predecessor speed ----------
      dtm = (ks - km(i))*dt;
      dta = min(dtm, cfg.ff_timeout);            % don't extrapolate accel for long
      vp_hat = max(0, vpm(i) + apm(i)*dta);
      g_hat  = gm(i) + vpm(i)*dtm + apm(i)*dta*(dtm - dta/2) - (x(i,ks) - x(i,km(i)));

      % ---------- CACC law (Ploeg 2014) ----------
      e    = g_hat - r - h*v(i,ks);
      edot = vp_hat - v(i,ks) - h*a(i,ks);
      if cacc && (ks - kff(i))*dt <= cfg.ff_timeout
        ff = uff(i); kp = cfg.kp; kdg = cfg.kd;          % CACC mode
      else
        ff = 0; kp = cfg.kp_acc; kdg = cfg.kd_acc;       % ACC / degraded mode
      end
      uc(i) = uc(i) + dt/h*(-uc(i) + kp*e + kdg*edot + ff);
      uc(i) = min(cfg.umax, max(cfg.umin, uc(i)));
      u(i,ks) = uc(i);
    end
    % ---------- vehicle dynamics ----------
    a(:,k) = a(:,ks) + dt*(u(:,ks) - a(:,ks))/cfg.tau;
    v(:,k) = max(0, v(:,ks) + dt*a(:,k));
    x(:,k) = x(:,ks) + dt*v(:,k);
  end
  u(:,N) = u(:,N-1);

  %% Metrics
  gap = x(1:end-1,:) - x(2:end,:);               % row j = gap in front of vehicle j+1
  err = gap - (r + h*v(2:end,:));
  out.t = t; out.x = x; out.v = v; out.a = a; out.gap = gap; out.err = err;
  out.collision = any(gap(:) <= 0);
  out.min_gap   = min(gap(:));
  out.rms_err   = sqrt(mean(err.^2, 2));         % per follower
  out.max_err   = max(abs(err), [], 2);
  an = sqrt(sum(a.^2, 2));                       % L2 norm of acceleration per vehicle
  out.gamma_a   = max(an(2:end) ./ max(an(1:end-1), eps));   % worst pairwise amplification
  out.amp_e     = out.max_err(end) / max(out.max_err(1), eps);
  closing = v(2:end,:) - v(1:end-1,:);
  ttc = gap ./ closing; ttc(closing <= 0) = inf;
  out.min_ttc   = min(ttc(:));
  out.meas_err  = merr(1:nme,:);
  out.loss_rate = nlost / max(npk, 1);
end

function lost = packet_lost(L, tk, bad)
  switch L.type
    case 'none',  lost = false;
    case 'burst', lost = tk > L.burst(1) && tk < L.burst(2);
    case 'ge'
      if bad, lost = rand < L.pl_bad; else, lost = rand < L.pl_good; end
    otherwise, error('unknown loss type');
  end
end

function bad = ge_next(L, bad)
  if bad, bad = ~(rand < L.p_bg); else, bad = rand < L.p_gb; end
end
