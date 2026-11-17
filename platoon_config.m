function cfg = platoon_config()
% PLATOON_CONFIG  Default parameters for platoon_sim.m (edit here, not in the sim).
% Values marked PLACEHOLDER must be replaced with a cited source before publishing.

  %% Simulation / vehicle
  cfg.dt  = 0.01;          % [s]
  cfg.T   = 40;            % [s]
  cfg.Nv  = 6;             % vehicles incl. leader
  cfg.tau = 0.4;           % actuator lag [s]
  cfg.umin = -8; cfg.umax = 3;   % accel limits [m/s^2]
  cfg.v0  = 25;            % initial speed [m/s]

  %% Spacing policy + controller (Ploeg et al., IEEE T-CST 2014 structure)
  %  e_i = gap_i - (r + h v_i);  h*du_i/dt = -u_i + kp e_i + kd de_i/dt + u_ff
  cfg.r  = 2.0;            % standstill distance [m]
  cfg.h  = 0.6;            % time headway [s]
  cfg.kp = 0.2;
  cfg.kd = 0.7;
  cfg.ff_timeout = 0.5;    % [s] drop feedforward when the CAM is older than this
  % Gains used when NO feedforward is available (ACC baseline and the CACC
  % fallback after ff_timeout). CACC gains are soft because they rely on the
  % feedforward; without it the follower needs much stiffer feedback.
  cfg.kp_acc = 1.5;
  cfg.kd_acc = 2.0;

  %% V2X packets (CAM/BSM)
  cfg.Tg    = 0.1;         % packet period [s] (10 Hz)
  cfg.lat_c = 0.02;        % packet latency [s]

  %% Packet loss: 'none' | 'burst' | 'ge' (Gilbert-Elliott, per link)
  cfg.loss.type  = 'none';
  cfg.loss.burst = [10 11];          % [s] window for 'burst' (all links)
  cfg.loss.p_gb  = 0.05;             % P(good->bad) per packet       PLACEHOLDER
  cfg.loss.p_bg  = 0.30;             % P(bad->good) per packet       PLACEHOLDER
  cfg.loss.pl_good = 0.01;           % loss prob in good state
  cfg.loss.pl_bad  = 0.90;           % loss prob in bad state

  %% GNSS (relative positioning via CAM)
  %  Error = common-mode part (shared by nearby receivers, cancels in the gap)
  %        + receiver-specific part (does NOT cancel).
  cfg.gnss.sig_common = 1.5;   % [m] standalone-level common error
  cfg.gnss.corr_common = 60;   % [s]
  cfg.gnss.sig_rel  = 0.5;     % [m] std of the GAP error (code DGNSS ~0.5 m; RTK ~0.02 m)
  cfg.gnss.corr_rel = 2;       % [s]

  %% Radar (Continental ARS408-21 class)  PLACEHOLDER: re-check against the PDF datasheet
  cfg.radar.Tr    = 0.072;     % cycle [s]
  cfg.radar.lat   = 0.072;     % latency [s] (one cycle)
  cfg.radar.sig_r = 0.10;      % range std, near range [m]
  cfg.radar.sig_v = 0.1/3.6;   % range-rate std [m/s]

  %% Near-field sidelink array (Decarli et al., JSAC 2024)
  cfg.nf.fc  = 5.9e9;
  cfg.nf.lam = 299792458/cfg.nf.fc;
  cfg.nf.Ka  = 41;
  x = (0:cfg.nf.Ka-1)*cfg.nf.lam/2;  cfg.nf.x = x - mean(x);
  cfg.nf.Mp  = 2; cfg.nf.Np = 120; cfg.nf.eta = 10^(6/10);
  cfg.nf.margin_dB = 10;       % PLACEHOLDER link margin
  cfg.nf.sig_cal = 0.0;        % per-element phase calibration error std [rad] PLACEHOLDER
  cfg.nf.theta = 0;            % straight-line platoon
  cfg.nf.n_gn  = 2;            % Gauss-Newton refinements after the quadratic fit
  cfg.nf.Apinv = pinv([cfg.nf.x(:) cfg.nf.x(:).^2]);

  %% Leader manoeuvre: 'brake' | 'stopgo'
  cfg.scenario = 'stopgo';
end
