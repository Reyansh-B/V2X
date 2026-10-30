% platoon_twin_v2.m
% Multi-vehicle CACC platoon digital twin with a FAIR comparison of
%   'gnss'  : GNSS positions via CAM/BSM (OU-correlated error, own + predecessor)
%   'nf'    : near-field sidelink gap (sigma = sqrt(PEB(d)^2 + sig_cal^2)), same packets as CAM
%   'radar' : radar gap+velocity at 20 Hz, no packet loss (CAM only for feedforward accel)
%
% What changed vs platoon_digital_twin.m:
%   * 6 vehicles, string-error metrics (not just 1 follower)
%   * 1st-order actuator lag (tau); leader broadcasts its commanded accel (Ploeg-style feedforward)
%   * measurements arrive at PACKET RATE (10 Hz) and are HELD + extrapolated; same delay for all
%   * packet loss = hold last packet (age grows), applied to GNSS and NF; NF shares the CAM packet
%   * SNR(d) from a link budget (free-space + margin) instead of a constant 20 dB
%   * calibration floor added on top of the CRLB
%   * seeds + Monte Carlo statistics
% Needs nearfieldBounds.m on the path.
clear; clc; close all;

%% Parameters
P.dt = 0.01; P.T = 20; P.Nv = 6; P.d0 = 10; P.tau = 0.4;
P.kp = 1.5; P.kd = 2.0;
P.Tg = 0.1;  P.dly = 2;                 % packet period [s], latency [steps of dt]
P.sig_gnss = 1.5; P.corr = 2.0;         % GNSS error std [m], correlation time [s]
P.sig_cal = 0.01;                       % PLACEHOLDER: array calibration/multipath floor [m]
P.sig_r = 0.1; P.sig_rv = 0.1; P.Tr = 0.05;   % PLACEHOLDER radar: take from a datasheet
P.ploss = 0.0;
Nruns = 20;

% Near-field physical layer (same as your other scripts)
fc = 5.9e9; c0 = 299792458; lam = c0/fc;
Ka = 41; xa = (0:Ka-1)*lam/2; xa = xa - mean(xa);
Mp = 2; Np = 120; eta = 10^(6/10);

% PEB lookup table vs range (theta = 0, platooning), SNR from link budget
dgrid = linspace(1, 60, 120); PEBg = zeros(size(dgrid));
for q = 1:numel(dgrid)
  o = nearfieldBounds(0, dgrid(q), xa, Mp, Np, snr_lin(dgrid(q), lam), eta, fc, 'paper');
  PEBg(q) = o.PEB;
end

%% Monte Carlo over two scenarios
scen = {'A: no packet loss', [0 0]; 'B: 1 s burst loss (t = 6..7 s)', [6 7]};
schemes = {'gnss', 'nf', 'radar'};
for s = 1:size(scen,1)
  P.burst = scen{s,2};
  fprintf('\n%s\n', scen{s,1});
  for m = 1:numel(schemes)
    rmsL = zeros(Nruns,1); maxe = rmsL; mingap = rmsL; amp = rmsL;
    for r = 1:Nruns
      [~, gap, err] = run_platoon(schemes{m}, r, P, dgrid, PEBg);
      rl = sqrt(mean(err.^2, 2));
      rmsL(r) = mean(rl); maxe(r) = max(abs(err(:))); mingap(r) = min(gap(:));
      amp(r) = rl(end)/rl(1);
    end
    fprintf('  %-6s RMS %.2f m | max|e| %.2f m | min gap %.2f m | P(gap<1m) %.2f | tail/head RMS %.2f\n', ...
      schemes{m}, mean(rmsL), mean(maxe), mean(mingap), mean(mingap<1), mean(amp));
  end
end

%% Representative plot (scenario B, last follower)
P.burst = [6 7]; figure('Color','w'); hold on;
cols = {'r','b','g'};
for m = 1:numel(schemes)
  [t, gap] = run_platoon(schemes{m}, 1, P, dgrid, PEBg);
  plot(t, gap(end,:), cols{m}, 'LineWidth', 1.8, 'DisplayName', schemes{m});
end
yline(P.d0, 'k--', 'DisplayName', 'desired gap');
patch([6 7 7 6], [0 0 20 20], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'packet-loss burst');
grid on; box on; ylim([0 16]); xlabel('Time [s]'); ylabel('Gap, last follower [m]');
legend('Location', 'best'); title('6-vehicle CACC platoon, leader brakes at -4 m/s^2 (4-8 s)');
print(gcf, 'platoon_v2_gap.png', '-dpng', '-r150');

%% ---------------- local functions ----------------
function s = snr_lin(d, lam)
  % LOS free-space link budget, Table II values; 10 dB extra margin (PLACEHOLDER)
  fspl = 20*log10(4*pi*d/lam);
  pr   = 23 + 2*3 - fspl - 10;                 % dBm (PT + Gtx + Grx - PL - margin)
  nz   = -174 + 10*log10(10e6) + 9;            % dBm
  s    = 10^((pr - nz)/10);                    % per-subcarrier SNR == total SNR
end

function [t, gap, err] = run_platoon(scheme, seed, P, dgrid, PEBg)
  rng(seed);
  dt = P.dt; t = 0:dt:P.T; N = numel(t); Nv = P.Nv; spp = round(P.Tg/dt);
  x = zeros(Nv,N); v = x; a = x; u = x;
  x(:,1) = -(0:Nv-1)'*P.d0; v(:,1) = 25;
  ulead = zeros(1,N); ulead(t > 4 & t < 8) = -4;       % leader commanded accel
  al = exp(-dt/P.corr); gerr = zeros(Nv,N);
  for k = 2:N, gerr(:,k) = al*gerr(:,k-1) + sqrt(1-al^2)*P.sig_gnss*randn(Nv,1); end
  hpos = zeros(Nv,1); hpos(2:end) = x(1:end-1,1);       % held predecessor data (CAM packet)
  hv = 25*ones(Nv,1); ha = zeros(Nv,1); htp = zeros(Nv,1);
  rpos = hpos; rv = hv; rtp = htp;                       % radar stream
  ph = randi(spp, Nv, 1) - 1;                            % packet phase offsets
  for k = 2:N
    u(1,k-1) = ulead(k-1);
    kd = max(1, k - P.dly);
    for i = 2:Nv
      p = i-1;
      if mod(k - ph(i), spp) == 0                        % a packet is received
        lost = (t(k) > P.burst(1) && t(k) < P.burst(2)) || rand < P.ploss;
        if ~lost
          gap_true = x(p,kd) - x(i,kd);
          switch scheme
            case 'gnss'
              pos = x(p,kd) + gerr(p,kd);
            case 'nf'
              gq = min(max(gap_true, dgrid(1)), dgrid(end));
              sg = hypot(interp1(dgrid, PEBg, gq), P.sig_cal);
              pos = x(p,kd) + sg*randn;
            otherwise
              pos = x(p,kd);
          end
          hpos(i) = pos; hv(i) = v(p,kd); ha(i) = u(p,kd); htp(i) = t(k);
        end
      end
      if strcmp(scheme,'radar') && mod(k, round(P.Tr/dt)) == 0
        rpos(i) = x(p,kd) + P.sig_r*randn; rv(i) = v(p,kd) + P.sig_rv*randn; rtp(i) = t(k);
      end
      if strcmp(scheme,'radar')
        ppos = rpos(i) + rv(i)*(t(k)-rtp(i)); pv = rv(i);
      else
        ppos = hpos(i) + hv(i)*(t(k)-htp(i)); pv = hv(i);   % dead-reckon between packets
      end
      own = x(i,k-1); if strcmp(scheme,'gnss'), own = own + gerr(i,k-1); end
      e = (ppos - own) - P.d0;
      u(i,k-1) = min(3, max(-8, P.kp*e + P.kd*(pv - v(i,k-1)) + ha(i)));
    end
    a(:,k) = a(:,k-1) + dt*(u(:,k-1) - a(:,k-1))/P.tau;  % actuator lag
    v(:,k) = max(0, v(:,k-1) + dt*a(:,k));
    x(:,k) = x(:,k-1) + dt*v(:,k);
  end
  gap = x(1:end-1,:) - x(2:end,:); err = gap - P.d0;
end
