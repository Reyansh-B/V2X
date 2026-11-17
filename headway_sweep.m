% headway_sweep.m
% Minimum time headway h at which each sensing scheme keeps the platoon
% collision-free and (empirically) string stable, with and without V2X loss.
%
% Empirical criterion over Nruns Monte Carlo runs of the manoeuvre:
%   min gap >= gap_safe in EVERY run (no collision, safety margin kept)  AND
%   mean worst-case acceleration amplification
%   gamma_a = max_i ||a_i||_2 / ||a_{i-1}||_2  <= 1 + tol.
% NOTE: this is a time-domain check on one manoeuvre. For the paper, pair it
% with the frequency-domain condition |Gamma(jw)| <= 1 (Ploeg et al. 2014).
clear; clc; close all;

Nruns = 10; tol = 0.01; gap_safe = 1.0;   % [m]
h_grid = 0.1:0.1:1.6;
base = platoon_config();
schemes = {
  'ideal',           'ideal', {};
  'ACC (radar)',     'acc',   {};
  'CACC radar',      'radar', {};
  'CACC GNSS-DGNSS', 'gnss',  {'gnss.sig_rel', 0.5};
  'CACC GNSS-RTK',   'gnss',  {'gnss.sig_rel', 0.02};
  'CACC NF',         'nf',    {'nf.sig_cal', 0};
  'CACC NF cal3deg', 'nf',    {'nf.sig_cal', 3*pi/180};
};
losses = {'none', 'ge'};

hmin = nan(size(schemes,1), numel(losses));
G = nan(size(schemes,1), numel(h_grid), numel(losses));
for L = 1:numel(losses)
  for m = 1:size(schemes,1)
    for j = 1:numel(h_grid)
      cfg = base; cfg.h = h_grid(j); cfg.loss.type = losses{L};
      ov = schemes{m,3};
      for q = 1:2:numel(ov), p = strsplit(ov{q}, '.'); cfg = setfield(cfg, p{:}, ov{q+1}); end
      mg = inf; g = zeros(Nruns,1);
      for r = 1:Nruns
        o = platoon_sim(cfg, schemes{m,2}, r);
        mg = min(mg, o.min_gap); g(r) = o.gamma_a;
        if mg < gap_safe, break; end      % already failed
      end
      G(m,j,L) = mean(g(1:r));
      if mg >= gap_safe && mean(g) <= 1 + tol
        hmin(m,L) = h_grid(j); break;   % smallest passing h
      end
    end
    fprintf('loss=%-5s %-16s h_min = %.1f s\n', losses{L}, schemes{m,1}, hmin(m,L));
  end
end

fid = fopen('headway_sweep.csv', 'w');
fprintf(fid, 'scheme,hmin_noloss,hmin_ge\n');
for m = 1:size(schemes,1)
  fprintf(fid, '%s,%.2f,%.2f\n', schemes{m,1}, hmin(m,1), hmin(m,2));
end
fclose(fid);

plot_headway_sweep;   % figure from headway_sweep.csv
