function [tw, th, d, qdof, healthy] = twin_step(tw, t, obs, s2, x, kw, kc, spacing, Tg)
% TWIN_STEP  One packet through the digital twin of a receive array.
%   [tw, th, d, qdof, healthy] = twin_step(tw, t, obs, s2, x, kw, kc, spacing, Tg)
%
%   tw   : twin state from twin_init (returned updated)
%   t    : packet time [s]
%   obs  : (Ka-1) measured differential phases [rad]
%   s2   : variance of ONE element's pilot-averaged phase = eta/(2*Mp*Np*SNR)
%   x    : (Ka-1) element positions (centre element removed), kw = 2*pi/lambda
%   kc   : centre-element index of the full array, spacing : element spacing [m]
%   Tg   : nominal packet period [s] (default 0.1), ages the bias uncertainty over lost packets
%
%   Per packet the twin:
%     1. PREDICTS  : subtracts its learned per-element phase bias
%     2. FITS      : position with nf_fit on the corrected phases
%     3. TESTS     : chi-square health test of the fit residual (noise + own bias uncertainty)
%     4. LOCATES   : after 3 failed tests in a row, finds the faulty element by
%                    leave-one-out and masks it (at most 3 elements)
%     5. LEARNS    : Kalman update of the bias, only from healthy packets
%
%   Limitation (by design, see README): from the packets alone the twin can only learn
%   the part of the bias that does NOT look like a change of (theta, d) -- a linear plus
%   quadratic phase profile across the array. That part needs an external reference
%   (a known-position beacon, or the radar as in platoon_twin_v3.m).
%
%   chi2inv without the Statistics Toolbox: chi2inv(p,k) = 2*gammaincinv(p,k/2)

  if nargin < 9, Tg = 0.1; end
  if isempty(tw.t_last), age = 1; else, age = max(1, (t - tw.t_last)/Tg); end
  tw.P = tw.P + tw.q*age;
  tw.t_last = t;

  oc = angle(exp(1j*(obs(:) - tw.bd)));
  if isempty(tw.p), p0 = nf_coarse(oc, x, kw, kc, spacing); else, p0 = tw.p; end
  [th, d, r, f] = nf_fit(oc, x, kw, tw.mask, p0);

  m   = sum(tw.mask);
  thr = 2*gammaincinv(1 - 1e-4, (m - 2)/2);
  s2e = s2 + tw.P/2;            % innovation variance: noise + uncertainty of our own bias estimate
  q   = f/s2e;

  if tw.warm <= 0
    if q > thr, tw.bad = tw.bad + 1; else, tw.bad = 0; end
    if tw.bad >= 3 && sum(~tw.mask) < 3
      best = inf; jb = 0;
      for j = find(tw.mask).'
        mk = tw.mask; mk(j) = false;
        [~, ~, ~, fj] = nf_fit(oc, x, kw, mk, [th; d]);
        if fj < best, best = fj; jb = j; end
      end
      tw.mask(jb) = false;
      tw.detect(end+1, :) = [t, jb];
      [th, d, r, f] = nf_fit(oc, x, kw, tw.mask, [th; d]);
      m   = sum(tw.mask);
      thr = 2*gammaincinv(1 - 1e-4, (m - 2)/2);
      q   = f/s2e;
      tw.bad = 0;
    end
  end

  healthy = q <= thr;
  if tw.warm > 0 || healthy
    Kg = tw.P/(tw.P + 2*s2);
    tw.bd(tw.mask) = tw.bd(tw.mask) + Kg*r(tw.mask);
    tw.P = tw.P*(1 - Kg);
  end
  tw.warm = tw.warm - 1;
  tw.p = [th; d];
  qdof = q/max(m - 2, 1);
end
