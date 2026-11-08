function [th, d, r, f] = nf_fit(obs, x, kw, mask, p0)
% NF_FIT  Fast near-field position fit (Levenberg-Marquardt, analytic Jacobian).
%   [th, d, r, f] = nf_fit(obs, x, kw, mask, p0)
%
%   obs  : (Ka-1) differential phases of the non-centre elements [rad] (wrapped is fine)
%   x    : (Ka-1) positions of those elements [m]
%   kw   : 2*pi/lambda
%   mask : (Ka-1) logical, elements to use (the twin switches faulty ones off)
%   p0   : [theta0; d0] starting point (e.g. from nf_coarse or the previous packet)
%
%   th, d : estimate [rad, m]
%   r     : (Ka-1) wrapped residual obs - model (0 for masked elements)
%   f     : whitened cost.  Differential phases share the centre element's
%           noise, so their covariance is s2*(I + 11'); we whiten with
%           chol(inv(I + 11')) = chol(I - 11'/(1+m)).
%
%   Much faster than fminsearch (needed: it runs once per packet per vehicle).
%   No toolboxes required.

  wrapph = @(a) angle(exp(1j*a));
  obs = obs(:); x = x(:); mask = logical(mask(:));
  xm = x(mask); om = obs(mask); m = numel(xm);
  R = chol(eye(m) - ones(m)/(1+m));
  resid = @(p) R*wrapph(om - nf_phase(p(1), p(2), xm, kw));

  p = p0(:); lam = 1e-3;
  r = resid(p); f = r.'*r;
  for it = 1:50
    dk = sqrt(p(2)^2 + xm.^2 - 2*p(2)*xm*sin(p(1)));
    dph_dth = kw*p(2)*xm*cos(p(1))./dk;
    dph_dd  = -kw*((p(2) - xm*sin(p(1)))./dk - 1);
    J = -R*[dph_dth, dph_dd];
    A = J.'*J; g = J.'*r;
    ok = false;
    for tr = 1:10
      delta = -(A + lam*diag(diag(A)) + 1e-30*eye(2)) \ g;
      pn = p + delta;
      pn(1) = min(max(pn(1), -1.3), 1.3);
      pn(2) = min(max(pn(2), 1), 500);
      rn = resid(pn); fn = rn.'*rn;
      if fn < f
        p = pn; r = rn; f = fn; lam = max(lam/10, 1e-12); ok = true;
        break;
      end
      lam = lam*10;
    end
    if ~ok || (abs(delta(1)) < 1e-12 && abs(delta(2)) < 1e-10*max(p(2), 1))
      break;
    end
  end
  th = p(1); d = p(2);
  r = zeros(numel(x), 1);
  r(mask) = wrapph(om - nf_phase(th, d, xm, kw));
end
