function [theta, d] = nf_estimate(meas, x, lam, n_gn, Apinv)
% NF_ESTIMATE  Low-complexity near-field position estimate.
%   [theta, d] = nf_estimate(meas, x, lam, n_gn, Apinv)
%   Step 1: quadratic (Fresnel) LS fit of the unwrapped differential phase,
%           phi_k ~ a*x_k + b*x_k^2,  a = k sin(theta),  b = -k cos^2(theta)/(2d),
%           i.e. ONE fixed 2xKa matrix (Apinv) times the phase vector.
%   Step 2: n_gn Gauss-Newton iterations on the exact spherical model with
%           wrapped residuals (n_gn = 0 -> quadratic fit only).
%   Apinv : optional precomputed pinv([x(:) x(:).^2]).

  if nargin < 4 || isempty(n_gn), n_gn = 0; end
  if nargin < 5 || isempty(Apinv), Apinv = pinv([x(:) x(:).^2]); end
  kw = 2*pi/lam; Ka = numel(x); kc = (Ka+1)/2;
  meas = meas(:).'; x = x(:).';

  % --- step 1: unwrap outward from the centre element, then fit ---
  ur = unwrap(meas(kc:end));
  ul = unwrap(meas(kc:-1:1));
  u  = [fliplr(ul(2:end)) ur];
  coef = Apinv*u(:);
  theta = asin(max(-1, min(1, coef(1)/kw)));
  if coef(2) < 0
    d = -kw*cos(theta)^2/(2*coef(2));
  else
    d = 1e3;                                    % no curvature seen: far field
  end
  d = min(max(d, 0.3), 1e3);

  % --- step 2: Gauss-Newton refinement on the exact model ---
  for it = 1:n_gn
    dk  = sqrt(d^2 + x.^2 - 2*d*x*sin(theta));
    mdl = -kw*(dk - d);
    r   = angle(exp(1j*(meas - mdl)));          % wrapped residual
    J   = [ -kw*(-d*x*cos(theta)./dk) ; -kw*((d - x*sin(theta))./dk - 1) ].';
    step = (J.'*J) \ (J.'*r(:));
    theta = theta + step(1);
    d     = max(0.3, d + step(2));
  end
end
