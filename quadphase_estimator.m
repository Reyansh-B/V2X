% quadphase_estimator.m
% Low-complexity (FPGA-friendly) near-field estimator vs ML vs CRLB.
%
% Idea: Fresnel expansion of the differential phase across the ULA
%   phi_k = -(2*pi/lam)*(d_k - d) ~= a*x_k + b*x_k^2
%   a = k*sin(theta),  b = -k*cos(theta)^2/(2*d)
% -> theta from linear term, d from quadratic term.
% The fit is ONE fixed 2xKa matrix times the unwrapped phase vector
% (precomputed pseudo-inverse) => trivial to put on FPGA.
% Optional: one ML refinement initialised from this estimate.
%
% Needs nearfieldBounds.m from your repo on the path.
clear; clc; close all; rng(1);

c0 = 299792458; fc = 5.9e9; lam = c0/fc; kw = 2*pi/lam;
Ka = 41; x = (0:Ka-1)*(lam/2); x = x - mean(x); kc = (Ka+1)/2;
Mp = 2; Np = 120; eta = 10^(6/10);

d_list   = [2 5 10 20 40];
th_list  = [0 0.3];          % rad
snr_list = [20 10];          % dB
Nt = 200;                    % Monte Carlo trials per point

Apinv = pinv([x(:) x(:).^2]);    % <-- the only "matrix" the FPGA needs (2 x Ka)

res = [];
for snr_dB = snr_list
  snr = 10^(snr_dB/10);
  sig_diff = sqrt(2*eta/(2*Mp*Np*snr));      % differential phase noise std
  for th = th_list
    for d = d_list
      o = nearfieldBounds(th, d, x, Mp, Np, snr, eta, fc, 'paper');
      peb = o.PEB;
      phi_clean = -kw*(sqrt(d^2 + x.^2 - 2*d*x*sin(th)) - d);
      eq = zeros(Nt,1); em = zeros(Nt,1);
      for t = 1:Nt
        n = sig_diff*randn(1,Ka); n(kc) = 0;
        meas = phi_clean + n; meas(kc) = 0;
        [thq, dq] = quadfit(meas, kc, kw, Apinv);
        cost = @(p) sum(angle(exp(1j*(meas - (-kw*(sqrt(p(2)^2 + x.^2 - 2*p(2)*x*sin(p(1))) - p(2)))))).^2);
        p = fminsearch(cost, [thq dq], optimset('Display','off','TolX',1e-8,'TolFun',1e-12));
        eq(t) = perr(thq, dq, th, d);
        em(t) = perr(p(1), p(2), th, d);
      end
      res(end+1,:) = [snr_dB th d peb sqrt(mean(eq.^2)) sqrt(mean(em.^2))]; %#ok<SAGROW>
      fprintf('SNR %2d dB th %.1f d %4.0f | PEB %.2e | quadLS %.2e | quad+ML %.2e\n', res(end,:));
    end
  end
end

% Plot: theta = 0, SNR = 20 dB
m = res(:,1)==20 & res(:,2)==0;
figure('Color','w'); semilogy(res(m,3), res(m,4), 'k--', 'LineWidth', 2); hold on;
semilogy(res(m,3), res(m,5), 'rs-', 'LineWidth', 1.5);
semilogy(res(m,3), res(m,6), 'bo-', 'LineWidth', 1.5);
grid on; xlabel('d [m]'); ylabel('Position RMSE [m]');
legend('PEB (CRLB)', 'Quadratic LS (FPGA-friendly)', 'Quadratic LS + 1 ML refinement', 'Location', 'northwest');
title('Low-complexity near-field estimator, SNR = 20 dB, \theta = 0');
print(gcf, 'quadphase_vs_peb.png', '-dpng', '-r150');

% ---------------- local functions ----------------
function [th, d] = quadfit(meas, kc, kw, Apinv)
  ur = unwrap(meas(kc:end));          % centre -> right
  ul = unwrap(meas(kc:-1:1));         % centre -> left
  u  = [fliplr(ul(2:end)) ur];        % full unwrapped profile, centre = 0
  coef = Apinv*u(:);
  a = coef(1); b = coef(2);
  th = asin(max(-1, min(1, a/kw)));
  if b < 0, d = -kw*cos(th)^2/(2*b); else, d = 1e3; end
end

function e = perr(th_h, d_h, th, d)
  e = hypot(d_h*sin(th_h) - d*sin(th), d_h*cos(th_h) - d*cos(th));
end
