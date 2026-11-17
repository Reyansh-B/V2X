function meas = nf_phase_meas(theta, d, x, snr, eta, Mp, Np, lam, cal_err)
% NF_PHASE_MEAS  Noisy differential phases across the ULA (Decarli et al. model).
%   meas = nf_phase_meas(theta, d, x, snr, eta, Mp, Np, lam, cal_err)
%   theta, d : true angle [rad] and range [m] of the transmitter
%   x        : 1xKa element positions [m] (centred, Ka odd)
%   snr      : linear per-subcarrier SNR
%   cal_err  : 1xKa fixed per-element phase calibration error [rad] (optional)
% The centre element is the reference, so its differential phase is 0.

  if nargin < 9 || isempty(cal_err), cal_err = zeros(size(x)); end
  Ka = numel(x); kc = (Ka+1)/2; kw = 2*pi/lam;
  dk = sqrt(d^2 + x.^2 - 2*d*x*sin(theta));
  phi = -kw*(dk - d);
  sig_diff = sqrt(2*eta/(2*Mp*Np*snr));        % differential phase noise std
  meas = phi + sig_diff*randn(1,Ka) + (cal_err - cal_err(kc));
  meas = angle(exp(1j*meas));                   % receiver only sees wrapped phase
  meas(kc) = 0;
end
