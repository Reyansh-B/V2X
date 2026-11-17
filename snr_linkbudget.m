function s = snr_linkbudget(d, lam, margin_dB)
% SNR_LINKBUDGET  LOS free-space per-subcarrier SNR (linear) at range d [m].
%   Values follow Decarli et al. Table II: PT = 23 dBm, 3 dBi antennas each side,
%   10 MHz bandwidth, 9 dB noise figure. margin_dB is an extra loss (default 10 dB,
%   PLACEHOLDER: replace with a measured/standard sidelink margin and cite it).
  if nargin < 3 || isempty(margin_dB), margin_dB = 10; end
  fspl = 20*log10(4*pi*max(d, 0.5)/lam);
  pr   = 23 + 2*3 - fspl - margin_dB;            % dBm
  nz   = -174 + 10*log10(10e6) + 9;              % dBm
  s    = 10.^((pr - nz)/10);
end
