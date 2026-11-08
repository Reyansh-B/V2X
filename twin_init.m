function tw = twin_init(Ka, drift_deg)
% TWIN_INIT  Create the digital twin of one receive array (see twin_step.m).
%   tw = twin_init(Ka, drift_deg)
%   Ka        : number of array elements
%   drift_deg : expected per-packet random-walk drift of each channel's phase bias [deg]

  tw.bd     = zeros(Ka-1, 1);              % learned differential phase bias per element [rad]
  tw.mask   = true(Ka-1, 1);               % elements still trusted
  tw.P      = (6*pi/180)^2;                % uncertainty of the bias estimate [rad^2]
  tw.q      = (1.5*drift_deg*pi/180)^2 + 1e-12;   % process noise per packet
  tw.warm   = 20;                          % packets before health testing starts
  tw.bad    = 0;                           % consecutive failed health tests
  tw.p      = [];                          % last [theta; d] estimate
  tw.detect = zeros(0, 2);                 % rows [time, masked index (1..Ka-1)]
  tw.t_last = [];
end
