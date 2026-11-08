function p0 = nf_coarse(obs, x, kw, kc, spacing)
% NF_COARSE  Starting point for nf_fit when there is no previous estimate.
%   p0 = nf_coarse(obs, x, kw, kc, spacing)
%   obs : (Ka-1) differential phases (centre element removed), x : their positions
%   kc  : index of the centre element in the full array, spacing : element spacing [m]
%   Angle from the mean phase step between neighbours (no wrapping for spacing <= lambda/2),
%   then range by a 1-D search at that angle.

  obs = obs(:); x = x(:);
  full  = [obs(1:kc-1); 0; obs(kc:end)];
  steps = angle(exp(1j*diff(full)));
  th0   = asin(max(-1, min(1, mean(steps)/(kw*spacing))));
  dgrid = logspace(0, log10(400), 120);
  cost  = zeros(size(dgrid));
  for q = 1:numel(dgrid)
    cost(q) = sum(angle(exp(1j*(obs - nf_phase(th0, dgrid(q), x, kw)))).^2);
  end
  [~, iq] = min(cost);
  p0 = [th0; dgrid(iq)];
end
