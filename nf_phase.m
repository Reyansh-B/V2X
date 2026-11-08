function ph = nf_phase(th, d, x, kw)
% NF_PHASE  Near-field differential phase of each element vs the array centre.
%   ph = nf_phase(th, d, x, kw)
%   th : angle from boresight [rad], d : range to the array centre [m]
%   x  : element positions [m] (any shape), kw : 2*pi/lambda
%   Same model as PhaseMeasurements.m:  phi_k = -kw*(d_k - d)

  dk = sqrt(d.^2 + x.^2 - 2*d.*x.*sin(th));
  ph = -kw*(dk - d);
end
