function u = leader_profile(t, scenario)
% LEADER_PROFILE  Commanded leader acceleration [m/s^2] over time vector t.
%   'brake'  : 25 m/s, brake at -4 m/s^2 for 4 s (t = 4..8 s), then cruise at 9 m/s
%   'stopgo' : 25 m/s -> brake -3 m/s^2 to ~1 m/s -> crawl -> accelerate +1.5 m/s^2
%              (brings the gaps down to the 2-5 m range where Fresnel bias matters)
  u = zeros(size(t));
  switch lower(scenario)
    case 'brake'
      u(t > 4 & t < 8) = -4;
    case 'stopgo'
      u(t > 4 & t < 12) = -3;          % 25 -> 1 m/s
      u(t > 20 & t < 32) = 1.5;        % 1 -> 19 m/s
    otherwise
      error('unknown scenario %s', scenario);
  end
end
