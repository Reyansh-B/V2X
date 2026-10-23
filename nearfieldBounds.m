function out = nearfieldBounds(theta, d, x, Mp, Np, snr, eta, fc, mode)
% NEARFIELD BOUNDS  Fisher information + BEB/REB/PEB for near-field V2X sidelink.
%
%   out = nearfieldBounds(theta, d, x, Mp, Np, snr, eta, fc, mode)
%
%   theta : angle(s) from boresight, rad (scalar or array)
%   d     : range(s) from the array centre, m (same size as theta)
%   x     : 1xK element positions, m
%   Mp,Np : pilot OFDM symbols / subcarriers
%   snr   : linear per-subcarrier SNR (NOT dB)
%   eta   : implementation loss factor (>=1)
%   fc    : carrier, Hz (default 5.9e9)
%   mode  : 'paper' (differential obs, variance 2 sigma_phi^2) or 'nuisance'
%
%   Returns: BEB [rad], REB [m], lateral [m], longitudinal [m], PEB [m],
%            and Jtt, Jtd, Jdd (polar FIM entries).

    if nargin < 8 || isempty(fc),   fc   = 5.9e9; end
    if nargin < 9 || isempty(mode), mode = 'paper'; end

    c0  = 299792458;
    lam = c0 / fc;
    k   = 2*pi / lam;

    th = theta(:);
    dd = d(:);
    xr = x(:).';

    % TX -> k-th antenna distance, N x K
    dk = sqrt(dd.^2 + xr.^2 - 2*dd.*xr.*sin(th));

    % Derivatives of dk wrt theta and d
    d_th = -dd .* xr .* cos(th) ./ dk;
    d_d  = (dd - xr .* sin(th)) ./ dk;

    switch lower(mode)
        case 'paper'
            g_th = -k * d_th;
            g_d  = -k * (d_d - 1);
            pre  = Mp * Np * snr / eta;
        case 'nuisance'
            g_th = -k * d_th;
            g_d  = -k * d_d;
            g_th = g_th - mean(g_th, 2);
            g_d  = g_d  - mean(g_d,  2);
            pre  = 2 * Mp * Np * snr / eta;
        otherwise
            error('mode must be paper or nuisance');
    end

    Jtt = pre * sum(g_th.^2,      2);
    Jtd = pre * sum(g_th.*g_d,    2);
    Jdd = pre * sum(g_d.^2,       2);

    % Polar -> Cartesian (paper Eq. 28)
    s = sin(th);  c = cos(th);
    c11 = c ./ dd;   c12 = -s ./ dd;
    c21 = s;         c22 =  c;

    Jxx = c11.^2 .* Jtt + 2*c11.*c21        .* Jtd + c21.^2 .* Jdd;
    Jxy = c11.*c12 .* Jtt + (c11.*c22+c12.*c21) .* Jtd + c21.*c22 .* Jdd;
    Jyy = c12.^2 .* Jtt + 2*c12.*c22        .* Jtd + c22.^2 .* Jdd;

    detJ  = Jxx.*Jyy - Jxy.^2;
    var_x =  Jyy ./ detJ;
    var_y =  Jxx ./ detJ;

    detp  = Jtt.*Jdd - Jtd.^2;

    out.BEB          = reshape(sqrt(Jdd ./ detp),          size(theta));
    out.REB          = reshape(sqrt(Jtt ./ detp),          size(theta));
    out.lateral      = reshape(sqrt(var_x),                size(theta));
    out.longitudinal = reshape(sqrt(var_y),                size(theta));
    out.PEB          = reshape(sqrt(var_x + var_y),        size(theta));
    out.Jtt          = reshape(Jtt, size(theta));
    out.Jtd          = reshape(Jtd, size(theta));
    out.Jdd          = reshape(Jdd, size(theta));
end