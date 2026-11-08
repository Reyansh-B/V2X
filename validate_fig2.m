% Reproduce Fig. 2 of Decarli et al., JSAC 2023.
% PEB/REB error map in front of a 41-element ULA at 5.9 GHz, SNR = 20 dB.

clear; clc; close all;

% --- Paper parameters (Fig. 2 + Table II) ---
fc      = 5.9e9;
SNR_dB  = 20;
SNR_lin = 10^(SNR_dB/10);
eta     = 10^(6/10);        % 6 dB implementation loss
Mp      = 2;
Np      = 120;              % 20 PRBs x 6 DMRS subcarriers/PRB

% --- ULA: 41 elements, lambda/2 spacing ---
Ka  = 41;
lam = 299792458 / fc;
x   = (0:Ka-1) * (lam/2);
x   = x - mean(x);

% --- Sanity checks ---
fprintf('aperture           = %.4f m   (expected ~1.0)\n', x(end) - x(1));
fprintf('Fraunhofer dist.   = %.2f m   (expected ~39 m at D=1m)\n', ...
        2*(x(end)-x(1))^2*fc/299792458);

% --- Grid in front of the array ---
xx = linspace(-10, 10, 81);
yy = linspace(  1, 50, 100);
[XX, YY] = meshgrid(xx, yy);
d     = sqrt(XX.^2 + YY.^2);
theta = atan2(XX, YY);

% --- Bound over the grid ---
out = nearfieldBounds(theta, d, x, Mp, Np, SNR_lin, eta, fc, 'paper');

% --- Point check at boresight, 10 m ---
chk = nearfieldBounds(0, 10, x, Mp, Np, SNR_lin, eta, fc, 'paper');
fprintf('PEB at (0, 10)     = %.3e m\n', chk.PEB);
fprintf('REB at (0, 10)     = %.3e m\n', chk.REB);

% --- Plot ---
figure('Color','w','Position',[100 100 950 430]);

subplot(1,2,1);
imagesc(xx, yy, log10(out.REB)); axis xy; axis equal;
cb = colorbar; cb.Label.String = 'log_{10} REB [m]';
xlabel('x [m]'); ylabel('y [m]'); title('REB');
colormap(flipud(hot));

subplot(1,2,2);
imagesc(xx, yy, log10(out.PEB)); axis xy; axis equal;
cb = colorbar; cb.Label.String = 'log_{10} PEB [m]';
xlabel('x [m]'); ylabel('y [m]'); title('PEB');
colormap(flipud(hot));

sgtitle(sprintf('Fig. 2 reproduction: D=%.2f m, f_c=%.1f GHz, SNR=%d dB', ...
        x(end)-x(1), fc/1e9, SNR_dB));

print(gcf, 'fig2_matlab.png', '-dpng', '-r150');