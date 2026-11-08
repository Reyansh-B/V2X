% Reproduce Fig. 6 of Decarli et al., JSAC 2023.
% Platooning PEB vs distance for different array configurations.

clear; clc; close all;

% --- Parameters ---
fc      = 5.9e9;
SNR_dB  = 20;
SNR_lin = 10^(SNR_dB/10);
eta     = 10^(6/10);        % 6 dB
Mp      = 2;
Np      = 120;
lam     = 299792458 / fc;

% --- Distances 1 to 10 m ---
d = linspace(1, 10, 50);
theta = 0; % platooning is straight ahead

% --- Configurations ---
Ka_list = [41, 31, 21, 11];
D_max = 1.20; % meters (120 cm)
colors = lines(length(Ka_list));

figure('Color','w','Position',[100 100 700 500]); hold on;

% 1) Fixed spacing = lambda/2 (Solid lines)
for i = 1:length(Ka_list)
    Ka = Ka_list(i);
    x = (0:Ka-1) * (lam/2);
    x = x - mean(x); % center on 0
    
    PEB_vals = zeros(1, length(d));
    for j = 1:length(d)
        out = nearfieldBounds(theta, d(j), x, Mp, Np, SNR_lin, eta, fc, 'paper');
        PEB_vals(j) = out.PEB;
    end
    semilogy(d, PEB_vals, '-', 'LineWidth', 1.5, 'Color', colors(i,:), ...
             'DisplayName', sprintf('K_a = %d, D = %.0f cm', Ka, (x(end)-x(1))*100));
end

% 2) Fixed array size D = 120 cm (Dashed lines)
for i = 1:length(Ka_list)
    Ka = Ka_list(i);
    x = linspace(-D_max/2, D_max/2, Ka);
    
    PEB_vals = zeros(1, length(d));
    for j = 1:length(d)
        out = nearfieldBounds(theta, d(j), x, Mp, Np, SNR_lin, eta, fc, 'paper');
        PEB_vals(j) = out.PEB;
    end
    semilogy(d, PEB_vals, '--', 'LineWidth', 1.5, 'Color', colors(i,:), ...
             'HandleVisibility', 'off'); % hide dashed from legend to match paper
end

% --- Formatting ---
grid on; box on;
% Add a dummy line to represent the dashed lines in the legend
h_dummy = plot(NaN, NaN, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Fixed D = 120 cm (dashed)');

% Let MATLAB build the legend from the DisplayNames
legend('Location', 'northwest');

% Ensure log scale is applied explicitly
set(gca, 'YScale', 'log');
ylim([1e-4, 1e-1]);

print(gcf, 'fig6_matlab.png', '-dpng', '-r150');