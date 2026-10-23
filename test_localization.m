% test_localization.m
clear; clc;

% Setup
fc = 5.9e9; lam = 299792458/fc;
Ka = 41; x = (0:Ka-1)*(lam/2); x = x - mean(x);
theta_true = 0; d_true = 5; SNR_dB = 20;

% 1. Generate Data (using logic from PhaseMeasurements.m)
eta = 10^(6/10); Mp = 2; Np = 120;
SNR_lin = 10^(SNR_dB/10);
sigma_diff = sqrt(2 * (eta / (2 * Mp * Np * SNR_lin)));

dk = sqrt(d_true^2 + x.^2 - 2*d_true*x*sin(theta_true));
phi_clean = -2*pi/lam * (dk - d_true);

noise = sigma_diff * randn(1, Ka); 
noise((Ka+1)/2) = 0; % Center antenna
meas = phi_clean + noise;

% 2. Estimate Position (using GridSearch.m)
[theta_hat, d_hat] = GridSearch(meas, x, lam, pi/3, 20);

% 3. Print Results
fprintf('\n--- Estimation Results ---\n');
fprintf('True Position     : theta = %.2f deg, d = %.2f m\n', rad2deg(theta_true), d_true);
fprintf('Estimated Position: theta = %.2f deg, d = %.2f m\n', rad2deg(theta_hat), d_hat);
fprintf('Distance Error    : %.2f mm\n', abs(d_hat - d_true) * 1000);