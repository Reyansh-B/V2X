% Step 3: Monte Carlo Estimator vs Bound
% Proves that the ML estimator achieves the PEB at high SNR.

clear; clc; close all;

% --- Parameters ---
fc      = 5.9e9;
lam     = 299792458 / fc;
SNR_dB  = 20;
SNR_lin = 10^(SNR_dB/10);
eta     = 10^(6/10);        
Mp      = 2;
Np      = 120;

% Array (41 elements, lambda/2)
Ka = 41;
x = (0:Ka-1) * (lam/2);
x = x - mean(x);
kc = (Ka+1)/2; % Center index

% Noise standard deviation for a SINGLE phase measurement
% The paper uses sigma_phi^2 = eta / (2 * Mp * SNR) per subcarrier
% For Np subcarriers, the effective variance is sigma_phi^2 / Np
sigma_phi = sqrt(eta / (2 * Mp * Np * SNR_lin));

% Differential noise variance is 2 * sigma_phi^2
sigma_diff = sqrt(2) * sigma_phi;

% --- Simulation Setup ---
d_test = linspace(2, 10, 9); % Test distances from 2 to 10 m
theta_true = 0;              % Boresight
N_trials = 200;              % Monte Carlo trials

rmse_vals = zeros(size(d_test));
peb_vals  = zeros(size(d_test));

fprintf('Running Monte Carlo (%d trials per distance)...\n', N_trials);

for i = 1:length(d_test)
    d_true = d_test(i);
    
    % 1. Theoretical Bound
    out_bound = nearfieldBounds(theta_true, d_true, x, Mp, Np, SNR_lin, eta, fc, 'paper');
    peb_vals(i) = out_bound.PEB;
    
    % 2. Monte Carlo Simulation
    errors = zeros(N_trials, 1);
    
    % Generate clean phase
    dk_true = sqrt(d_true^2 + x.^2 - 2*d_true*x*sin(theta_true));
    phi_clean = -2*pi/lam * (dk_true - d_true);
    phi_clean(kc) = 0; % Differential phase at center is exactly 0
    
    for t = 1:N_trials
        % Add noise (center antenna gets 0 noise)
        noise = sigma_diff * randn(1, Ka);
        noise(kc) = 0;
        meas = phi_clean + noise;
        
        % Estimation using fminsearch
        % Cost function: wrapped squared error
        cost_fn = @(p) sum( angle(exp(1j * (meas - (-2*pi/lam * (sqrt(p(2)^2 + x.^2 - 2*p(2)*x*sin(p(1))) - p(2)))))).^2 );
        
        % Initial guess: perturb the true position slightly
        p0 = [theta_true + randn()*0.01, d_true + randn()*0.5];
        
        % Optimize (suppress output)
        options = optimset('Display', 'off', 'MaxFunEvals', 2000, 'MaxIter', 2000);
        p_hat = fminsearch(cost_fn, p0, options);
        
        % Convert polar estimate to Cartesian to compute error
        x_hat = p_hat(2) * sin(p_hat(1));
        y_hat = p_hat(2) * cos(p_hat(1));
        x_true = d_true * sin(theta_true);
        y_true = d_true * cos(theta_true);
        
        % Store error magnitude
        errors(t) = sqrt((x_hat - x_true)^2 + (y_hat - y_true)^2);
    end
    
    % Compute RMSE for this distance
    rmse_vals(i) = sqrt(mean(errors.^2));
    fprintf('d = %2.1f m | PEB = %.4f m | RMSE = %.4f m\n', d_true, peb_vals(i), rmse_vals(i));
end

% --- Plotting ---
figure('Color','w','Position',[100 100 600 450]);
semilogy(d_test, peb_vals, 'k--', 'LineWidth', 2, 'DisplayName', 'Theoretical PEB');
hold on;
semilogy(d_test, rmse_vals, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b', 'DisplayName', 'Estimator RMSE');
grid on; box on;
xlabel('Distance d [m]');
ylabel('Error [m]');
title(sprintf('Estimator RMSE vs PEB (SNR = %d dB, %d trials)', SNR_dB, N_trials));
legend('Location', 'northwest');
print(gcf, 'rmse_vs_peb.png', '-dpng', '-r150');