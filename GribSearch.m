%% Part 4: Grid Search Position Estimator
% Find the transmitter position that best matches the phase measurements

clear; close all; clc;

% ============================================
% 1. SETUP: Constants and Array Configuration
% ============================================

c = 3e8;                    % Speed of light (m/s)
fc = 5.9e9;                 % Carrier frequency: 5.9 GHz
lambda = c / fc;            % Wavelength (about 5 cm)
d_antenna = lambda / 2;     % Half-wavelength spacing

% Array configuration
K = 31;                     % 41 antennas
D = (K - 1) * d_antenna;    % Total array size (about 1 meter)
x_ant = linspace(-D/2, D/2, K);

% Make x_ant a column vector
x_ant = x_ant(:);

% ============================================
% 2. TRUE TRANSMITTER POSITION (Unknown to estimator)
% ============================================

d_true = 15;                % 15 meters away
theta_true = 5 * pi/180;    % 5 degrees off boresight

fprintf('=== True Transmitter Position ===\n');
fprintf('Distance: %.2f m\n', d_true);
fprintf('Angle: %.2f degrees\n', theta_true * 180/pi);
fprintf('Array size: %.2f m (%d antennas)\n\n', D, K);

% ============================================
% 3. GENERATE NOISY MEASUREMENTS
% ============================================

% Ideal phase (no noise)
d_k_true = sqrt(x_ant.^2 + d_true^2 - 2*x_ant*d_true*sin(theta_true));
phase_ideal = -2*pi/lambda * (d_k_true - d_true);

% Add noise
SNR_dB = 5;                % High SNR for good estimation
SNR_linear = 10^(SNR_dB/10);
phase_noise_std = sqrt(1 / (2 * SNR_linear));
phase_measured = phase_ideal + phase_noise_std * randn(K, 1);

% ★★★ FIX: Force column vector ★★★
phase_measured = phase_measured(:);

fprintf('=== Measurement Parameters ===\n');
fprintf('SNR: %d dB\n', SNR_dB);
fprintf('Phase noise std: %.2f degrees\n\n', phase_noise_std * 180/pi);

% ============================================
% 4. GRID SEARCH: Try All Possible Positions
% ============================================

% Define the search grid
search_d = 5:0.5:50;            % Distances from 5m to 50m (step 0.5m)
search_theta = -30:0.5:30;      % Angles from -30 to +30 degrees (step 0.5°)
search_theta_rad = search_theta * pi/180;

fprintf('=== Grid Search Parameters ===\n');
fprintf('Distance range: %.1f to %.1f m (step %.1f m)\n', ...
        search_d(1), search_d(end), search_d(2)-search_d(1));
fprintf('Angle range: %.1f to %.1f degrees (step %.1f°)\n', ...
        search_theta(1), search_theta(end), search_theta(2)-search_theta(1));
fprintf('Total evaluations: %d\n\n', length(search_d) * length(search_theta));

% Initialize error matrix
error_matrix = zeros(length(search_d), length(search_theta));

% Loop over all possible positions
fprintf('Searching grid...\n');
for i = 1:length(search_d)
    d_candidate = search_d(i);
    
    for j = 1:length(search_theta)
        theta_candidate = search_theta_rad(j);
        
        % Calculate expected phase for this candidate position
        d_k_candidate = sqrt(x_ant.^2 + d_candidate^2 - ...
                             2*x_ant*d_candidate*sin(theta_candidate));
        phase_candidate = -2*pi/lambda * (d_k_candidate - d_candidate);
        
        % ★★★ FIX: Force column vector ★★★
        phase_candidate = phase_candidate(:);
        
        % Calculate error (sum of squared differences)
        error_matrix(i, j) = sum((phase_measured - phase_candidate).^2);
    end
end
fprintf('Search complete!\n\n');

% ============================================
% 5. FIND THE BEST MATCH
% ============================================

% Find the minimum error in the matrix
[min_error, min_idx] = min(error_matrix(:));

% Convert linear index to (distance, angle) indices
[d_idx, theta_idx] = ind2sub(size(error_matrix), min_idx);

% Get the estimated position
d_estimated = search_d(d_idx);
theta_estimated = search_theta(theta_idx);

% ============================================
% 6. DISPLAY RESULTS
% ============================================

fprintf('=== Estimation Results ===\n');
fprintf('True position:\n');
fprintf('  Distance: %.2f m\n', d_true);
fprintf('  Angle: %.2f degrees\n', theta_true * 180/pi);
fprintf('\nEstimated position:\n');
fprintf('  Distance: %.2f m\n', d_estimated);
fprintf('  Angle: %.2f degrees\n', theta_estimated);
fprintf('\nErrors:\n');
fprintf('  Distance error: %.2f m (%.1f%%)\n', ...
        abs(d_estimated - d_true), ...
        100 * abs(d_estimated - d_true) / d_true);
fprintf('  Angle error: %.2f degrees (%.1f%%)\n', ...
        abs(theta_estimated - theta_true*180/pi), ...
        100 * abs(theta_estimated - theta_true*180/pi) / (theta_true*180/pi));

% ============================================
% 7. PLOT RESULTS
% ============================================

figure('Position', [100, 100, 1200, 800]);

% --- Plot 1: Error Surface ---

subplot(2, 2, 1);
imagesc(search_theta, search_d, log10(error_matrix));
colorbar;
xlabel('Angle (degrees)', 'FontSize', 12);
ylabel('Distance (m)', 'FontSize', 12);
title('Log10(Estimation Error Surface)', 'FontSize', 14);
hold on;

% Mark the true position (white star)
plot(theta_true*180/pi, d_true, 'w*', 'MarkerSize', 15, 'LineWidth', 2, ...
     'DisplayName', 'True Position');

% Mark the estimated position (red circle)
plot(theta_estimated, d_estimated, 'ro', 'MarkerSize', 10, 'LineWidth', 2, ...
     'DisplayName', 'Estimated Position');

legend('Location', 'best');

% --- Plot 2: Phase Matching ---

subplot(2, 2, 2);

% Calculate phase for the estimated position
d_k_best = sqrt(x_ant.^2 + d_estimated^2 - ...
                2*x_ant*d_estimated*sin(theta_estimated*pi/180));
phase_best = -2*pi/lambda * (d_k_best - d_estimated);

% Plot all three phase patterns
plot(x_ant*100, phase_ideal*180/pi, 'b-', 'LineWidth', 2, ...
     'DisplayName', 'Ideal (No Noise)');
hold on;
plot(x_ant*100, phase_measured*180/pi, 'ro', 'MarkerSize', 4, ...
     'DisplayName', 'Measured (Noisy)');
plot(x_ant*100, phase_best*180/pi, 'g--', 'LineWidth', 2, ...
     'DisplayName', 'Best Fit (Estimated)');

xlabel('Antenna Position (cm)', 'FontSize', 12);
ylabel('Phase Difference (degrees)', 'FontSize', 12);
title('Phase Matching: Measured vs Best Fit', 'FontSize', 14);
legend('Location', 'best');
grid on;
xlim([-D/2*100, D/2*100]);
ylim([-100, 100]);

% --- Plot 3: Error vs Distance ---

subplot(2, 2, 3);

% For each distance, find the minimum error across angles
[min_error_vs_dist, ~] = min(error_matrix, [], 2);

plot(search_d, min_error_vs_dist, 'b-', 'LineWidth', 2);
xlabel('Distance (m)', 'FontSize', 12);
ylabel('Minimum Error', 'FontSize', 12);
title('Error vs Distance (Best Angle at Each Distance)', 'FontSize', 14);
grid on;

% Mark the true distance
line([d_true, d_true], ylim, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
text(d_true, max(ylim)*0.9, sprintf('True d=%.1fm', d_true), ...
     'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'r');

% --- Plot 4: Error vs Angle ---

subplot(2, 2, 4);

% For each angle, find the minimum error across distances
[min_error_vs_angle, ~] = min(error_matrix, [], 1);

plot(search_theta, min_error_vs_angle, 'b-', 'LineWidth', 2);
xlabel('Angle (degrees)', 'FontSize', 12);
ylabel('Minimum Error', 'FontSize', 12);
title('Error vs Angle (Best Distance at Each Angle)', 'FontSize', 14);
grid on;

% Mark the true angle
line([theta_true*180/pi, theta_true*180/pi], ylim, ...
     'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
text(theta_true*180/pi, max(ylim)*0.9, sprintf('True θ=%.1f°', theta_true*180/pi), ...
     'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'r');

fprintf('\n=== Part 4 Complete ===\n');
fprintf('You should see a figure with 4 subplots:\n');
fprintf('1. Error surface (2D heatmap)\n');
fprintf('2. Phase matching (measured vs best fit)\n');
fprintf('3. Error vs distance\n');
fprintf('4. Error vs angle\n');