%% Part 3: Simulate Received Signal with Phase Noise
% Generate realistic phase measurements by adding noise

clear; close all; clc;  % ← START FRESH!

% ============================================
% 1. SETUP: Constants and Array Configuration
% ============================================

c = 3e8;                    % Speed of light (m/s)
fc = 5.9e9;                 % Carrier frequency: 5.9 GHz
lambda = c / fc;            % Wavelength (about 5 cm)
d_antenna = lambda / 2;     % Half-wavelength spacing

% Array configuration
K = 41;                     % 41 antennas
D = (K - 1) * d_antenna;    % Total array size (about 1 meter)

% Make sure x_ant is a ROW VECTOR (1x41)
x_ant = linspace(-D/2, D/2, K);

% Display the size to verify
fprintf('Size of x_ant: %d x %d\n', size(x_ant, 1), size(x_ant, 2));

% ============================================
% 2. TRUE TRANSMITTER POSITION
% ============================================

d_true = 15;                % 15 meters away
theta_true = 5 * pi/180;    % 5 degrees off boresight

fprintf('=== True Transmitter Position ===\n');
fprintf('Distance: %.2f m\n', d_true);
fprintf('Angle: %.2f degrees\n', theta_true * 180/pi);
fprintf('Array size: %.2f m (%d antennas)\n\n', D, K);

% ============================================
% 3. IDEAL (NOISE-FREE) PHASE
% ============================================

% Distance from transmitter to each antenna
% x_ant is a row vector (1x41)
d_k_true = sqrt(x_ant.^2 + d_true^2 - 2*x_ant*d_true*sin(theta_true));

% Check the size of d_k_true
fprintf('Size of d_k_true: %d x %d\n', size(d_k_true, 1), size(d_k_true, 2));

% Ideal phase (no noise) - should be same size as x_ant
phase_ideal = -2*pi/lambda * (d_k_true - d_true);

% Check the size of phase_ideal
fprintf('Size of phase_ideal: %d x %d\n', size(phase_ideal, 1), size(phase_ideal, 2));

% Ensure phase_ideal is a column vector (41x1)
if size(phase_ideal, 1) == 1 && size(phase_ideal, 2) > 1
    phase_ideal = phase_ideal';  % Transpose to make it a column vector
    fprintf('Transposed phase_ideal to column vector\n');
end

% ============================================
% 4. ADD NOISE TO SIMULATE REAL MEASUREMENTS
% ============================================

% Signal-to-Noise Ratio (SNR)
SNR_dB = 10;                % 20 dB (moderate noise)
SNR_linear = 10^(SNR_dB/10);

% Phase noise standard deviation
phase_noise_std = sqrt(1 / (2 * SNR_linear));

% Generate multiple noisy realizations
num_realizations = 100;

% Pre-allocate: K antennas x num_realizations
phase_noisy = zeros(K, num_realizations);

for r = 1:num_realizations
    % Add Gaussian noise to each antenna's phase
    % phase_ideal is column vector (41x1)
    % randn(K,1) is also column vector (41x1)
    phase_noisy(:, r) = phase_ideal + phase_noise_std * randn(K, 1);
end

fprintf('=== Noise Parameters ===\n');
fprintf('SNR: %d dB\n', SNR_dB);
fprintf('Phase noise standard deviation: %.2f degrees\n', phase_noise_std * 180/pi);
fprintf('Number of realizations: %d\n\n', num_realizations);

% ============================================
% 5. PLOT RESULTS
% ============================================

figure('Position', [100, 100, 1200, 600]);

% --- Plot 1: Single realization with error bars ---

subplot(1, 2, 1);

% Plot ideal phase (no noise)
% Convert x_ant from meters to cm, phase from radians to degrees
plot(x_ant*100, phase_ideal*180/pi, 'b-', 'LineWidth', 3, 'DisplayName', 'Ideal (No Noise)');
hold on;

% Calculate mean and standard deviation across realizations
mean_phase = mean(phase_noisy, 2);
std_phase = std(phase_noisy, 0, 2);

% Plot mean with error bars
errorbar(x_ant*100, mean_phase*180/pi, std_phase*180/pi, ...
         'ro', 'LineWidth', 1.5, 'MarkerSize', 4, ...
         'DisplayName', ['Measured (SNR = ', num2str(SNR_dB), ' dB)']);

xlabel('Antenna Position (cm)', 'FontSize', 12);
ylabel('Phase Difference (degrees)', 'FontSize', 12);
title('Phase Across the Array', 'FontSize', 14);
legend('Location', 'best');
grid on;
xlim([-D/2*100, D/2*100]);
ylim([-100, 100]);

text(0, 80, sprintf('SNR = %d dB', SNR_dB), ...
     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

% --- Plot 2: Histogram of phase noise at one antenna ---

subplot(1, 2, 2);

% Pick the center antenna (index 21)
antenna_idx = 21;

% Get the phase measurements at this antenna across all realizations
phase_at_antenna = phase_noisy(antenna_idx, :) * 180/pi;

% Plot histogram
histogram(phase_at_antenna, 20, 'Normalization', 'pdf', 'FaceColor', [0.8, 0.2, 0.2]);
hold on;

% Overlay theoretical Gaussian distribution
x_range = linspace(-3, 3, 100) * (phase_noise_std * 180/pi);
theoretical_pdf = (1/(sqrt(2*pi) * phase_noise_std * 180/pi)) * ...
                  exp(-x_range.^2 / (2 * (phase_noise_std * 180/pi)^2));
plot(x_range, theoretical_pdf, 'k-', 'LineWidth', 2, 'DisplayName', 'Theoretical Gaussian');

xlabel('Phase Error (degrees)', 'FontSize', 12);
ylabel('Probability Density', 'FontSize', 12);
title(sprintf('Phase Noise Distribution\nAntenna %d (Center)', antenna_idx), 'FontSize', 14);
legend('Simulated', 'Theoretical Gaussian', 'Location', 'best');
grid on;

% ============================================
% 6. ADDITIONAL VISUALIZATION: All Realizations
% ============================================

figure('Position', [100, 100, 800, 600]);

% Plot ALL realizations (shows the spread)
plot(x_ant*100, phase_noisy*180/pi, 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.5);
hold on;

% Overlay the ideal phase in bold
plot(x_ant*100, phase_ideal*180/pi, 'b-', 'LineWidth', 3, 'DisplayName', 'Ideal (No Noise)');

% Overlay the mean in red
plot(x_ant*100, mean_phase*180/pi, 'r-', 'LineWidth', 2, 'DisplayName', 'Mean of All Realizations');

xlabel('Antenna Position (cm)', 'FontSize', 12);
ylabel('Phase Difference (degrees)', 'FontSize', 12);
title(sprintf('100 Realizations of Noisy Phase Measurements (SNR = %d dB)', SNR_dB), 'FontSize', 14);
legend('Location', 'best');
grid on;
xlim([-D/2*100, D/2*100]);
ylim([-100, 100]);

fprintf('\n=== Part 3 Complete ===\n');
fprintf('You should see two figures:\n');
fprintf('1. Phase pattern with error bars + noise distribution\n');
fprintf('2. All 100 realizations showing the spread\n');