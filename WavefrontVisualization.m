%% Part 2: Wavefront Curvature Visualization
% Shows why near-field gives distance information
% This is the KEY INSIGHT of the paper!

clear; close all; clc;

% ============================================
% 1. SETUP: Constants and Array Configuration
% ============================================

c = 3e8;                    % Speed of light (m/s)
fc = 5.9e9;                 % Carrier frequency: 5.9 GHz (current V2X)
lambda = c / fc;            % Wavelength (about 5 cm)
d_antenna = lambda / 2;     % Half-wavelength spacing (standard)

% Array configuration
K = 41;                     % Number of antennas (odd number for a center)
D = (K - 1) * d_antenna;    % Total array size (about 1 meter)

% Display array info
fprintf('=== Array Configuration ===\n');
fprintf('Number of antennas: %d\n', K);
fprintf('Antenna spacing: %.2f cm\n', d_antenna*100);
fprintf('Total array size: %.2f m (%.1f cm)\n', D, D*100);
fprintf('Fraunhofer distance: %.2f m\n\n', 2*D^2/lambda);

% Antenna positions (centered at 0)
x_ant = linspace(-D/2, D/2, K);

% ============================================
% 2. TEST TWO SCENARIOS: Near vs Far
% ============================================

% Two test distances
distances = [10, 200];      % 10m = near-field, 200m = far-field
labels = {'Near-Field (d=10m)', 'Far-Field (d=200m)'};
theta = 0;                  % Boresight direction (straight ahead)

% Create figure with two subplots
figure('Position', [100, 100, 1200, 500]);

for idx = 1:2
    d_tx = distances(idx);
    
    % ============================================
    % 3. CALCULATE PHASE AT EACH ANTENNA
    % ============================================
    
    % Distance from transmitter to each antenna
    % Transmitter is at position (0, d_tx) in polar coordinates
    % Formula: d_k = sqrt(x_ant^2 + d_tx^2 - 2*x_ant*d_tx*sin(theta))
    d_k = sqrt(x_ant.^2 + d_tx^2 - 2*x_ant*d_tx*sin(theta));
    
    % Phase relative to the central antenna
    % Formula: phase = -2*pi/lambda * (d_k - d_tx)
    phase = -2*pi/lambda * (d_k - d_tx);
    
    % ============================================
    % 4. PLOT THE RESULTS
    % ============================================
    
    subplot(1, 2, idx);
    
    % Plot phase vs antenna position
    plot(x_ant*100, phase*180/pi, 'b-o', 'LineWidth', 2, 'MarkerSize', 5);
    
    xlabel('Antenna Position (cm)', 'FontSize', 12);
    ylabel('Phase Difference (degrees)', 'FontSize', 12);
    title(sprintf('%s\nArray spans %.1f cm', labels{idx}, D*100), 'FontSize', 12);
    grid on;
    
    xlim([-D/2*100, D/2*100]);
    ylim([-100, 100]);
    
    % Add annotation explaining what we can estimate
    if idx == 1
        % Near-field: curved pattern
        text(0, 80, 'CURVED → Distance + Angle info!', ...
             'HorizontalAlignment', 'center', ...
             'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
        text(0, 60, sprintf('(d = %d m < dF = %.0f m)', d_tx, 2*D^2/lambda), ...
             'HorizontalAlignment', 'center', ...
             'Color', 'r', 'FontSize', 10);
    else
        % Far-field: straight line
        text(0, 80, 'STRAIGHT → Angle ONLY!', ...
             'HorizontalAlignment', 'center', ...
             'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
        text(0, 60, sprintf('(d = %d m > dF = %.0f m)', d_tx, 2*D^2/lambda), ...
             'HorizontalAlignment', 'center', ...
             'Color', 'r', 'FontSize', 10);
    end
end

% ============================================
% 5. ADD A 3D SURFACE PLOT (BONUS)
% ============================================

% Create a new figure showing the wavefront in 2D
figure('Position', [100, 100, 800, 600]);

% Visualize wavefront for near-field case
d_tx = 10;
theta = 0;

% Create a grid of points to visualize the wavefront
x_range = linspace(-1, 1, 50);
y_range = linspace(0, 12, 100);
[X, Y] = meshgrid(x_range, y_range);

% Calculate distance from transmitter (at 0, d_tx) to each point
d_grid = sqrt(X.^2 + (Y - d_tx).^2);

% Calculate phase at each point (relative to transmitter)
phase_grid = -2*pi/lambda * (d_grid);

% Plot the wavefront (phase contours)
contourf(X, Y, phase_grid, 20);
colorbar;
xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
title('Near-Field Wavefront (d=10m) - Phase Contours', 'FontSize', 14);

% Mark transmitter position
hold on;
plot(0, d_tx, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Transmitter (NV)');
plot(0, 0, 'bs', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Receiver Center (AV)');

% Mark the receiving array
line([-0.5, 0.5], [0, 0], 'Color', 'k', 'LineWidth', 3, ...
     'DisplayName', 'Receiving Array (1m)');
legend('Location', 'northeast');
grid on;

fprintf('\n=== Part 2 Complete ===\n');
fprintf('You should see two figures:\n');
fprintf('1. Phase across the array (near vs far)\n');
fprintf('2. 2D wavefront visualization\n');