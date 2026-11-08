%% Part 1: Fraunhofer Distance Calculator
% This shows when near-field localization is possible

clear; close all; clc;

% Constants
c = 3e8; % Speed of light (m/s)

% Parameters to vary
D = 0.1:0.05:3.0;         % Array size from 10 cm to 3 m
frequencies = [5.9e9, 28e9, 60e9];
freq_labels = {'5.9 GHz (Current V2X)', '28 GHz (mmWave)', '60 GHz (mmWave)'};
colors = {'b', 'r', 'g'};

% Create figure
figure('Position', [100, 100, 800, 600]);
hold on;

% Plot curves with DisplayName
for i = 1:length(frequencies)
    lambda = c / frequencies(i);
    dF = 2 * D.^2 / lambda;
    plot(D, dF, colors{i}, 'LineWidth', 2, 'DisplayName', freq_labels{i});
end

% === YELLOW PLATOONING STRIP ===
patch([0, 50, 50, 0], [0, 0, max(ylim), max(ylim)], 'y', ...
      'FaceAlpha', 0.08, 'EdgeColor', 'none');

% Label the yellow strip
text(25, max(ylim)*0.92, 'Platooning Distance (0-50m)', ...
     'HorizontalAlignment', 'center', 'FontSize', 11, ...
     'FontWeight', 'bold', 'Color', [0.4, 0.4, 0]);

% Labels and styling
xlabel('Antenna Array Size D (meters)', 'FontSize', 12);
ylabel('Fraunhofer Distance d_F (meters)', 'FontSize', 12);
title('Near-Field vs Far-Field Threshold', 'FontSize', 14);

% === LEGEND WITH CUSTOM NAMES ===
legend('Location', 'northwest');  % This will use DisplayName from above

grid on;

% Add reference lines for typical vehicle widths
line([1.7, 1.7], ylim, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
line([2.2, 2.2], ylim, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
text(1.7, max(ylim)*0.85, 'Car (1.7m)', 'HorizontalAlignment', 'center', 'FontSize', 10);
text(2.2, max(ylim)*0.85, 'Truck (2.2m)', 'HorizontalAlignment', 'center', 'FontSize', 10);


hold off;

% Set axis limits
xlim([0, 3.5]);
ylim([0, 2500]);

% Print key values
fprintf('\n=== Fraunhofer Distance Examples ===\n');
fprintf('Car array (D=1.7m):\n');
fprintf('  at 5.9 GHz: dF = %.1f m\n', 2*1.7^2/(c/5.9e9));
fprintf('  at 28 GHz:  dF = %.1f m\n', 2*1.7^2/(c/28e9));
fprintf('  at 60 GHz:  dF = %.1f m\n', 2*1.7^2/(c/60e9));
fprintf('\nTruck array (D=2.2m):\n');
fprintf('  at 5.9 GHz: dF = %.1f m\n', 2*2.2^2/(c/5.9e9));
fprintf('  at 28 GHz:  dF = %.1f m\n', 2*2.2^2/(c/28e9));
fprintf('  at 60 GHz:  dF = %.1f m\n', 2*2.2^2/(c/60e9));