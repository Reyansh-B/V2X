% PhaseMeasurements.m
% Generates noisy differential phase observations for the V2X array.
% Implements the noise model from Decarli et al., Eq. (7)-(10).

clear; clc;

% --- System Parameters ---
fc      = 5.9e9;            % Carrier frequency (5.9 GHz)
c0      = 299792458;
lam     = c0 / fc;          % Wavelength
SNR_dB  = 20;               % Signal-to-Noise Ratio (dB)
SNR_lin = 10^(SNR_dB/10);   % Linear SNR
eta     = 10^(6/10);        % Implementation loss (6 dB, paper Table II)
Mp      = 2;                % Number of pilot OFDM symbols
Np      = 120;              % Number of pilot subcarriers

% --- Array Geometry ---
Ka = 41;                    % Number of antennas (odd)
x = (0:Ka-1) * (lam/2);     % Antenna positions
x = x - mean(x);            % Center the array at x = 0
kc = (Ka+1)/2;              % Index of the central antenna

% --- True Vehicle Position ---
theta_true = 0;             % Angle from boresight (radians)
d_true     = 5.0;           % Distance from array center (meters)

% --- Clean Phase Generation ---
% Distance from TX to each RX antenna
dk_true = sqrt(d_true^2 + x.^2 - 2*d_true*x*sin(theta_true));

% Differential phase (relative to central antenna)
phi_clean = -2*pi/lam * (dk_true - d_true);

% --- Add Noise ---
% Phase noise variance per pilot (Eq. 8)
sigma_phi_sq = eta / (2 * Mp * Np * SNR_lin); 
% Differential phase noise variance is doubled (Eq. 9)
sigma_diff = sqrt(2 * sigma_phi_sq);

% Generate Gaussian noise
noise = sigma_diff * randn(1, Ka);

% The central antenna's differential phase is exactly 0 by definition
noise(kc) = 0; 

% Create the final noisy measurements
meas = phi_clean + noise;
meas(kc) = 0; 

% --- Display Results ---
fprintf('Generated measurements for d = %.1f m, theta = %.1f deg\n', d_true, rad2deg(theta_true));
fprintf('Phase noise std (single): %.4f rad\n', sqrt(sigma_phi_sq));
fprintf('Differential noise std  : %.4f rad\n', sigma_diff);
fprintf('First 5 measured phases : [%.3f, %.3f, %.3f, %.3f, %.3f] rad\n', meas(1:5));