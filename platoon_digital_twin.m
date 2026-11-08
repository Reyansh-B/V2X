% final_digital_twin.m
% Paper-Quality Simulation: String Stability with Feedforward CACC

clear; clc; close all;

%% 1. Simulation Parameters
dt = 0.01;            % 100 Hz
T_total = 12;         
t = 0:dt:T_total;
N = length(t);

desired_gap = 10.0;   
kp = 1.5; kd = 2.0;   % CACC Feedback Gains

% Near-Field Physical Parameters
fc = 5.9e9; lam = 299792458/fc;
Ka = 41; x = (0:Ka-1)*(lam/2); x = x - mean(x);
Mp = 2; Np = 120; eta = 10^(6/10);
SNR_dB = 20; SNR_lin = 10^(SNR_dB/10);

% GNSS/BSM Parameters
sigma_gnss = 1.5; corr_time = 2.0; alpha = exp(-dt / corr_time); 

%% 2. Generate Leader Profile
v_leader = 25 * ones(1, N); 
a_leader = zeros(1, N); 

% Braking event
idx_brake = (t > 4) & (t < 8);
a_leader(idx_brake) = -4; % Brake at -4 m/s^2

% Integrate acceleration to get velocity and position
for k = 2:N
    v_leader(k) = max(0, v_leader(k-1) + a_leader(k) * dt);
end
x_leader = cumsum(v_leader) * dt;

%% 3. Simulation Setup
x_f_bsm = zeros(1, N); v_f_bsm = zeros(1, N);
x_f_nf  = zeros(1, N); v_f_nf  = zeros(1, N);

x_f_bsm(1) = x_leader(1) - desired_gap; v_f_bsm(1) = 25;
x_f_nf(1)  = x_leader(1) - desired_gap; v_f_nf(1)  = 25;

% Generate correlated GNSS noise
gnss_err = zeros(1, N);
for k = 2:N
    gnss_err(k) = alpha * gnss_err(k-1) + sqrt(1-alpha^2) * sigma_gnss * randn();
end

%% 4. Control Loop with Feedforward
for k = 2:N
    % --- 1. BSM/GNSS Control ---
    % Simulate a 1-second packet loss from t=6 to t=7
    if t(k) > 6 && t(k) < 7
        idx_b = max(1, k - 100); % Freeze on old data
    else
        idx_b = max(1, k - 10);  % 100ms delay
    end
    
    x_lead_est_b = x_leader(idx_b) + gnss_err(k);
    gap_b = x_lead_est_b - x_f_bsm(k-1);
    
    % True CACC: Feedback + Feedforward (leader acceleration)
    acc_b = kp*(gap_b - desired_gap) + kd*(v_leader(idx_b) - v_f_bsm(k-1)) + a_leader(idx_b);
    acc_b = max(-8, min(3, acc_b)); 
    
    v_f_bsm(k) = max(0, v_f_bsm(k-1) + acc_b * dt);
    x_f_bsm(k) = x_f_bsm(k-1) + v_f_bsm(k) * dt;
    
    % --- 2. Near-Field V2X Control ---
    idx_c = max(1, k - 2); % 20ms delay
    true_gap_nf = x_leader(idx_c) - x_f_nf(k-1);
    
    bound = nearfieldBounds(0, true_gap_nf, x, Mp, Np, SNR_lin, eta, fc, 'paper');
    sigma_nf_real = bound.PEB; 
    
    gap_c = true_gap_nf + sigma_nf_real * randn();
    
    % Near-Field gets perfectly synced acceleration via sidelink
    acc_c = kp*(gap_c - desired_gap) + kd*(v_leader(idx_c) - v_f_nf(k-1)) + a_leader(k);
    acc_c = max(-8, min(3, acc_c));
    
    v_f_nf(k) = max(0, v_f_nf(k-1) + acc_c * dt);
    x_f_nf(k) = x_f_nf(k-1) + v_f_nf(k) * dt;
end

%% 5. Plotting
gap_bsm = x_leader - x_f_bsm;
gap_nf  = x_leader - x_f_nf;

figure('Color','w','Position',[100 100 800 450]); hold on;
plot(t, gap_bsm, 'r-', 'LineWidth', 2, 'DisplayName', 'BSM/GNSS (w/ 1s Packet Loss)');
plot(t, gap_nf, 'b-', 'LineWidth', 2, 'DisplayName', 'Near-Field V2X (Dynamic CRLB)');
yline(desired_gap, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired Gap (10m)');

% Highlight the packet loss window
patch([6 7 7 6], [0 0 16 16], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'BSM Packet Loss');
grid on; box on;
xlabel('Time [s]');
ylabel('Inter-Vehicle Gap [m]');
title('CACC String Stability with Dynamic CRLB and BSM Packet Loss');
legend('Location', 'best');
ylim([0, 16]);