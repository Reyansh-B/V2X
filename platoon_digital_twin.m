% platoon_digital_twin.m
% Lightweight Kinematic Digital Twin for Platoon Control
% Compares Perfect, BSM/GNSS, and Near-Field V2X Localization.

clear; clc; close all;

%% 1. Simulation Parameters
dt = 0.01;            % Simulation time step (100 Hz)
T_total = 15;         % Total time (seconds)
t = 0:dt:T_total;
N = length(t);

% Vehicle dynamics (Point-mass kinematics)
desired_gap = 5.0;    % meters
kp = 1.5; kd = 2.0;   % CACC Controller gains

% Leader profile (Emergency braking)
v_leader = 20 * ones(1, N); % 72 km/h
for i = 1:N
    if t(i) > 5 && t(i) < 7
        v_leader(i:end) = v_leader(i:end) - 5 * dt; % Brake at -5 m/s^2
    elseif t(i) >= 7
        v_leader(i:end) = max(v_leader(i:end), 10); % Hold at 36 km/h
    end
end
% Generate leader position
x_leader = cumsum(v_leader) * dt;

% Follower initial state
x_follower = x_leader - desired_gap;
v_follower = 20;

% Measurement delays (in steps)
delay_bsm = 0.10 / dt; % 100 ms (10 Hz)
delay_nf  = 0.02 / dt; % 20 ms (50 Hz)

% Noise standard deviations
sigma_bsm = 1.5; % 1.5 meters (GNSS/BSM error)
sigma_nf  = 0.01; % 1 cm (Decarli near-field error at 10m)

%% 2. Storage Arrays
gap_true = zeros(1, N);
gap_measured_perfect = zeros(1, N);
gap_measured_bsm = zeros(1, N);
gap_measured_nf  = zeros(1, N);

% For delay buffer
buf_bsm = repmat(desired_gap, 1, ceil(delay_bsm)+1);
buf_nf  = repmat(desired_gap, 1, ceil(delay_nf)+1);

%% 3. Main Simulation Loop
for k = 2:N
    % --- 1. True Physics Update ---
    gap_true(k) = x_leader(k) - x_follower(k-1);
    
    % --- 2. Generate Measurements (The "Digital Twin" Channel) ---
    % Perfect knowledge
    gap_measured_perfect(k) = gap_true(k);
    
    % BSM/GNSS (High noise, High delay)
    noisy_bsm = gap_true(k) + sigma_bsm * randn();
    buf_bsm = [buf_bsm(2:end), noisy_bsm];
    gap_measured_bsm(k) = buf_bsm(1); % Delayed reading
    
    % Near-Field V2X (Low noise, Low delay)
    noisy_nf = gap_true(k) + sigma_nf * randn();
    buf_nf = [buf_nf(2:end), noisy_nf];
    gap_measured_nf(k) = buf_nf(1); % Delayed reading
    
    % --- 3. Controller (CACC) ---
    % We run the controller 3 times for the 3 cases to see divergence
    % (In a real system, only one controller runs)
    
    % Case A: Perfect
    accel_perfect = kp*(gap_measured_perfect(k) - desired_gap) - kd*(v_leader(k) - v_follower);
    
    % Case B: BSM
    % Note: We assume the follower gets leader velocity via BSM too
    accel_bsm = kp*(gap_measured_bsm(k) - desired_gap) - kd*(v_leader(k) - v_follower);
    
    % Case C: Near-Field
    accel_nf = kp*(gap_measured_nf(k) - desired_gap) - kd*(v_leader(k) - v_follower);
    
    % Update Follower Velocity and Position (Using BSM controller as the "active" one for physical truth)
    % For plotting, we will simulate 3 separate follower trajectories:
    v_f_perfect = v_follower + accel_perfect * dt;
    v_f_bsm     = v_follower + accel_bsm * dt;
    v_f_nf      = v_follower + accel_nf * dt;
    
    % We store the three trajectories
    if k == 2
        x_f_perfect = x_follower; x_f_bsm = x_follower; x_f_nf = x_follower;
    end
    
    x_f_perfect = x_f_perfect + v_f_perfect * dt;
    x_f_bsm     = x_f_bsm + v_f_bsm * dt;
    x_f_nf      = x_f_nf + v_f_nf * dt;
    
    % For simplicity in this loop, just update the "true" follower with perfect dynamics
    v_follower = v_f_perfect;
    x_follower = x_f_perfect;
    
    % Calculate the gaps for the 3 cases relative to the leader
    gap_perfect(k) = x_leader(k) - x_f_perfect;
    gap_bsm(k)     = x_leader(k) - x_f_bsm;
    gap_nf(k)      = x_leader(k) - x_f_nf;
end

%% 4. Plotting Results
figure('Color','w','Position',[100 100 800 500]); hold on;
plot(t, gap_perfect, 'k-', 'LineWidth', 2, 'DisplayName', 'Perfect Knowledge');
plot(t, gap_bsm, 'r-', 'LineWidth', 2, 'DisplayName', 'BSM/GNSS (High Noise, Delayed)');
plot(t, gap_nf, 'b-', 'LineWidth', 2, 'DisplayName', 'Near-Field V2X (Proposed)');

yline(0, 'k--', 'Collision', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
grid on; box on;
xlabel('Time [s]');
ylabel('Inter-Vehicle Gap [m]');
title('Digital Twin: Platooning Safety under Emergency Braking');
legend('Location', 'best');
ylim([-2, 10]);

fprintf('Minimum gap (Perfect) : %.2f m\n', min(gap_perfect));
fprintf('Minimum gap (BSM)     : %.2f m\n', min(gap_bsm));
fprintf('Minimum gap (Near-Field): %.2f m\n', min(gap_nf));

if min(gap_bsm) < 0
    fprintf('Result: BSM/GNSS caused a CRASH.\n');
end
if min(gap_nf) > 0
    fprintf('Result: Near-Field V2X prevented the crash.\n');
end