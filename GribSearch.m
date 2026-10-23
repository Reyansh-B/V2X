function [theta_hat, d_hat] = GridSearch(meas, x, lam, theta_max, d_max)
% GRIDSEARCH ML position estimator for near-field V2X localization.
%   [theta_hat, d_hat] = GridSearch(meas, x, lam, theta_max, d_max)
%
%   Inputs:
%       meas      : 1 x K_a vector of noisy differential phases
%       x         : 1 x K_a vector of antenna positions
%       lam       : Wavelength
%       theta_max : Maximum search angle (e.g., pi/3)
%       d_max     : Maximum search distance (e.g., 20 m)
    
    % Ensure row vectors
    meas = meas(:).';
    x = x(:).';
    
    % --- 1. Coarse Grid Search ---
    th_grid = linspace(-theta_max, theta_max, 121);
    d_grid  = linspace(0.5, d_max, 100);
    
    [TH, DD] = meshgrid(th_grid, d_grid);
    
    % Vectorized distance calculation (3D array: 100 x 121 x Ka)
    % Reshape x to (1 x 1 x Ka) to broadcast across the grid
    x_3d = reshape(x, 1, 1, []);
    
    dk = sqrt(DD.^2 + x_3d.^2 - 2*DD.*x_3d.*sin(TH));
    
    % Candidate differential phases
    phi_cand = -2*pi/lam * (dk - DD);
    
    % Wrapped residual: angle(exp(1j*(meas - cand)))
    % Reshape meas to (1 x 1 x Ka) to subtract from phi_cand
    meas_3d = reshape(meas, 1, 1, []);
    residuals = angle(exp(1j * (meas_3d - phi_cand)));
    
    % Sum squared error over the antennas (3rd dimension)
    cost_grid = sum(residuals.^2, 3);
    
    % Find best grid point
    [~, idx] = min(cost_grid(:));
    [row, col] = ind2sub(size(TH), idx);
    p0 = [TH(row, col), DD(row, col)];
    
    % --- 2. Local Refinement (fminsearch) ---
    % Define cost function for continuous optimization
    cost_fn = @(p) sum( angle(exp(1j * (meas - (-2*pi/lam * (sqrt(p(2)^2 + x.^2 - 2*p(2)*x*sin(p(1))) - p(2)))))).^2 );
    
    % Options to suppress output and limit iterations
    options = optimset('Display', 'off', 'MaxFunEvals', 2000, 'MaxIter', 2000);
    
    % Run optimization starting from the best grid point
    p_hat = fminsearch(cost_fn, p0, options);
    
    % Extract outputs
    theta_hat = p_hat(1);
    d_hat     = p_hat(2);
end