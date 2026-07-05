function [t_out, u_opt, x_out, lambda_out, G] = ...
    solveLQR_Infinite_Euler(n, m, A, B, Q, R, x0, T_final, N_steps)
% Solves the Infinite Horizon LQR problem using manual Forward Euler.

    dt = T_final / (N_steps - 1);
    t_span = linspace(0, T_final, N_steps);
    
    % --- 1. Solve the Algebraic Riccati Equation (ARE) ---
    try
        [G, K_are, ~] = lqr(A, B, Q, R);
    catch ME
        fprintf('Error solving LQR (ARE): %s\n', ME.message);
        fprintf('Check if the system (A, B) is controllable.\n');
        t_out = []; u_opt = []; x_out = []; lambda_out = []; G = [];
        return;
    end
    
    % --- 2. Solve State Forward Integration ---
    A_cl = A - B * G; % Constant closed-loop matrix
    x_history = zeros(n, N_steps);
    x_history(:, 1) = x0; 
    for i = 1:N_steps-1
        x_history(:, i+1) = x_history(:, i) + (A_cl * x_history(:, i)) * dt;
    end
    
    % --- 3. Compute Control Inputs and Costates ---
    u_history = -G * x_history;
    lambda_history = K_are * x_history; 
    
    % --- Format Output Arrays ---
    t_out = t_span';
    u_opt = u_history';
    x_out = x_history';
    lambda_out = lambda_history';
end