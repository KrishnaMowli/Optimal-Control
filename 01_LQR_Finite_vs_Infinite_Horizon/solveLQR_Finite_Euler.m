function [t_out, u_opt, x_out, lambda_out, K_history] = ...
    solveLQR_Finite_Euler(n, m, A, B, H, Q, R, x0, T_final, N_steps)
% Solves the finite-horizon LQR problem using Forward Euler integration
% of the Differential Riccati Equation (DRE).

    % --- Controllability Check ---
    if rank(ctrb(A, B)) < n
        warning('The system may not be fully controllable.');
    end

    % --- Setup and Parameter Initialization ---
    R_inv = R \ eye(size(R)); 
    dt = T_final / (N_steps - 1);
    t_span = linspace(0, T_final, N_steps);
    
    % Pre-allocate memory buffers
    K_history = zeros(n, n, N_steps);
    x_history = zeros(n, N_steps);
    u_history = zeros(m, N_steps);
    lambda_history = zeros(n, N_steps); 
    
    % --- 1. Solve Differential Riccati Equation Backward ---
    K_history(:, :, N_steps) = H; 
    
    % Integrate the DRE backward in time from K(T) = H.
    for i = N_steps:-1:2
        K = K_history(:, :, i);
        K_dot = -A'*K - K*A + K*B*R_inv*B'*K - Q;
        K_history(:, :, i-1) = K - K_dot * dt;
    end
    
    % --- 2. Solve State Forward Integration ---
    x_history(:, 1) = x0; 
    for i = 1:N_steps-1
        x = x_history(:, i);
        K = K_history(:, :, i);
        A_cl = A - B*R_inv*B'*K;
        x_dot = A_cl * x;
        x_history(:, i+1) = x + x_dot * dt;
    end
    
    % --- 3. Compute Control Inputs and Algebraically Determine Costates ---
    for i = 1:N_steps
        K_i = K_history(:, :, i);
        x_i = x_history(:, i);
        
        lambda_i = K_i * x_i; 
        lambda_history(:, i) = lambda_i;
        
        u_history(:, i) = -R_inv * B' * lambda_i; 
    end
    
    % --- Format Output Arrays ---
    t_out = t_span';
    u_opt = u_history';
    x_out = x_history';
    lambda_out = lambda_history';
end