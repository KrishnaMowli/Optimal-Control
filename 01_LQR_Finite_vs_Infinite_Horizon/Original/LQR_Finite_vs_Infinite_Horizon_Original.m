%% ------------------------------------------------------------------------
% MASTER SCRIPT: LQR COMPARISON (3-STATE, 1-INPUT SYSTEM)
% -------------------------------------------------------------------------
% 1. Defines a system (n=3, m=1).
% 2. Runs both finite and infinite horizon LQR.
% 3. Performs analytical verification by checking costate dynamics.
% 4. Plots 3 comparison figures (state, input, costate).
% -------------------------------------------------------------------------
clear; 
clc; 
close all; 
n = 3; % Number of states
m = 1; % Number of inputs
A = [0.1   1   0;
     0     0.2 1;
     0.5  -1   0];
B = [0 ;1; 1];

Q = [10  0  0;
     0   1  0;
     0   0  1];

R = 1; 
H = 10 * Q; 
% --- Initial Conditions and Time ---
x0 = [4; 1; -2];  % Initial state (3x1 vector)
T_final = 5;      % Final time (shortened to see finite/infinite diff)
N_steps = 2001;   % More steps for better Euler accuracy
dt = T_final / (N_steps - 1); % Time step
fprintf('--- LQR Comparison Simulation Start ---\n');
fprintf('System: %d states, %d inputs.\n', n, m);
fprintf('Time: 0 to %.1f seconds, %d steps (dt=%.4f).\n', T_final, N_steps, dt);
fprintf('Initial State: x0 = [%s]\n', num2str(x0'));

% 2. RUN FINITE HORIZON LQR SIMULATION
fprintf('\nRunning Finite Horizon Simulation...\n');
[t_fin, u_fin, x_fin, lambda_fin] = ...
    solveLQR_Finite_Euler(n, m, A, B, H, Q, R, x0, T_final, N_steps);
fprintf('Finite Horizon complete.\n');

% 3. RUN INFINITE HORIZON LQR SIMULATION
fprintf('\nRunning Infinite Horizon Simulation...\n');
[t_inf, u_inf, x_inf, lambda_inf] = ...
    solveLQR_Infinite_Euler(n, m, A, B, Q, R, x0, T_final, N_steps);
fprintf('Infinite Horizon complete.\n');
fprintf('\nAll simulations finished. Verifying and plotting...\n');

% 4. ANALYTICAL VERIFICATION 
% Check if the costate equation (lambda_dot = -Qx - A'*lambda) holds true.
t_comp = t_fin(1:end-1);
% --- Finite Horizon Verification ---
lambda_dot_numeric_fin = diff(lambda_fin, 1, 1) / dt;
analytic_lambda_dot_fin = zeros(N_steps-1, n);
for i = 1:(N_steps-1)
    analytic_lambda_dot_fin(i, :) = ...
        (-Q * x_fin(i, :)' - A' * lambda_fin(i, :)')';
end
error_fin = lambda_dot_numeric_fin - analytic_lambda_dot_fin;
% --- Infinite Horizon Verification ---
lambda_dot_numeric_inf = diff(lambda_inf, 1, 1) / dt;
analytic_lambda_dot_inf = zeros(N_steps-1, n);
for i = 1:(N_steps-1)
    analytic_lambda_dot_inf(i, :) = ...
        (-Q * x_inf(i, :)' - A' * lambda_inf(i, :)')';
end
error_inf = lambda_dot_numeric_inf - analytic_lambda_dot_inf;
% --- Plot Verification Results ---
figure('Name', 'ANALYTICAL VERIFICATION');
sgtitle('Verification Error: \lambda_{dot} (numeric) vs. \lambda_{dot} (analytic)', 'FontSize', 12);
legend_labels = cell(n, 1);
for i = 1:n, legend_labels{i} = sprintf('Error \\lambda_%d', i); end
% Plot Finite Error
subplot(2, 1, 1);
plot(t_comp, error_fin);
grid on;
title('Finite Horizon: Costate Equation Error');
xlabel('Time (t)');
ylabel('Error');
legend(legend_labels, 'Location', 'best');
% Plot Infinite Error
subplot(2, 1, 2);
plot(t_comp, error_inf);
grid on;
title('Infinite Horizon: Costate Equation Error');
xlabel('Time (t)');
ylabel('Error');
legend(legend_labels, 'Location', 'best');

% 5. PLOT COMPARISON RESULTS 
colors = get(gca, 'ColorOrder'); % Get standard plot colors
% --- Figure 2: State Comparison  ---
figure('Name', 'State Comparison');
hold on;
legend_labels = cell(2*n, 1);
for i = 1:n
    plot(t_fin, x_fin(:,i), '-', 'LineWidth', 1.5, 'Color', colors(i,:));
    plot(t_inf, x_inf(:,i), '--', 'LineWidth', 1.5, 'Color', colors(i,:));
    legend_labels{2*i-1} = sprintf('x_%d (Finite)', i);
    legend_labels{2*i}   = sprintf('x_%d (Infinite)', i);
end
hold off;
grid on;
title('State Trajectories x(t): Finite vs. Infinite Horizon');
xlabel('Time (t)');
ylabel('State Value');
legend(legend_labels, 'Location', 'best');

% --- Figure 3: Input Comparison ---
figure('Name', 'Input Comparison');
hold on;
legend_labels = cell(2*m, 1);
for i = 1:m
    color_idx = mod(i-1, size(colors,1)) + 1;
    plot(t_fin, u_fin(:,i), '-', 'LineWidth', 1.5, 'Color', colors(color_idx,:));
    plot(t_inf, u_inf(:,i), '--', 'LineWidth', 1.5, 'Color', colors(color_idx,:));
    legend_labels{2*i-1} = sprintf('u_%d (Finite)', i);
    legend_labels{2*i}   = sprintf('u_%d (Infinite)', i);
end
hold off;
grid on;
title('Optimal Input u(t): Finite vs. Infinite Horizon');
xlabel('Time (t)');
ylabel('Input Value');
legend(legend_labels, 'Location', 'best');

% --- Figure 4: Costate Comparison ---
figure('Name', 'Costate Comparison');
hold on;
legend_labels = cell(2*n, 1);
for i = 1:n
    plot(t_fin, lambda_fin(:,i), '-', 'LineWidth', 1.5, 'Color', colors(i,:));
    plot(t_inf, lambda_inf(:,i), '--', 'LineWidth', 1.5, 'Color', colors(i,:));
    legend_labels{2*i-1} = sprintf('\\lambda_%d (Finite)', i);
    legend_labels{2*i}   = sprintf('\\lambda_%d (Infinite)', i);
end
hold off;
grid on;
title('Costate \lambda(t): Finite vs. Infinite Horizon');
xlabel('Time (t)');
ylabel('Costate Value');
legend(legend_labels, 'Location', 'best');

% LOCAL FUNCTION DEFINITIONS
function [t_out, u_opt, x_out, lambda_out] = ...
    solveLQR_Finite_Euler(n, m, A, B, H, Q, R, x0, T_final, N_steps)
% Solves the Finite Horizon LQR problem using manual Forward Euler.
    % --- 0. Setup ---
    R_inv = inv(R);
    dt = T_final / (N_steps - 1);
    t_span = linspace(0, T_final, N_steps);
    % Pre-allocate
    K_history = zeros(n, n, N_steps);
    x_history = zeros(n, N_steps);
    u_history = zeros(m, N_steps);
    lambda_history = zeros(n, N_steps); 
    % --- 1. Solve DRE Backward (Euler) ---
    K_history(:, :, N_steps) = H; 
    for i = N_steps:-1:2
        K = K_history(:, :, i);
        K_dot = -A'*K - K*A + K*B*R_inv*B'*K - Q;
        K_history(:, :, i-1) = K - K_dot * dt;
    end
    % --- 2. Solve State Forward (Euler) ---
    x_history(:, 1) = x0; 
    for i = 1:N_steps-1
        x = x_history(:, i);
        K = K_history(:, :, i);
        A_cl = A - B*R_inv*B'*K;
        x_dot = A_cl * x;
        x_history(:, i+1) = x + x_dot * dt;
    end
    % --- 3. Calculate Input u(t) and Costate lambda(t) ---
    for i = 1:N_steps
        K_i = K_history(:, :, i);
        x_i = x_history(:, i); % Was x_history(:, i)'
        
        % lambda = K * x
        lambda_i = K_i * x_i; % Was K_i .* x_i
        lambda_history(:, i) = lambda_i;
        
        % u = -R_inv * B' * lambda
        u_history(:, i) = -R_inv * B' * lambda_i; 
    end
    % --- 4. Format Output ---
    t_out = t_span';
    u_opt = u_history';
    x_out = x_history';
    lambda_out = lambda_history';
end
% -------------------------------------------------------------------------
function [t_out, u_opt, x_out, lambda_out] = ...
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
        t_out = []; u_opt = []; x_out = []; lambda_out = [];
        return;
    end
    % --- 2. Solve State Forward (Euler) ---
    A_cl = A - B * G; % Constant closed-loop matrix
    x_history = zeros(n, N_steps);
    x_history(:, 1) = x0; 
    for i = 1:N_steps-1
        x_history(:, i+1) = x_history(:, i) + (A_cl * x_history(:, i)) * dt;
    end
    % --- 3. Calculate Input u(t) and Costate lambda(t) ---
    u_history = -G * x_history;
    lambda_history = K_are * x_history; % lambda = K_are * x
    % --- 4. Format Output ---
    t_out = t_span';
    u_opt = u_history';
    x_out = x_history';
    lambda_out = lambda_history';
end