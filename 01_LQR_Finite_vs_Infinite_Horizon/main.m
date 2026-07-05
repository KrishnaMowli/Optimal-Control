%% =========================================================================
% MASTER SCRIPT: LQR COMPARISON (3-STATE, 1-INPUT SYSTEM)
% =========================================================================
% Core Architecture:
% 1. Centralized System Definition
% 2. Finite Horizon & Infinite Horizon LQR Execution (with safety checks)
% 3. Automated Performance Evaluation & Analytical Verification
% 4. Publication-Quality Visualization and High-Resolution Exporting
% =========================================================================

clear; 
clc; 
close all; 

%% System Definition
n = 3;                        % Number of states
m = 1;                        % Number of inputs
A = [0.1   1   0;
     0     0.2 1;
     0.5  -1   0];
B = [0; 1; 1];

Q = [10  0  0;
     0   1  0;
     0   0  1];

R = 1; 
H = 10 * Q;                   % Terminal cost weight

% --- Initial Conditions and Time ---
x0 = [4; 1; -2];              % Initial state vector
T_final = 5;                  % Final time (seconds)
N_steps = 2001;               % Discretization steps for Euler accuracy
dt = T_final / (N_steps - 1);  % Time step

% Setup output directory for high-quality figures using robust cross-platform paths
imageFolder = fullfile(pwd, 'images');
if ~exist(imageFolder, 'dir')
    mkdir(imageFolder);
end

fprintf('=================================================\n');
fprintf('LQR COMPARISON SIMULATION INITIALIZATION\n');
fprintf('=================================================\n');
fprintf('• System Dimensions : %d states, %d inputs.\n', n, m);
fprintf('• Simulation Window : 0 to %.1f seconds (%d steps, dt=%.4f).\n', T_final, N_steps, dt);
fprintf('• Initial State Vector : x0 = [%s]\n\n', num2str(x0'));

%% Execute Finite Horizon LQR Simulation
fprintf('=================================================\n');
fprintf('FINITE HORIZON LQR\n');
fprintf('=================================================\n');
fprintf('• Solving Differential Riccati Equation...\n');
[t_fin, u_fin, x_fin, lambda_fin, K_history_fin] = ...
    solveLQR_Finite_Euler(n, m, A, B, H, Q, R, x0, T_final, N_steps);

if isempty(t_fin)
    error('Finite Horizon LQR solver failed.');
end
fprintf('✓ Completed\n\n');

%% Execute Infinite Horizon LQR Simulation
fprintf('=================================================\n');
fprintf('INFINITE HORIZON LQR\n');
fprintf('=================================================\n');
fprintf('• Solving Algebraic Riccati Equation...\n');
[t_inf, u_inf, x_inf, lambda_inf, G_inf] = ...
    solveLQR_Infinite_Euler(n, m, A, B, Q, R, x0, T_final, N_steps);

if isempty(t_inf)
    error('Infinite Horizon LQR solver failed.');
end
fprintf('✓ Completed\n\n');

%% Performance Metrics & Verification Engine
fprintf('=================================================\n');
fprintf('METRICS & ANALYTICAL VERIFICATION\n');
fprintf('=================================================\n');
fprintf('• Processing transient response metrics...\n');
metrics = compute_Performance_Metrics(t_fin, x_fin, u_fin, lambda_fin, K_history_fin, ...
                                    t_inf, x_inf, u_inf, lambda_inf, G_inf, ...
                                    A, B, Q, R, dt, n);
fprintf('✓ Completed\n\n');

%% Visualization and Exporting
fprintf('=================================================\n');
fprintf('GENERATING PUBLICATION GRAPHICS\n');
fprintf('=================================================\n');
fprintf('• Plotting and exporting high-resolution assets...\n');
plot_Results(t_fin, x_fin, u_fin, lambda_fin, t_inf, x_inf, u_inf, lambda_inf, metrics, imageFolder, n, m);
fprintf('✓ Completed. All assets exported to:\n  %s\n', imageFolder);