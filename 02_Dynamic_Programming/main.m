%% =========================================================================
% MASTER SCRIPT: CONSTRAINED OPTIMAL CONTROL USING DYNAMIC PROGRAMMING
%% =========================================================================
% Architecture:
% 1. Centralized Parameter & Constraint Definition
% 2. Multi-Dimensional Grid Initialization
% 3. Backward DP Pass with N-Dimensional Gridded Interpolation
% 4. Forward Trajectory Simulation & Asset Generation
% =========================================================================

clear; 
clc; 
close all; 

%% System Definition
% --- 1. Define System Parameters (n=2 states, m=1 input) ---
n = 2; % States: [position; velocity]
m = 1; % Inputs: [force]

h_step = 0.1; % Time step (seconds)
N = 10;       % Time horizon steps

A = [0.9974, 0.0539; -0.1078, 1.16];
B = [0.0013; 0.0539];

% --- 2. Define Cost Matrices ---
Q = diag([0.25, 0.05]); 
R = 0.05; 
G = zeros(n, n); % Terminal state cost matrix

% --- 3. Define State and Input Constraints ---
xmin = [-5; -5]; 
xmax = [ 5;  5];
umin = -2;  
umax =  2;     

% --- 4. Define Initial State Configuration ---
x0 = [2; 1]; 

% --- 5. Set Grid Discretization Resolution ---
grid_points_x = 31; 
grid_points_u = 51; 

% Setup cross-platform asset path for image storage
imageFolder = fullfile(pwd, 'images');
if ~exist(imageFolder, 'dir')
    mkdir(imageFolder);
end

fprintf('=================================================\n');
fprintf('DYNAMIC PROGRAMMING SOLVER INITIALIZATION\n');
fprintf('=================================================\n');
fprintf('• State Dimensions  : %dD State Grid (%d points/dim)\n', n, grid_points_x);
fprintf('• Input Dimensions  : %dD Input Grid (%d points/dim)\n', m, grid_points_u);
fprintf('• Horizon Steps     : N = %d (Total Time = %.2f s)\n', N, N * h_step);
fprintf('• Initial Vector x0 : [%s]\n\n', num2str(x0'));

%% Execute Dynamic Programming Solver Suite
fprintf('=================================================\n');
fprintf('RUNNING GENERAL DP SOLVER PASSES\n');
fprintf('=================================================\n');
tic;
[J_tables, U_tables, X_opt, U_opt, J_opt, grid_vecs, J_star, U_star] = ...
    solveDP_General(A, B, n, m, G, Q, R, N, h_step, ...
                    xmin, xmax, umin, umax, ...
                    x0, grid_points_x, grid_points_u);
elapsedTime = toc;

% Solver Verification Engine
if isempty(X_opt) || isempty(U_opt)
    error('Dynamic Programming Solver Suite tracking failed to converge.');
end
fprintf('\n✓ DP Passes Complete (Execution Time: %.4f seconds)\n\n', elapsedTime);
fprintf('Optimal Cost-to-Go : %.4f\n\n', J_opt);

%% Run Visualization Suite & Export Asset Portfolios
fprintf('=================================================\n');
fprintf('GENERATING HIGH-RESOLUTION PORTFOLIO GRAPHICS\n');
fprintf('=================================================\n');
fprintf('• Rendering visualization canvases...\n');

plotResults(X_opt, U_opt, J_opt, N, h_step, xmin, xmax, umin, umax, ...
            grid_vecs, J_star, U_star, imageFolder, n, m);

fprintf('✓ Completed. Publication-ready assets exported to:\n  %s\n', imageFolder);