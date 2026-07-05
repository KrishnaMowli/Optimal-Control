function [J_star_tables, U_star_tables, X_opt, U_opt, J_opt, grid_vecs, J_star, U_star] = ...
    solveDP_General(A, B, n, m, G, Q, R, N, h, xmin, xmax, umin, umax, x0, grid_points_x, grid_points_u)
% Solves a finite-horizon constrained optimal control problem using
% Bellman Dynamic Programming with multi-dimensional state and input grids.

    %% 1. Space Grid Discretization
    fprintf('• Structuring %dD State and %dD Input discrete mesh grids...\n', n, m);

    % Define state space grid paths
    grid_vecs = cell(1, n);
    for i = 1:n
        grid_vecs{i} = linspace(xmin(i), xmax(i), grid_points_x);
    end
    [X_grids{1:n}] = ndgrid(grid_vecs{:});

    % Flatten state combinations for high-performance vectorized referencing
    total_grid_points = grid_points_x^n;
    X_grid_points = zeros(total_grid_points, n);
    for i = 1:n
        X_grid_points(:, i) = X_grids{i}(:);
    end
    grid_dims = cellfun(@length, grid_vecs); 

    % Define control input space grid paths
    u_grid_vecs = cell(1, m);
    for i = 1:m
        u_grid_vecs{i} = linspace(umin(i), umax(i), grid_points_u);
    end
    [U_grids{1:m}] = ndgrid(u_grid_vecs{:});

    % Flatten control combinations 
    total_input_points = grid_points_u^m;
    U_grid_list = zeros(total_input_points, m);
    for i = 1:m
        U_grid_list(:, i) = U_grids{i}(:);
    end

    % Memory Allocation Allocation Pipeline
    J_star = cell(N + 1, 1);
    U_star = cell(N, 1);
    J_star_tables = cell(N + 1, 1); 
    U_star_tables = cell(N, 1); 

    %% 2. Backward Dynamic Programming Pass (Bellman Recursion)
    fprintf('• Launching backward optimization sequence (k = N down to 0)...\n');

    % --- Terminal State Penalty Assignment (k = N) ---
    k_idx = N + 1; 
    J_N_values = zeros(total_grid_points, 1);
    for i = 1:total_grid_points
        x = X_grid_points(i, :)'; 
        J_N_values(i) = x' * G * x;
    end
    J_star{k_idx} = reshape(J_N_values, grid_dims);

    % --- Step-by-Step Backwards Cost-to-Go Recursion ---
    for k = (N - 1):-1:0
        k_idx = k + 1;       
        k_next_idx = k + 2;  
        
        % Generate multi-dimensional cost-to-go function handle for step k+1
        J_k_plus_1_grid = J_star{k_next_idx};
        J_k_plus_1_interp = griddedInterpolant(grid_vecs, J_k_plus_1_grid, 'linear', 'nearest');
        
        J_k_values = zeros(total_grid_points, 1);
        U_k_values_flat = zeros(total_grid_points, m);
        
        % Loop through all points across the state mesh
        for i = 1:total_grid_points
            x_k = X_grid_points(i, :)'; 
            stage_cost_x = h * (x_k' * Q * x_k);
            
            min_cost_for_this_x = inf;
            best_u_for_this_x = nan(m, 1);
            
            % Exhaustive grid search over permissible input permutations
            for j = 1:total_input_points
                u_k = U_grid_list(j, :)'; 
                stage_cost_u = h * (u_k' * R * u_k);
                
                % Propagate state dynamic transition
                x_k_plus_1 = A * x_k + B * u_k;
                
                % Impose hard box state tracking boundaries
                if any(x_k_plus_1 < xmin) || any(x_k_plus_1 > xmax)
                    cost_to_go = inf; 
                else
                    x_k_plus_1_cell = num2cell(x_k_plus_1');
                    cost_to_go = J_k_plus_1_interp(x_k_plus_1_cell{:});
                end
                
                total_cost = stage_cost_x + stage_cost_u + cost_to_go;
                
                if total_cost < min_cost_for_this_x
                    min_cost_for_this_x = total_cost;
                    best_u_for_this_x = u_k;
                end
            end 
            
            J_k_values(i) = min_cost_for_this_x;
            U_k_values_flat(i, :) = best_u_for_this_x';
        end 
        
        J_star{k_idx} = reshape(J_k_values, grid_dims);
        U_star{k_idx} = reshape(U_k_values_flat, [grid_dims, m]);
    end 

    %% 3. Compile Optimization Policy Lookup Maps
    for k = 0:N
        k_idx = k + 1;
        J_star_tables{k_idx} = griddedInterpolant(grid_vecs, J_star{k_idx}, 'linear', 'nearest');
    end

    for k = 0:(N - 1)
        k_idx = k + 1;
        U_data_k = U_star{k_idx};
        U_interp_m = cell(m, 1); 
        
        S.type = '()';
        S.subs = [repmat({':'}, 1, n), 1]; 
        
        for j_m = 1:m
            S.subs{n + 1} = j_m; 
            U_i_data = subsref(U_data_k, S);
            U_interp_m{j_m} = griddedInterpolant(grid_vecs, U_i_data, 'linear', 'nearest');
        end
        U_star_tables{k_idx} = U_interp_m;
    end

    %% 4. Forward Dynamic Simulation Rollout
    fprintf('• Executing forward integration trajectory rollout from x0...\n');
    X_opt = zeros(n, N + 1);
    U_opt = zeros(m, N);
    J_opt_calc = 0; 

    X_opt(:, 1) = x0;

    for k = 0:(N - 1)
        k_idx = k + 1; 
        x_k = X_opt(:, k_idx);
        
        U_k_interp = U_star_tables{k_idx};
        u_k = zeros(m, 1);
        x_k_cell = num2cell(x_k'); 
        
        for j_m = 1:m
            u_k(j_m) = U_k_interp{j_m}(x_k_cell{:});
        end
        
        U_opt(:, k_idx) = u_k;
        stage_cost = h * (x_k' * Q * x_k + u_k' * R * u_k);
        J_opt_calc = J_opt_calc + stage_cost;   
        
        x_k_plus_1 = A * x_k + B * u_k;    
        x_k_plus_1 = max(min(x_k_plus_1, xmax), xmin); % Clamp the state within the admissible bounds to avoid numerical drift.
        X_opt(:, k_idx + 1) = x_k_plus_1;
    end

    x_N = X_opt(:, N + 1);
    terminal_cost = x_N' * G * x_N;
    J_opt = J_opt_calc + terminal_cost;
    
    fprintf('  Optimal Closed-Loop Trajectory Cost Verified: %.4f\n', J_opt);
end