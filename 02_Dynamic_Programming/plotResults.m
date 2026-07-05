function plotResults(X_opt, U_opt, J_opt, N, h, xmin, xmax, umin, umax, ...
                    grid_vecs, J_star, U_star, imageFolder, n, m)
% Handles advanced visual parsing, grid mappings, 3D surface configurations, 
% and vectorized graphic file export.

    % Define high-contrast aesthetic asset color tokens
    c_blue   = [0.00, 0.45, 0.74];
    c_orange = [0.85, 0.33, 0.10];
    c_green  = [0.47, 0.67, 0.19];
    
    t_vec   = (0:N) * h;    
    t_u_vec = (0:N-1) * h; 

    %% Internal layout tool for plot standardization
    function applyStyle(ax, xlbl, ylbl, titleStr)
        set(ax, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
        grid(ax, 'on'); box(ax, 'on');
        xlabel(ax, xlbl, 'FontWeight', 'bold');
        ylabel(ax, ylbl, 'FontWeight', 'bold');
        title(ax, titleStr, 'FontSize', 12, 'FontWeight', 'bold');
    end

    function saveAsset(figHandle, filename)
        exportgraphics(figHandle, fullfile(imageFolder, filename), 'Resolution', 300);
    end

    %% Figure 1 & 2: Trajectory Profiles (State and Inputs)
    f1 = figure('Name', 'System Operations Trajectories', 'Position', [150, 150, 800, 600]);
    
    ax1 = subplot(2, 1, 1);
    plot(t_vec, X_opt(1, :), '-o', 'Color', c_blue, 'LineWidth', 2, 'DisplayName', 'x_1 (Position)');
    hold on;
    plot(t_vec, X_opt(2, :), '-s', 'Color', c_orange, 'LineWidth', 2, 'DisplayName', 'x_2 (Velocity)');
    plot(t_vec, ones(size(t_vec)) * xmax(1), 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(t_vec, ones(size(t_vec)) * xmin(1), 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    hold off;
    applyStyle(ax1, 'Time (s)', 'State Vectors', 'Optimal State Trajectories vs. Boundaries');
    legend(ax1, 'show', 'Location', 'best');

    ax2 = subplot(2, 1, 2);
    stairs(t_u_vec, U_opt(1, :), 'Color', c_green, 'LineWidth', 2, 'DisplayName', 'u_1 (Control Effort)');
    hold on;
    plot(t_u_vec, ones(size(t_u_vec)) * umax(1), 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(t_u_vec, ones(size(t_u_vec)) * umin(1), 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    hold off;
    applyStyle(ax2, 'Time (s)', 'Control Input Value', 'Optimal Step Control Input Profile');
    ylim([umin(1) - 0.5, umax(1) + 0.5]);
    legend(ax2, 'show', 'Location', 'best');
    saveAsset(f1, '01_SystemTrajectories.png');

    %% Figure 3: State-Space Geometry Phase Portrait
    f2 = figure('Name', 'Phase Portrait');
    plot(X_opt(1, :), X_opt(2, :), '-o', 'Color', 'k', 'LineWidth', 2, 'MarkerFaceColor', c_blue);
    hold on;
    plot(X_opt(1, 1), X_opt(2, 1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Initial State');
    plot(X_opt(1, end), X_opt(2, end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Equilibrium Target');
    
    % Draw the geometric rectangle safely without properties it doesn't recognize
    rectangle('Position', [xmin(1), xmin(2), xmax(1)-xmin(1), xmax(2)-xmin(2)], ...
              'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1.2);
    % Provide an invisible dummy line to act as the legend placeholder
    plot(nan, nan, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Constraint Boundary');
    
    hold off;
    applyStyle(gca, 'State x_1 (Position)', 'State x_2 (Velocity)', 'Optimal Trajectory Phase Portrait Space');
    axis([xmin(1)-1, xmax(1)+1, xmin(2)-1, xmax(2)+1]);
    legend('Location', 'best');
    saveAsset(f2, '02_PhasePortrait.png');

    %% Figure 4: Instantaneous Dynamic Operating Cost Running Profile
    f3 = figure('Name', 'Running Cost Profile');
    running_cost = zeros(N, 1);
    for k = 1:N
        running_cost(k) = h * (X_opt(:, k)' * diag([0.25, 0.05]) * X_opt(:, k) + U_opt(:, k)' * 0.05 * U_opt(:, k));
    end
    plot(t_u_vec, running_cost, '-^', 'Color', c_orange, 'LineWidth', 2);
    applyStyle(gca, 'Time (s)', 'Stage Cost L(x,u)', 'Instantaneous Stage Operational Cost Profile');
    saveAsset(f3, '03_RunningCost.png');

    %% 2D Slicing Configuration for Surface Visualizations (n=2 check)
    if n == 2
        [X1_mesh, X2_mesh] = meshgrid(grid_vecs{1}, grid_vecs{2});
        
        %% Figure 5: Cost-To-Go Profile Map at Initial State k=0
        f4 = figure('Name', 'Value Function Surface', 'Position', [200, 200, 700, 550]);
        J_vals_k0 = J_star{1}; 
        J_vals_k0(isinf(J_vals_k0)) = nan; % Strip infinite bounds elements for clean shading
        
        surf(X1_mesh, X2_mesh, J_vals_k0', 'EdgeColor', 'none');
        colormap(jet); colorbar; shading interp;
        applyStyle(gca, 'State x_1', 'State x_2', 'Optimal Cost-To-Go Surface Vector J_0(x)');
        zlabel('Cost-To-Go J*'); view(-45, 30);
        saveAsset(f4, '04_ValueFunctionSurface.png');

        %% Figure 6: Control Policy Surface Field Mapping at Initial Stage k=0
        f5 = figure('Name', 'Control Policy Surface', 'Position', [250, 250, 700, 550]);
        U_vals_k0 = U_star{1}(:, :, 1); 
        
        surf(X1_mesh, X2_mesh, U_vals_k0', 'EdgeColor', 'none');
        colormap(parula); colorbar; shading interp;
        applyStyle(gca, 'State x_1', 'State x_2', 'Optimal State Feedback Control Policy Surface U_0(x)');
        zlabel('Control Value u*'); view(-45, 30);
        saveAsset(f5, '05_ControlPolicySurface.png');
    end
end