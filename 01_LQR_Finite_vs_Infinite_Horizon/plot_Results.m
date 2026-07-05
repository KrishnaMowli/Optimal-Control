function plotResults(t_fin, x_fin, u_fin, lambda_fin, t_inf, x_inf, u_inf, lambda_inf, metrics, imageFolder, n, m)
% Manages plotting layouts and automated file saving pipelines.

    colors = [0.00, 0.45, 0.74;  % Deep Blue
              0.85, 0.33, 0.10;  % Rust Orange
              0.93, 0.69, 0.13]; % Warm Yellow

    %% Helper function for styling plots to publication standards
    function applyStyle(ax, xlbl, ylbl, titleStr)
        set(ax, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
        grid(ax, 'on');
        box(ax, 'on');
        xlabel(ax, xlbl, 'FontSize', 11, 'FontWeight', 'bold');
        ylabel(ax, ylbl, 'FontSize', 11, 'FontWeight', 'bold');
        title(ax, titleStr, 'FontSize', 12, 'FontWeight', 'bold');
    end

    function saveAsset(figHandle, filename)
        exportgraphics(figHandle, fullfile(imageFolder, filename), 'Resolution', 300);
    end

    %% Figure 1: Analytical Costate Equation Verification
    f1 = figure('Name', 'Analytical Verification', 'Position', [100 100 800 600]);
    sgtitle('Verification Error: \lambda_{dot} (Numeric) vs. \lambda_{dot} (Analytic)', 'FontSize', 13, 'FontWeight', 'bold');
    lbls = cell(n, 1); for i = 1:n, lbls{i} = sprintf('Error \\lambda_%d', i); end
    
    ax1 = subplot(2, 1, 1); plot(metrics.t_comp, metrics.error_fin, 'LineWidth', 1.5);
    applyStyle(ax1, 'Time (t)', 'Error', 'Finite Horizon: Costate Equation Error'); legend(ax1, lbls, 'Location', 'best');
    
    ax2 = subplot(2, 1, 2); plot(metrics.t_comp, metrics.error_inf, 'LineWidth', 1.5);
    applyStyle(ax2, 'Time (t)', 'Error', 'Infinite Horizon: Costate Equation Error'); legend(ax2, lbls, 'Location', 'best');
    saveAsset(f1, '01_AnalyticalVerification.png');

    %% Figure 2: State Trajectories Comparison
    f2 = figure('Name', 'State Comparison'); hold on;
    lbls = cell(2*n, 1);
    for i = 1:n
        plot(t_fin, x_fin(:,i), '-', 'LineWidth', 1.8, 'Color', colors(i,:));
        plot(t_inf, x_inf(:,i), '--', 'LineWidth', 1.8, 'Color', colors(i,:));
        lbls{2*i-1} = sprintf('x_%d (Finite)', i); lbls{2*i} = sprintf('x_%d (Infinite)', i);
    end
    hold off; applyStyle(gca, 'Time (t)', 'State Value', 'State Trajectories x(t): Finite vs. Infinite Horizon');
    legend(lbls, 'Location', 'best'); saveAsset(f2, '02_StateComparison.png');

    %% Figure 3: Control Input Comparison
    f3 = figure('Name', 'Input Comparison'); hold on;
    lbls = cell(2*m, 1);
    for i = 1:m
        c_idx = mod(i-1, size(colors,1)) + 1;
        plot(t_fin, u_fin(:,i), '-', 'LineWidth', 1.8, 'Color', colors(c_idx,:));
        plot(t_inf, u_inf(:,i), '--', 'LineWidth', 1.8, 'Color', colors(c_idx,:));
        lbls{2*i-1} = sprintf('u_%d (Finite)', i); lbls{2*i} = sprintf('u_%d (Infinite)', i);
    end
    hold off; applyStyle(gca, 'Time (t)', 'Input Value', 'Optimal Input u(t): Finite vs. Infinite Horizon');
    legend(lbls, 'Location', 'best'); saveAsset(f3, '03_InputComparison.png');

    %% Figure 4: Costate Variable Comparison
    f4 = figure('Name', 'Costate Comparison'); hold on;
    lbls = cell(2*n, 1);
    for i = 1:n
        plot(t_fin, lambda_fin(:,i), '-', 'LineWidth', 1.8, 'Color', colors(i,:));
        plot(t_inf, lambda_inf(:,i), '--', 'LineWidth', 1.8, 'Color', colors(i,:));
        lbls{2*i-1} = sprintf('\\lambda_%d (Finite)', i); lbls{2*i} = sprintf('\\lambda_%d (Infinite)', i);
    end
    hold off; applyStyle(gca, 'Time (t)', 'Costate Value', 'Costate Trajectories \lambda(t): Finite vs. Infinite Horizon');
    legend(lbls, 'Location', 'best'); saveAsset(f4, '04_CostateComparison.png');

    %% Figure 5: State Vector Trajectory Norms
    f5 = figure('Name', 'State Norm'); hold on;
    plot(t_fin, metrics.norm_x_fin, '-', 'LineWidth', 2, 'Color', colors(1,:));
    plot(t_inf, metrics.norm_x_inf, '--', 'LineWidth', 2, 'Color', colors(2,:));
    hold off; applyStyle(gca, 'Time (t)', '||x(t)||_2', 'System State Vector Norm Evolution');
    legend({'Finite Horizon', 'Infinite Horizon'}, 'Location', 'best'); saveAsset(f5, '05_StateNorm.png');

    %% Figure 6: Transmitted Control Energy Integration
    f6 = figure('Name', 'Control Energy'); hold on;
    plot(t_fin, metrics.energy_u_fin, '-', 'LineWidth', 2, 'Color', colors(1,:));
    plot(t_inf, metrics.energy_u_inf, '--', 'LineWidth', 2, 'Color', colors(2,:));
    hold off; applyStyle(gca, 'Time (t)', '\int u^2 dt', 'Accumulated Control Energy');
    legend({'Finite Horizon', 'Infinite Horizon'}, 'Location', 'best'); saveAsset(f6, '06_ControlEnergy.png');

    %% Figure 7: Instantaneous Running Costs
    f7 = figure('Name', 'Running Cost'); hold on;
    plot(t_fin, metrics.running_cost_fin, '-', 'LineWidth', 2, 'Color', colors(1,:));
    plot(t_inf, metrics.running_cost_inf, '--', 'LineWidth', 2, 'Color', colors(2,:));
    hold off; applyStyle(gca, 'Time (t)', 'x^TQx + u^TRu', 'Instantaneous Quadratic Running Cost');
    legend({'Finite Horizon', 'Infinite Horizon'}, 'Location', 'best'); saveAsset(f7, '07_RunningCost.png');

    %% Figure 8: Total Integrated Cost Profile
    f8 = figure('Name', 'Accumulated Cost'); hold on;
    plot(t_fin, metrics.acc_cost_fin, '-', 'LineWidth', 2, 'Color', colors(1,:));
    plot(t_inf, metrics.acc_cost_inf, '--', 'LineWidth', 2, 'Color', colors(2,:));
    hold off; applyStyle(gca, 'Time (t)', 'J(t)', 'Accumulated Cost Evolution Over Time');
    legend({'Finite Horizon', 'Infinite Horizon'}, 'Location', 'best'); saveAsset(f8, '08_AccumulatedCost.png');

    %% Figure 9: Complex Plane Pole-Zero Spectrums
    f9 = figure('Name', 'Closed-Loop Eigenvalues'); hold on;
    plot(real(metrics.eig_open_loop), imag(metrics.eig_open_loop), 'ro', 'MarkerSize', 9, 'LineWidth', 2);
    plot(real(metrics.eig_closed_loop), imag(metrics.eig_closed_loop), 'bx', 'MarkerSize', 11, 'LineWidth', 2);
    xline(0, 'k--', 'LineWidth', 1.2); yline(0, 'k--', 'LineWidth', 1.2); hold off;
    applyStyle(gca, 'Real Axis (Real)', 'Imaginary Axis (Imag)', 'System Pole Configuration: Stability Map');
    legend({'Open-Loop Poles', 'Infinite-Horizon Closed-Loop'}, 'Location', 'best'); saveAsset(f9, '09_Eigenvalues.png');

    %% Figure 10: Finite Horizon Matrix Time Evolution
    f10 = figure('Name', 'Riccati Evolution'); hold on;
    plot(t_fin, metrics.K11, 'LineWidth', 2, 'Color', colors(1,:));
    plot(t_fin, metrics.K22, 'LineWidth', 2, 'Color', colors(2,:));
    plot(t_fin, metrics.K33, 'LineWidth', 2, 'Color', colors(3,:));
    hold off; applyStyle(gca, 'Time (t)', 'K_{ii}(t)', 'Finite Horizon Riccati Gain Evolution');
    legend({'K_{11}', 'K_{22}', 'K_{33}'}, 'Location', 'best'); saveAsset(f10, '10_RiccatiEvolution.png');
end