function metrics = computePerformanceMetrics(t_fin, x_fin, u_fin, lambda_fin, K_history_fin, ...
                                            t_inf, x_inf, u_inf, lambda_inf, G_inf, ...
                                            A, B, Q, R, dt, n)
% Handles numerical differentiations, performance metrics, and cost parsing.

    metrics = struct();
    t_comp = t_fin(1:end-1);
    metrics.t_comp = t_comp;

    % --- 1. Costate Dynamics Verification ---
    lambda_dot_numeric_fin = diff(lambda_fin, 1, 1) / dt;
    analytic_lambda_dot_fin = zeros(length(t_comp), n);
    for i = 1:length(t_comp)
        analytic_lambda_dot_fin(i, :) = (-Q * x_fin(i, :)' - A' * lambda_fin(i, :)')';
    end
    metrics.error_fin = lambda_dot_numeric_fin - analytic_lambda_dot_fin;

    lambda_dot_numeric_inf = diff(lambda_inf, 1, 1) / dt;
    analytic_lambda_dot_inf = zeros(length(t_comp), n);
    for i = 1:length(t_comp)
        analytic_lambda_dot_inf(i, :) = (-Q * x_inf(i, :)' - A' * lambda_inf(i, :)')';
    end
    metrics.error_inf = lambda_dot_numeric_inf - analytic_lambda_dot_inf;

    % --- 2. State Norms (Figure 5) ---
    metrics.norm_x_fin = sqrt(sum(x_fin.^2, 2));
    metrics.norm_x_inf = sqrt(sum(x_inf.^2, 2));

    % --- 3. Control Energy Accumulation (Figure 6) ---
    metrics.energy_u_fin = cumtrapz(t_fin, sum(u_fin.^2, 2));
    metrics.energy_u_inf = cumtrapz(t_inf, sum(u_inf.^2, 2));

    % --- 4. Running Costs (Figure 7) ---
    N_steps = length(t_fin);
    metrics.running_cost_fin = zeros(N_steps, 1);
    metrics.running_cost_inf = zeros(N_steps, 1);
    for i = 1:N_steps
        metrics.running_cost_fin(i) = x_fin(i,:)*Q*x_fin(i,:)' + u_fin(i,:)*R*u_fin(i,:)';
        metrics.running_cost_inf(i) = x_inf(i,:)*Q*x_inf(i,:)' + u_inf(i,:)*R*u_inf(i,:)';
    end

    % --- 5. Accumulated System Costs (Figure 8) ---
    metrics.acc_cost_fin = cumtrapz(t_fin, metrics.running_cost_fin);
    metrics.acc_cost_inf = cumtrapz(t_inf, metrics.running_cost_inf);

    % --- 6. Closed-Loop Spectrum Parsing (Figure 9) ---
    metrics.eig_open_loop = eig(A);
    metrics.eig_closed_loop = eig(A - B * G_inf);

    % --- 7. Riccati Time Histories (Figure 10) ---
    metrics.K11 = squeeze(K_history_fin(1, 1, :));
    metrics.K22 = squeeze(K_history_fin(2, 2, :));
    metrics.K33 = squeeze(K_history_fin(3, 3, :));
end