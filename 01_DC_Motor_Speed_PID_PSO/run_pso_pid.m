%% DC Motor Speed Control: PID Tuning via Particle Swarm Optimization
clear; clc; close all;

%% 1. Search Space Boundaries [Kp, Ki, Kd]
lb = [0, 0, 0];
ub = [500, 100, 10];
num_vars = 3;

%% 2. PSO Optimizer Configuration
options = optimoptions('particleswarm', ...
    'SwarmSize', 15, ...
    'MaxIterations', 20, ...
    'Display', 'iter');

%% 3. Run Optimization
disp('Starting PSO optimization loop...');
tic;
[best_params, best_cost] = particleswarm(@pid_cost, num_vars, lb, ub, options);
elapsed_time = toc;

fprintf('\nOptimization finished in %.2f seconds.\n', elapsed_time);
fprintf('Optimal Parameters: Kp = %.4f, Ki = %.4f, Kd = %.4f\n', best_params);
fprintf('Minimum Cost Value: %.4f\n', best_cost);

%% 4. Baseline vs. Optimized Comparative Validation
% Simulate baseline manually tuned controller
baseline_params = [10, 5, 0.1];
pid_cost(baseline_params);
simOut_manual = sim('dc_motor_model_cost', 'StopTime', '3');
y_manual = simOut_manual.get('y');

% Simulate PSO-optimized controller
pid_cost(best_params);
simOut_pso = sim('dc_motor_model_cost', 'StopTime', '3');
y_pso = simOut_pso.get('y');

%% 5. Visualization
figure('Color', 'w', 'Position', [200, 200, 800, 450]);
plot(y_manual.Time, y_manual.Data, 'r--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Manual Baseline (Kp=%.1f, Ki=%.1f, Kd=%.1f)', baseline_params));
hold on;
plot(y_pso.Time, y_pso.Data, 'b-', 'LineWidth', 2.0, ...
    'DisplayName', sprintf('PSO Optimized (Kp=%.2f, Ki=%.2f, Kd=%.2f)', best_params));
yline(1.0, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Target Reference (1.0 rad/s)');

grid on;
box on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Rotor Speed (rad/s)', 'FontSize', 11);
title('DC Motor Velocity Response: Manual Tuning vs. PSO Optimization', 'FontSize', 12);
legend('Location', 'southeast', 'FontSize', 10);
