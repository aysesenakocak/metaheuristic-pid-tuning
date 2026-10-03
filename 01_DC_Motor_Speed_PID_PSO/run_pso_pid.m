clear; clc; close all;

% 1. Plant physical parameters (CTMS Benchmark)
J = 0.01;   % Rotor inertia (kg.m^2)
b = 0.1;    % Viscous friction constant (N.m.s)
K = 0.01;   % Electromotive force / Torque constant (V/(rad/s) or N.m/A)
R = 1.0;    % Electric resistance (Ohm)
L = 0.5;    % Electric inductance (H)

% 2. Search boundaries and PSO options
lb = [0 0 0];
ub = [500 300 10];
options = optimoptions('particleswarm', 'SwarmSize', 15, 'MaxIterations', 20, 'Display', 'iter');

% 3. Run optimization
[best_params, best_cost] = particleswarm(@pid_cost, 3, lb, ub, options);

% 4. Display results
disp('Optimal parameters (Kp, Ki, Kd):');
disp(best_params);
disp('Minimum cost:');
disp(best_cost);

% 5. Simulate with optimal gains and plot response
pid_cost(best_params);
simOut = sim('dc_motor_model_cost', 'StopTime', '3');
y = simOut.get('y');

figure;
plot(y.Time, y.Data, 'b', 'LineWidth', 2);
hold on;
yline(1.0, 'r--', 'LineWidth', 1.5); % Target velocity reference (1.0 rad/s)
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('PSO-Tuned DC Motor Speed Response');
legend('Motor Speed (y)', 'Reference (1.0 rad/s)', 'Location', 'Southeast');
