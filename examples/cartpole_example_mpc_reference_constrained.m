% MATLAB equivalent of cartpole_example_mpc_reference_constrained.py
% MPC simulation with input constraints and reference tracking

clear all; close all; clc;

fprintf('=== TinyMPC MATLAB: Cartpole Constrained MPC Example ===\n\n');

% Same cartpole dynamics as Python example
A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];

B = [0.0;
     0.02;
     0.0;
     0.067];

Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);

N = 20;

% Problem dimensions
nx = size(A, 1);
nu = size(B, 2);

% Affine dynamics term (zeros for linear system)
fdyn = zeros(nx, 1);

% Input constraints (matching Python example)
u_min_val = -0.45;
u_max_val = 0.45;

% Set up bounds
infty = 1e17;
x_min = -infty * ones(nx, N);
x_max = infty * ones(nx, N);
u_min = u_min_val * ones(nu, N-1);
u_max = u_max_val * ones(nu, N-1);

% Setup parameters
rho = 1.0;
verbose = 0;  % Set to 1 for debug output

fprintf('Setting up TinyMPC solver with input constraints [%.2f, %.2f]...\n', u_min_val, u_max_val);

% Setup solver
status = tinympc_matlab('setup', A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose);

if status ~= 0
    error('Setup failed with status %d', status);
end

% Set reference trajectory (goal position x=1.0, matching Python)
x_ref = repmat([1.0; 0; 0; 0], 1, N);  % Reference to equilibrium at x=1.0
u_ref = zeros(nu, N-1);
tinympc_matlab('set_x_ref', x_ref, verbose);
tinympc_matlab('set_u_ref', u_ref, verbose);

% Initial condition (same as Python)
x0 = [0.5; 0; 0; 0];

% Simulation parameters
Nsim = 1000;
xs = zeros(Nsim-N, 4);
us = zeros(Nsim-N, 1);

fprintf('Running constrained MPC simulation for %d steps...\n', Nsim-N);

% MPC loop
for i = 1:(Nsim-N)
    % Set current state as initial condition
    tinympc_matlab('set_x0', x0, verbose);
    
    % Solve MPC problem
    solve_status = tinympc_matlab('solve', verbose);
    
    % Get solution
    [x_sol, u_sol] = tinympc_matlab('get_solution', verbose);
    
    % Apply first control and simulate forward
    u_current = u_sol(:, 1);
    x0 = A * x0 + B * u_current;
    
    % Store results
    xs(i, :) = x0';
    us(i, :) = u_current';
    
    % Progress indicator
    if mod(i, 100) == 0
        fprintf('  Step %d/%d completed\n', i, Nsim-N);
    end
end

fprintf('Constrained MPC simulation completed!\n');

% Plot results
figure('Name', 'Cartpole Constrained MPC Results', 'Position', [100, 100, 800, 600]);

subplot(2, 1, 1);
plot(xs);
hold on;
plot([1, size(xs,1)], [1.0, 1.0], 'r--', 'LineWidth', 2);  % Reference line
legend({'x (meters)', 'theta (radians)', 'x\_dot (m/s)', 'theta\_dot (rad/s)', 'x reference'}, 'Location', 'northeast');
title('Cartpole trajectory with reference tracking');
ylabel('State values');
grid on;

subplot(2, 1, 2);
plot(us);
hold on;
plot([1, size(us,1)], [u_min_val, u_min_val], 'r--', 'LineWidth', 1);  % Lower bound
plot([1, size(us,1)], [u_max_val, u_max_val], 'r--', 'LineWidth', 1);  % Upper bound
legend({'control (Newtons)', 'constraints'}, 'Location', 'northeast');
xlabel('time steps (100Hz)');
ylabel('Control input');
ylim([u_min_val-0.1, u_max_val+0.1]);
grid on;

% Display final statistics
fprintf('\nFinal Results:\n');
fprintf('  Final position: %.4f meters (target: 1.0)\n', xs(end, 1));
fprintf('  Position error: %.4f meters\n', abs(xs(end, 1) - 1.0));
fprintf('  Final angle: %.4f radians\n', xs(end, 2));
fprintf('  Control range: [%.4f, %.4f] Newtons\n', min(us), max(us));
fprintf('  Constraint violations: %d\n', sum(us < u_min_val | us > u_max_val));

% Clean up
tinympc_matlab('reset', verbose);

fprintf('\nExample completed successfully!\n');
