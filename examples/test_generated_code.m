% MATLAB equivalent of test_generated_code.py
% Test the generated C code for cartpole problem

clear all; close all; clc;

fprintf('=== TinyMPC MATLAB: Test Generated Code Example ===\n\n');

% Note: This example requires that code generation has been run first
% Run cartpole_example_code_generation.m to generate the C code

% Check if generated code exists
if ~exist('out', 'dir') || ~exist('out/src/tiny_data.cpp', 'file')
    fprintf('❌ Generated code not found!\n');
    fprintf('Please run cartpole_example_code_generation.m first to generate the C code.\n');
    return;
end

fprintf('Found generated code in "out" directory\n');

% For simulation purposes (same as Python example)
A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];

B = [0.0;
     0.02;
     0.0;
     0.067];

% Initial condition
x0 = [0.5; 0; 0; 0];

% Reference trajectory (same as Python)
x_ref = repmat([1.0; 0; 0; 0], 1, 20);

fprintf('Note: This example demonstrates the concept of testing generated code.\n');
fprintf('In a real implementation, you would:\n');
fprintf('1. Compile the generated C code into a MEX function\n');
fprintf('2. Call the generated solver instead of the main TinyMPC interface\n');
fprintf('3. Compare performance between interpreted and generated code\n\n');

% For demonstration, we'll simulate what the generated code would do
% by using the regular TinyMPC interface with the same problem setup

% Problem dimensions
nx = size(A, 1);
nu = size(B, 2);
N = 20;

% Affine dynamics term (zeros for linear system)
fdyn = zeros(nx, 1);

% Same setup as the code generation example
Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);
rho = 1.0;
verbose = 0;

% Input constraints (same as code generation)
u_min_val = -0.5;
u_max_val = 0.5;

% Set up bounds
infty = 1e17;
x_min = -infty * ones(nx, N);
x_max = infty * ones(nx, N);
u_min = u_min_val * ones(nu, N-1);
u_max = u_max_val * ones(nu, N-1);

fprintf('Setting up TinyMPC solver (simulating generated code behavior)...\n');

% Setup solver
status = tinympc_matlab('setup', A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose);

if status ~= 0
    error('Setup failed with status %d', status);
end

% Set reference trajectory
tinympc_matlab('set_x_ref', x_ref, verbose);
tinympc_matlab('set_u_ref', zeros(nu, N-1), verbose);

% Simulation parameters
Nsim = 1000;
xs = zeros(Nsim, 4);
us = zeros(Nsim, 1);

fprintf('Running MPC simulation (demonstrating generated code functionality)...\n');

% MPC loop (simulating what generated code would do)
for i = 1:Nsim
    % Set current state as initial condition
    tinympc_matlab('set_x0', x0, verbose);
    
    % Solve MPC problem (in real implementation, this would call generated code)
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
    if mod(i, 200) == 0
        fprintf('  Step %d/%d completed\n', i, Nsim);
    end
end

fprintf('Simulation completed!\n');

% Plot results (same as Python example)
figure('Name', 'Generated Code Test Results', 'Position', [100, 100, 800, 600]);

subplot(2, 1, 1);
plot(xs);
hold on;
plot([1, size(xs,1)], [1.0, 1.0], 'r--', 'LineWidth', 2);  % Reference line
legend({'x (meters)', 'theta (radians)', 'x\_dot (m/s)', 'theta\_dot (rad/s)', 'x reference'}, 'Location', 'northeast');
title('Cartpole trajectory over time (generated code test)');
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

fprintf('\nGenerated code test example completed!\n');
fprintf('In a production environment, this would use the actual generated C code.\n');
