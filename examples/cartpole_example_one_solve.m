% MATLAB equivalent of cartpole_example_one_solve.py
% Single solve example for cartpole stabilization

clear all; close all; clc;

fprintf('=== TinyMPC MATLAB: Cartpole One Solve Example ===\n\n');

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

% Default bounds (large values, matching Python default)
infty = 1e17;
x_min = -infty * ones(nx, N);
x_max = infty * ones(nx, N);
u_min = -infty * ones(nu, N-1);
u_max = infty * ones(nu, N-1);

% Setup parameters (matching Python example)
rho = 1.0;
verbose = 0;  % Set to 1 for debug output

fprintf('Setting up TinyMPC solver...\n');

% Setup solver
status = tinympc_matlab('setup', A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose);

if status ~= 0
    error('Setup failed with status %d', status);
end

% Set initial condition (same as Python)
x0 = [0.5; 0; 0; 0];
tinympc_matlab('set_x0', x0, verbose);

% Set reference trajectories (zeros - regulation problem)
x_ref = zeros(nx, N);
u_ref = zeros(nu, N-1);
tinympc_matlab('set_x_ref', x_ref, verbose);
tinympc_matlab('set_u_ref', u_ref, verbose);

fprintf('Solving...\n');

% Solve
solve_status = tinympc_matlab('solve', verbose);

% Get solution
[x_sol, u_sol] = tinympc_matlab('get_solution', verbose);

% Get statistics
[iter, status, pri_res_state, pri_res_input] = tinympc_matlab('get_stats', verbose);

fprintf('Solver converged in %d iterations\n', iter);
fprintf('Control output: [%.8f]\n', u_sol(1));

% Display results
fprintf('\nResults:\n');
fprintf('  Status: %d\n', status);
fprintf('  Iterations: %d\n', iter);
fprintf('  First control: %.8f\n', u_sol(1));
fprintf('  Solution shapes: x_sol=%dx%d, u_sol=%dx%d\n', size(x_sol), size(u_sol));

% Clean up
tinympc_matlab('reset', verbose);

fprintf('\nExample completed successfully!\n');
