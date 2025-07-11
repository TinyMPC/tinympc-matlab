% Cartpole MPC Reference Constrained Example (MATLAB)
clear; close all;

% Add TinyMPC class to path

currentFile = mfilename('fullpath');
[scriptPath, ~, ~] = fileparts(currentFile);
repoRoot = fileparts(scriptPath);
addpath(fullfile(repoRoot, 'src', 'matlab_wrapper'));
addpath(fullfile(repoRoot, 'build'));

% Cartpole system matrices
A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];
B = [0.0; 0.02; 0.0; 0.067];
Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);
N = 20;

% Constraint bounds
u_min = -0.45;
u_max = 0.45;

% Create solver
solver = TinyMPC();

% Setup solver with matrices and constraints
solver.setup(A, B, Q, R, N, 'u_min', u_min, 'u_max', u_max);

% Set reference trajectory (goal must be another equilibrium position)
x_ref = repmat([1.0; 0; 0; 0], 1, N);  % 4x20 matrix
solver.set_x_ref(x_ref);

% Set initial condition
x0 = [0.5; 0; 0; 0];

% Simulation loop
Nsim = 1000;
xs = zeros(Nsim-N, 4);
us = zeros(Nsim-N, 1);

for i = 1:(Nsim-N)
    solver.set_x0(x0);
    solution = solver.solve();
    u = solution.controls;
    x0 = A * x0 + B * u;
    xs(i,:) = x0';
    us(i) = u;
end

% Plot results
figure;
subplot(2,1,1);
plot(xs);
legend({'x (meters)', 'theta (radians)', 'x_dot (m/s)', 'theta_dot (rad/s)'}, 'Location', 'northeast');
title('cartpole trajectory over time');
grid on;

subplot(2,1,2);
plot(us);
legend({'control (Newtons)'}, 'Location', 'northeast');
xlabel('time steps (100Hz)');
grid on;
