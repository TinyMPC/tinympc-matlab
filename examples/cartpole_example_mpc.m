% Concise Cartpole MPC Example (MATLAB) - matches Python version
clear; close all;

% Add TinyMPC class to path
currentFile = mfilename('fullpath');
[scriptPath, ~, ~] = fileparts(currentFile);
repoRoot = fileparts(scriptPath);
addpath(fullfile(repoRoot, 'src', 'matlab_wrapper'));

% Cartpole system matrices
A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];
B = [0.0; 0.02; 0.0; 0.067];
Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);
N = 20;

% Create and setup solver (Python-like interface)
prob = TinyMPC(size(A,1), size(B,2), N, A, B, Q, R);
prob.setup('rho', 1.0, 'verbose', false);

% Set initial condition
x0 = [0.5; 0; 0; 0];

% Simulation loop
Nsim = 1000;
xs = zeros(Nsim-N, 4);
us = zeros(Nsim-N, 1);

for i = 1:(Nsim-N)
    prob.set_initial_state(x0);
    prob.solve();
    [~, u_traj] = prob.get_solution();
    u = u_traj(:,1);
    x0 = A * x0 + B * u;
    xs(i,:) = x0';
    us(i) = u;
end

% Plot results
figure;
subplot(2,1,1);
plot(xs);
legend({'x (meters)', 'theta (radians)', 'x\_dot (m/s)', 'theta\_dot (rad/s)'}, 'Location', 'northeast');
title('Cartpole trajectory over time');
grid on;

subplot(2,1,2);
plot(us);
legend({'control (Newtons)'}, 'Location', 'northeast');
xlabel('time steps (100Hz)');
grid on;
