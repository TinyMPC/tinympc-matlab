% Concise Cartpole One Solve Example (MATLAB) - matches Python version
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

% Create and setup solver
prob = TinyMPC(size(A,1), size(B,2), N, A, B, Q, R);
prob.setup('rho', 1.0);

% Set initial condition and solve
x0 = [0.5; 0; 0; 0];
prob.set_initial_state(x0);
prob.solve();

% Get and display solution
[~, u_traj] = prob.get_solution();
disp(u_traj);
