% Cartpole One Solve Example (MATLAB)
clear; close all;

% Add TinyMPC class to path

currentFile = mfilename('fullpath');
[scriptPath, ~, ~] = fileparts(currentFile);
repoRoot = fileparts(scriptPath);
addpath(fullfile(repoRoot, 'src'));
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

% Create solver
solver = TinyMPC();

% Setup solver with matrices
solver.setup(A, B, Q, R, N, 'rho', 1.0);

% Set initial condition and solve
x0 = [0.5; 0; 0; 0];
solver.set_x0(x0);
status = solver.solve();
solution = solver.get_solution();

% Display solution
disp(solution.controls);
