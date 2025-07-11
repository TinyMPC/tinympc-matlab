% Concise Cartpole Code Generation Example (MATLAB) - matches Python version
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

% Set constraint bounds
u_min = -0.5;
u_max = 0.5;

% Setup solver
prob.setup('u_min', u_min, 'u_max', u_max, 'rho', 1.0, 'verbose', true);

% Generate code
prob.codegen('out');
