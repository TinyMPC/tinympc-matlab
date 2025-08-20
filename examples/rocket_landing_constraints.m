%% Rocket Landing with Constraints
% Based on: https://github.com/TinyMPC/TinyMPC/blob/main/examples/rocket_landing_mpc.cpp

% Add TinyMPC class to path 
currentFile = mfilename('fullpath');
[scriptPath, ~, ~] = fileparts(currentFile);
repoRoot = fileparts(scriptPath);
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'build'));

% Problem dimensions
NSTATES = 6;  % [x, y, z, vx, vy, vz] 
NINPUTS = 3;  % [thrust_x, thrust_y, thrust_z]
NHORIZON = 10;

% System dynamics (from rocket_landing_params_20hz.hpp)
A = [1.0, 0.0, 0.0, 0.05, 0.0, 0.0;
     0.0, 1.0, 0.0, 0.0, 0.05, 0.0;
     0.0, 0.0, 1.0, 0.0, 0.0, 0.05;
     0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
     0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
     0.0, 0.0, 0.0, 0.0, 0.0, 1.0];

B = [0.000125, 0.0, 0.0;
     0.0, 0.000125, 0.0;
     0.0, 0.0, 0.000125;
     0.005, 0.0, 0.0;
     0.0, 0.005, 0.0;
     0.0, 0.0, 0.005];

fdyn = [0.0; 0.0; -0.0122625; 0.0; 0.0; -0.4905];
Q = diag([101.0, 101.0, 101.0, 101.0, 101.0, 101.0]);  % From parameter file
R = diag([2.0, 2.0, 2.0]);  % From parameter file

% Box constraints
x_min = [-5.0; -5.0; -0.5; -10.0; -10.0; -20.0];
x_max = [5.0; 5.0; 100.0; 10.0; 10.0; 20.0];
u_min = [-10.0; -10.0; -10.0];
u_max = [105.0; 105.0; 105.0];

% SOC constraints 
cx = [0.5];    % coefficients for state cones (mu)
cu = [0.25];   % coefficients for input cones (mu)
Acx = [0];     % start indices for state cones 
Acu = [0];     % start indices for input cones 
qcx = [3];     % dimensions for state cones
qcu = [3];     % dimensions for input cones

% Setup solver
solver = TinyMPC();
solver.setup(A, B, Q, R, NHORIZON, 'rho', 1.0, 'fdyn', fdyn, ...
             'max_iter', 100, 'abs_pri_tol', 2e-3, 'verbose', true);

solver.set_bound_constraints(x_min, x_max, u_min, u_max);

% Set cone constraints 
solver.set_cone_constraints(Acu, qcu, cu, Acx, qcx, cx);

% Initial and goal states 
xinit = [4.0; 2.0; 20.0; -3.0; 2.0; -4.5];
xgoal = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
xinit_scaled = xinit * 1.1;

% Initial reference trajectory 
x_ref = zeros(NSTATES, NHORIZON);
u_ref = zeros(NINPUTS, NHORIZON-1);

NTOTAL = 100;  
x_current = xinit_scaled;  

% Set initial reference 
for i = 1:NHORIZON
    x_ref(:, i) = xinit + (xgoal - xinit) * (i-1) / (NTOTAL - 1);  
end
u_ref(3, :) = 10.0;  % Hover thrust

solver.set_x_ref(x_ref);
solver.set_u_ref(u_ref);

% Store trajectory for plotting
trajectory = [];
controls = [];
constraint_violations = [];

fprintf('Starting rocket landing simulation...\n');
for k = 1:(NTOTAL - NHORIZON)
    % Calculate tracking error 
    tracking_error = norm(x_current - x_ref(:, 2));  % MATLAB is 1-indexed
    fprintf('tracking error: %.5f\n', tracking_error);
    
    % 1. Update measurement (set current state)
    solver.set_x0(x_current);
    
    % 2. Update reference trajectory 
    for i = 1:NHORIZON
        x_ref(:, i) = xinit + (xgoal - xinit) * (i + k - 2) / (NTOTAL - 1);
        if i <= NHORIZON - 1
            u_ref(3, i) = 10.0;  % uref stays constant
        end
    end
    
    solver.set_x_ref(x_ref);
    solver.set_u_ref(u_ref);
    
    % 3. Solve MPC problem
    solver.solve();
    solution = solver.get_solution();
    
    % 4. Simulate forward (apply first control)
    u_opt = solution.controls(:, 1);
    x_current = A * x_current + B * u_opt + fdyn;
    
    % Store data for plotting
    trajectory = [trajectory, x_current];
    controls = [controls, u_opt];
    
    % Check constraint violations
    altitude_violation = x_current(3) < 0;  % Ground constraint
    thrust_violation = norm(u_opt(1:2)) > 0.25 * abs(u_opt(3));  % Cone constraint
    constraint_violations = [constraint_violations, (altitude_violation || thrust_violation)];
end

fprintf('\nSimulation completed!\n');
fprintf('Initial state was: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', xinit_scaled');
fprintf('Final position: [%.2f, %.2f, %.2f]\n', x_current(1:3)');
fprintf('Final velocity: [%.2f, %.2f, %.2f]\n', x_current(4:6)');
fprintf('Distance to goal: %.3f m\n', norm(x_current(1:3)));
fprintf('Constraint violations: %d/%d\n', sum(constraint_violations), length(constraint_violations));

% Plotting
figure('Position', [100, 100, 1200, 900]);
sgtitle('Rocket Landing with Constraints', 'FontSize', 16);

% 2D trajectory (X-Y view)
subplot(2, 2, 1);
plot(trajectory(1, :), trajectory(2, :), 'b-', 'LineWidth', 2); hold on;
scatter(xinit_scaled(1), xinit_scaled(2), 100, 'red', 'filled');
scatter(xgoal(1), xgoal(2), 100, 'green', 'filled');
xlabel('X (m)'); ylabel('Y (m)');
legend('Trajectory', 'Start', 'Goal', 'Location', 'best');
title('2D Trajectory (X-Y)');
grid on;

% Position vs time
subplot(2, 2, 2);
time_steps = 1:size(trajectory, 2);
plot(time_steps, trajectory(1, :), 'r-', 'LineWidth', 1.5); hold on;
plot(time_steps, trajectory(2, :), 'g-', 'LineWidth', 1.5);
plot(time_steps, trajectory(3, :), 'b-', 'LineWidth', 1.5);
yline(0, 'k--', 'Alpha', 0.5);
xlabel('Time Step'); ylabel('Position (m)');
legend('X', 'Y', 'Z', 'Ground', 'Location', 'best');
title('Position vs Time');
grid on;

% Velocity vs time
subplot(2, 2, 3);
plot(time_steps, trajectory(4, :), 'r-', 'LineWidth', 1.5); hold on;
plot(time_steps, trajectory(5, :), 'g-', 'LineWidth', 1.5);
plot(time_steps, trajectory(6, :), 'b-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Velocity (m/s)');
legend('Vx', 'Vy', 'Vz', 'Location', 'best');
title('Velocity vs Time');
grid on;

% Thrust vs time
subplot(2, 2, 4);
plot(time_steps, controls(1, :), 'r-', 'LineWidth', 1.5); hold on;
plot(time_steps, controls(2, :), 'g-', 'LineWidth', 1.5);
plot(time_steps, controls(3, :), 'b-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Thrust (N)');
legend('Thrust X', 'Thrust Y', 'Thrust Z', 'Location', 'best');
title('Thrust vs Time');
grid on;
