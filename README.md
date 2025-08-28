# TinyMPC MATLAB Interface

MATLAB wrapper for [TinyMPC](https://tinympc.org/). Supports code generation and interaction with the C/C++ backend. Tested on Ubuntu and macOS.


## Prerequisites

- MATLAB (R2024b +)
- C++ compiler with C++17 support


## Building

1. **Clone this repo (with submodules):**
   ```bash
   git clone --recurse-submodules https://github.com/TinyMPC/tinympc-matlab.git
   ```
   If you already cloned without `--recurse-submodules`, run:
   ```bash
   git submodule update --init --recursive
   ```

2. **Build the C++ library:**
   ```bash
   cd tinympc-matlab
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Verify installation:**
   ```matlab
   % Add paths and test that the module loads correctly
   addpath('src'); addpath('build');
   solver = TinyMPC();
   disp('TinyMPC MATLAB ready to use!');
   ```

## Examples

The `examples/` directory contains scripts demonstrating TinyMPC features:
- `interactive_cartpole.m` - Interactive cartpole control
- `cartpole_example_one_solve.m` - One-step solve
- `cartpole_example_mpc.m` - Full MPC loop
- `cartpole_example_mpc_reference_constrained.m` - Reference tracking and constraints
- `cartpole_example_code_generation.m` - Code generation
- `quadrotor_hover_code_generation.m` - Quadrotor codegen

## Usage Example

### Basic MPC Workflow

```matlab
% Add required paths
addpath('src'); addpath('build');

% System matrices (cartpole example)
A = [1.0  0.01  0.0   0.0;
     0.0  1.0   0.039 0.0;
     0.0  0.0   1.002 0.01;
     0.0  0.0   0.458 1.002];
B = [0.0; 0.02; 0.0; 0.067];
Q = diag([10.0, 1.0, 10.0, 1.0]);
R = 1.0;
N = 20;  % Horizon length

% Create and setup solver
solver = TinyMPC();
solver.setup(A, B, Q, R, N, 'rho', 1.0, 'verbose', false);

% Set initial state and references
x0 = [0.5; 0; 0; 0];  % Initial state
solver.set_x0(x0);
solver.set_x_ref(zeros(4, N));      % State reference trajectory
solver.set_u_ref(zeros(1, N-1));    % Control reference trajectory

% Solve and get solution
status = solver.solve();  % Returns status code (0 = success)
solution = solver.get_solution();  % Get actual solution

% Access solution
fprintf('First control: %.3f\n', solution.controls(1));
states_trajectory = solution.states;     % All predicted states (4×20)
controls_trajectory = solution.controls; % All predicted controls (1×19)
```

### Code Generation Workflow

```matlab
% Setup solver
solver = TinyMPC();
u_min = -0.5; u_max = 0.5;  % Control bounds
solver.setup(A, B, Q, R, N, 'rho', 1.0);

% Set bounds explicitly
solver.set_bound_constraints([], [], u_min, u_max);

% Generate C++ code
solver.codegen('out');
```

### Adaptive Rho Workflow

```matlab
% Setup solver first
solver = TinyMPC();
solver.setup(A, B, Q, R, N, 'rho', 1.0, 'adaptive_rho', true);

% Compute sensitivity matrices using built-in numerical differentiation
[dK, dP, dC1, dC2] = solver.compute_sensitivity_autograd();

% Generate code with sensitivity matrices
solver.codegen_with_sensitivity('out', dK, dP, dC1, dC2);
```

See `examples/quadrotor_hover_code_generation.m` for a complete example.

## API Reference

### Core Functions

```matlab
% Setup solver with system matrices
solver.setup(A, B, Q, R, N, 'rho', rho, 'verbose', verbose, ...)

% Set initial state and references
solver.set_x0(x0)
solver.set_x_ref(x_ref)
solver.set_u_ref(u_ref)

% Solve and get solution
status = solver.solve()          % Returns status code (0 = success)
solution = solver.get_solution() % Get actual solution
```

### Code Generation

```matlab
% Generate standalone C++ code
solver.codegen(output_dir)

% Generate code with sensitivity matrices
solver.codegen_with_sensitivity(output_dir, dK, dP, dC1, dC2)
```

### Sensitivity Analysis

```matlab
% Compute sensitivity matrices using built-in autograd function
[dK, dP, dC1, dC2] = solver.compute_sensitivity_autograd()
```

### Constraints API

```matlab
% Bounds (box constraints)
solver.set_bound_constraints(x_min, x_max, u_min, u_max);

% Linear inequalities
solver.set_linear_constraints(Alin_x, blin_x, Alin_u, blin_u);

% Second-order cones (states first, then inputs)
solver.set_cone_constraints(Acx, qcx, cx, Acu, qcu, cu);

% Equality constraints (implemented as paired inequalities)
% Aeq_x * x == beq_x, Aeq_u * u == beq_u
solver.set_equality_constraints(Aeq_x, beq_x, Aeq_u, beq_u);
```

Each call auto-enables the corresponding constraint(s) in the C++ layer.

### Configuration

```matlab
% Update solver settings
solver.update_settings('abs_pri_tol', 1e-6, 'abs_dua_tol', 1e-6, 'max_iter', 100, ...)

% Reset solver
solver.reset()
```

## Solution Structure

The `solver.get_solution()` function returns a struct with:
- `solution.states` - Full state trajectory (nx × N matrix)
- `solution.controls` - Optimal control sequence (nu × (N-1) matrix)

## Testing

Run the test suite:
```matlab
addpath('tests');
run_all_tests;
```

See [https://tinympc.org/](https://tinympc.org/) for full documentation.
