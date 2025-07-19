# TinyMPC MATLAB Interface

MATLAB wrapper for [TinyMPC](https://tinympc.org/). Supports code generation and interaction with the C/C++ backend. Tested on Ubuntu and macOS.


## Prerequisites

- MATLAB (tested with R2024b and later)
- C++ compiler with C++17 support
- No additional toolboxes required (numerical differentiation used instead of symbolic)


## Building

1. Clone this repo (with submodules):
   ```bash
   git clone --recurse-submodules https://github.com/TinyMPC/tinympc-matlab.git
   ```
   If you already cloned without `--recurse-submodules`, run:
   ```bash
   git submodule update --init --recursive
   ```

2. Create a build directory and configure the project with CMake:
   ```bash
   mkdir build
   cd build
   cmake ..
   ```

3. Build the MATLAB wrapper and examples:
   ```bash
   make
   ```

4. Run any example in the `examples/` directory (e.g. `interactive_cartpole.m`)

## Examples

The `examples/` directory contains scripts demonstrating TinyMPC features:
- `interactive_cartpole.m` - Interactive cartpole control
- `cartpole_example_one_solve.m` - One-step solve
- `cartpole_example_mpc.m` - Full MPC loop
- `cartpole_example_mpc_reference_constrained.m` - Reference tracking and constraints
- `cartpole_example_code_generation.m` - Code generation
- `quadrotor_hover_code_generation.m` - Quadrotor codegen
- `test_adaptive_rho_functionality.m` - Adaptive rho and sensitivity test

## Usage Example

### Basic MPC Workflow

```matlab
% Create and setup solver
solver = TinyMPC();
solver.setup(A, B, Q, R, N, 'rho', 1.0, 'verbose', false);

% Set initial state and solve
x0 = [0.5; 0; 0; 0];  % Initial state
solver.set_x0(x0);
solution = solver.solve();

% Access solution
fprintf('First control: %.3f\n', solution.controls(1));
states_trajectory = solution.states_all;     % All predicted states
controls_trajectory = solution.controls_all; % All predicted controls
```

### Code Generation Workflow

```matlab
% Setup solver with constraints
solver = TinyMPC();
u_min = -0.5; u_max = 0.5;  % Control bounds
solver.setup(A, B, Q, R, N, 'u_min', u_min, 'u_max', u_max, 'rho', 1.0);


% Generate C++ code
solver.codegen('out');
```

### Sensitivity & Adaptive Rho Workflow

```matlab
% Setup solver with adaptive rho enabled
prob = TinyMPC();
prob.setup(A, B, Q, R, N, 'rho', 5.0, 'adaptive_rho', true);

% Set reference trajectories
Xref = zeros(nx, N);      % State reference (nx × N)
Uref = zeros(nu, N-1);    % Input reference (nu × N-1)
prob.set_x_ref(Xref);
prob.set_u_ref(Uref);

% Compute LQR cache terms
[Kinf, Pinf, Quu_inv, AmBKt] = prob.compute_cache_terms();

% Compute sensitivity matrices using numerical differentiation
[dK, dP, dC1, dC2] = prob.compute_sensitivity_autograd();

% Generate code with sensitivity support for adaptive rho
prob.codegen_with_sensitivity('out', dK, dP, dC1, dC2);
```

See `examples/quadrotor_hover_code_generation.m` for a complete example.

## API Reference

### Constructor and Setup
- `solver = TinyMPC()` - Create solver instance
- `solver.setup(A, B, Q, R, N, ...)` - Setup MPC problem with system matrices
  - Required: `A` (nx×nx), `B` (nx×nu), `Q` (nx×nx), `R` (nu×nu), `N` (horizon)
  - Optional: `'rho', value`, `'verbose', true/false`, `'adaptive_rho', true/false`

### Problem Configuration
- `solver.set_x0(x0)` - Set initial state (nx×1)
- `solver.set_x_ref(x_ref)` - Set state reference trajectory (nx×N)
- `solver.set_u_ref(u_ref)` - Set input reference trajectory (nu×(N-1))
- Set constraints in `setup()` using: `'u_min', value`, `'u_max', value`, `'x_min', value`, `'x_max', value`

### Solving
- `solution = solver.solve()` - Solve MPC optimization problem and return solution
  - `solution.controls` - First optimal control input
  - `solution.states_all` - Complete state trajectory (nx × N)
  - `solution.controls_all` - Complete control trajectory (nu × N-1)
- `solver.reset()` - Reset solver state

### Advanced Features
- `[Kinf, Pinf, Quu_inv, AmBKt] = solver.compute_cache_terms()` - Compute LQR matrices
- `[dK, dP, dC1, dC2] = solver.compute_sensitivity_autograd()` - Compute sensitivity matrices
  - Uses numerical finite differences: `d/drho ≈ (f(rho+h) - f(rho)) / h`
  - Returns derivatives of K, P, C1, C2 with respect to rho
- `solver.set_sensitivity_matrices(dK, dP, dC1, dC2)` - Set sensitivity for codegen

### Code Generation
- `solver.codegen(output_dir)` - Generate standalone C++ code
- `solver.codegen_with_sensitivity(output_dir, dK, dP, dC1, dC2)` - Generate code with adaptive rho support

## Tests

The `tests/` directory contains scripts for building and testing:
- `compile_tinympc_matlab.m` - Compile the MEX interface
- `run_all_tests.m` - Run all tests

## Build Artifacts

When you compile, the following files are generated:
- `tinympc_matlab.mexa64` - Linux
- `tinympc_matlab.mexmaci64` - Intel macOS
- `tinympc_matlab.mexmaca64` - Apple Silicon macOS

## Documentation

See [https://tinympc.org/](https://tinympc.org/) for full documentation.