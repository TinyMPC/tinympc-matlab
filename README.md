# TinyMPC MATLAB Interface

MATLAB wrapper for [TinyMPC](https://tinympc.org/). Supports code generation and interaction with the C/C++ backend. Tested on Ubuntu and macOS.


## Prerequisites

- MATLAB (tested with R2024b and later)
- C++ compiler with C++17 support
- **Symbolic Math Toolbox** (required for sensitivity computation and adaptive rho workflows)

### macOS
- Xcode or Command Line Tools
- Accept Xcode license: `sudo xcodebuild -license accept`
- Or install Command Line Tools: `xcode-select --install`
- Eigen is bundled, but can also be installed via `brew install eigen`

### Ubuntu/Linux
- GCC with C++17 support
- Eigen is bundled, but can also be installed via `sudo apt-get install libeigen3-dev`

## Building

1. Clone this repo:
   ```bash
   git clone https://github.com/TinyMPC/tinympc-matlab.git
   ```
2. Compile the MATLAB wrapper:
   ```matlab
   cd tests
   compile_tinympc_matlab
   ```
3. Run any example in the `examples/` directory (e.g. `interactive_cartpole.m`)

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

### Standard Workflow (No Sensitivity)

```matlab
prob = TinyMPC();
prob.setup(A, B, Q, R, N, 'rho', 5.0, 'verbose', true);
prob.set_x_ref(Xref);  % Set state reference trajectory
prob.set_u_ref(Uref);  % Set input reference trajectory
prob.codegen('output_dir');
```

### Adaptive Rho / Sensitivity Workflow

**Requires Symbolic Math Toolbox**

```matlab
prob = TinyMPC();
prob.setup(A, B, Q, R, N, 'rho', 5.0, 'verbose', true, 'adaptive_rho', true);
prob.set_x_ref(Xref);
prob.set_u_ref(Uref);

% Compute cache terms (LQR solution)
[Kinf, Pinf, Quu_inv, AmBKt] = prob.compute_cache_terms();

% Compute exact sensitivity matrices using Symbolic Math Toolbox
[dK, dP, dC1, dC2] = prob.compute_sensitivity_autograd();

% Set sensitivity matrices for codegen
prob.set_sensitivity_matrices(dK, dP, dC1, dC2);

% Generate code with sensitivity
prob.codegen_with_sensitivity('output_dir', dK, dP, dC1, dC2);
```

See `examples/quadrotor_hover_code_generation.m` for a complete example.
## Notes on Symbolic Math Toolbox

- The method `compute_sensitivity_autograd()` uses symbolic differentiation to compute exact derivatives of the LQR solution with respect to `rho`.
- This is required for adaptive rho workflows and for generating code with sensitivity matrices.
- For large systems, symbolic computation may be slow. For typical MPC problems (e.g., cartpole, quadrotor), it is practical and robust.


## Class Methods

**Core:**
- `TinyMPC(nx, nu, N, A, B, Q, R)`
- `setup(...)`
- `solve()`
- `get_solution()`
- `reset()`

**State/Reference:**
- `set_initial_state(x0)`
- `set_state_reference(x_ref)`
- `set_input_reference(u_ref)`
- `set_bounds(x_min, x_max, u_min, u_max)`

**Advanced:**
- `compute_cache_terms()`
- `compute_sensitivity_autograd()`
- `set_sensitivity_matrices(dK, dP, dC1, dC2)`

**Code Generation:**
- `codegen(output_dir)`
- `codegen_with_sensitivity(output_dir, dK, dP, dC1, dC2)`

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