# TinyMPC MATLAB Interface

MATLAB wrapper for [TinyMPC](https://tinympc.org/). Supports code generation and interaction with the C/C++ backend. Tested on Ubuntu and macOS.

## Prerequisites

- MATLAB (tested with R2024b and later)
- C++ compiler with C++17 support

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

## Features

- Basic MPC setup and solve
- Code generation
- Adaptive rho (automatic penalty adaptation)
- Sensitivity matrix support
- Cache term computation
- Constraint and reference setting

## Usage Example

```matlab
solver = TinyMPC(nx, nu, N, A, B, Q, R);
solver.setup('adaptive_rho', true, ...
            'adaptive_rho_min', 0.1, ...
            'adaptive_rho_max', 10.0, ...
            'adaptive_rho_enable_clipping', true, ...
            'verbose', true);
[Kinf, Pinf, Quu_inv, AmBKt] = solver.compute_cache_terms();
[dK, dP, dC1, dC2] = solver.compute_sensitivity_autograd();
solver.set_sensitivity_matrices(dK, dP, dC1, dC2);
solver.codegen_with_sensitivity('./output_dir', dK, dP, dC1, dC2);
```

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

## Troubleshooting

### macOS
1. **"Supported compiler not detected":**
   - Make sure Xcode is installed and its license is accepted: `sudo xcodebuild -license accept`
   - Or install Command Line Tools: `xcode-select --install`
   - Configure MATLAB's MEX compiler: `mex -setup C++`
2. **"Xcode license not accepted":**
   - Run `sudo xcodebuild -license accept`
   - Or open Xcode and accept the license through the GUI
3. **Eigen not found:**
   - The library includes a bundled Eigen
   - Or install: `brew install eigen`

### Linux
1. **Missing compiler:**
   - Install GCC: `sudo apt-get install build-essential`
   - Install MEX dependencies: `sudo apt-get install libc6-dev`
2. **Eigen not found:**
   - The library includes a bundled Eigen
   - Or install: `sudo apt-get install libeigen3-dev`
