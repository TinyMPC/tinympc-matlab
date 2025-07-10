# MATLAB Interface for TinyMPC

MATLAB wrapper for [TinyMPC](https://tinympc.org/). It supports code generation and interaction with the C/C++ code. This has been tested on Ubuntu.

## Installation

1. Clone this repo: 

```bash
git clone https://github.com/TinyMPC/tinympc-matlab.git
```

2. Compile the MATLAB wrapper by running the compilation script:

```matlab
cd tests
compile_tinympc_matlab
```

3. Run the interactive example `interactive_cartpole.mlx` under the examples directory.

## Repository Structure

### Examples

The `examples` directory contains MATLAB scripts demonstrating various features of TinyMPC:

- `interactive_cartpole.mlx` - An interactive live script for controlling a cartpole system
- `cartpole_example_one_solve.m` - Simple one-step solve example for a cartpole system
- `cartpole_example_mpc.m` - Full MPC implementation for cartpole control
- `cartpole_example_mpc_reference_constrained.m` - MPC with reference tracking and constraints
- `cartpole_example_code_generation.m` - Demonstrates code generation capabilities
- `quadrotor_hover_code_generation.m` - Code generation example for a quadrotor hover system

### Tests

The `tests` directory contains scripts for testing and building the TinyMPC MATLAB interface:

- `compile_tinympc_matlab.m` - Script to compile the MEX interface
- `run_all_tests.m` - Runs a comprehensive test suite for the MATLAB interface
- `compare_examples_comprehensive.py` - Python script to compare results between MATLAB and Python implementations

## Build Artifacts

When you compile the library, the following files are generated:
- `tinympc_matlab.mexa64` - The compiled MEX file for Linux (or equivalent for other platforms)

## Documentation

The interface is documented [here](https://tinympc.org/).
