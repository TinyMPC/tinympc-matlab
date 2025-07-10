# MATLAB Interface for TinyMPC

MATLAB wrapper for [TinyMPC](https://tinympc.org/). It supports code generation and interaction with the C/C++ code. This has been tested on Ubuntu and macOS.

### Prerequisites

**Required for all platforms:**
- MATLAB (tested with R2024b and later)
- C++ compiler with C++17 support

**On macOS:**
- Xcode (recommended) or Command Line Tools for Xcode
- If using Xcode: accept the license agreement by running `sudo xcodebuild -license accept`
- If using Command Line Tools only: `xcode-select --install`
- Optional: Eigen can be installed via `brew install eigen` (the library includes a bundled version)

**On Ubuntu/Linux:**
- GCC with C++17 support
- Optional: Eigen can be installed via `sudo apt-get install libeigen3-dev` (the library includes a bundled version)

### Building

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
- `tinympc_matlab.mexa64` - The compiled MEX file for Linux
- `tinympc_matlab.mexmaci64` - The compiled MEX file for Intel-based macOS
- `tinympc_matlab.mexmaca64` - The compiled MEX file for Apple Silicon macOS

## Documentation

The interface is documented [here](https://tinympc.org/).

## Troubleshooting

**macOS Compilation Issues:**

1. **"Supported compiler not detected":**
   - Make sure Xcode is installed and its license is accepted: `sudo xcodebuild -license accept`
   - Or install Command Line Tools: `xcode-select --install`
   - Configure MATLAB's MEX compiler: `mex -setup C++`

2. **"Xcode license not accepted":**
   - Run `sudo xcodebuild -license accept` in Terminal
   - Or open Xcode and accept the license through the GUI

3. **Eigen not found:**
   - The library includes a bundled version of Eigen that should work automatically
   - If needed, install system Eigen: `brew install eigen`

**Linux Compilation Issues:**

1. **Missing compiler:**
   - Install GCC: `sudo apt-get install build-essential`
   - Install MEX dependencies: `sudo apt-get install libc6-dev`

2. **Eigen not found:**
   - The library includes a bundled version of Eigen that should work automatically
   - If needed, install system Eigen: `sudo apt-get install libeigen3-dev`
