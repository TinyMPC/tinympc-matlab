# TinyMPC MATLAB Tests

This directory contains comprehensive tests for the TinyMPC MATLAB interface.

## Test Files

### 1. `run_all_tests.m`
Basic interface tests that don't require the compiled library:
- Constructor validation
- Dimension checking
- Error handling
- Interface completeness

**Usage:**
```matlab
run_all_tests()
```

### 2. `test_cartpole_example.m`
Comprehensive cartpole MPC example demonstrating:
- Problem setup and validation
- Multiple test scenarios
- System analysis
- Expected workflow

**Usage:**
```matlab
test_cartpole_example()
```

### 3. `test_with_library.m`
Integration tests requiring the compiled library:
- Library loading
- Full solve workflow
- Constraint handling
- Reference tracking
- Code generation

**Usage:**
```matlab
test_with_library()
% or with custom paths:
test_with_library('lib_path', '/path/to/lib', 'header_path', '/path/to/header')
```

### 4. `compile_tinympc_matlab.m`
Helper script to compile the TinyMPC MATLAB interface:
- Checks prerequisites
- Builds MEX command
- Compiles wrapper and TinyMPC core
- Validates output

**Usage:**
```matlab
compile_tinympc_matlab()
```

## Running Tests

### Prerequisites

1. **MATLAB** with MEX compiler configured:
   ```matlab
   mex -setup
   ```

2. **TinyMPC submodule** initialized:
   ```bash
   git submodule update --init --recursive
   ```

3. **C++17 compatible compiler** (Visual Studio 2019+, GCC 7+, Clang 5+)

### Basic Tests (No Compilation Required)

```matlab
cd tests
run_all_tests()
test_cartpole_example()
```

### Full Integration Tests

1. **Compile the library:**
   ```matlab
   cd tests
   compile_tinympc_matlab()
   ```

2. **Run integration tests:**
   ```matlab
   test_with_library()
   ```

## Test Structure

### Interface Tests
- ✅ Constructor validation
- ✅ Dimension checking
- ✅ Matrix validation
- ✅ Error handling
- ✅ Method signatures

### Functionality Tests (Require Library)
- 🔄 Solver setup
- 🔄 Initial state setting
- 🔄 Constraint handling
- 🔄 Reference tracking
- 🔄 Solve execution
- 🔄 Solution retrieval
- 🔄 Code generation

### Example Problems
- **Cartpole**: 4-state nonlinear system (linearized)
- **Double Integrator**: Simple 2-state system
- **Reference Tracking**: Sinusoidal trajectory following

## Expected Outputs

### Successful Compilation
```
=== TinyMPC MATLAB Compilation ===
✓ All source files found
✓ Include directories verified
✓ Compilation successful!
Generated library: tinympc_matlab.mexw64
```

### Successful Tests
```
=== TinyMPC MATLAB Test Suite ===
Running test 1/7: Basic Setup and Validation
  ✓ PASSED

...

=== Test Summary ===
Tests passed: 7
Tests failed: 0
🎉 All tests passed!
```

## Troubleshooting

### Compilation Issues

1. **MEX not configured:**
   ```matlab
   mex -setup
   ```

2. **Missing TinyMPC submodule:**
   ```bash
   git submodule update --init --recursive
   ```

3. **C++17 not supported:**
   - Update compiler (Visual Studio 2019+, GCC 7+)
   - Check MATLAB compatibility

4. **Eigen not found:**
   - Ensure TinyMPC submodule includes Eigen headers
   - Check include paths

### Runtime Issues

1. **Library not loaded:**
   ```matlab
   % For MEX files, no explicit loading needed
   % For shared libraries:
   solver.load_library('path/to/lib', 'path/to/header')
   ```

2. **Solve failures:**
   - Check problem conditioning
   - Verify constraint feasibility
   - Adjust ADMM parameters (rho)

3. **Memory issues:**
   - Restart MATLAB
   - Check for memory leaks in wrapper

## Development

### Adding New Tests

1. Create test function in appropriate file
2. Add to test list in `run_all_tests.m`
3. Follow naming convention: `test_feature_name()`
4. Include both positive and negative test cases

### Test Categories

- **Unit Tests**: Individual function testing
- **Integration Tests**: Full workflow testing
- **Performance Tests**: Timing and memory usage
- **Regression Tests**: Preventing regressions

### Example Test Template

```matlab
function test_new_feature()
    % Test description
    
    try
        % Setup
        solver = TinyMPC(...);
        
        % Test positive case
        result = solver.new_method();
        assert(condition, 'Error message');
        
        % Test negative case
        try
            solver.invalid_call();
            error('Should have failed');
        catch ME
            assert(contains(ME.message, 'expected'), 'Wrong error');
        end
        
        fprintf('    - Feature test: OK\n');
        
    catch ME
        % Cleanup if needed
        rethrow(ME);
    end
end
```

## Platform Notes

### Windows
- Uses `.mexw64` extension
- Requires Visual Studio compiler
- May need Windows SDK

### macOS
- Uses `.mexmaci64` extension
- Requires Xcode command line tools
- May need Homebrew packages

### Linux
- Uses `.mexa64` extension
- Requires GCC/Clang
- May need development packages

## Continuous Integration

For CI/CD integration:

```yaml
# Example GitHub Actions workflow
- name: Setup MATLAB
  uses: matlab-actions/setup-matlab@v1
  
- name: Initialize submodules
  run: git submodule update --init --recursive
  
- name: Run MATLAB tests
  uses: matlab-actions/run-command@v1
  with:
    command: |
      cd tests
      try
        compile_tinympc_matlab()
        run_all_tests()
        test_with_library()
      catch ME
        disp(ME.message)
        exit(1)
      end
```
