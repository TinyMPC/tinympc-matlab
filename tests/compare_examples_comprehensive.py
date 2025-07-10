#!/usr/bin/env python3
"""
Comprehensive comparison script for TinyMPC Python and MATLAB examples.
Runs equivalent examples and compares results to ensure consistency.
"""

import subprocess
import sys
import os
import numpy as np
import scipy.io
import tempfile
from pathlib import Path
import importlib.util
import re

def run_python_example(example_name):
    """Run a Python example and return results."""
    print(f"=== Running Python {example_name}.py ===")
    
    # Path to Python examples
    python_examples_dir = Path(__file__).parent / ".." / ".." / "tinympc-python" / "examples"
    example_path = python_examples_dir / f"{example_name}.py"
    
    if not example_path.exists():
        print(f"Python example {example_path} not found")
        return None
    
    # Change to Python examples directory
    original_dir = os.getcwd()
    os.chdir(python_examples_dir)
    
    try:
        # Run the Python example
        result = subprocess.run([
            sys.executable, str(example_path)
        ], capture_output=True, text=True, timeout=300)
        
        if result.returncode != 0:
            print(f"Python example {example_name} failed!")
            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            return None
        
        print(f"Python example {example_name} completed successfully")
        
        # Parse results from output
        output_lines = result.stdout.split('\n')
        
        results = {
            'output': result.stdout,
            'success': True
        }
        
        # Extract specific values based on example type
        if "one_solve" in example_name:
            # Look for iteration count and control value
            for line in output_lines:
                if 'converged in' in line and 'iterations' in line:
                    try:
                        iter_val = int(line.split('converged in')[1].split('iterations')[0].strip())
                        results['iterations'] = iter_val
                    except:
                        pass
                        
        elif "mpc" in example_name:
            # Look for final control value or cost
            for line in output_lines:
                if 'Final cost' in line:
                    try:
                        cost_val = float(line.split(':')[1].strip())
                        results['final_cost'] = cost_val
                    except:
                        pass
                        
        elif "code_generation" in example_name:
            # Check if code generation files were created
            out_dir = python_examples_dir / "out"
            if out_dir.exists():
                results['code_generated'] = True
                results['generated_files'] = [f.name for f in out_dir.glob("*")]
            else:
                results['code_generated'] = False
                
        elif "test_generated_code" in example_name:
            # Look for test results
            for line in output_lines:
                if 'Test' in line and ('passed' in line or 'failed' in line):
                    results['test_result'] = 'passed' if 'passed' in line else 'failed'
        
        return results
        
    except subprocess.TimeoutExpired:
        print(f"Python example {example_name} timed out")
        return None
    except Exception as e:
        print(f"Error running Python example {example_name}: {e}")
        return None
    finally:
        os.chdir(original_dir)

def run_matlab_example(example_name):
    """Run a MATLAB example and return results."""
    print(f"\n=== Running MATLAB {example_name}.m ===")
    
    # Path to MATLAB examples
    matlab_examples_dir = Path(__file__).parent / ".." / "examples"
    example_path = matlab_examples_dir / f"{example_name}.m"
    
    if not example_path.exists():
        print(f"MATLAB example {example_path} not found")
        return None
    
    # Change to MATLAB examples directory
    original_dir = os.getcwd()
    os.chdir(matlab_examples_dir)
    
    try:
        # Run MATLAB example
        result = subprocess.run([
            'matlab', '-batch', example_name
        ], capture_output=True, text=True, timeout=300)
        
        if result.returncode != 0:
            print(f"MATLAB example {example_name} failed!")
            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            return None
        
        print(f"MATLAB example {example_name} completed successfully")
        
        # Parse results from output
        output_lines = result.stdout.split('\n')
        
        results = {
            'output': result.stdout,
            'success': True
        }
        
        # Extract specific values based on example type
        if "one_solve" in example_name:
            # Look for iteration count and control value
            for line in output_lines:
                if 'Iterations:' in line:
                    try:
                        iter_val = int(line.split(':')[1].strip())
                        results['iterations'] = iter_val
                    except:
                        pass
                        
        elif "mpc" in example_name:
            # Look for final control value or cost
            for line in output_lines:
                if 'Final cost' in line:
                    try:
                        cost_val = float(line.split(':')[1].strip())
                        results['final_cost'] = cost_val
                    except:
                        pass
                        
        elif "code_generation" in example_name:
            # Check if code generation files were created
            out_dir = matlab_examples_dir / "out"
            if out_dir.exists():
                results['code_generated'] = True
                results['generated_files'] = [f.name for f in out_dir.glob("*")]
            else:
                results['code_generated'] = False
                
        elif "test_generated_code" in example_name:
            # Look for test results
            for line in output_lines:
                if 'Test' in line and ('passed' in line or 'failed' in line):
                    results['test_result'] = 'passed' if 'passed' in line else 'failed'
        
        return results
        
    except subprocess.TimeoutExpired:
        print(f"MATLAB example {example_name} timed out")
        return None
    except Exception as e:
        print(f"Error running MATLAB example {example_name}: {e}")
        return None
    finally:
        os.chdir(original_dir)

def compare_example_results(example_name, python_results, matlab_results):
    """Compare Python and MATLAB results for a specific example."""
    print(f"\n=== Comparing {example_name} Results ===")
    
    if python_results is None or matlab_results is None:
        print("Cannot compare - missing results")
        return False
    
    if not python_results['success'] or not matlab_results['success']:
        print("Cannot compare - examples failed")
        return False
    
    # Compare based on example type
    if "one_solve" in example_name:
        # Compare iterations
        if 'iterations' in python_results and 'iterations' in matlab_results:
            iter_diff = abs(python_results['iterations'] - matlab_results['iterations'])
            print(f"Iterations:")
            print(f"  Python: {python_results['iterations']}")
            print(f"  MATLAB: {matlab_results['iterations']}")
            print(f"  Difference: {iter_diff}")
            
            if iter_diff > 2:
                print(f"❌ Iteration difference {iter_diff} is too large")
                return False
        
    elif "mpc" in example_name:
        # Compare final cost if available
        if 'final_cost' in python_results and 'final_cost' in matlab_results:
            cost_diff = abs(python_results['final_cost'] - matlab_results['final_cost'])
            print(f"Final cost:")
            print(f"  Python: {python_results['final_cost']:.6f}")
            print(f"  MATLAB: {matlab_results['final_cost']:.6f}")
            print(f"  Difference: {cost_diff:.2e}")
            
            if cost_diff > 1e-4:
                print(f"❌ Cost difference {cost_diff:.2e} exceeds tolerance")
                return False
        
    elif "code_generation" in example_name:
        # Compare if code was generated
        py_generated = python_results.get('code_generated', False)
        mat_generated = matlab_results.get('code_generated', False)
        
        print(f"Code generation:")
        print(f"  Python: {'✅' if py_generated else '❌'}")
        print(f"  MATLAB: {'✅' if mat_generated else '❌'}")
        
        if not (py_generated and mat_generated):
            print("❌ Code generation failed in one or both examples")
            return False
            
    elif "test_generated_code" in example_name:
        # Compare test results
        py_test = python_results.get('test_result', 'unknown')
        mat_test = matlab_results.get('test_result', 'unknown')
        
        print(f"Test results:")
        print(f"  Python: {py_test}")
        print(f"  MATLAB: {mat_test}")
        
        if py_test != 'passed' or mat_test != 'passed':
            print("❌ Tests failed in one or both examples")
            return False
    
    print(f"✅ {example_name} comparison passed!")
    return True

def test_compilation():
    """Test if MATLAB wrapper can be compiled."""
    print("\n=== Testing MATLAB Compilation ===")
    
    script_dir = Path(__file__).parent
    test_dir = script_dir / ".." / "tests"
    
    original_dir = os.getcwd()
    os.chdir(test_dir)
    
    try:
        # Try to compile
        result = subprocess.run([
            'matlab', '-batch', 'compile_tinympc_matlab'
        ], capture_output=True, text=True, timeout=600)
        
        if result.returncode == 0:
            print("✅ MATLAB compilation successful")
            return True
        else:
            print("❌ MATLAB compilation failed")
            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ MATLAB compilation timed out")
        return False
    finally:
        os.chdir(original_dir)

def main():
    """Main comparison runner."""
    print("TinyMPC Python vs MATLAB Examples Comprehensive Comparison")
    print("=" * 80)
    
    # List of examples to compare
    examples_to_compare = [
        "cartpole_example_one_solve",
        "cartpole_example_mpc",
        "cartpole_example_mpc_reference_constrained",
        "cartpole_example_code_generation",
        "quadrotor_hover_code_generation",
        "test_generated_code"
    ]
    
    try:
        # Test compilation first
        if not test_compilation():
            print("\n❌ Cannot proceed without successful compilation")
            sys.exit(1)
        
        all_passed = True
        
        for example in examples_to_compare:
            print(f"\n{'='*60}")
            print(f"Testing: {example}")
            print('='*60)
            
            # Run Python example
            python_results = run_python_example(example)
            
            # Run MATLAB example
            matlab_results = run_matlab_example(example)
            
            # Compare results
            success = compare_example_results(example, python_results, matlab_results)
            
            if not success:
                all_passed = False
                print(f"❌ {example} comparison failed")
            else:
                print(f"✅ {example} comparison passed")
        
        print(f"\n{'='*80}")
        if all_passed:
            print("🎉 All comparisons passed! Python and MATLAB examples produce consistent results.")
            sys.exit(0)
        else:
            print("❌ Some comparisons failed! Results do not match.")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n❌ Comparison runner failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
