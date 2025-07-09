function compile_tinympc_matlab()
    % COMPILE_TINYMPC_MATLAB - Helper script to compile TinyMPC MATLAB interface
    %
    % This script attempts to compile the TinyMPC MATLAB interface using MEX.
    % It assumes the TinyMPC submodule is properly initialized.
    %
    % Prerequisites:
    %   1. MATLAB with MEX compiler configured (run 'mex -setup' if needed)
    %   2. TinyMPC submodule initialized (git submodule update --init --recursive)
    %   3. C++17 compatible compiler
    %
    % Usage:
    %   compile_tinympc_matlab()
    
    fprintf('=== TinyMPC MATLAB Compilation ===\n\n');
    
    % Get current directory and paths
    current_dir = pwd;
    [repo_root, ~, ~] = fileparts(fileparts(current_dir));
    wrapper_dir = fullfile(repo_root, 'src', 'matlab_wrapper');
    tinympc_dir = fullfile(repo_root, 'src', 'tinympc', 'TinyMPC');
    
    fprintf('Repository root: %s\n', repo_root);
    fprintf('Wrapper directory: %s\n', wrapper_dir);
    fprintf('TinyMPC directory: %s\n', tinympc_dir);
    
    % Check if directories exist
    if ~exist(wrapper_dir, 'dir')
        error('Wrapper directory not found: %s', wrapper_dir);
    end
    
    if ~exist(tinympc_dir, 'dir')
        error('TinyMPC directory not found: %s\nRun: git submodule update --init --recursive', tinympc_dir);
    end
    
    % Source files to compile
    wrapper_files = {
        fullfile(wrapper_dir, 'wrapper.cpp'),
        fullfile(wrapper_dir, 'codegen.cpp')
    };
    
    tinympc_src_files = {
        fullfile(tinympc_dir, 'src', 'tinympc', 'tiny_api.cpp'),
        fullfile(tinympc_dir, 'src', 'tinympc', 'admm.cpp'),
        fullfile(tinympc_dir, 'src', 'tinympc', 'codegen.cpp'),
        fullfile(tinympc_dir, 'src', 'tinympc', 'rho_benchmark.cpp')
    };
    
    % Check if source files exist
    all_files = [wrapper_files, tinympc_src_files];
    fprintf('\nChecking source files:\n');
    for i = 1:length(all_files)
        if exist(all_files{i}, 'file')
            fprintf('  ✓ %s\n', all_files{i});
        else
            fprintf('  ✗ %s (MISSING)\n', all_files{i});
            error('Missing source file: %s', all_files{i});
        end
    end
    
    % Include directories
    include_dirs = {
        fullfile(tinympc_dir, 'src'),
        fullfile(tinympc_dir, 'include'),
        wrapper_dir
    };
    
    fprintf('\nInclude directories:\n');
    for i = 1:length(include_dirs)
        if exist(include_dirs{i}, 'dir')
            fprintf('  ✓ %s\n', include_dirs{i});
        else
            fprintf('  ✗ %s (MISSING)\n', include_dirs{i});
        end
    end
    
    % Build MEX command
    mex_cmd = 'mex';
    
    % Add C++ standard flag
    mex_cmd = [mex_cmd, ' CXXFLAGS="$CXXFLAGS -std=c++17"'];
    
    % Add include directories
    for i = 1:length(include_dirs)
        mex_cmd = [mex_cmd, sprintf(' -I"%s"', include_dirs{i})];
    end
    
    % Add source files
    for i = 1:length(all_files)
        mex_cmd = [mex_cmd, sprintf(' "%s"', all_files{i})];
    end
    
    % Output name
    mex_cmd = [mex_cmd, ' -output tinympc_matlab'];
    
    % Add optimization flags
    mex_cmd = [mex_cmd, ' -O'];
    
    fprintf('\nMEX command:\n%s\n\n', mex_cmd);
    
    % Attempt compilation
    try
        fprintf('Starting compilation...\n');
        eval(mex_cmd);
        fprintf('✓ Compilation successful!\n\n');
        
        % Check output
        if ispc
            lib_file = 'tinympc_matlab.mexw64';
        elseif ismac
            lib_file = 'tinympc_matlab.mexmaci64';
        else
            lib_file = 'tinympc_matlab.mexa64';
        end
        
        if exist(lib_file, 'file')
            fprintf('Generated library: %s\n', lib_file);
            
            % Test loading
            try
                if libisloaded('tinympc_matlab')
                    unloadlibrary('tinympc_matlab');
                end
                
                % For MEX files, we don't use loadlibrary
                fprintf('✓ MEX file created successfully\n');
                
            catch ME
                fprintf('⚠ Warning: Could not test library loading: %s\n', ME.message);
            end
        else
            fprintf('⚠ Warning: Expected output file not found: %s\n', lib_file);
        end
        
        fprintf('\nNext steps:\n');
        fprintf('1. Run the test suite: run_all_tests()\n');
        fprintf('2. Try the cartpole example: test_cartpole_example()\n');
        fprintf('3. Run integration tests: test_with_library()\n');
        
    catch ME
        fprintf('✗ Compilation failed: %s\n', ME.message);
        fprintf('\nTroubleshooting:\n');
        fprintf('1. Check that MEX is configured: mex -setup\n');
        fprintf('2. Ensure C++17 compiler is available\n');
        fprintf('3. Verify TinyMPC submodule: git submodule status\n');
        fprintf('4. Check file paths and permissions\n');
        
        rethrow(ME);
    end
end
