function compile_tinympc_matlab()
    % COMPILE_TINYMPC_MATLAB - Helper script to compile TinyMPC MATLAB interface
    
    fprintf('=== TinyMPC MATLAB Compilation ===\n\n');
    
    % Get current directory and paths
    current_dir = pwd;
    if endsWith(current_dir, 'tests')
        repo_root = fileparts(current_dir);
    else
        repo_root = current_dir;
    end
    
    wrapper_dir = fullfile(repo_root, 'src', 'matlab_wrapper');
    tinympc_dir = fullfile(repo_root, 'src', 'tinympc', 'TinyMPC');
    
    fprintf('Repository root: %s\n', repo_root);
    fprintf('Wrapper directory: %s\n', wrapper_dir);
    fprintf('TinyMPC directory: %s\n', tinympc_dir);
    
    % Check directories exist
    if ~exist(wrapper_dir, 'dir')
        error('Wrapper directory not found: %s', wrapper_dir);
    end
    
    if ~exist(tinympc_dir, 'dir')
        error('TinyMPC directory not found: %s\nRun: git submodule update --init --recursive', tinympc_dir);
    end
    
    % Install Eigen if not found
    fprintf('Checking for Eigen...\n');
    if ~check_eigen_system()
        fprintf('Installing Eigen via apt...\n');
        [status, ~] = system('sudo apt-get update && sudo apt-get install -y libeigen3-dev');
        if status ~= 0
            error('Failed to install Eigen. Please install manually: sudo apt-get install libeigen3-dev');
        end
    end
    
    % Simple approach: compile all source files directly (like Python does)
    fprintf('Compiling TinyMPC MATLAB interface...\n');
    
    % Source files - only the essentials
    source_files = {
        fullfile(wrapper_dir, 'wrapper.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'tiny_api.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'admm.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'codegen.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'rho_benchmark.cpp');
    };
    
    % Include directories
    include_dirs = {
        fullfile(tinympc_dir, 'src');
        fullfile(tinympc_dir, 'include');
        wrapper_dir;
        '/usr/include/eigen3';  % Standard Eigen location on Linux
    };
    
    % Check files exist
    fprintf('\nChecking source files:\n');
    for i = 1:length(source_files)
        if exist(source_files{i}, 'file')
            fprintf('  ✓ %s\n', source_files{i});
        else
            error('Missing file: %s', source_files{i});
        end
    end
    
    % Build MEX command
    mex_cmd = 'mex -v';  % Add verbose for debugging
    
    % C++ flags
    mex_cmd = [mex_cmd, ' CXXFLAGS="$CXXFLAGS -std=c++17 -O3"'];
    
    % Include directories
    for i = 1:length(include_dirs)
        if exist(include_dirs{i}, 'dir')
            mex_cmd = [mex_cmd, sprintf(' -I"%s"', include_dirs{i})];
            fprintf('  ✓ Include: %s\n', include_dirs{i});
        end
    end
    
    % Source files
    for i = 1:length(source_files)
        mex_cmd = [mex_cmd, sprintf(' "%s"', source_files{i})];
    end
    
    % Output
    mex_cmd = [mex_cmd, ' -output tinympc_matlab'];
    
    fprintf('\nMEX command:\n%s\n\n', mex_cmd);
    
    % Compile
    try
        fprintf('Starting compilation...\n');
        eval(mex_cmd);
        fprintf('✓ Compilation successful!\n');
        
        % Check output
        if exist('tinympc_matlab.mexa64', 'file')
            fprintf('✓ Created: tinympc_matlab.mexa64\n');
        else
            fprintf('⚠ Output file not found\n');
        end
        
    catch ME
        fprintf('✗ Compilation failed: %s\n', ME.message);
        
        % Try without the wrapper's codegen.cpp to avoid conflicts
        fprintf('\nTrying alternative compilation without wrapper codegen...\n');
        
        alt_source_files = {
            fullfile(wrapper_dir, 'wrapper.cpp');  % Only wrapper.cpp, not codegen.cpp
            fullfile(tinympc_dir, 'src', 'tinympc', 'tiny_api.cpp');
            fullfile(tinympc_dir, 'src', 'tinympc', 'admm.cpp');
            fullfile(tinympc_dir, 'src', 'tinympc', 'codegen.cpp');
            fullfile(tinympc_dir, 'src', 'tinympc', 'rho_benchmark.cpp');  % ADD THIS LINE TOO
        };
        
        % Rebuild command with alternative files
        mex_cmd = 'mex -v CXXFLAGS="$CXXFLAGS -std=c++17 -O3"';
        
        for i = 1:length(include_dirs)
            if exist(include_dirs{i}, 'dir')
                mex_cmd = [mex_cmd, sprintf(' -I"%s"', include_dirs{i})];
            end
        end
        
        for i = 1:length(alt_source_files)
            mex_cmd = [mex_cmd, sprintf(' "%s"', alt_source_files{i})];
        end
        
        mex_cmd = [mex_cmd, ' -output tinympc_matlab'];
        
        fprintf('Alternative MEX command:\n%s\n\n', mex_cmd);
        
        try
            eval(mex_cmd);
            fprintf('✓ Alternative compilation successful!\n');
        catch ME2
            fprintf('✗ Both compilation attempts failed\n');
            fprintf('Original error: %s\n', ME.message);
            fprintf('Alternative error: %s\n', ME2.message);
            rethrow(ME2);
        end
    end
end

function has_eigen = check_eigen_system()
    % Check if Eigen is available system-wide
    has_eigen = exist('/usr/include/eigen3/Eigen/Dense', 'file') == 2 || ...
                exist('/usr/local/include/eigen3/Eigen/Dense', 'file') == 2;
end