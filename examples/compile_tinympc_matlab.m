function compile_tinympc_matlab()
    % COMPILE_TINYMPC_MATLAB - Helper script to compile TinyMPC MATLAB interface
    % Based on the working tinympc-python structure
    % This script should be run from the examples directory.
    
    fprintf('=== TinyMPC MATLAB Compilation ===\n\n');
    
    % Get current directory and paths
    current_dir = pwd;
    if endsWith(current_dir, 'examples')
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
    
    % Ensure Eigen is available
    fprintf('Checking for Eigen...\n');
    if ~check_eigen_system()
        fprintf('Warning: Eigen not found in expected locations.\n');
        fprintf('Attempting to install system Eigen as fallback...\n');
        install_eigen_system();
    end
    
    % Compile TinyMPC static library first (like Python does)
    fprintf('Compiling TinyMPC static library...\n');
    compile_tinympc_static(tinympc_dir);
    
    % Now compile the MATLAB wrapper
    fprintf('Compiling TinyMPC MATLAB interface...\n');
    
    % Essential source files (following Python structure)
    source_files = {
        fullfile(wrapper_dir, 'bindings.cpp');
    };
    
    % TinyMPC library files (pre-compiled)
    library_files = {
        fullfile(tinympc_dir, 'src', 'tinympc', 'tiny_api.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'admm.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'codegen.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'rho_benchmark.cpp');
    };
    
    % Add library files to source files
    source_files = [source_files; library_files];
    
    % Include directories (following Python structure)
    include_dirs = get_eigen_include_paths();
    % Add TinyMPC specific directories
    include_dirs = [include_dirs; {
        fullfile(tinympc_dir, 'src');         % For tinympc headers
        fullfile(tinympc_dir, 'include');     % For tinympc includes
        wrapper_dir;                          % For wrapper headers
    }];
    
    % Check source files exist
    fprintf('\nChecking source files:\n');
    for i = 1:length(source_files)
        if exist(source_files{i}, 'file')
            fprintf('  ✓ %s\n', source_files{i});
        else
            error('Missing file: %s', source_files{i});
        end
    end
    
    % Check include directories exist
    fprintf('\nChecking include directories:\n');
    for i = 1:length(include_dirs)
        if exist(include_dirs{i}, 'dir')
            fprintf('  ✓ %s\n', include_dirs{i});
        else
            fprintf('  ⚠ Missing: %s\n', include_dirs{i});
        end
    end
    
    % Build MEX command (following Python compiler flags)
    mex_cmd = 'mex -v';
    
    % C++ flags (matching Python's approach) 
    % Add preprocessor definitions to help with Eigen compatibility
    mex_cmd = [mex_cmd, ' CXXFLAGS="$CXXFLAGS -std=c++17 -O3 -fPIC -DEIGEN_DONT_PARALLELIZE"'];
    mex_cmd = [mex_cmd, ' -DEIGEN_MPL2_ONLY'];  % Add Eigen compatibility flag
    
    % Add include directories
    for i = 1:length(include_dirs)
        if exist(include_dirs{i}, 'dir')
            mex_cmd = [mex_cmd, sprintf(' -I"%s"', include_dirs{i})];
        end
    end
    
    % Add source files
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
        mex_extension = get_mex_extension();
        output_file = ['tinympc_matlab.' mex_extension];
        if exist(output_file, 'file')
            fprintf('✓ Created: %s\n', output_file);
            
            % Copy to examples folder (already here)
            fprintf('✓ MEX file is in the examples directory\n');
        else
            fprintf('⚠ Output file not found\n');
        end
        
    catch ME
        fprintf('✗ Compilation failed: %s\n', ME.message);
        
        % Print detailed error information:
        fprintf('\nDetailed error analysis:\n');
        fprintf('Error identifier: %s\n', ME.identifier);
        fprintf('Error message: %s\n', ME.message);
        
        if contains(ME.message, 'Eigen')
            fprintf('\n=== Eigen Error Detected ===\n');
            fprintf('This appears to be an Eigen header issue.\n');
            fprintf('Suggestions:\n');
            if is_macos()
                fprintf('1. Ensure Eigen is installed: brew install eigen\n');
                fprintf('2. Check if Eigen headers are in Homebrew location\n');
            else
                fprintf('1. Ensure Eigen is installed: sudo apt-get install libeigen3-dev\n');
                fprintf('2. Check if Eigen headers are in the expected location\n');
            end
            fprintf('3. The TinyMPC library may need to be modified to use standard Eigen includes\n');
        end
        
        rethrow(ME);
    end
end

function compile_tinympc_static(~)
    % Compile TinyMPC as a static library (following Python's approach)
    fprintf('Building TinyMPC static library...\n');
    % This is simplified - in a full implementation, you'd use CMake
    % For now, we'll compile the sources directly with the MEX command
    fprintf('  Note: TinyMPC sources will be compiled directly with the wrapper\n');
end

function has_eigen = check_eigen_system()
    % Check if Eigen is available system-wide or bundled
    eigen_paths = get_eigen_include_paths();
    has_eigen = false;
    for i = 1:length(eigen_paths)
        eigen_path = eigen_paths{i};
        if endsWith(eigen_path, 'Eigen') && exist(fullfile(eigen_path, 'Eigen', 'Dense'), 'file') == 2
            has_eigen = true;
            fprintf('  ✓ Bundled Eigen found at: %s\n', eigen_path);
            break;
        elseif exist(fullfile(eigen_path, 'Eigen', 'Eigen', 'Dense'), 'file') == 2
            has_eigen = true;
            fprintf('  ✓ Bundled Eigen found at: %s\n', eigen_path);
            break;
        elseif exist(fullfile(eigen_path, 'Eigen', 'Dense'), 'file') == 2
            has_eigen = true;
            fprintf('  ✓ System Eigen found at: %s\n', eigen_path);
            break;
        end
    end
    if ~has_eigen
        fprintf('  ✗ Eigen not found in any standard location\n');
    end
end

function install_eigen_system()
    if is_macos()
        fprintf('Installing Eigen via Homebrew...\n');
        [status, ~] = system('brew install eigen');
        if status ~= 0
            error('Failed to install Eigen. Please install manually: brew install eigen');
        end
    else
        fprintf('Installing Eigen via apt...\n');
        [status, ~] = system('sudo apt-get update && sudo apt-get install -y libeigen3-dev');
        if status ~= 0
            error('Failed to install Eigen. Please install manually: sudo apt-get install libeigen3-dev');
        end
    end
end

function paths = get_eigen_include_paths()
    current_dir = pwd;
    if endsWith(current_dir, 'examples')
        repo_root = fileparts(current_dir);
    else
        repo_root = current_dir;
    end
    tinympc_dir = fullfile(repo_root, 'src', 'tinympc', 'TinyMPC');
    paths = {
        fullfile(tinympc_dir, 'include', 'Eigen');
        fullfile(tinympc_dir, 'include');
    };
    if is_macos()
        system_paths = {
            '/opt/homebrew/include/eigen3';
            '/opt/homebrew/include/eigen3/Eigen';
            '/usr/local/include/eigen3';
            '/usr/local/include/eigen3/Eigen';
            '/opt/local/include/eigen3';
            '/opt/local/include/eigen3/Eigen';
        };
    else
        system_paths = {
            '/usr/include/eigen3';
            '/usr/include/eigen3/Eigen';
            '/usr/local/include/eigen3';
            '/usr/local/include/eigen3/Eigen';
        };
    end
    paths = [paths; system_paths];
end

function is_mac = is_macos()
    is_mac = ismac();
end

function ext = get_mex_extension()
    if ismac()
        if contains(computer('arch'), 'arm64')
            ext = 'mexmaca64';
        else
            ext = 'mexmaci64';
        end
    elseif isunix()
        ext = 'mexa64';
    elseif ispc()
        ext = 'mexw64';
    else
        error('Unsupported platform');
    end
end
