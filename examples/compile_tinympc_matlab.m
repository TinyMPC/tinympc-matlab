function compile_tinympc_matlab()
    % COMPILE_TINYMPC_MATLAB - Bare bones compilation script for TinyMPC MATLAB interface
    % Works on Linux and Mac (Intel/Apple Silicon)
    % Run from the examples directory.
    
    fprintf('=== TinyMPC MATLAB Compilation ===\n\n');
    
    % Get paths
    current_dir = pwd;
    if endsWith(current_dir, 'examples')
        repo_root = fileparts(current_dir);
    else
        repo_root = current_dir;
    end
    
    wrapper_dir = fullfile(repo_root, 'src', 'matlab_wrapper');
    tinympc_dir = fullfile(repo_root, 'src', 'tinympc', 'TinyMPC');
    
    % Check essential directories
    if ~exist(wrapper_dir, 'dir')
        error('Wrapper directory not found: %s', wrapper_dir);
    end
    if ~exist(tinympc_dir, 'dir')
        error('TinyMPC directory not found: %s\nRun: git submodule update --init --recursive', tinympc_dir);
    end
    
    % Source files
    source_files = {
        fullfile(wrapper_dir, 'bindings.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'tiny_api.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'admm.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'codegen.cpp');
        fullfile(tinympc_dir, 'src', 'tinympc', 'rho_benchmark.cpp');
    };
    
    % Include directories
    include_dirs = {
        fullfile(tinympc_dir, 'include');
        fullfile(tinympc_dir, 'src');
        wrapper_dir;
    };
    
    % Add Eigen paths
    eigen_paths = get_eigen_paths();
    if ~isempty(eigen_paths)
        include_dirs = [include_dirs; eigen_paths(:)];  % Ensure column vector
    end
    
    % Build MEX command
    mex_cmd = 'mex -v';
    mex_cmd = [mex_cmd, ' CXXFLAGS="$CXXFLAGS -std=c++17 -O3 -fPIC"'];
    
    % Add includes
    for i = 1:length(include_dirs)
        if exist(include_dirs{i}, 'dir')
            mex_cmd = [mex_cmd, sprintf(' -I"%s"', include_dirs{i})];
        end
    end
    
    % Add sources
    for i = 1:length(source_files)
        mex_cmd = [mex_cmd, sprintf(' "%s"', source_files{i})];
    end
    
    mex_cmd = [mex_cmd, ' -output tinympc_matlab'];
    
    % Compile
    fprintf('Compiling...\n');
    eval(mex_cmd);
    
    % Check result
    mex_ext = get_mex_extension();
    output_file = ['tinympc_matlab.' mex_ext];
    if exist(output_file, 'file')
        fprintf('✓ Success: %s\n', output_file);
    else
        error('Compilation failed - no output file found');
    end
end

function paths = get_eigen_paths()
    paths = {};
    
    % Try bundled Eigen first
    current_dir = pwd;
    if endsWith(current_dir, 'examples')
        repo_root = fileparts(current_dir);
    else
        repo_root = current_dir;
    end
    tinympc_dir = fullfile(repo_root, 'src', 'tinympc', 'TinyMPC');
    bundled_eigen = fullfile(tinympc_dir, 'include');
    if exist(bundled_eigen, 'dir')
        paths{end+1, 1} = bundled_eigen;  % Ensure column vector
    end
    
    % Add system Eigen paths
    if ismac()
        system_paths = {
            '/opt/homebrew/include/eigen3';
            '/usr/local/include/eigen3';
        };
    else
        system_paths = {
            '/usr/include/eigen3';
            '/usr/local/include/eigen3';
        };
    end
    
    for i = 1:length(system_paths)
        if exist(system_paths{i}, 'dir')
            paths{end+1, 1} = system_paths{i};  % Ensure column vector
        end
    end
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
