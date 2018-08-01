%% total_grid: global grid map with all the information stored over time
% Each dimension of the total grid stores: 
% 1: evidential mass of free state
% 2: evidential mass of occupied state
% 3: evidential mass of conflict state
% 4: evidential mass of unknown state
% 5: perception state (look perceptionGrid.m for specification)
% 6: counter hits occupied
total_grid = zeros(grid_parameters.size_grid_x, grid_parameters.size_grid_y,6);
total_grid(:,:,4) = 1.0; % initialize with evidential mass at unknown
total_grid(:,:,5) = 0; % initialize with perception state unknown

%% stereo_grid: stereo camera grid map of the current timestamp
% Each dimension stores the evidential mass of the 4 states
stereo_grid = zeros(grid_parameters.size_grid_x, grid_parameters.size_grid_y,4);
stereo_grid(:,:,4) = 1.0; % initialize with evidential mass at unknown

%% aging_stereo: matrix with the last updated timestamp of a cell
% Stores the last timestamp when a cell from the stereo_grid was updated
aging_stereo = ones(grid_parameters.size_grid_x, grid_parameters.size_grid_y,1) * ...
    camera_timestamps(1);

%% lidar_grid: lidar grid map of the current timestamp
% Each dimension stores the evidential mass of the 4 states
lidar_grid = zeros(grid_parameters.size_grid_x, grid_parameters.size_grid_y,4);
lidar_grid(:,:,4) = 1.0; % initialize with evidential mass at unknown

%% aging_lidar: matrix with the last updated timestamp of a cell
% Stores the last timestamp when a cell from the lidar_grid was updated
aging_lidar = ones(grid_parameters.size_grid_x, grid_parameters.size_grid_y,1) * ...
    camera_timestamps(1);

%% fusion_grid: fusion grid map of the current timestamp
% Each dimension stores the evidential mass of the 4 states after
% performing the fusion between stereo_grid and lidar_grid
fusion_grid = zeros(grid_parameters.size_grid_x, grid_parameters.size_grid_y,4);
fusion_grid(:,:,4) = 1.0; % initialize with evidential mass at unknown