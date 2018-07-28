%% Read Camera extrinsics
camera = 'stereo';
camera_extrinsics = dlmread([extrinsics_dir  camera '.txt']);
camera_timestamps = dlmread([main_dir '/' camera '.timestamps']);

%% Read LIDAR extrinsics
laser = 'ldmrs';
laser_extrinisics = dlmread([extrinsics_dir laser '.txt']);
laser_timestamps = dlmread([laser_dir '../' laser '.timestamps']);

%% Read INS extrinsics
extrinsics_path = [extrinsics_dir 'ins.txt'];
ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);

G_camera = SE3MatrixFromComponents(camera_extrinsics);
G_ins = SE3MatrixFromComponents(ins_extrinsics);
G_laser = SE3MatrixFromComponents(laser_extrinisics);
G_ins_laser = G_ins \ G_laser;
G_ins_camera = G_ins \  G_camera ;

%% Camera Internal parameters
[fx_left, fy_left, cx_left, cy_left, G_camera_image_left, LUT_left] = ...
    ReadCameraModel(image_dir_left, models_dir);
[fx_right, fy_right, cx_right, cy_right, G_camera_image_right, LUT_right] = ...
    ReadCameraModel(image_dir_right, models_dir);
camera_params.f = fx_left;
camera_params.cu = cx_left;
camera_params.cv = cy_left;
camera_params.b = 0.239983;

%% Define camera timestamps
[~, initial_camera_timestamp] = min(abs(camera_timestamps-initial_image_timestamp));
[~, end_camera_timestamp] = min(abs(camera_timestamps-end_image_timestamp));
camera_timestamps = camera_timestamps(initial_camera_timestamp:end_camera_timestamp);

%% Define LIDAR timestamps
laser_timestamps = laser_timestamps(:,1);
[~, initial_laser_timestamp] = min(abs(laser_timestamps-initial_image_timestamp));
[~, end_laser_timestamp] = min(abs(laser_timestamps-end_image_timestamp));
laser_timestamps = laser_timestamps(initial_laser_timestamp:end_laser_timestamp);

%% Find poses for each timestamp 
[ins_timestamps, ins_poses, ins_quaternions, pitches, ins_states] = readINS(ins_file);
ins_poses_camera = InterpolatePoses(camera_timestamps(1,:), camera_timestamps(1,1), ...
        ins_timestamps, ins_poses, ins_quaternions,0);