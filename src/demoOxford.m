close all;

%% Testing parameters
capture_video = false;
calculate_grid_lidar = true;
calculate_grid_stereo = true;
show_camera_image = true;
initial_image_timestamp = 1418381798076682;
end_image_timestamp = 1418381817136573;

%% Algorithm parameters 
scale_image = 0.5;
grid_parameters.resolution = 0.3;
grid_parameters.size_grid_x = round(150 / grid_parameters.resolution);
grid_parameters.size_grid_y = round(100 / grid_parameters.resolution);
grid_parameters.origin = [ 1, round(grid_parameters.size_grid_y / 2.0)];

%% Folders
computer_folder = strcat(pwd,'/../');
main_dir = strcat(computer_folder,'/sample_small');
image_dir = strcat(main_dir, '/stereo/left/');
image_dir_left = strcat(main_dir, '/stereo/left/');
image_dir_right = strcat(main_dir, '/stereo/right/');
laser_dir = strcat(main_dir, '/ldmrs/');
ins_file = strcat(main_dir, '/gps/ins.csv');
models_dir = strcat(computer_folder,'/models/');
extrinsics_dir = strcat(computer_folder,'/extrinsics/');

%% Save video of testing sequence
if capture_video 
    name_video = 'demo_video';
    fileName = 'name_video.avi';
    videoObj = VideoWriter(fileName, 'Uncompressed AVI');
    videoObj.FrameRate = 6;
    open(videoObj);
end

%% Load all parameters 
loadParams();

%% Execute the mapping 
executeMapping();

%% Close video
if capture_video 
    close(videoObj);
end