%% Grid Initialization
initGrids();

%% Main loop
figure(1);
positions = [];
max_loop = size(ins_poses_camera,1);

for t = 1:max_loop
    fprintf("Timestamp: %d\n", t)
    
    %% Get LIDAR pose related to camera (different timestamp)
    [pose_lidar_camera, laser_timestamp_camera] = getPoseLidarCamera(t, ...
        laser_timestamps, camera_timestamps, ins_timestamps, ins_poses,...
        ins_quaternions);

    %% Open stereo camera images
    if calculate_grid_stereo 
            image_left = LoadImage(image_dir_left, camera_timestamps(t), ...
                LUT_left);
            if ~image_left
                error(['No image found for timestamp: ' ...
                    num2str(camera_timestamps(t))]);
            end
            image_right = LoadImage(image_dir_right, camera_timestamps(t), ...
                LUT_right);
            if ~image_right
                error(['No image found for timestamp: ' ...
                    num2str(camera_timestamps(t))]);
            end
            u_disparity = processImage(image_left, image_right, scale_image);
    end

    %% Show image from left camera for demo 
    if show_camera_image 
        if ~calculate_grid_stereo
            image_left = LoadImage(image_dir_left, camera_timestamps(t), ...
                LUT_left);
            if ~image_left
                error(['No image found for timestamp: ' ...
                    num2str(camera_timestamps(t))]);
            end
        end
        figure(2);
        subplot(1,2,1);
        imshow(image_left);
    end

    %% Read laser scanner
    pointcloud = readScan(laser_dir, laser_timestamp_camera, 3);

    %% Get pose and timestamp at new camera acquision 
    pose = ins_poses_camera{t};
    timestamp = camera_timestamps(t);

    %% Mapping
    Mapping();

    %% Show global life-long map 
    showEvidentialMap(total_grid);
    if capture_video
        writeVideo(videoObj, getframe(gcf));
    end
    previous_timestamp = timestamp;
end