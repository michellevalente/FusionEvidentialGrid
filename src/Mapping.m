%% LIDAR Mapping
if calculate_grid_lidar 
    % Create object reference for computational speed
    obj_aging_lidar=Reference;
    obj_aging_lidar.X=aging_lidar; clear aging_lidar;
    obj_lidar_grid=Reference;
    obj_lidar_grid.X=lidar_grid; clear lidar_grid;
    
    % Project pointcloud to INS frame in camera timestamp
    pose_lidar = pose * pose_lidar_camera * G_ins_laser;
    pointcloud = pose_lidar  *...
    [pointcloud; ones(1, size(pointcloud,2))];

    % Create the lidar grid
    [lidar_grid, aging_lidar, changes_lidar, limits] = lidarMapping(...
        pointcloud, grid_parameters, timestamp, pose_lidar, ...
        obj_lidar_grid, obj_aging_lidar);
end

%% Stereo Camera Mapping
if calculate_grid_stereo 
    % Create object reference for computational speed
    obj_aging_stereo=Reference;
    obj_aging_stereo.X=aging_stereo; clear aging_stereo;
    obj_stereo_grid=Reference;
    obj_stereo_grid.X=stereo_grid; clear stereo_grid;
    
    if ~calculate_grid_lidar 
        limits =[1 grid_parameters.size_grid_x 1 grid_parameters.size_grid_y];
    end
    
     % Create the stereo grid
    [stereo_grid, aging_stereo, changes_stereo] = stereoMapping(...
        obj_stereo_grid, u_disparity, grid_parameters, pose, ...,
        obj_aging_stereo, G_ins_camera, timestamp, camera_params, limits, ...
        scale_image);
end

%% Mapping Fusion ( time and sensor )
if calculate_grid_lidar && calculate_grid_stereo
    %% Fusion between stereo camera grid and lidar grid
    changes = union(changes_stereo, changes_lidar, 'rows');
    changes(1,:) = [];
    obj_lidar_grid=Reference; obj_lidar_grid.X=lidar_grid; 
    clear lidar_grid;
    obj_stereo_grid=Reference; obj_stereo_grid.X=stereo_grid; 
    clear stereo_grid;
    obj_fusion_grid=Reference; obj_fusion_grid.X=fusion_grid; 
    clear fusion_grid;
   
    [fusion_grid, lidar_grid, stereo_grid] = sensorFusion(obj_lidar_grid,...
        obj_stereo_grid, obj_fusion_grid, changes);
    
    %% Fusion between local grid and global grid
    if t > 1
        obj_total_grid=Reference;
        obj_total_grid.X=total_grid; 
        clear total_grid;
        obj_fusion_grid = Reference; obj_fusion_grid.X=fusion_grid;
        clear fusion_grid;
        
        [total_grid, fusion_grid] = timeFusion(obj_fusion_grid, ...
            obj_total_grid, changes);
    end

else
    if calculate_grid_lidar 
        %% Fusion between local lidar grid and global grid  
        changes = changes_lidar;
        obj=Reference;
        obj.X=total_grid; 
        clear total_grid;
        obj_fusion_grid = Reference; obj_fusion_grid.X=lidar_grid;
        clear lidar_grid;
        [total_grid, lidar_grid] = timeFusion(obj_fusion_grid, obj, ...
        timestamp, previous_timestamp, changes);
    else
        %% Fusion between local stereo camera grid and global grid 
        changes = changes_stereo;
        obj=Reference; obj.X=total_grid; 
        clear total_grid;
        obj_fusion_grid = Reference; obj_fusion_grid.X=stereo_grid;
        clear stereo_grid;
        [total_grid, stereo_grid] = timeFusion(obj_fusion_grid, obj, ...
        timestamp, previous_timestamp, changes); 
    end
end

%% Update perception layer
if mod(t,1) == 0
    obj=Reference;
    obj.X=total_grid; clear total_grid;
    [total_grid] = perceptionGrid(obj, aging_lidar, aging_stereo, ...
        camera_timestamps(t), limits);
end