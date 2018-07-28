function [pointcloud] = readScan(laser_dir, laser_timestamp_camera, layer)
        %% Read scan file
        scan_path = [laser_dir num2str(laser_timestamp_camera) '.bin'];
        if ~exist(scan_path, 'file')
            error('No laser scan found ');
        end
        
        scan_file = fopen(scan_path);
        scan = fread(scan_file, 'double');
        fclose(scan_file);
        
        scan = reshape(scan, [3 numel(scan)/3]);      
        %% Extract one layer of the LIDAR pointcloud
        pointcloud = [];
        pointcloud = extractLayerPointcloud(scan,layer);
end

