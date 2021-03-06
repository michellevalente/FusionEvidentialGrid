function [grid_evidential, aging_lidar, changes, limits] = lidarMapping(...
    pointcloud, grid_parameters, time, pose, obj_grid_evidential, ...
    obj_aging_lidar)
    
    %% Read from object
    aging_lidar=obj_aging_lidar.X; obj_aging_lidar.X=[]; 
    grid_evidential=obj_grid_evidential.X; obj_grid_evidential.X=[]; 

    %% Transform scan to INS frame
    position_world = [pose(1,4), pose(2,4)];
    position = round(position_world/grid_parameters.resolution + grid_parameters.origin);
   
    %% Limits current map
    map_range = round(80 / grid_parameters.resolution);
    
    limit_x_min = position(1) - map_range;
    if limit_x_min < 1
      limit_x_min = 1;
    end
    limit_x_max = position(1) + map_range;
    if limit_x_max > grid_parameters.size_grid_x
      limit_x_max = grid_parameters.size_grid_x;
    end

    limit_y_min = position(2) - map_range ;
    if limit_y_min < 1
      limit_y_min = 1;
    end
    limit_y_max = position(2) + map_range ;
    if limit_y_max > grid_parameters.size_grid_y
      limit_y_max = grid_parameters.size_grid_y;
    end
    limits = [limit_x_min,limit_x_max, limit_y_min, limit_y_max ];
  
    %% Create evidential grid map
    confidence_lidar_occ = 0.8;  
    confidence_lidar_free = 0.6;  
    pointcloud = sortrows(pointcloud', 1)';
    size_pc = size(pointcloud);
    changes = zeros(1000,2);
    count_changes = 1;

    for i = 1:size_pc(2)

      %% Define occupied space
      p = pointcloud(:,i);
      row = ceil((p(1))/grid_parameters.resolution + ...
          grid_parameters.origin(1));
      col = ceil((p(2))/grid_parameters.resolution + ...
          grid_parameters.origin(2));
      [free_x,free_y] = bresenham(row,col,position(1),position(2));

      if(row >= limits(1) && row <= limits(2) && col >= limits(3) && col <= limits(4) && ...
              grid_evidential(row,col,2) ~= confidence_lidar_occ)
        changes(count_changes,:) = [row,col];
        count_changes = count_changes + 1;
        grid_evidential(row,col,1) = 0.0;
        grid_evidential(row,col,2) = confidence_lidar_occ;
        grid_evidential(row,col,3) = 0.0;
        grid_evidential(row,col,4) = 1 - confidence_lidar_occ;
        aging_lidar(row,col,1) = time;
      end

      %% Define free space
      if ~isempty(free_x)
          for j = size(free_x):-1:1
              row = free_x(j);
              col = free_y(j);

              if(row >= limits(1) && row <= limits(2) && col >= limits(3) && col <= limits(4) )
                if col > grid_parameters.size_grid_y
                    col = grid_parameters.size_grid_y ;
                end
                if grid_evidential(row,col,2) == confidence_lidar_occ
                    break;
                else
                    if grid_evidential(row,col,1) ~= confidence_lidar_free
                       grid_evidential(row,col,1) = confidence_lidar_free;
                       grid_evidential(row,col,2) = 0.0;
                       grid_evidential(row,col,3) = 0.0;
                       grid_evidential(row,col,4) = 1 - confidence_lidar_free;
                       aging_lidar(row,col,1) = time;
                       changes(count_changes,:) = [row,col];
                       count_changes = count_changes + 1;
                    end
                end
              end
          end
      end
    end
end
