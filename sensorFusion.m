function [fusion_grid, lidar_grid, stereo_grid] = sensorFusion( obj_lidar_grid, obj_stereo_grid, obj_fusion_grid, changes)

lidar_grid=obj_lidar_grid.X; obj_lidar_grid.X=[]; 
stereo_grid=obj_stereo_grid.X; obj_stereo_grid.X=[]; 
fusion_grid=obj_fusion_grid.X; obj_fusion_grid.X=[]; 
dempster_normalization = 1;

fusion_grid(:,:,1) = 0;
fusion_grid(:,:,2) = 0;
fusion_grid(:,:,3) = 0;
fusion_grid(:,:,4) = 1;

for i = 1:size(changes,1)
    col =  changes(i,1);
    row = changes(i,2);
    m1 = lidar_grid(col,row,:);
    m2 = stereo_grid(col,row,:);
    lidar_grid(col,row,:) = [0,0,0,1];
    stereo_grid(col,row,:) = [0,0,0,1];

    if (m1(4) ~= 1.0 && m2(4) ~= 1.0) || ~isequal(m1,m2)
        free = m1(4) * m2(1) + m1(1) * m2(1) + m1(1) * m2(4); 
        occ = m1(4) * m2(2) + m1(2) * m2(2) + m1(2) * m2(4); 
        conf = m1(3) * m2(1) + m1(3) * m2(2) + m1(3) * m2(3) + ...
            m1(3) * m2(4) + m2(3) * m1(1) + m2(3) * m1(2) + ...
            m2(3) * m1(3) + m1(1) * m2(2) + m1(2) * m2(1);
        unk = m1(4) * m2(4);
    else
        free = m1(1);
        occ = m1(2);
        conf = m1(3);
        unk = m1(4);
    end

    if dempster_normalization 
        free =  fix((free / (1 - conf)) * 100) / 100;
        occ = fix((occ / (1 - conf)) * 100) / 100;
        unk = fix((unk / (1 - conf)) * 100) / 100;
        conf = 0.0;
    end
    fusion_grid(col,row,:) = [free,occ,conf,unk];
end

end