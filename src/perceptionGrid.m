function [ grid] = perceptionGrid( obj, aging_lidar, aging_stereo,time,limits)
%% Life-long grid states:
% 1 - free/currently free , 2 - free/currently unk
% 3 - occ/currently occ , 4 - occ/fixed
% 0 - unk

%% Get object grid
grid=obj.X; obj.X=[];

%% Parameters
thr_occ = 8;
timeout = 10;

%% Define region of interest to not go through the entire map
i_min = limits(1);
i_max = limits(2);
j_min = limits(3);
j_max = limits(4);

%% Loop in region of interest
for j = j_min:j_max
    for i = i_min:i_max
        %% Update life-long grid states
        age_lidar = aging_lidar(i,j);
        age_stereo = aging_stereo(i,j);
        if age_lidar > age_stereo
           last_age = age_lidar;
        else
            last_age = age_stereo;
        end
        if last_age == time
            [~,max_value] = max([grid(i,j,1),grid(i,j,2),grid(i,j,3),grid(i,j,4)]);
            if max_value == 1
               grid(i,j,5) = 1;
            elseif max_value == 2
               grid(i,j,6) = grid(i,j,6) + 1;
               if grid(i,j,6) > thr_occ
                   grid(i,j,5) = 3;
               else
                   grid(i,j,5) = 4;
               end
            else
                if grid(i,j,5) == 4
                    grid(i,j,5) = 0;
                end
            end
        else
            if time - last_age > timeout
                if grid(i,j,5) == 1 
                    grid(i,j,5) = 2;
                elseif grid(i,j,5) == 4
                    grid(i,j,5) = 0;
                end
            end
        end
    end
end
end

