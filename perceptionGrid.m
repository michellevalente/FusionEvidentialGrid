function [ grid, average_entropy, average_specificity] = perceptionGrid( obj, aging_lidar, aging_stereo,time,limits, step)
% Perception : 1- free, 2-occ, 3-conf, 4-unk
% % 1 - free/currently_free , 2 - free/unk
% % 3 - occ/fixed , 4 - occ/currently
% % 0 - unk
grid=obj.X; obj.X=[];
calculate_entropy = 0;
size_grid_i = size(grid,1);
size_grid_j = size(grid,2);
clustering = 0;

% current_state_grid = zeros(size_grid_i, size_grid_j,2);
% current_state_grid(:,:,1) = 4;
% perception_grid = zeros(size_grid_i, size_grid_j,2);
% perception_grid(:,:,1) = 5;
entropy_grid = zeros(size_grid_i, size_grid_j,1);
specificity_grid = zeros(size_grid_i, size_grid_j,1);
thr_occ = 8;

if clustering == 1
    grid(:,:,7) = 1;
    [grid, clusters] = segmentGrid(grid);
    dynamic_cells = zeros(size(clusters,1));
end
% previous_state = grid(:,:,5);
% grid(:,:,5) = 4;

new_states = 1;
i_min = limits(3);
i_max = limits(4);
j_min = limits(1);
j_max = limits(2);

for j = j_min:j_max
    for i = i_min:i_max
        if grid(i,j,4) ~= 1.0
            if new_states == 0
                [~,I] = max([grid(i,j,1),grid(i,j,2),grid(i,j,3),grid(i,j,4)]);
                grid(i,j,5) = I;
            else
                last_time_lidar = aging_lidar(i,j);
                last_time_stereo = aging_stereo(i,j);
                if last_time_lidar > last_time_stereo
                    last_time = last_time_lidar;
                else
                    last_time = last_time_stereo;
                end
                if (time - last_time)/100000 < 7
                    if grid(i,j,4) == 1.0
                        if grid(i,j,5) == 1
                            if (time - last_time)/100000 > 4
                                grid(i,j,5) = 2;
                            else
                                grid(i,j,5) = 1;
                            end
                        elseif grid(i,j,5) == 4
                            grid(i,j,5) = 0;
                        end
                    else
                        [~,I] = max([grid(i,j,1),grid(i,j,2),grid(i,j,3),grid(i,j,4)]);
                        %                         previous_state = grid(i,j,5);
                        %                         if grid(i,j,5) == 0
                        if I == 1
                            if (time - last_time)/100000 > 4 
                                grid(i,j,5) = 2;
                            else
                                grid(i,j,5) = 1;
                            end
                            grid(i,j,6) = 0;
                        elseif I == 2
                            if (time - last_time)/100000 < 4
                                %                             if move > 7.0
                                %                                 grid(i,j,6) = grid(i,j,6) + 2;
                                %                             elseif move > 4 && move <= 6

                                %                             end
                                if grid(i,j,6) > thr_occ
                                    grid(i,j,5) = 3;
                                else
                                    grid(i,j,5) = 4;
                                    grid(i,j,6) = grid(i,j,6) + 1;
                                    if clustering == 1
                                        if grid(i,j,7) > 0.0
                                            dynamic_cells(grid(i,j,7)) = dynamic_cells(grid(i,j,7)) + 1;
                                            if (dynamic_cells(grid(i,j,7)) / size(clusters{grid(i,j,7)},1)) > 0.5  && step > 5
                                                cells = clusters{grid(i,j,7)};
                                                for c = 1:size(cells)
                                                    grid(cells(c,1),cells(c,2),5) = 4;

                                                end
                                            end
                                        end
                                    end
                                end
                            else
                                if grid(i,j,5) == 4
                                    grid(i,j,5) = 0;
                                end
                            end

                        elseif I == 3
                            grid(i,j,5) = 4;
                            if clustering == 1
                                dynamic_cells(grid(i,j,7)) = dynamic_cells(grid(i,j,7)) + 1;
                                if grid(i,j,7) > 0.0 && step > 5
                                    if (dynamic_cells(grid(i,j,7)) / size(clusters{grid(i,j,7)},1)) > 0.5 && step > 5
                                        cells = clusters{grid(i,j,7)};
                                        for c = 1:size(cells)
                                            grid(cells(c,1),cells(c,2),5) = 4;
                                        end
                                    end
                                end
                            end
                        else
                            if grid(i,j,5) == 1
                                if (time - last_time)/100000 > 4
                                    grid(i,j,5) = 2;
                                else
                                    grid(i,j,5) = 1;
                                end
                            elseif grid(i,j,5) == 4
                                grid(i,j,5) = 0;
                            end
                        end
                    end
                else
                    if grid(i,j,5) == 4
                        grid(i,j,5) = 0 ;
                    end
                    %                         if previous_state(i,j) ~= 4
                    %                             grid(i,j,5) = previous_state(i,j);
                    %                         else
                    %                             grid(i,j,5) = 0;
                    %                         end
                end
                %             else
                %                 grid(i,j,5) = 0;
                %             end
            end
        end
    end
end

if calculate_entropy == 1
    average_entropy = mean2(entropy_grid);
    average_specificity = mean2(specificity_grid);
else
    average_entropy = 0;
    average_specificity = 0;
end
end

