function [ ] = showEvidentialMap( total_grid, path,id)

% [perception_grid, average_entropy, average_specificity] = perceptionGrid(map, previous_perception_grid, aging_lidar, aging_stereo, time);
% size_map = size(total_grid);
% out_total_grid = zeros(size_map(1), size_map(2),3);
% out_total_grid(:,:,:) = 0.75;
% dynamic = zeros(size_map(1), size_map(2));

% if mod(id,1) == 0
%     for i = 1:size_map(1)
%         for j = 1:size_map(2)
%             I = total_grid(i,j,5);
%             if I == 1
%                 out_total_grid(i,j,2) = 1;
%                 out_total_grid(i,j,1) = 0;
%                 out_total_grid(i,j,3) = 0;
%             elseif I == 2
%                 out_total_grid(i,j,1) = 0.75;
%                 out_total_grid(i,j,2) = 0.75;
%                 out_total_grid(i,j,3) = 0.75;
%             elseif I == 3
%                 out_total_grid(i,j,1) = 0.0;
%                 out_total_grid(i,j,2) = 0.4;
%                 out_total_grid(i,j,3) = 0.8;
%             elseif I == 4
%                 out_total_grid(i,j,1) = 1;
%                 out_total_grid(i,j,2) = 0;
%                 out_total_grid(i,j,3) = 0;
%             end
%         end
%     end
    

%     for i = 1:size(path,1)
%         out_total_grid(path(i,1),path(i,2),1) = 1.0;
%         out_total_grid(path(i,1),path(i,2),2) = 1.0;
%     end
    % 
% figure('name','Total evidential grid');
    figure(2);
    subplot(1,2,2);
    imagesc((total_grid(:,:,5)));
%     imshow(flipud(out_total_grid));
    pause(0.01)
%     if id == 227
%        imwrite(flipud(out_total_grid),['map_227_error_stereo_lidar.png']); 
%     end
%     imwrite(flipud(out_total_grid),['map' num2str(id) '.png']);
% end

end

