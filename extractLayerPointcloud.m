%% extractLayerPointcloud function
% Extracts one layer of the pointcloud
function [ pointcloud ] = extractLayerPointcloud( scan, layer)
pointcloud = [];
size_scan = size(scan);
for i = 1:size_scan(2)
   y = scan(:,i);
   a = atan2d(y(3), y(1));
   b = atan2(y(3),y(1));
   if(a > (layer - 3) && a < (layer - 2) )
       pointcloud = [pointcloud scan(1:3,i)];
   end
end
end

