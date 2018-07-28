function [ pointcloud ] = extractLayerPointcloud( scan, layer)

%
% height -  -> -1 ; -1 -> 0 ; 0 -> 1 ; 1->2
%%
pointcloud = [];
size_scan = size(scan);
angles = [];
for i = 1:size_scan(2)
   y = scan(:,i);
   a = atan2d(y(3), y(1));
   b = atan2(y(3),y(1));
%    a = round(atan2(y(3), y(1)),2);
   if(a > (layer - 3) && a < (layer - 2) )
       pointcloud = [pointcloud scan(1:3,i)];
   end
   angles(i) = b;
end
end

