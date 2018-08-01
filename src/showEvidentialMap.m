function showEvidentialMap(total_grid)
    %% Plot life-long grid
    % Black: unknown space
    % Green: free space - currently free
    % Gray: free space - currently unknown
    % Red: occupied space - currently occupied
    % Blue: occupied space - fixed occupied
    
    figure(2);
    subplot(1,2,2);
    map = [[0 0 0]; 
           [0 1 0]; 
           [0.75 0.75 0.75];
           [0 0.4 0.8];
           [1 0 0]];
    image = mat2gray(total_grid(:,:,5));
    imshow(flipud(image));
    colormap(map);
    
end

