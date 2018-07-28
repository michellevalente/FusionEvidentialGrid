function U = processImage(image_left, image_right)
    %% variables
    scale = 0.5;
    show_all_images = 0;
    var_ground = -30;
    
    image_left = im2double((image_left));
    image_right = im2double((image_right));
    image_left = imresize(((image_left)),scale);
    image_right = imresize(((image_right)),scale);
    % image_left = adapthisteq(image_left,'clipLimit',0.02,'Distribution','rayleigh');
    % image_right = adapthisteq(image_right,'clipLimit',0.02,'Distribution','rayleigh');

    %% Disparity

    max_d = 32;
    min_d = 0;
    blocks = 15;

    disparityRange = [min_d max_d];
    image_left = rgb2gray(image_left);
    image_right = rgb2gray(image_right);
    
%     disp = disparityMap(image_left, image_right, 2, 0, 16);
    
    I = disparity((image_left),(image_right),'Method', 'BlockMatching', 'BlockSize',...
        blocks,'DisparityRange',disparityRange, 'UniquenessThreshold', 0, ...
        'ContrastThreshold', 0.5);
    
    
%     I = imresize(I, scale);
    [m,n] = size(I);
    start_j = 1;
    end_j = n ;
    start_i = 300 * scale;
    end_i = 750 * scale;
%     
    I(I == -1) = 0;
    I = uint8(I);
    I(I > max_d) = max_d;

    [m,n] = size(I);
    U = zeros(max_d,n);
    
    

    if show_all_images == 1
        figure 
        imshow(I,[0 32]);
        title('Disparity Map');
        colormap(gca,jet) 
        colorbar
    end

    %% v-disparity
    V = zeros(m,max_d);
    for i = start_i:end_i
        for j = start_j:end_j
            d = I(i,j);
         
            if d ~= 0
                V(i,d) = V(i,d) + 1;
            end
        end
    end

    Vmean = uint8(mean(V(V~=0))) + 5;
    ZV = uint8(V);
    ZV(V > Vmean) = 255;
    ZV(V <= Vmean) = 0;
    if show_all_images == 1
        figure('Name','v-disparity');
        imshow(ZV);
    end

    %% Ground computation
    G = zeros(1,max_d);
    for i = 1:max_d
        [Gx,~] = size(find(ZV(:,i)>0));
        if Gx > 0
            if find(ZV(:,i)>0, 1, 'last' ) > m/2
                G(i) = find(ZV(:,i)>0, 1, 'last' );
            else
                G(i) = 0;
            end
        else
            G(i) = 0;
        end
    end
    Ground = zeros(m,max_d);
    Groundpoint = [0 0];
    Ground(:,:) = 255;
    for i = 1:max_d
        if G(i) > 0
            Ground(G(i),i) = 0;
            Groundpoint = [Groundpoint;i,G(i)];
        end
    end

    Groundpoint = Groundpoint(2:end,:);
    try
        [Groundline] = robustfit(Groundpoint(:,1),Groundpoint(:,2));
    catch
        return;
    end

    for i = 1:max_d
        linep(i) = Groundline(1) + Groundline(2)*i;
    end

    %% Seperate Ground and Obstacle
    I_ground = zeros(m,n);
    I_obstacle = zeros(m,n);

    for i = start_i:end_i
        for j = start_j:end_j
            d = I(i,j);
            if d ~= 0
                if (i - linep(d)) < var_ground
                    I_obstacle(i,j) = I(i,j);
                else
                    I_ground(i,j) = I(i,j);
                end
            end
        end
    end

    if show_all_images == 1
        figure;
        imagesc(uint8(I_ground));
        colormap('default');axis image;
        figure('Name', 'I_obstacle');
        imagesc(uint8(I_obstacle));
        colormap('default');axis image;
    end

    %% u-disparity
    for i = start_i:end_i
        for j = start_j:end_j
            d = I_obstacle(i,j);
            if d >= 5 && d <= 25
                U(d,j) = U(d,j) + 1;
            end
        end
    end

    Z = uint8(U);
    Z(U > 5) = 255;
    Z(U <= 5) = 0;

    if show_all_images == 1
        figure('Name','u-disparity');
        imshow(Z)
    end
%     imshow(U);
end

