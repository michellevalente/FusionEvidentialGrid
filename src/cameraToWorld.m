function [ points] = cameraToWorld( x, y, u, d, resolution, camera_params) 
%% Apply gausian observation model in the obstacles detected by the camera
    b = camera_params.b;
    f = camera_params.f;
    u0 = camera_params.cu;

    JG = [b/d  -b*(u-u0)/d^2; 0  -b*f/d^2];
    omega_d = 0.5;
    omega_u = 20.0;

    KU = [omega_u^2 0 ;  0 omega_d^2;];
    sigma = JG * KU * JG';

    mu = [x, y];
    xi = x-(resolution*2):resolution:x+(resolution*2); 
    yi = y-(resolution*3):resolution:y+(resolution*3);

    [X,Y] = meshgrid(xi,yi);
    W = mvnpdf([X(:) Y(:)],mu,sigma); 

    max_w = max(W(:,:,1));

    points = [];
    v = [X(:) Y(:)];
    for i = 1:size(v,1)
        if W(i)/max_w > 0.1
            points = [points; [v(i,1) v(i,2) W(i)/max_w]];
        end
    end
end