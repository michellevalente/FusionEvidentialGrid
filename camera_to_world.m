function [ points] = camera_to_world( x, y, u, d, resolution, max_distance,...
                                    camera_params)
    b = camera_params.b;
    f = camera_params.f;
    u0 = camera_params.cu;
    
    distance = sqrt(x^2 + y^2);
    if distance > max_distance
       points = [];
       return
    end

    JG = [b/d  -b*(u-u0)/d^2; 0  -b*f/d^2];
    omega_d = 0.5;
    omega_u = 10.0;

    KU = [omega_u^2 0 ;  0 omega_d^2;];
    sigma = JG * KU * JG';

    mu = [x, y];
    xi = x-0.8:resolution:x+0.8; 
    yi = y-0.8:resolution:y+0.8;

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