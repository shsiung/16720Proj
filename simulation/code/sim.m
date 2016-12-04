
function [xi, T] = GaussianNewtonOptimize(I_ref, D_ref, I, cam_intrinsics)
    if ndims(I) == 3
        I_ref = rgb2gray(I_ref);
        I = rgb2gray(I);
    end
    
    % undistort the shit
    [f, fy, cx, cy, d0, d1, d2, d3, d4] = cam_intrinsics{:};
    I_ref_origin = I_ref;
    I_origin = I;
    D_ref_origin = D_ref;
    D_ref = undistortimage(D_ref_origin, f, cx, cy, d0, d1, d2, d3, d4);
    I_ref = undistortimage(I_ref_origin, f, cx, cy, d0, d1, d2, d3, d4);
    I =  undistortimage(I_origin, f, cx, cy, d0, d1, d2, d3, d4);
    
   
    max_iter = 100;
    thres = 1e-4;
    iter = 0;
    xi = zeros(1, 6);
    l = loss(I_ref, D_ref, I, xi);
    while iter < max_iter || l < thres
        jacobian = jacob(I_ref, D_ref, I, xi);
        delta_xi =  - inv(jacobian'*jacobian) * jacobian' * l;
        
        delta_T = expMapSE3(delta_xi);
        T = expMapSE3(xi);
        T = delta_T * T;
        xi = logSE3(T);
    end
    
end

function xi = logSE3(T)
    R = T(1:3,1:3);
    Vu = T(1:3, 4);
    theta = arccos( (trace(R) - 1) / 2.0 );
    w = theta/( 2* sin(theta) ) * (R - R');

    A = sin(theta) / theta;
    B = (1-cos(theta)) / theta^2;
    C = (1-A) / theta^2;
    V = eye(3) + B * wx + C * wx2;    
    
    u = inv(V) * Vu;
    
    xi = [u; w]';    
end

function l = loss(I_ref, D_ref, I, xi)
    [w, h] = size(I);
    px = repmat( [1:w] - int32(w/2), [h, 1]);
    py = repmat( [1:h]' - int32(h/2), [1, w]);
    p = [ px(:);
          py(:)];
    d = D_ref(1:w*h);
    T = expMapSE3(xi);
    w = warp(p, d, T);
    I_interp = interp2(I, w(1,:), w(2, :));
    E_xi =  I_ref(p) - I(w);
    l = norm(E_xi);
    l = l*l;
end

function jacobian = jacob(I_ref, D_ref, I, xi)
    [h, w] = size(I);

    
    jacobian = zeros(1, 6);
    
    % small pertubance of u and w in each dimension
    delta_u = 0.1;
    delta_w = 0.1;
    
    delta1 = [delta_u, 0, 0, 0, 0, 0];
    delta2 = [0, delta_u, 0, 0, 0, 0];
    delta3 = [0, 0, delta_u, 0, 0, 0];
    delta4 = [0, 0, 0, delta_w, 0, 0];
    delta5 = [0, 0, 0, 0, delta_w, 0];
    delta6 = [0, 0, 0, 0, 0, delta_w];
    delta = [delta1; delta2, delta3; delta4; detla5; delta6];
   
    px = repmat( [1:w] - int32(w/2), [h, 1]);
    py = repmat( [1:h]' - int32(h/2), [1, w]);
    p = [ px(:);
          py(:)];
    d = D_ref(1:w*h);
    for i = 1 : 6
        delta_xi = delta(i, :);
        xi_plus = expMapSE3(delta_xi);
        xi_minus = expMapSE3(-delta_xi);
        T = expMapSE3(xi);
        T_plus = xi_plus * T;
        T_minus= xi_minus* T;
        
        w_plus = warp(p, d, T_plus);
        w_minus= warp(p, d, T_minus);
        
        I_plus = interp2(I, w_plus(1,:), w_plus(2, :));
        I_minus= interp2(I, w_minus(1,:),w_minus(2,:));
        
        E_xi =  I_minus(w_minus) - I(w_plus);
        jacobian(i) = norm(E_xi);
        jacobian(i) = jacobian(i)^2;
    end
       
end

function w = warp(p, d, T)
    P = [p(1, :)./ d; p(2, :)./ d; 1 ./ d];
    A = [ T(1, :) ./ T(3, :);
          T(2, :) ./ T(3, :)];
    w = A*P;
end
function T = expMapSE3(xi)
    %
    assert(length(xi) == 6);
    x = xi(1);
    y = xi(2);
    z = xi(3);
    w1 = xi(4);
    w2 = xi(5);
    w3 = xi(6);
    u = [x, y, z]';
    w = [w1, w2, w3];
    wx = [  0, -w3,  w2;
           w3,   0, -w1;
          -w2,  w1,  0];
    wx2 = [ - w2^2 - w3^2,         w1*w2,        w1*w3;
                   w1*w2, - w1^2 - w3^2,         w2*w3;
                   w1*w3,         w2*w3, - w1^2 - w2^2];
    theta = sqrt(w1^2 + w2^2 + w3^2);
    
    if abs(theta) > 1e-20 % when theta is not zero
        A = sin(theta) / theta;
        B = (1-cos(theta)) / theta^2;
        C = (1-A) / theta^2;
        
        R = eye(3) + A * wx + C * wx2;
        V = eye(3) + B * wx + C * wx2;
        
        T = [        R, V*u;
             zeros(1,3), 1 ];
    else % when theta is zero
        T = [zeros(3,3), u;
             zeros(1,3), 1];
    end
end

function nim = undistortimage(im, f, ppx, ppy, k1, k2, k3, p1, p2)
    
    % Strategy: Generate a grid of coordinate values corresponding to an ideal
    % undistorted image.  We then apply the imaging process to these
    % coordinates, including lens distortion, to obtain the actual distorted
    % image locations.  In this process these distorted image coordinates end up
    % being stored in a matrix that is indexed via the original ideal,
    % undistorted coords.  Thus for every undistorted pixel location we can
    % determine the location in the distorted image that we should map the grey
    % value from.

    % Start off generating a grid of ideal values in the undistorted image.
    [rows,cols,chan] = size(im);        
    [xu,yu] = meshgrid(1:cols, 1:rows);
    
    % Convert grid values to normalised values with the origin at the principal
    % point.  Dividing pixel coordinates by the focal length (defined in pixels)
    % gives us normalised coords corresponding to z = 1
    x = (xu-ppx)/f;
    y = (yu-ppy)/f;    

    % Radial lens distortion component
    r2 = x.^2+y.^2;                    % Squared normalized radius.
    dr = k1*r2 + k2*r2.^2 + k3*r2.^3;  % Distortion scaling factor.
    
    % Tangential distortion component (Beware of different p1,p2
    % orderings used in the literature)
    dtx =    2*p1*x.*y      +  p2*(r2 + 2*x.^2);
    dty = p1*(r2 + 2*y.^2)  +    2*p2*x.*y;    
    
    % Apply the radial and tangential distortion components to x and y
    x = x + dr.*x + dtx;
    y = y + dr.*y + dty;
    
    % Now rescale by f and add the principal point back to get distorted x
    % and y coordinates
    xd = x*f + ppx;
    yd = y*f + ppy;
    
    % Interpolate values from distorted image to their ideal locations
    if ndims(im) == 2   % Greyscale
        nim = interp2(xu,yu,double(im),xd,yd); 
    else % Colour
        nim = zeros(size(im));
        for n = 1:chan
            nim(:,:,n) = interp2(xu,yu,double(im(:,:,n)),xd,yd); 
        end
    end

    if isa(im, 'uint8')      % Cast back to uint8 if needed
        nim = uint8(nim);
    end
    
end