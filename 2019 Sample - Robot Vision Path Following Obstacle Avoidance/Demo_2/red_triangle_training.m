%% Startup 
%close all
clear all
close all
addpath('Functions')
addpath('WorkspacesDemo2'); 
addpath('WorkspacesDemo1'); 

%% File 1 
filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_obstacle_forward_triangle.mat'; 
load(filename)
figure(99); 
R_rgb = []; 
nR_rgb = []; 
R_hsv = []; 
nR_hsv = []; 
[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,1));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,2));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,3));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,4));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,5));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 


save('file1.mat', 'R_rgb', 'nR_rgb', 'R_hsv', 'nR_hsv'); 

%% File 2
filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_backward_triangles.mat';
load(filename)
figure(99); 
R_rgb = []; 
nR_rgb = []; 
R_hsv = []; 
nR_hsv = []; 
[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,1));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,2));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,3));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,4));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,5));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 


save('file2.mat', 'R_rgb', 'nR_rgb', 'R_hsv', 'nR_hsv'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% File 3
filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_forward_triangle_facing_yellow.mat';
load(filename)
figure(99); 
R_rgb = []; 
nR_rgb = []; 
R_hsv = []; 
nR_hsv = []; 
[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,1));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,2));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,3));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,4));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,5));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 


save('file3.mat', 'R_rgb', 'nR_rgb', 'R_hsv', 'nR_hsv'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% File 4
filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_wall_2.mat';
load(filename)
figure(99); 
R_rgb = []; 
nR_rgb = []; 
R_hsv = []; 
nR_hsv = []; 
[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,1));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,2));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,3));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,4));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 

[R_temp_rgb, nR_temp_rgb, R_temp_hsv, nR_temp_hsv] = getTpnts(imageCircBuff(:,:,:,5));
R_rgb = cat(1, R_temp_rgb, R_rgb); 
nR_rgb = cat(1, nR_temp_rgb, nR_rgb); 
R_hsv = cat(1, R_temp_hsv, R_hsv); 
nR_hsv = cat(1, nR_temp_hsv, nR_hsv); 


save('file4.mat', 'R_rgb', 'nR_rgb', 'R_hsv', 'nR_hsv'); 

%% Helper function
function [redT, notRedT, redT_hsv, notRedT_hsv] = getTpnts(I)
    % expects RGB image as I 
    my_mask = zeros(size(I(:,:,1)));
    I_hsv = rgb2hsv(I);
    imshow(I)
    hold on

    [x, y] = getpts;
    if length(x) <= 2
        redT = [];
        notRedT = [];
        redT_hsv = [];
        notRedT_hsv = [];
        return 
    end
    t_bounds = [ceil(x), ceil(y)];
    perimeter = [];
    for k = 1:(length(t_bounds))
        if (k == length(t_bounds))
            point1 = [t_bounds(k,1), t_bounds(k,2)];
            point2 = [t_bounds(1,1), t_bounds(1,2)];
        else
            point1 = [t_bounds(k,1), t_bounds(k,2)];
            point2 = [t_bounds(k+1,1), t_bounds(k+1,2)];
        end
       pnts = make_line(point1, point2); 
       perimeter = cat(1, pnts, perimeter);
    end
    
    try
        inds = sub2ind(size(my_mask), perimeter(:,2), perimeter(:,1));
    catch
        redT = [];
        notRedT = [];
        redT_hsv = [];
        notRedT_hsv = [];
        disp('DO NOT PICK OUT OF BOUNDS PIXELS'); 
        return 
    end
    my_mask(inds) = true; 
    mask_pop_1 = imfill(my_mask,'holes');
    mask_pop_2 = ~mask_pop_1;
    
    im_pop_1 = uint8(mask_pop_1).*I;
    im_pop_2 = uint8(mask_pop_2).*I;
    
    im_pop_1_hsv = double(mask_pop_1).*(I_hsv+1); % clever
    im_pop_2_hsv = double(mask_pop_2).*(I_hsv+1);
    %imshowpair(im_pop_1, im_pop_2, 'montage');

    redT = reshape(im_pop_1, [], size(I, 3));
    ind = (redT(:,1) ~= 0)&(redT(:,2) ~= 0)&(redT(:,3) ~= 0);
    redT = redT(ind, :);
    notRedT = reshape(im_pop_2, [], size(I, 3));
    ind = (notRedT(:,1) ~= 0)&(notRedT(:,2) ~= 0)&(notRedT(:,3) ~= 0); 
    notRedT = notRedT(ind, :);
    
    redT_hsv = reshape(im_pop_1_hsv, [], size(I, 3));
    ind = (redT_hsv(:,1) ~= 0)&(redT_hsv(:,2) ~= 0)&(redT_hsv(:,3) ~= 0); 
    redT_hsv = redT_hsv(ind, :)-1;
    notRedT_hsv = reshape(im_pop_2_hsv, [], size(I, 3));
    ind = (notRedT_hsv(:,1) ~= 0)&(notRedT_hsv(:,2) ~= 0)&(notRedT_hsv(:,3) ~= 0);
    notRedT_hsv = notRedT_hsv(ind, :)+1;    
end

function [pnts] = make_line(point1, point2)
      % Find the slope for the most extreme points on the line 
      lineM = (point2(2) -  point1(2))./(point2(1) - point1(1));
      lineC =  (point2(2)) - lineM*(point2(1));
      start = min([point1(1), point2(1)]); 
      fin = max([point1(1), point2(1)]);
      x = start:fin; 
      y = lineM.*x+lineC; 
      plot(x,y,'b'); 
      pnts = [ceil(x)',ceil(y)'];
end
