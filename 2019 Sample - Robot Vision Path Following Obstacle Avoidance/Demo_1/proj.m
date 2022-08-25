% close all; clear all; clc;
% 
% % try2vision contains 10 images, select one using try2vision(:,:,:,i)
% load 'images/images.mat'
% 
% % Images from professor
% im1 = imread('images/Ex1.png');
% im2 = imread('images/Ex2.jpg');


%% Select an image from try2vision
figure(2)
for i=1:4
    subplot(2,2,i); imshow(imageCircBuff(:,:,:,i))
end

[P, K, R, t] = cameraCalibration();
% load cameraParams.mat

for i = 1:5
    im = imageCircBuff(:,:,:,i);
    imbw = im2single(rgb2gray(im));
    imbw = im2bw(imbw, 0.5);
    figure(1); 
    subplot(1,2,1); imshow(imbw);
    subplot(1,2,2); histogram(imbw); % 20 bins

    % Notes:    -90 deg -> horizontal, robot is looking perpendicular to line
    %           0 deg -> vertical, robot is facing along line
    extracted_lines = findEdges(imbw, 1);

    % Try to move from 2D to 3D
    ex = extracted_lines(1);
    f = 3.04*10^-3; % Focal length m;
    h = 1; % ground plane

    WorldPoints = ImagePlane2ObjPlane(ex, cameraParameters, R, t)
end
% close all; % use this to close all figures for now