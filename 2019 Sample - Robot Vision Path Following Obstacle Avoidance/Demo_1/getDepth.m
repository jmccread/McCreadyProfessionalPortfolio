function [ depthMap ] = getDepth( I1, I2 )
%getDepth This function takes two images and outputs a depth map for them
%   I1: image 1
%   I2: image 2

depthMap = disparity(rgb2gray(I1), rgb2gray(I2));
figure; imshow(depthMap);
title('Disparity Map');
colormap(gca,jet) 
colorbar

end

