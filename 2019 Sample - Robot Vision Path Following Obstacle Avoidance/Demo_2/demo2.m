%% Trial code for part 2 of demo
%  Goals: Orient along arrow and travel to center

close all; clear all; clc;

test_case = 1; % 1: arrow detect, 2: yellow pillar detect

if test_case == 1
	load 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_backward_triangles.mat'
elseif test_case == 2
	load 'WorkspacesDemo2/WorkspacesDemo2_yellow.mat'
elseif test_case == 3
	load 'WorkspacesDemo2/WorkspacesDemo2_yellow4.mat'
end

img = imageCircBuff(:,:,:,5); % select channel that has least information!
img_hsv = rgb2hsv(img);
% img = img(1,:,:0;)


arrow_filt = [	0 0 1 0 0; 
				0 1 1 1 0; 
				0 0 0 0 0	];

% im_corr = xcorr2(img, arrow_filt);

% Analyze HSV image
figure
subplot(2,2,1); imshow(img)
subplot(2,2,2); imshow(img_hsv(:,:,1)) % HUE
subplot(2,2,3); imshow(img_hsv(:,:,2)) % SATURATION
subplot(2,2,4); imshow(img_hsv(:,:,3)) % INTENSITY

saturation_mask = img_hsv(:,:,2) > .3 & img_hsv(:,:,2) < .5;

[H,T,R] = hough(saturation_mask,'RhoResolution',0.1,'ThetaResolution',0.1);
numpeaks = 5; %Specify the number of peaks
P  = houghpeaks(H,numpeaks);
lines = houghlines(saturation_mask,T,R,P,'FillGap',2,'MinLength',5);

% Compute the length of each detected line
lineLength = [];
for i = 1:length(lines)
	lineLength(i) = norm(lines(i).point1 - lines(i).point2);
end
[~, idx] = sort(lineLength, 2, 'ascend');

figure
subplot(121); imshow(img)
title('Original Image')
subplot(122); imshow(saturation_mask)
title('Edges')
hold on

max_len = 0;
top_lines = 1; % plot x longest lines
for k = 1:length(lines) % uncomment to plot all lines
	% for k = 1:top_lines
	xy = [lines(k).point1; lines(k).point2]; % uncomment to plot all lines
	% xy = [lines(idx(k)).point1; lines(idx(k)).point2];
	plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');

	% Plot beginnings and ends of lines
	plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
	plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

	% Determine the endpoints of the longest line segment
	len = norm(lines(k).point1 - lines(k).point2);
	if ( len > max_len)
		max_len = len;
		xy_long = xy;
	end
end

% figure
% subplot(2,2,1); imshow(img)
% subplot(2,2,2); imshow(saturation_mask)
