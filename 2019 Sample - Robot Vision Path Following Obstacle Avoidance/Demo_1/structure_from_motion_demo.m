clc; close all; clear all;
%% Demo of structure from motion

load Workspace_dashed_fast.mat
I = imageCircBuff(:,:,:,1);
I = rgb2gray(I);
I = undistortImage(I, cameraParams);

pointsPrev = detectSURFFeatures(I);
[featuresPrev,pointsPrev] = extractFeatures(I,pointsPrev);

vSet = viewSet;
vSet = addView(vSet, 1,'Points',pointsPrev,'Orientation',cameraPoses(:,:,1),'Location',locations(1,:));