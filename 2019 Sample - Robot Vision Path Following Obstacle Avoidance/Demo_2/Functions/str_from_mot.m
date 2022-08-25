function [xyzPoints,errors, cameraPoses] = str_from_mot(images, rotations, locations, cameraParams)
    % Compute features for the first image.
    I = rgb2gray(images(:,:,:, 1));
    I = undistortImage(I,cameraParams);
    pointsPrev = detectSURFFeatures(I);
    [featuresPrev,pointsPrev] = extractFeatures(I,pointsPrev);
    
    %Create a viewSet object.
    vSet = viewSet;
    vSet = addView(vSet, 1,'Points',pointsPrev,'Orientation',...
                   rotations(:,:,1),'Location',locations(1,:));
    for i = 2:size(images, 4)
      I = rgb2gray(images(:,:,:, i));
      %I = undistortImage(I, cameraParams); Already done
      points = detectSURFFeatures(I);
      [features, points] = extractFeatures(I, points);
      vSet = addView(vSet,i,'Points',points,'Orientation',...
                     rotations(:,:,i),'Location',locations(i,:));
      pairsIdx = matchFeatures(featuresPrev,features,'MatchThreshold',5);
      vSet = addConnection(vSet,i-1,i,'Matches',pairsIdx);
      featuresPrev = features;
    end
    % Find point tracks.
    tracks = findTracks(vSet);
    % Get camera poses.
    cameraPoses = poses(vSet);
    %Find 3-D world points.
    [xyzPoints,errors] = triangulateMultiview(tracks,cameraPoses,cameraParams);
end