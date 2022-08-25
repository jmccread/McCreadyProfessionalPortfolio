% Find 3-D World Points Across Multiple Images Using Triangulation
%open('workspace1.mat'); 
% Compute features for the first image.
I = rgb2gray(C1);
%I = undistortImage(I,cameraParams); % done already
pointsPrev = detectSURFFeatures(I);
[featuresPrev,pointsPrev] = extractFeatures(I,pointsPrev);

%Create a viewSet object.
vSet = viewSet;
loc1 = [robot_Position1.X, robot_Position1.Y, robot_Position1.Z]; 
vSet = addView(vSet, 1,'Points',pointsPrev,'Orientation',...
    robot_Rotation1,'Location',loc1);

% Do it again

% Compute features for the first image.
J = rgb2gray(C2);
%I = undistortImage(I,cameraParams); % done already
points = detectSURFFeatures(J);
[features, points] = extractFeatures(J, points);
loc2 = [robot_Position2.X, robot_Position2.Y, robot_Position2.Z]; 
vSet = addView(vSet,2,'Points',points,'Orientation',...
      robot_Rotation2,'Location',loc2);

  % identify image invariants
 pairsIdx = matchFeatures(featuresPrev,features,'MatchThreshold',5);
 vSet = addConnection(vSet,1,2,'Matches',pairsIdx);
 %featuresPrev = features;
 
 % Find point tracks 
 tracks = findTracks(vSet);
 
 % Get camera poses
 cameraPoses = poses(vSet);
 
 % Find 3-D world points.
 figure
 [xyzPoints,errors] = triangulateMultiview(tracks,cameraPoses,cameraParams);
 z = xyzPoints(:,3);
 idx = errors < 5 & z > 0 & z < 20;
 pcshow(xyzPoints(idx, :),'VerticalAxis','y','VerticalAxisDir','down','MarkerSize',30);
 hold on
 plotCamera(cameraPoses, 'Size', 0.1);
 hold off