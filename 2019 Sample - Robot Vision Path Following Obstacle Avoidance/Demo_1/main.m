clear all
close all
startup
%try
    load('cameraParams.mat')
    perc = Percieve;
    % Call init functions and load function library 
    [odom_sub, imu_sub, image_sub] = init_Perception(TurtleBot_Topic);
    [velocity_pub, velocity_msg] = init_Motion(TurtleBot_Topic);

    %% Create data structures 
    buffSize = 12;
    posCircBuff = nan(buffSize,1);
    slopeCircBuff = nan(buffSize,1);
    thetaCircBuff = nan(buffSize,1);    
    errCircBuff = nan(buffSize,1);

    imageCircBuff = nan(cameraParams.ImageSize(1), cameraParams.ImageSize(2),3, buffSize);
    rotCircBuff = nan(3,3,buffSize); 

    %% Flag definition
    %% Flag definition
    case_spin = 1; % line not found, spin 360; 
    case_move_spin = 2;
    case_go2line = 3; 
    case_spin_on_line = 4;
    case_walk_line = 5;
    case_stop = 6;
    state = 1;
    line = []; 
    

    
%% Begin demo     
k_rots = 0; 
delta_rot = pi/32; 
num_rots = abs(2*pi/delta_rot); 
target_abs_theta_d = 90;
single_error_flag = 0; 
target_err = 4; %Error to horizontal line 
err_debounce = 0.5; % Half a degree of error growth tolerated 
d_err_bound = 5; % Error should not change more than 5 degrees in a single step
case_go2_line_flag1 = 1; 
initial_depth_line = []; 
rho = 1; 
first_run_flag_garbage = 0;
walk_err_count = 0;
err_count = 0;
turn_flag = 0;

 while(1)
    C = TurtleBot3_Image(image_sub);
    % Get odometry data
    %[robot_Position, robot_Rotation] = TurtleBot3_Odom(odom_sub, imu_sub);    
    %imageCircBuff = nan(cameraParams.ImageSize(1), cameraParams.ImageSize(2),3, buffSize);
    imshow(C);
    hold on
    % Define case_spin
    
    %% State machine
    switch state
    %% case_spin
       case case_spin
            % rotate until k_rots == num_rots
                tic
        first_run_flag_garbage
        if first_run_flag_garbage
              if (0 ~= errCircBuff(1)) && (errCircBuff(1) < 10 ) 
                [line, lineM, lineC]= getLine(C,0, 0.6, 0); % Bright room
                %[line, lineM, lineC]= getLine(C,0, 0.55, 0); % darker room
              else 
                  [line, lineM, lineC]= getLine(C,1, 0.6, 0); % Bright room       
                  %[line, lineM, lineC]= getLine(C,1, 0.55, 0); % darker room
              end 
        else
            [line, lineM, lineC]= getLine(C,1, 0.60,0); % Bright room
            %[line, lineM, lineC]= getLine(C,1, 0.55,0); % darker room
            %first_run_flag_garbage = 1;
        end
                toc
               
                if (isempty(line)) % if no line found move on
                    move(velocity_msg, velocity_pub, 0, delta_rot)
                    if k_rots == num_rots % if we went round decide where to go
                        state = case_move_spin;
                    else
                        k_rots = k_rots+1
                    end 
                else

                   first_run_flag_garbage = 1; % move this here so that we always get a line for error purposes
                   disp(line.theta)
                   slopeCircBuff = cat(1, lineM, slopeCircBuff(1:end-1,:)); 
                   thetaCircBuff = cat(1, line.theta, thetaCircBuff(1:end-1,:));
                   
                   abs_error  = abs(target_abs_theta_d - abs(line.theta)); 
                   errCircBuff = cat(1, abs_error, errCircBuff(1:end-1,:));
                  
                   
                   % Decide if I am going the right direction based on
                   % if theta is going towards or away from 90. 
                   if ~isnan(errCircBuff(1:2))
                       d_err = errCircBuff(1) - errCircBuff(2);  
                       
                        % Do a check to see if err is going crazy if so
                        % make line detection less sensitive
                       if d_err > d_err_bound
                           first_run_flag_garbage = 0;    
                       end
                       if (d_err_bound> d_err) &&(d_err > err_debounce) % error is growing - bad                         
                         if  single_error_flag == 1 % two time there have been increasing errors 
                             delta_rot = - delta_rot;
                             single_error_flag = 0; %%#ok<NASGU>
                         else
                             single_error_flag = 1;
                         end 
                            
                       else % error is shrinking - good
                            delta_rot = delta_rot; % do nothing 
                            single_error_flag = 0;
                       end                            
                   end                    
                   move(velocity_msg, velocity_pub, 0, delta_rot)
                   k_rots = k_rots+1                
                   % Act on data 
                   if k_rots == num_rots % if we went round decide where to go
                        if (0 ~= abs_error) && (abs_error < target_err ) % error thresh is 5 degrees
                             state = case_go2line;
                        else  % try to move somewhere else
                            state = case_move_spin;
                        end

                   else % just check if we met our target 
                       if (0 ~= abs_error) && (abs_error < target_err ) % error thresh is 5 degrees
                            state = case_go2line;
                       end 
                   end
                    
                end
             if k_rots == num_rots % if we went round decide where to go
                 state = case_move_spin;
             end
        %% case_move_spin
        case case_move_spin
            %We need to decide where we want it to move after we don't see
            %a line from a full circle view
            move(velocity_msg, velocity_pub, .5, pi); %turn 90 degrees from start position and move forward half a meter
            k_rots = 0; %reset k_rots
            %delta_rot = - delta_rot;
            first_run_flag_garbage = 0; 
            single_error_flag = 0;
            slopeCircBuff = nan(buffSize,1);
            thetaCircBuff = nan(buffSize,1);    
            errCircBuff = nan(buffSize,1);
            state = case_spin; %search for lines again
         
        %% case_go2line            
        case case_go2line
            tic
                [line, lineM, lineC]= getLine(C,0, 0.6,0); % bright room
                %[line, lineM, lineC]= getLine(C,1, 0.55,0); % darker room
            toc
            if ~isempty(line) && case_go2_line_flag1 % set initial depth
                case_go2_line_flag1 = 0; 
                initial_depth_line = line; 
                rho = abs(line.rho)/abs(initial_depth_line.rho);
                
            elseif ~isempty(line) && ~case_go2_line_flag1 % calc rho
                rho = abs(line.rho)/abs(initial_depth_line.rho);
            end
            
            if  ~isempty(line) % need valid line to move
                if abs(line.rho) > 300
                    move(velocity_msg, velocity_pub, .5, 0);
                    state = case_spin_on_line;
                %elseif (rho <= .3) && ~(abs(line.rho) > 400)%Distance to line is 30cm or less away so stop after move
                %    state = case_stop;
                
                elseif (rho >1.3 ) % Gone to far
                    move(velocity_msg, velocity_pub, -.2, 0); 
                else % see line go to it 
                    move(velocity_msg, velocity_pub, .05, 0);
                end  
            else
                state = case_move_spin;
            end
        %% case_spin_on_line    
        case case_spin_on_line
            move(velocity_msg, velocity_pub, 0, pi/2); %We made it to the line, so spin the robot 90 degrees
            state = case_walk_line; %once rotated, walk the line
            errCircBuff = nan(buffSize,1);
            
        
        %% case_walk_line
        case case_walk_line
            tic
                [line, lineM, lineC]= getLine(C,0,.60,1); % bright 
            toc
            turn_distance = 0;           
            if(~isempty(line))
                theta_corrected = abs(line.theta)-17; 
                vert_err = 90 - theta_corrected % left is positive error, right is positive
                
                if vert_err > 7 && ~turn_flag
                    if vert_err ~= 107
                        turn_distance = -pi/16;
                    else
                        err_count = err_count + 1;
                    end
                elseif vert_err < -7 && ~turn_flag
                    turn_distance = pi/16;
                else
                    turn_flag = 1;
                    turn_distance = 0;
                end
            end
 %           move(velocity_msg, velocity_pub, 0.05, turn_distance);   %Need to figure out rho for input
            if vert_err > 50 || vert_err < -13
                move(velocity_msg, velocity_pub, 0.05, turn_distance);
            else
                move(velocity_msg, velocity_pub, 0.05, turn_distance);
                move(velocity_msg, velocity_pub, 0, -turn_distance);
            end
%             if err_count == 3
%                 state = case_stop;
%             end
        %% case_stop
        case case_stop
            move(velocity_msg, velocity_pub, 0, 0); %ensure the robot is stopped
            state = 99; %Shutdown the ros instance
        
        %% default state - shutdown for now
        otherwise
            rosshutdown   
            return
    end 
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [odom_sub, imu_sub, image_sub] = init_Perception(TurtleBot_Topic)
%init_Perception - starts up tasks for subscibers in ROS 
    if ismember(TurtleBot_Topic.odom, rostopic('list'))
    %     use rostopic info topicname to detemine the message type
        odom_sub = rossubscriber(TurtleBot_Topic.odom, 'nav_msgs/Odometry');
    else
        error('no odom member'); 
    end
% read imu
    if ismember(TurtleBot_Topic.imu, rostopic('list'))
        imu_sub = rossubscriber(TurtleBot_Topic.imu, 'sensor_msgs/Imu');
    else
        error('no imu member'); 
    end
%read images
    % images captured by Pi camera, if you are using Gazebo, the topic list is different.
    if ismember(TurtleBot_Topic.picam, rostopic('list'))
        image_sub = rossubscriber(TurtleBot_Topic.picam);
    else
        error('no picam member'); 
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [velocity_pub, velocity_msg] = init_Motion(TurtleBot_Topic)
    if ismember(TurtleBot_Topic.vel, rostopic('list'))
          velocity_pub = rospublisher(TurtleBot_Topic.vel, 'geometry_msgs/Twist');
    %     velocity_sub = rossubscriber('cmd_vel', 'geometry_msgs/Twist');
    end
    velocity_msg = rosmessage(velocity_pub);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [C] = TurtleBot3_Image(image_sub)
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    C = readImage(image_compressed); 
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [robot_Position, robot_Rotation] = TurtleBot3_Odom(odom_sub, imu_sub)
    odom_data = receive(odom_sub);
    imu_data = receive(imu_sub);
    robot_Position = [odom_data.Pose.Pose.Position.X, odom_data.Pose.Pose.Position.Y, odom_data.Pose.Pose.Position.Z];
    robot_Orientation1 = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
    robot_Rotation = quat2rotm(robot_Orientation1);
end

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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [C_fore_mask,rng] = foreground_mask(I)
%% Foreground detection function, detects the foreground based on the
% largest blob appearing in the image as found by bwboundaries with
% noholes.
% Input:
%       C               1 channel grayscale image
% Outputs: 
%       C_fore_mask     image mask with foreground as 1's
    BW = imbinarize(I); % Threshold image according to otsu's method
    [B, ~] = bwboundaries(BW, 'noholes'); 
    [~, index] = maxk(cellfun('length', B),3);
    C_fore_mask = zeros(size(BW),'uint8'); 
    if max(max(B{index(1)}(:,1))) == size(BW, 1)
        rng = [min(B{index(1)}(:,1)), max(B{index(1)}(:,1)); min(B{index(1)}(:,2)),max(B{index(1)}(:,2))];
        
    elseif max(max(B{index(2)}(:,1))) == size(BW, 1)
        rng = [min(B{index(2)}(:,1)), max(B{index(2)}(:,1)); min(B{index(2)}(:,2)),max(B{index(2)}(:,2))];
        
    elseif max(max(B{index(3)}(:,1))) == size(BW, 1)
        rng = [min(B{index(3)}(:,1)), max(B{index(3)}(:,1)); min(B{index(3)}(:,2)),max(B{index(3)}(:,2))];
        
    else
        rng = [215 480; 1 640];
    end
    C_fore_mask((rng(1,1)+10):rng(1,2), rng(2,1):rng(2,2)) = 1; 
    C_fore_mask = logical(C_fore_mask);
%     figure;
%     imshow(logical(C_fore_mask));
end


function [lines, lineM, lineC] = getLine(C, enableLineCheck, Adapt_sense, doVert)
%% Do preprocessing/noise reduction
    %subplot(2,2,m);
    %C = histeq(C);
    I = rgb2gray(C);
    I = histeq(I);
    [fore_mask, rng] = foreground_mask(I); 
%     if sum(sum(fore_mask)) < 
%     
%     end
    J = histeq(I);
    J = J.*uint8(fore_mask);
    J = histeq(J);
% Second round of reduction
    %imshowpair(I,J,'montage')
    BW = imbinarize(J, .85); 
    specular_mask = imgaussfilt(uint8(BW), 5);
    M = J.*specular_mask;
    %imshowpair(J,M,'montage')

    N = histeq(M);%.*specular_mask.*fore_mask;
    %imshowpair(M,N,'montage')

% Third round    
    %BW = imbinarize(N, [0.9, 0.97]); 
    sumB4 = sum(sum(N)); 
    sumNow =  sum(sum(N)); 
    dec = 0;
    BWadapt = imbinarize(N, 'adaptive','Sensitivity',Adapt_sense+dec);
    Q = N.*uint8(BWadapt);
    target =  sum(sum(N.*Q))/sumB4/25;
    P = N;
    while (sumNow/sumB4 > target) && (dec> -Adapt_sense)
        BWadapt = imbinarize(N, 'adaptive','Sensitivity',Adapt_sense+dec);
        dec = dec - 0.01;
        %BWadapt =imgaussfilt(uint8(BWadapt), 5);
        P = N.*uint8(BWadapt);
        sumNow = sum(sum(P));
        %P = histeq(P);
        %BW = imbinarize(P, .9); 
        %imshow(BWadapt)
        %imshow(P)
    end
   
 %% Detected boundaries for reduced image with only specular objects   
 BW_filt = logical(uint8(medfilt2(BWadapt)).*uint8(specular_mask).*uint8(fore_mask));   
 %imshow(BW_filt)
 [B,L] = bwboundaries(BW_filt,'noholes');
    %imshow(B)
    %title('Labeled Image')
    %imshow(label2rgb(L, @jet, [.5 .5 .5]))
    
% Turn boundaries into masks and calc selection metrics
    masks = nan(size(BW,1),size(BW,2), length(B)); 
    edges = nan(size(BW,1),size(BW,2), length(B));
    sums = nan(length(B),1);
    size_B = cellfun('length',B);
    num_elts = 5;%ceil(length(B)*1/3) % hopefully reduce comp time. 
    [bigBs, idx_B] = maxk(size_B, num_elts);
    B = B(idx_B); 
    if length(B) == 1
        my_mask = zeros(size(BW));
        inds = sub2ind(size(my_mask), B{1}(:,1), B{1}(:,2));
        my_mask(inds) = 1; 
        edges =cat(3, my_mask, edges(:,:, 1:end-1));
        BW = imfill(my_mask,'holes');
        masks = my_mask; 
        sums = sum(sum(BW));

    else 
        for k = 1:length(B)
            my_mask = zeros(size(BW));
            inds = sub2ind(size(my_mask), B{k}(:,1), B{k}(:,2));
            my_mask(inds) = 1; 
            edges =cat(3, my_mask, edges(:,:, 1:end-1));
            BW = imfill(my_mask,'holes');
            masks =cat(3, BW, masks(:,:, 1:end-1));
            sums =cat(1, sum(sum(BW)), sums(1:end-1));
        end
    end
    
%% Select "best" mask by size - future by size and degree of specularness
    [sorted_sums, I_sum] = sort(sums); 
    
    % Just choose the biggest
    [~, candidate_1] = max(sums); 
%     figure(1);
%     subplot(1,1,1)
%     imshow(edges(:,:,candidate_1));
%% Detect lines from calcuated boundaries (no canny needed) 
    [lines, all_lines] = findEdgesThick(edges(:,:,candidate_1),enableLineCheck, 1);
       
    if ~isempty(lines) 
        if enableLineCheck
            lines = checkLines(lines);
        end
    end
    lineM = []; 
    lineC = []; 
    if ~isempty(lines)
        if ~doVert
            [long_line, lineM, lineC]  = elongateLines(lines, rng, 1);
            lines = long_line;
        else 
            [long_line, lineM, lineC]  = elongateLinesVert(lines, rng, 1);
            lines = long_line;
        end
    end
end


function [good_lines] = checkLines(lines)
bad_lines = zeros(size(lines)); % bad line means 1   
    for k = 1:length(lines)
        % bottom horz case 
        A =(lines(k).point1(2) == 480 || lines(k).point2(2) == 480);
        % top horz case
        B =(lines(k).point1(2) == 1 || lines(k).point2(2) == 1);
        % left vert case
        C =(lines(k).point1(1) == 1 || lines(k).point2(1) == 1);
        % right vert case
        D =(lines(k).point1(1) == 640 || lines(k).point2(1) == 640);
        % left vert case
        %E =(lines(k).point1(1) == 0 || lines(k).point2(1) == 0);
    
        bad_lines(k) = (A||B||C||D);
    end 
 good_lines = lines(~bad_lines); 
end 

function [long_line, lineM, lineC] = elongateLinesVert(lines, rng, verbose)
     try 
        % Compute the length of each detected line
      lines_tb = struct2table(lines);
      lineLength = zeros(length(lines),1); 
      points_1 = lines_tb.point1;
      points_2 = lines_tb.point2;
      % Find most extreme line points 
      [~, min_p1_index] = min(points_1(:,2)); 
      [~, max_p2_index] = max(points_2(:,2)); 
      
      point1 = lines(min_p1_index).point1; % bottom most point detected
      point2 = lines(max_p2_index).point2; % top most point detected 
      
      % Find the slope for the most extreme points on the line 
      lineM = ((point2(2) -  point1(2))./(point2(1) - point1(1)))^-1; % inverse of before
      lineC =  (point2(1)) - lineM*(point2(2));
      
%       x = 1:640;
%       y = lineM.*x+lineC; 
%       y = y(y>215);  % y(y>rng(1,1))
%       x = ceil((y - lineC)./lineM);
%       lineTheta = -atan2(1, lineM);
%       lineRho = lineC * sin(lineTheta);
%       if isnan(sum(sum(x)))
%         x = 1:640;
%         y = lineM.*x+lineC; 
%         %return;
%       end 

      y = 1:480;
      x = lineM.*y+lineC; 

      lineTheta = -atan2(1, lineM);
      lineRho = lineC * sin(lineTheta);
%       if isnan(sum(sum(x)))
%         x = 1:640;
%         y = lineM.*x+lineC; 
%         %return;
%       end 
      for k = 1:length(lines)
        lineLength(k) = norm(lines_tb.point1(k,:) -  lines_tb.point2(k,:));
      end
       longest_lines = lines(lineLength == max(lineLength));
       longest_line = longest_lines(1);
    
    field1 = 'point1';  value1 = [0,0];
    field2 = 'point2';  value2 = [0, 0];
    field3 = 'theta';  value3 = 0;
    field4 = 'rho';  value4 = 0;
    long_line = struct(field1,value1,field2,value2,field3,value3,field4,value4);
    
    long_line.point1 = [x(1), y(1)];
    long_line.point2 = [x(end), y(end)];           
    long_line.theta =rad2deg(lineTheta); 
    long_line.rho = lineRho; 
    
    if (verbose == 1) 
        hold on; 
        xy = [long_line.point1; long_line.point2];
        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');  
        plot(x,y,'LineWidth',2,'Color','cyan');
        
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');
            % Plot beginnings and ends of lines
            plot(xy(1,1),xy(1,2),'x','LineWidth',1,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',1,'Color','red');
        end
    end
     catch
        long_line = []; 
        lineM = NaN;  
        lineC = NaN;
     end 
end

function [long_line, lineM, lineC] = elongateLines(lines, rng, verbose)
     try 
        % Compute the length of each detected line
      lines_tb = struct2table(lines);
      lineLength = zeros(length(lines),1); 
      points_1 = lines_tb.point1;
      points_2 = lines_tb.point2;
      % Find most extreme line points 
      [~, min_p1_index] = min(points_1(:,1));
      [~, max_p2_index] = max(points_2(:,1));
      
      point1 = lines(min_p1_index).point1; % left most point detected
      point2 = lines(max_p2_index).point2; % right most point detected 
      
      % Find the slope for the most extreme points on the line 
      lineM = (point2(2) -  point1(2))./(point2(1) - point1(1));
      lineC =  (point2(2)) - lineM*(point2(1));
      
      x = 1:640;
      y = lineM.*x+lineC; 
      y = y(y>215);  % y(y>rng(1,1))
      x = ceil((y - lineC)./lineM);
      lineTheta = -atan2(1, lineM);
      lineRho = lineC * sin(lineTheta);
      if isnan(sum(sum(x)))
        x = 1:640;
        y = lineM.*x+lineC; 
        %return;
      end 
      for k = 1:length(lines)
        lineLength(k) = norm(lines_tb.point1(k,:) -  lines_tb.point2(k,:));
      end
       longest_lines = lines(lineLength == max(lineLength));
       longest_line = longest_lines(1);
    
    field1 = 'point1';  value1 = [0,0];
    field2 = 'point2';  value2 = [0, 0];
    field3 = 'theta';  value3 = 0;
    field4 = 'rho';  value4 = 0;
    long_line = struct(field1,value1,field2,value2,field3,value3,field4,value4);
    
    long_line.point1 = [x(1), y(1)];
    long_line.point2 = [x(end), y(end)];           
    long_line.theta =rad2deg(lineTheta); 
    long_line.rho = lineRho; 
    
    if (verbose == 1) 
        hold on; 
        xy = [long_line.point1; long_line.point2];
        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');  
        plot(x,y,'LineWidth',2,'Color','cyan');
        
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');
            % Plot beginnings and ends of lines
            plot(xy(1,1),xy(1,2),'x','LineWidth',1,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',1,'Color','red');
        end
    end
     catch
        long_line = []; 
        lineM = NaN;  
        lineC = NaN;
     end 
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function move(velocity_msg, velocity_pub, rho, theta)
% Theta becomes rad/s 
    
    spinVelocity = pi/4;       % Angular velocity (rad/s)
    forwardVelocity = .1;    % Linear velocity (m/s)
    backwardVelocity = -.1;    % Linear velocity (m/s)
    %spinTime = 0.5; %theta/spinVelocity;
    
    if abs(theta) > 0 
        spinTime = abs(theta)/spinVelocity;
        velocity_msg.Angular.Z = sign(theta)*spinVelocity;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
        pause(spinTime); 
        
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
    end 
    
    if rho > 0
 
        forwardTime = rho/forwardVelocity;
       
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = forwardVelocity;
        send(velocity_pub, velocity_msg);
        pause(forwardTime); 
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
        
    elseif rho < 0
        forwardTime = abs(rho)/backwardVelocity;
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = backwardVelocity;
        send(velocity_pub, velocity_msg);
        pause(forwardTime); 
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
    else
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
    end
end