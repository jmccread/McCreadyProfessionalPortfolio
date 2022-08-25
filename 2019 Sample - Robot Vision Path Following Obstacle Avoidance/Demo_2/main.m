clear all
close all
startup

addpath('Functions');
load('lda_rgb.mat'); 
%try
    load('cameraParams.mat')
    % Call init functions and load function library 
    [odom_sub, imu_sub, image_sub, laser_sub] = init_Perception(TurtleBot_Topic);
    [velocity_pub, velocity_msg] = init_Motion(TurtleBot_Topic);

    %% Create data structures 
    buffSize = 12;
    posCircBuff = nan(buffSize,3);
    slopeCircBuff = nan(buffSize,1);
    thetaCircBuff = nan(buffSize,1);    
    errCircBuff = nan(buffSize,1);
    laserCircBuff = cell(1,buffSize);


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
    case_turn_around = 7;
    state = 5;
    line = []; 
    
    
    % Obs Avoidance initialized variables
    point_angle = 0;
    detected = 0;
    waited = 0;
    case_obs_detect = 50;
    case_obs_wait = 51;
    case_obs_avoid = 52;
    case_obs_return = 53; 
    %go_forward = 59;
    min_point_detected = 0;
    %state = go_forward;
    extra_spin = 0.1;
    total_distance = 0;
    extra_distance = 1.5;
    f1 = figure;
    f2 = figure;
    

    
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
backwards_dir_count = 0; 

 while(1)
 %% Collect Telemetry 
    % Get image data
     C = TurtleBot3_Image(image_sub);
     imageCircBuff =cat(4, C, imageCircBuff(:,:,:, 1:end-1));

    % Get odometry data
    [robot_Position, robot_Rotation] = TurtleBot3_Odom(odom_sub, imu_sub); 
    posCircBuff = cat(1, robot_Position, posCircBuff(1:end-1,:));
    rotCircBuff = cat(3, robot_Rotation,rotCircBuff(:,:, 1:end-1));

    % Get laser data 
    [scan_data, data_xy] = TurtleBot3_LaserRanger(laser_sub); 
    if ~isempty(data_xy)
        c_data_xy = mat2cell(data_xy, size(data_xy,1), size(data_xy,2)); 
    else
        c_data_xy = {nan};
    end 
    laserCircBuff = cat(2, c_data_xy, laserCircBuff(:,1:end-1));

    imshow(C);
    hold on
    % Define case_spin
    
    
    % May need this if above is not working for scan_data
    %scan_data = receive(laser_sub);
    
    
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
            backwards_dir_count = 0; 

        %% case_walk_line
        case case_walk_line
            tic
            [line, lineM, lineC]= getLine(C,0,.60,1); % bright 
            toc
            [triangle_dir] = getTriangle(C, W, thresh_LD2, 1);
            triangle_dir
            if (triangle_dir == 0)
                backwards_dir_count = backwards_dir_count +1;  
                
                if backwards_dir_count == 2
                    state = case_turn_around; 
                    backwards_dir_count = 0; 
                end
            else
       
                turn_distance = 0;           
                if(~isempty(line))
                    theta_corrected = abs(line.theta)-17; 
                    vert_err = 90 - theta_corrected; % left is positive error, right is positive

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
        %           if err_count == 3
        %               state = case_stop;
        %           end
            state = case_obs_detect;
            end
            
        %case_obs_detect
        case case_obs_detect
            [detections, min_point] = detect_obstacles(scan_data);
            if detections > 3 
                velocity_msg.Angular.Z = 0.0;
                velocity_msg.Linear.X = 0.0;
                send(velocity_pub, velocity_msg);
                if waited == 4
                    waited = 0;
                    state = case_obs_avoid;
                else
                    state = case_obs_wait;
                end
            else%if waited ~= 0
                state = case_walk_line;
                waited = 0;
            end
        %%case_obs_wait
        case case_obs_wait
            pause(.2);
            waited = waited + 1;
            state = case_obs_detect;
        %% case_obs_avoid
        case case_obs_avoid
            [detections, min_point] = detect_obstacles(scan_data);
            pause(.1);
            move(velocity_msg, velocity_pub, -(min_point(3)+0.45), 0);
            if min_point(5) == 0
                point_angle = pi/8 + extra_spin;
            else
                point_angle = -pi/8 - extra_spin;
            end
            move(velocity_msg, velocity_pub, 0, -point_angle);
            total_distance = total_distance + 0.45*extra_distance;
            move(velocity_msg, velocity_pub, 0.45*extra_distance, 0);
            min_point_detect = min_point(1);
            state = case_obs_return;
        %% case_obs_return
        case case_obs_return
            pause(.1);
            move(velocity_msg, velocity_pub, 0, point_angle*2);
            pause(.2);
            [detections, min_point] = detect_obstacles(scan_data);
            if detections > 3
                state = case_obs_detect;
            else
                move(velocity_msg, velocity_pub, total_distance, 0);
                move(velocity_msg, velocity_pub, 0, -point_angle);
                point_angle = 0;
                total_distance = 0;
                min_point_detect = 0;
                state = case_walk_line;
            end
        %% case_stop
        case case_stop
            move(velocity_msg, velocity_pub, 0, 0); %ensure the robot is stopped
            state = 99; %Shutdown the ros instance
        
        case case_turn_around
            move(velocity_msg, velocity_pub, 0, 0); %ensure the robot is stopped
            move(velocity_msg, velocity_pub, 0, pi); %We made it to the line, so spin the robot 90 degrees
            state = case_walk_line; %once rotated, walk the line
        %% default state - shutdown for now
        otherwise
            rosshutdown   
            return
    end 
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End main
















