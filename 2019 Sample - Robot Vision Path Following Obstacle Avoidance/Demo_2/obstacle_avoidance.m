clear all
close all
startup

addpath('Functions');
%try
load('cameraParams.mat')
% Call init functions and load function library 
[odom_sub, imu_sub, image_sub, laser_sub] = init_Perception(TurtleBot_Topic);
[velocity_pub, velocity_msg] = init_Motion(TurtleBot_Topic);

load('saved_scan_data.mat');
%141.215.223.203


point_angle = 0;
detected = 0;
waited = 0;
case_obs_detect = 50;
case_obs_wait = 51;
case_obs_avoid = 52;
case_obs_return = 53; 
go_forward = 59;
min_point_detected = 0;
state = go_forward;
extra_spin = 0.1;
total_distance = 0;
extra_distance = 1.5;
f1 = figure;
f2 = figure;

%% State machine
while 1
    C = TurtleBot3_Image(image_sub);
%     plot(x(120:240), y(120:240))
    hold off
    scan_data = receive(laser_sub);
    set(0, 'CurrentFigure', f2)
    plot(scan_data)
    set(0, 'CurrentFigure', f1)
    imshow(C);
    hold on
    switch state
        case go_forward
            velocity_msg.Angular.Z = 0;
            velocity_msg.Linear.X = .05;
            send(velocity_pub, velocity_msg);
            state = case_obs_detect;
        %% case_obs_detect
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
            elseif waited ~= 0
                state = go_forward;
                waited = 0;
            end

        case case_obs_wait
            pause(.2);
            waited = waited + 1;
            state = case_obs_detect;
        case case_obs_avoid
            [detections, min_point] = detect_obstacles(scan_data);
            move(velocity_msg, velocity_pub, -(min_point(3)+0.45), 0);
            if min_point(5) == 0
                point_angle = pi/8 + extra_spin;
            else
                point_angle = -pi/8 - extra_spin;
            end
            move(velocity_msg, velocity_pub, 0, -point_angle);
            total_distance = total_distance + min_point(1)*extra_distance;
            move(velocity_msg, velocity_pub, min_point(1)*extra_distance, 0);
            min_point_detect = min_point(1);
            state = case_obs_return;
        case case_obs_return
            pause(.1);
            move(velocity_msg, velocity_pub, 0, point_angle*2);
            [detections, min_point] = detect_obstacles(scan_data);
            if detections > 3
                state = case_obs_detect;
            else
                move(velocity_msg, velocity_pub, total_distance, 0);
                move(velocity_msg, velocity_pub, 0, -point_angle);
                point_angle = 0;
                total_distance = 0;
                min_point_detect = 0;
                state = go_forward;
            end
        otherwise
                rosshutdown   
                return
    end
end


function [detections, min_point] = detect_obstacles(filtered_scan_data)
    %x = zeros(1, 360);
    %y = zeros(1, 360);
    detections_left = 0;
    detections_right = 0;
    %detection_angles = zeros(1, 360);
    min_point_left = [100, 360, 100, 100, 2];
    min_point_right = [100, 360, 100, 100, 2];
    detections = 0;
    min_point = [100, 360, 100, 100, 2];
    for i=1:size(filtered_scan_data, 2)
        pause(.1)
        detected = 0;
        for jr=91:180
            xr = filtered_scan_data(i).Ranges(jr)*cos(jr*pi/180);
            yr = filtered_scan_data(i).Ranges(jr)*sin(jr*pi/180);
            %x(jr) = xr;
            %y(jr) = yr;
            if xr < 0 && abs(yr) < .07 && xr > -.45
                detected = 1;
                if jr > 148
                    xline((640*(211-jr)/(211-149)), 'LineWidth', 2, 'Color', 'g');
                    %detection_angles(jr) = jr;
                end
                set(gca,'Color','r')
                if min_point_right(1) > filtered_scan_data(i).Ranges(jr)
                    min_point_right = [filtered_scan_data(i).Ranges(jr), jr, xr, yr, 1];
                end
                detections_right = detections_right + 1;

            end
        end
        for jl=181:269
            xl = filtered_scan_data(i).Ranges(jl)*cos(jl*pi/180);
            yl = filtered_scan_data(i).Ranges(jl)*sin(jl*pi/180);
            %x(jl) = xl;
            %y(jl) = yl;
            if xl < 0 && abs(yl) < .16 && xl > -.45
                detected = 1;
                if jl < 212
                    xline((640*(211-jl)/(211-149)), 'LineWidth', 2, 'Color', 'g');
                    %detection_angles(jl) = jl;
                end
                set(gca,'Color','r')
                if min_point_left(1) > filtered_scan_data(i).Ranges(jl)
                    min_point_left = [filtered_scan_data(i).Ranges(jl), jl, xl, yl, 0];
                end
                detections_left = detections_left + 1;

            end
        end
        %all_detection_angles = detection_angles(detection_angles ~= 0);
        
        
        if detected == 0
            set(gca,'Color','w')
        else
            if detections_left > detections_right
                detections = detections_left;
                if detections_right == 0
                    min_point = min_point_left;
                else
                    min_point = min_point_right;
                end
            else
                detections = detections_right;
                if detections_left == 0
                    min_point = min_point_right;
                else
                    min_point = min_point_left;
                end
            end
        end
    end
    
end

function avoid_obstacles()
end

function return_from_avoid()
end




% 
% for i = 1:size(filtered_scan_data, 2)
%     hold on
%     pause(.1)
%     plot(filtered_scan_data(i))
% end

%tic;
%while toc < 40% Collect information from laser scan
    %scan_data = receive(laser_sub);
%     plot(scan_data);
%     data = readCartesian(scan_data);
%     x = data(:,1);
%     y = data(:,2);% Compute distance of the closest obstacle
%     dist = sqrt(x.^2 + y.^2);
%     minDist = min(dist);% Command robot action
%     if minDist < distanceThreshold% If close to obstacle, back up slightly and spin
%         velocity_msg.Angular.Z = spinVelocity;
%         velocity_msg.Linear.X = backwardVelocity;
%     else% Continue on forward path
%         velocity_msg.Linear.X = forwardVelocity;
%         velocity_msg.Angular.Z = 0;
%     end
%     %send(velocity_pub, velocity_msg);
% %end% let TurtleBot stop before disconnect from it
% velocity_msg.Angular.Z = 0.0;
% velocity_msg.Linear.X = 0.0;
%send(velocity_pub, velocity_msg);