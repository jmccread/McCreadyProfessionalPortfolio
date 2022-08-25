
%%% read lidar data
if ismember(TurtleBot_Topic.laser, rostopic('list'))
    laser_sub = rossubscriber(TurtleBot_Topic.laser);
end

if ismember(TurtleBot_Topic.vel, rostopic('list'))
    velocity_pub = rospublisher(TurtleBot_Topic.vel, 'geometry_msgs/Twist');
    %     velocity_sub = rossubscriber('cmd_vel', 'geometry_msgs/Twist');
end
velocity_msg = rosmessage(velocity_pub);

% Set some parameters that will be used in the processing loop.
% You can modify these values for different behavior.
spinVelocity = -1/32*pi;       % Angular velocity (rad/s)
forwardVelocity = 0.1;    % Linear velocity (m/s)
backwardVelocity = -0.02; % Linear velocity (reverse) (m/s)
distanceThreshold = 0.6;  % Distance threshold (m) for turning

% Run a loop to move the robot forward and compute the closest obstacles to
% the robot. When an obstacle is within the limits of the distanceThreshold,
% the robot turns. This loop stops after 20 seconds of run time.
% CTRL+C (or Control+C on the Mac) also stops this loop.
tic;
while toc < 10
% 
%     velocity_msg.Linear.X = 0;%forwardVelocity;
%     velocity_msg.Angular.Z = spinVelocity; 
%     send(velocity_pub, velocity_msg);
    
    scan_data = receive(laser_sub);
    plot(scan_data);
end
% let TurtleBot stop before disconnect from it
velocity_msg.Angular.Z = 0.0;
velocity_msg.Linear.X = 0.0;
send(velocity_pub, velocity_msg);
% 
clear
rosshutdown