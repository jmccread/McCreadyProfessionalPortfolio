%% read odometry
if ismember(TurtleBot_Topic.odom, rostopic('list'))
%     use rostopic info topicname to detemine the message type
    odom_sub = rossubscriber(TurtleBot_Topic.odom, 'nav_msgs/Odometry');
end
% odom_data = receive(odom_sub);

%%% read imu
if ismember(TurtleBot_Topic.imu, rostopic('list'))
    imu_sub = rossubscriber(TurtleBot_Topic.imu, 'sensor_msgs/Imu');
end
% imu_data = receive(imu_sub);

draw_odomData = animatedline('Color', 'c', 'LineWidth', 1.5, 'LineStyle', '-');
draw_imuData = animatedline('Color', 'm', 'LineWidth', 1.5, 'LineStyle', ':');

%% read images
% images captured by Pi camera, if you are using Gazebo, the topic list is different.
if ismember(TurtleBot_Topic.picam, rostopic('list'))
    image_sub = rossubscriber(TurtleBot_Topic.picam);
end

%% First capture image
image_compressed = receive(image_sub);
image_compressed.Format = 'bgr8; jpeg compressed bgr8';
C1 = readImage(image_compressed);
[C1,newOrigin] = undistortImage(C1,cameraParams);
figure
imshow(C1);
title('C1');

%% Do odometery
odom_data = receive(odom_sub);
imu_data = receive(imu_sub);
robot_Position1 = odom_data.Pose.Pose.Position;
robot_Orientation1 = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
robot_Rotation1 = quat2rotm(robot_Orientation1);
addpoints(draw_odomData, robot_Position1.X, robot_Position1.Y, robot_Position1.Z);
drawnow

%% Do motion 
if ismember(TurtleBot_Topic.vel, rostopic('list'))
    velocity_pub = rospublisher(TurtleBot_Topic.vel, 'geometry_msgs/Twist');
    %     velocity_sub = rossubscriber('cmd_vel', 'geometry_msgs/Twist');
end
velocity_msg = rosmessage(velocity_pub);

% Set some parameters that will be used in the processing loop.
% You can modify these values for different behavior.
spinVelocity = 1/32*pi;       % Angular velocity (rad/s)
forwardVelocity = 0.1;    % Linear velocity (m/s)
backwardVelocity = -0.02; % Linear velocity (reverse) (m/s)
distanceThreshold = 0.6;  % Distance threshold (m) for turning

% Run a loop to move the robot forward and compute the closest obstacles to
% the robot. When an obstacle is within the limits of the distanceThreshold,
% the robot turns. This loop stops after 20 seconds of run time.
% CTRL+C (or Control+C on the Mac) also stops this loop.
tic;
while toc < 1

    velocity_msg.Linear.X = forwardVelocity;
    velocity_msg.Angular.Z = spinVelocity; 
    send(velocity_pub, velocity_msg);
end
% let TurtleBot stop before disconnect from it
velocity_msg.Angular.Z = 0.0;
velocity_msg.Linear.X = 0.0;
send(velocity_pub, velocity_msg);
% 

%% First capture image
image_compressed = receive(image_sub);
image_compressed.Format = 'bgr8; jpeg compressed bgr8';
C2 = readImage(image_compressed); 
[C2,newOrigin] = undistortImage(C2,cameraParams);
figure
imshow(C2);
title('C2'); 

%% Do odometery
odom_data = receive(odom_sub);
imu_data = receive(imu_sub);
robot_Position2 = odom_data.Pose.Pose.Position;
robot_Orientation2 = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
robot_Rotation2 = quat2rotm(robot_Orientation2);
addpoints(draw_odomData, robot_Position2.X, robot_Position2.Y, robot_Position2.Z);
drawnow
save('workspace1');