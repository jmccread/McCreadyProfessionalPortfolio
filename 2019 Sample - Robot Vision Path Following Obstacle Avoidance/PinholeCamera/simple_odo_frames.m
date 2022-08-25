%% Idiot checking 

%%% read odometry
if ismember(TurtleBot_Topic.odom, rostopic('list'))
%     use rostopic info topicname to detemine the message type
    odom_sub = rossubscriber(TurtleBot_Topic.odom, 'nav_msgs/Odometry');
end
% odom_data = receive(odom_sub);

if ismember(TurtleBot_Topic.vel, rostopic('list'))
    velocity_pub = rospublisher(TurtleBot_Topic.vel, 'geometry_msgs/Twist');
    %     velocity_sub = rossubscriber('cmd_vel', 'geometry_msgs/Twist');
end
velocity_msg = rosmessage(velocity_pub);

%%% read imu
if ismember(TurtleBot_Topic.imu, rostopic('list'))
    imu_sub = rossubscriber(TurtleBot_Topic.imu, 'sensor_msgs/Imu');
end
% imu_data = receive(imu_sub);

%% Setup

draw_odomData = animatedline('Color', 'c', 'LineWidth', 1.5, 'LineStyle', '-');
draw_imuData = animatedline('Color', 'm', 'LineWidth', 1.5, 'LineStyle', ':');
tic;

%% Do loop

odom_data = receive(odom_sub);
imu_data = receive(imu_sub);
robot_Position = odom_data.Pose.Pose.Position;
robot_Orientation = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
robot_Rotation = quat2rotm(robot_Orientation);
addpoints(draw_odomData, robot_Position.X, robot_Position.Y, robot_Position.Z);
drawnow