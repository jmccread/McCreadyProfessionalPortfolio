function [odom_sub, imu_sub, image_sub, laser_sub] = init_Perception(TurtleBot_Topic)
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
    
    if ismember(TurtleBot_Topic.laser, rostopic('list'))
        laser_sub = rossubscriber('/scan');
    else
        error('no laser member'); 
    end
end