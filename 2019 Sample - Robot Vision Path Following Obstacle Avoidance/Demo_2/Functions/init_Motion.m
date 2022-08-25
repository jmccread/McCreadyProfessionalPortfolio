%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [velocity_pub, velocity_msg] = init_Motion(TurtleBot_Topic)
    if ismember(TurtleBot_Topic.vel, rostopic('list'))
          velocity_pub = rospublisher(TurtleBot_Topic.vel, 'geometry_msgs/Twist');
    %     velocity_sub = rossubscriber('cmd_vel', 'geometry_msgs/Twist');
    end
    velocity_msg = rosmessage(velocity_pub);
end