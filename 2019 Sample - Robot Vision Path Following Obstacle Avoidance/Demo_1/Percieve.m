classdef Percieve
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        odom_sub
        imu_sub
        image_sub
    end
    
    methods
        function obj = init_Perc(tb_topic)
        %UNTITLED3 Summary of this function goes here
        %   Detailed explanation goes here
        %read odometry
            if ismember(TurtleBot_Topic.odom, rostopic('list'))
            %     use rostopic info topicname to detemine the message type
                obj.odom_sub = rossubscriber(tb_topic.odom, 'nav_msgs/Odometry');
            else
                error('no odom member'); 
            end
        % read imu
            if ismember(TurtleBot_Topic.imu, rostopic('list'))
                obj.imu_sub = rossubscriber(TurtleBot_Topic.imu, 'sensor_msgs/Imu');
            else
                error('no imu member'); 
            end
        %read images
            % images captured by Pi camera, if you are using Gazebo, the topic list is different.
            if ismember(TurtleBot_Topic.picam, rostopic('list'))
                obj.image_sub = rossubscriber(TurtleBot_Topic.picam);
            else
                error('no picam member'); 
            end
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

