function [robot_Position, robot_Rotation] = TurtleBot3_Odom(odom_sub, imu_sub)
    odom_data = receive(odom_sub);
    imu_data = receive(imu_sub);
    robot_Position = [odom_data.Pose.Pose.Position.X, odom_data.Pose.Pose.Position.Y, odom_data.Pose.Pose.Position.Z];
    robot_Orientation1 = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
    robot_Rotation = quat2rotm(robot_Orientation1);
end