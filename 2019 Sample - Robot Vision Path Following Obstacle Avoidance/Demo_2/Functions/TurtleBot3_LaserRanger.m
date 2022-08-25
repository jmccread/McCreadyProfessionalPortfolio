function [scan_data, data_xy] = TurtleBot3_LaserRanger(laser_sub)
    scan_data = receive(laser_sub);
    data_xy = readCartesian(scan_data);
end
