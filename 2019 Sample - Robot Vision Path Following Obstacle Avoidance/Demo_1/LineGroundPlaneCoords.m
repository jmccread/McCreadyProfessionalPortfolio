%% LineGroundPlaneCoords: Maps a line in the image plane to the objext plane; mapping from 2D to 3D
%     Inputs: line, f
%     line, Hough Line 1 in image plane
%	  f, 1x2 matrix of the cameras x and y focal lengths
%	  h, ground plane of camera
%     Outputs: coords - spatial coordinates are in millimeters
%     coords, (x, z) coordinates in the ground plane (3D) along with theta

%% Notes on the Hough transform:
%	y = mx + c 							Cartesian Coordinates
%	x*cos(theta) + y*sin(theta) = rho	Hough Transform / Polar Representation
%	m = -cos(theta)/sin(theta)
%	c = rho/sin(theta)
% 	We need to code the above relationship!


%% Notes on 3D to 2D projection of pinhole camera
%	z = -h/yi * f, h = ground plane
%	x = -xi/f * z
%	We need to code this mapping!

function [coords] = LineGroundPlaneCoords(line, f, h)
    % Define constants
    px_size = 1.12e-3;
    xc = 240;
    yc = 320;
    
	% Extract all information fron Hough line
	theta = line.theta;
	rho = line.rho;
	point1 = line.point1 ; % units in mm
    xi = (point1(1) - xc) * px_size;
    yi = (point1(2) - yc) * px_size; % correcting the index
    
% 	point2 = line.point2 * 0.0057;

	m = -cos(deg2rad(theta)) / sin(deg2rad(theta));
	c = rho / sin(deg2rad(theta));

    z = -h/yi*f; 
	x = -xi/f*z;
    
	coords = [x z theta];
end