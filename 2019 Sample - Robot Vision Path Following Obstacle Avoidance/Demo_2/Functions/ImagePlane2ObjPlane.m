%% ImagePlane2ObjPlane: Maps a line in the image plane to the objext plane; mapping from 2D to 3D
%     Inputs: line, f
%     line, Hough Line 1 in image plane
%	  f, focal length of camera
%	  h, ground plane of camera
%     Outputs: lines
%     (x, y, x), coordinates in the object plane (3D)

%% Notes on the Hough transform:
%	y = mx + c 							Cartesian Coordinates
%	x*cos(theta) + y*sin(theta) = rho	Hough Transform / Polar Representation
%	m = -cos(theta)/sin(theta)
%	c = rho/sin(theta)
% 	We need to code the above relationship!


%% Notes on 3D to 2D projection of pinhole camera
%	z = h/yi * f, h = ground plane
%	x = -xi/f * z
%	We need to code this mapping!

function [x, y, z] = ImagePlane2ObjPlane(line, f, h)
	% Extract all information fron Hough line
	theta = line.theta
	rho = line.rho
	point1 = line.point1
	point2 = line.point2

	m = -cos(deg2rad(theta)) / sin(deg2rad(theta))
	c = rho / sin(deg2rad(theta))

	%% Let's verify this is correct...
	yt = m*point2(1) + c
	if yt == point2(2)
		display('It works')
	else
		display('I think you did the calculation wrong')
		error = yt - point2(2)

	y = h;
	z = h/point1(2);
	x = -point1(1)/f*z;

	% x=1;y=1;z=1; % placeholder for now
end