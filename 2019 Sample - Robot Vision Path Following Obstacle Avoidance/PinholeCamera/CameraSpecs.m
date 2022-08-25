Num_rows = 480; Num_cols = 640; 
im_size = [Num_rows, Num_cols]; % Num_rows, Num_cols
pixel_side = 1.12*10^-6; % Pixel side length (square) m
f = 3.04*10^-3; % Focal length m;
horz_view_deg = 62.2;
vert_view_deg = 48.8; 
F_stop = 2; 
t = [0; 113*10^-3; 33*10^-3];
R = [-1 0 0; 
      0 -1 0;
      0  0 1];
x_o = size(try2vision(:,:,:,1),1)/2*pixel_side; 
y_o = size(try2vision(:,:,:,1),2)/2*pixel_side; 
  principal_point = [ x_o, y_o];
K = [f 0 principal_point(1);
     0 f principal_point(2);
     0 0 1];
 
 P = K*[R,t]
 

