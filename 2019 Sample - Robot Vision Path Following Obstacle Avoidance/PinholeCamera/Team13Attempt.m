%%% read images
% images captured by Pi camera, if you are using Gazebo, the topic list is different.
if ismember(TurtleBot_Topic.picam, rostopic('list'))
    image_sub = rossubscriber(TurtleBot_Topic.picam);
end
% 
% % First image
% image_compressed = receive(image_sub);
% image_compressed.Format = 'bgr8; jpeg compressed bgr8';
% C = readImage(image_compressed); 
% figure
% imshow(C1);

tic
figure; 
%while toc < 20
for i = 1:20
    % Collect images 
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    C = readImage(image_compressed); 
    imwrite(C, sprintf('CamCal%i.jpg', i), 'jpg');
    % Display image capture
    imshow(C);
end
%%% convert qu