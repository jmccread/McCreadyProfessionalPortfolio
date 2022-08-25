%% findEdges: Function takes in an image and outputs the Hough lines
%     Inputs: im, verbose
%     im, image that will be processed on
%     verbose, if set to 1, plot the image and its detected edges
%     Outputes: lines
%     lines, Hough lines that were extracted

function [lines] = findEdges(im, verbose)
   % Run Edge Detection
   % im = rgb2gray(im);
%    imbw = im2bw(im,graythresh(im)); % threshold, this should be done outside of the edge detection
   im_Canny = edge(im,'Canny'); %Detect edges using Canny
   [H,T,R] = hough(im_Canny,'RhoResolution',0.1,'ThetaResolution',0.1);
   numpeaks = 5; %Specify the number of peaks
   P  = houghpeaks(H,numpeaks);
   lines = houghlines(im_Canny,T,R,P,'FillGap',27,'MinLength',15);
   
   % Compute the length of each detected line
   lineLength = [];
   for i = 1:length(lines)
      lineLength(i) = norm(lines(i).point1 - lines(i).point2);
   end
   [~, idx] = sort(lineLength, 2, 'descend');

   if verbose == 1
      figure(3)
      subplot(121); imshow(im)
      title('Original Image')
      subplot(122); imshow(im_Canny)
      title('Canny')
      hold on

      max_len = 0;
      top_lines = 1; % plot x longest lines
%       for k = 1:length(lines) % uncomment to plot all lines
      for k = 1:top_lines
%          xy = [lines(k).point1; lines(k).point2]; % uncomment to plot all lines
         xy = [lines(idx(k)).point1; lines(idx(k)).point2];
         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');

         % Plot beginnings and ends of lines
         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

         % Determine the endpoints of the longest line segment
         len = norm(lines(k).point1 - lines(k).point2);
         if ( len > max_len)
            max_len = len;
            xy_long = xy;
         end
      end

      % Show the rho-theta voting
      figure(4)
      imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
            'InitialMagnification','fit');
      title('Hough transform results');
      xlabel('\theta'), ylabel('\rho');
      axis on, axis normal, hold on;
      colormap(gca,hot);

      imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
      xlabel('\theta'), ylabel('\rho');
      axis on, axis normal, hold on;
      plot(T(P(:,2)),R(P(:,1)),'s','color','white');
   end
   lines = lines(idx);