function [out_lines, all_lines] = findEdgesThick(im, robust_detect_flag, verbose)
% findEdges: Function takes in an image and outputs the Hough lines
%     Inputs: im, verbose
%     im, image that will be processed on
%     verbose, if set to 1, plot the image and its detected edges
%     Outputes: lines
%     lines, Hough lines that were extracted
  try
%% Do hough lines stuff 
       im_Canny = im; 
       [H,T,R] = hough(im_Canny,'RhoResolution',0.1,'ThetaResolution',0.1);
       numpeaks = 20; %Specify the number of peaks
       P  = houghpeaks(H,numpeaks);
       lines = houghlines(im_Canny,T,R,P,'FillGap',150,'MinLength',10); % more seems to be better with fillgap

% Take care of not having enough lines for clustering
       if length(lines) == 1 
           if robust_detect_flag
              out_lines = [];
              all_lines = [];
              return
           else
               lines = [lines, lines,lines];
               out_lines = lines;
               all_lines = lines; 
           end
           %return
       elseif length(lines) == 2
           lines = [lines, lines];
           all_lines = lines; 
       end 

       % Compute the length of each detected line - not used 
       lineLength = [];
       for i = 1:length(lines)
          lineLength(i) = norm(lines(i).point1 - lines(i).point2);
       end
       [~, idx] = sort(lineLength, 2, 'descend');

       lines = lines(idx);
       lines_tb = struct2table(lines);
       theta_std = std(lines_tb.theta); 
       abs_T = abs(lines_tb.theta);
       idx = kmeans(abs_T,3);
       abs_rho = abs(lines_tb.rho);
       idx = kmeans([abs_T, abs_rho],3);
       idx_rho = kmeans(abs_rho,3);

       means = [mean(abs_T(idx == 1)), mean(abs_T(idx == 2)), mean(abs_T(idx == 3))];
       std_devs = [std(idx == 1)*1.1, std(idx == 2)*1.1, std(idx == 3)*1.1];
       
%        means_rho = [mean(abs_rho(idx_rho == 1)), mean(abs_rho(idx_rho == 2)), mean(abs_rho(idx_rho == 3))];
%        std_dev_rho = [std(idx_rho == 1), std(idx_rho == 2)*1.1, std(idx_rho == 3)*1.1];

%% Figure out line families using theta and length
% Compute intervals and then create sets 
       intervals = [ means - 0.05*means; means + 0.05*means];
       intervals(1,:) = floor(intervals(1,:)); 
       intervals(2,:) = ceil(intervals(2,:));
       A = intervals(1,1):intervals(2,1);
       B = intervals(1,2):intervals(2,2);
       C = intervals(1,3):intervals(2,3); 
% Judge intersections of sets 
       AB = ~isempty(intersect(A,B)); % 1 if there are elements
       AC = ~isempty(intersect(A,C)); % 1 if there are elements
       BC = ~isempty(intersect(B,C)); 
       ABC = ~isempty(intersect(C, intersect(A,B)));
% Check for what sets have members and set output 
       if AB && ~AC && ~BC && ~ABC % Just AB 
          id_sel_lines_t = idx == 1 | idx == 2; 
       elseif AC && ~AB && ~BC && ~ABC
           id_sel_lines_t = idx == 1 | idx == 3; 
       elseif BC && ~AB && ~AC && ~ABC
           id_sel_lines_t = idx == 2 | idx == 3; 
       elseif ABC
            id_sel_lines_t = idx == 1 | idx == 2 | idx == 3;
       else % If no sets have any in common return the line family with the longest line 
          lineLength = zeros(length(lines),1); 
          for i = 1:length(lines)
            lineLength(i) = norm(lines_tb.point1(i,:) -  lines_tb.point2(i,:));
          end
          whatAcluster = idx(lineLength==max(lineLength)); 
          id_sel_lines_t = (idx == whatAcluster);
       end

%% Figure out line families using rho and length
% Compute intervals and then create sets 
       means = [mean(abs_rho(idx == 1)), mean(abs_rho(idx == 2)), mean(abs_rho(idx == 3))];
       intervals = [ means - 0.05*means; means + 0.05*means];
       intervals(1,:) = floor(intervals(1,:)); 
       intervals(2,:) = ceil(intervals(2,:));
       A = intervals(1,1):intervals(2,1);
       B = intervals(1,2):intervals(2,2);
       C = intervals(1,3):intervals(2,3); 
% Judge intersections of sets 
       AB = ~isempty(intersect(A,B)); % 1 if there are elements
       AC = ~isempty(intersect(A,C)); % 1 if there are elements
       BC = ~isempty(intersect(B,C)); 
       ABC = ~isempty(intersect(C, intersect(A,B)));
% Check for what sets have members and set output 
       if AB && ~AC && ~BC && ~ABC % Just AB 
          id_sel_lines_rho = idx == 1 | idx == 2; 
       elseif AC && ~AB && ~BC && ~ABC
           id_sel_lines_rho = idx == 1 | idx == 3; 
       elseif BC && ~AB && ~AC && ~ABC
           id_sel_lines_rho = idx == 2 | idx == 3; 
       elseif ABC
            id_sel_lines_rho = idx == 1 | idx == 2 | idx == 3;
       else % If no sets have any in common return the line family with the longest line 
          lineLength = zeros(length(lines),1); 
          for i = 1:length(lines)
            lineLength(i) = norm(lines_tb.point1(i,:) -  lines_tb.point2(i,:));
          end
          whatAcluster = idx(lineLength==max(lineLength)); 
          id_sel_lines_rho = (idx == whatAcluster);
       end
    out_lines = lines(id_sel_lines_t & id_sel_lines_rho);
    all_lines = lines; 
%         out_lines_tb = struct2table(out_lines);
%         abs_rho = abs(out_lines_tb.rho)
%        BadLines = isoutlier(abs_rho, 'gesd'); 
%        out_lines = out_lines(~BadLines);
        
%% Plotting if desired
       if verbose == 1
          hold on
          for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2]; % uncomment to plot all lines
             plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');
             % Plot beginnings and ends of lines
             plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
             plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
          end

          % Show the rho-theta voting
    %       figure(4)
    %       imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
    %             'InitialMagnification','fit');
    %       title('Hough transform results');
    %       xlabel('\theta'), ylabel('\rho');
    %       axis on, axis normal, hold on;
    %       colormap(gca,hot);
    % 
    %       imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
    %       xlabel('\theta'), ylabel('\rho');
    %       axis on, axis normal, hold on;
    %       plot(T(P(:,2)),R(P(:,1)),'s','color','red');
       end
   catch % catch if broken 
        out_lines = []; 
        all_lines = []; 
   end
end