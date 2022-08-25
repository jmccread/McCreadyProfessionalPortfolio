
function [long_line, lineM, lineC] = elongateLinesVert(lines, rng, verbose)
     try 
        % Compute the length of each detected line
      lines_tb = struct2table(lines);
      lineLength = zeros(length(lines),1); 
      points_1 = lines_tb.point1;
      points_2 = lines_tb.point2;
      % Find most extreme line points 
      [~, min_p1_index] = min(points_1(:,2)); 
      [~, max_p2_index] = max(points_2(:,2)); 
      
      point1 = lines(min_p1_index).point1; % bottom most point detected
      point2 = lines(max_p2_index).point2; % top most point detected 
      
      % Find the slope for the most extreme points on the line 
      lineM = ((point2(2) -  point1(2))./(point2(1) - point1(1)))^-1; % inverse of before
      lineC =  (point2(1)) - lineM*(point2(2));
      
%       x = 1:640;
%       y = lineM.*x+lineC; 
%       y = y(y>215);  % y(y>rng(1,1))
%       x = ceil((y - lineC)./lineM);
%       lineTheta = -atan2(1, lineM);
%       lineRho = lineC * sin(lineTheta);
%       if isnan(sum(sum(x)))
%         x = 1:640;
%         y = lineM.*x+lineC; 
%         %return;
%       end 

      y = 1:480;
      x = lineM.*y+lineC; 

      lineTheta = -atan2(1, lineM);
      lineRho = lineC * sin(lineTheta);
%       if isnan(sum(sum(x)))
%         x = 1:640;
%         y = lineM.*x+lineC; 
%         %return;
%       end 
      for k = 1:length(lines)
        lineLength(k) = norm(lines_tb.point1(k,:) -  lines_tb.point2(k,:));
      end
       longest_lines = lines(lineLength == max(lineLength));
       longest_line = longest_lines(1);
    
    field1 = 'point1';  value1 = [0,0];
    field2 = 'point2';  value2 = [0, 0];
    field3 = 'theta';  value3 = 0;
    field4 = 'rho';  value4 = 0;
    long_line = struct(field1,value1,field2,value2,field3,value3,field4,value4);
    
    long_line.point1 = [x(1), y(1)];
    long_line.point2 = [x(end), y(end)];           
    long_line.theta =rad2deg(lineTheta); 
    long_line.rho = lineRho; 
    
    if (verbose == 1) 
        hold on; 
        xy = [long_line.point1; long_line.point2];
        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');  
        plot(x,y,'LineWidth',2,'Color','cyan');
        
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');
            % Plot beginnings and ends of lines
            plot(xy(1,1),xy(1,2),'x','LineWidth',1,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',1,'Color','red');
        end
    end
     catch
        long_line = []; 
        lineM = NaN;  
        lineC = NaN;
     end 
end


