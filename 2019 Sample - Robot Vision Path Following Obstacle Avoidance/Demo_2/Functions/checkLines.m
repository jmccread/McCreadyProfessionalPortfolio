function [good_lines] = checkLines(lines)
bad_lines = zeros(size(lines)); % bad line means 1   
    for k = 1:length(lines)
        % bottom horz case 
        A =(lines(k).point1(2) == 480 || lines(k).point2(2) == 480);
        % top horz case
        B =(lines(k).point1(2) == 1 || lines(k).point2(2) == 1);
        % left vert case
        C =(lines(k).point1(1) == 1 || lines(k).point2(1) == 1);
        % right vert case
        D =(lines(k).point1(1) == 640 || lines(k).point2(1) == 640);
        % left vert case
        %E =(lines(k).point1(1) == 0 || lines(k).point2(1) == 0);
    
        bad_lines(k) = (A||B||C||D);
    end 
 good_lines = lines(~bad_lines); 
end 
