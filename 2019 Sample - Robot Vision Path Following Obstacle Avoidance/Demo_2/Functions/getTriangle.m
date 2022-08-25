function [right_dir] = getTriangle(C, W, thresh_LD2, verbose)
% Inputs
%   C               rgb image
%   W               LDA transform matrix (3X2)
%   thresh_LD2      threshold for red pixel mapping 
%   verbose         1 if plot, 0 no plot
% Outputs
%   right_dir       1 if robot is going the right way
%                   0 if robot is going the wrong way
%                   -1 if no red triangle found
    row_C = reshape(C, [], 3); 
    C_lda = double(row_C)*W; 
    R_mask = C_lda(:,2) < thresh_LD2; 
    R_mask = reshape(R_mask, size(C,1), size(C,2)); 
    
    fore_mask = zeros(size(R_mask),'logical'); 
    rng = [215 480; 1 640];
    fore_mask((rng(1,1)):rng(1,2), rng(2,1):rng(2,2)) = 1; 
    
    R_mask = logical(R_mask.*fore_mask);
    [B,~] = bwboundaries(R_mask,'noholes');
    
    total_valued_pixels =  nan(length(B),1);
    if isempty(B)
        long_line = []; 
        right_dir = -1; 
        if verbose
            %imshow(C);
        end
        return
    end 
    
    for k = 1:length(B)
        my_mask = zeros(size(R_mask));
        inds = sub2ind(size(my_mask), B{k}(:,1), B{k}(:,2));
        my_mask(inds) = 1; 
        BW = imfill(my_mask,'holes');
        C_BW = uint8(BW).*C;
        total_valued_pixels(k) = sum(sum(double(BW))); 
        if verbose 
            %imshow(C_BW)
        end
    end
    
    [~, ind_red_blob] = max(total_valued_pixels);
    
    [l_corner_x, l_ind] = min(B{ind_red_blob}(:,2));
    [r_corner_x, r_ind] = max(B{ind_red_blob}(:,2));
    l_corner = [B{ind_red_blob}(l_ind,1), l_corner_x];
    r_corner = [ B{ind_red_blob}(r_ind,1), r_corner_x,];
    mid_pnt_base = (l_corner+r_corner)./2;
    point1 = [mid_pnt_base(2), mid_pnt_base(1)]; 

    
    my_mask = zeros(size(R_mask));
    inds = sub2ind(size(my_mask), B{ind_red_blob}(:,1), B{ind_red_blob}(:,2));
    my_mask(inds) = 1; 
    BW = imfill(my_mask,'holes');
    below_baseline = sum(sum(my_mask(mid_pnt_base(1)+5:end,:)));
    above_baseline = sum(sum(my_mask(1:mid_pnt_base(1)-5,:)));
    
    if above_baseline>below_baseline
        right_dir = 1;
        [tip_corner_y, tip_ind] = min(B{ind_red_blob}(:,1));
        tip_corner = [tip_corner_y, B{ind_red_blob}(tip_ind,2)];
    elseif above_baseline<below_baseline
        right_dir = 0;
        [tip_corner_y, tip_ind] = max(B{ind_red_blob}(:,1));
        tip_corner = [tip_corner_y, B{ind_red_blob}(tip_ind,2)];
    else
        long_line = []; 
        right_dir = -1; 
        if verbose
            %imshow(C);
        end
        return
    end
    
    point2 = [tip_corner(2), tip_corner(1)]; 
    if right_dir ~= -1 
        lineM = ((point2(2) -  point1(2))./(point2(1) - point1(1)))^-1; % inverse of before
        lineC =  (point2(1)) - lineM*(point2(2));
        y = 1:480;
        x = lineM.*y+lineC;
        lineTheta = -atan2(1, lineM);
        lineRho = lineC * sin(lineTheta);
        field1 = 'point1';  value1 = [0,0];
        field2 = 'point2';  value2 = [0, 0];
        field3 = 'theta';  value3 = 0;
        field4 = 'rho';  value4 = 0;
        long_line = struct(field1,value1,field2,value2,field3,value3,field4,value4);

        long_line.point1 = [x(1), y(1)];
        long_line.point2 = [x(end), y(end)];           
        long_line.theta =rad2deg(lineTheta); 
        long_line.rho = lineRho;
    else
        long_line = [];
    end 

    if verbose == 1 && right_dir ~= -1
            %imshow(C)
            hold on;
            if right_dir == 1
                plot(tip_corner(2),tip_corner(1) ,'x','LineWidth',1.5,'Color','blue');
                plot(mid_pnt_base(2),mid_pnt_base(1),'x','LineWidth',1.5,'Color','blue');
                plot([tip_corner(2), mid_pnt_base(2)], [tip_corner(1), mid_pnt_base(1)], 'LineWidth',1.5,'Color','green');
                plot(l_corner(2),l_corner(1),'x','LineWidth',1.5,'Color','blue');
                plot(r_corner(2),r_corner(1),'x','LineWidth',1.5,'Color','blue');
                plot([r_corner(2), l_corner(2)], [r_corner(1), l_corner(1)], 'LineWidth',1.5,'Color','white');
            else
                plot(tip_corner(2),tip_corner(1) ,'x','LineWidth',1.5,'Color','blue');
                plot(mid_pnt_base(2),mid_pnt_base(1),'x','LineWidth',1.5,'Color','blue');
                plot([tip_corner(2), mid_pnt_base(2)], [tip_corner(1), mid_pnt_base(1)], 'LineWidth',1.5,'Color','white');
                plot(l_corner(2),l_corner(1),'x','LineWidth',1.5,'Color','blue');
                plot(r_corner(2),r_corner(1),'x','LineWidth',1.5,'Color','blue');
                plot([r_corner(2), l_corner(2)], [r_corner(1), l_corner(1)], 'LineWidth',1.5,'Color','white');
            end
            
            xy = [long_line.point1; long_line.point2];
            % Plot beginnings and ends of lines
            plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');  
            plot(x,y,'LineWidth',2,'Color','cyan');

    end
    
end 




 