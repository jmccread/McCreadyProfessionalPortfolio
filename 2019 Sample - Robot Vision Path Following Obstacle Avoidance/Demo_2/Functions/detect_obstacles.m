function [detections, min_point] = detect_obstacles(filtered_scan_data)
    %x = zeros(1, 360);
    %y = zeros(1, 360);
    detections_left = 0;
    detections_right = 0;
    %detection_angles = zeros(1, 360);
    min_point_left = [100, 360, 100, 100, 2];
    min_point_right = [100, 360, 100, 100, 2];
    detections = 0;
    min_point = [100, 360, 100, 100, 2];
    for i=1:size(filtered_scan_data, 2)
        pause(.1)
        detected = 0;
        for jr=91:180
            xr = filtered_scan_data(i).Ranges(jr)*cos(jr*pi/180);
            yr = filtered_scan_data(i).Ranges(jr)*sin(jr*pi/180);
            %x(jr) = xr;
            %y(jr) = yr;
            if xr < 0 && abs(yr) < .07 && xr > -.45
                detected = 1;
                if jr > 148
                    xline((640*(211-jr)/(211-149)), 'LineWidth', 2, 'Color', 'g');
                    %detection_angles(jr) = jr;
                end
                set(gca,'Color','r')
                if min_point_right(1) > filtered_scan_data(i).Ranges(jr)
                    min_point_right = [filtered_scan_data(i).Ranges(jr), jr, xr, yr, 1];
                end
                detections_right = detections_right + 1;

            end
        end
        for jl=181:269
            xl = filtered_scan_data(i).Ranges(jl)*cos(jl*pi/180);
            yl = filtered_scan_data(i).Ranges(jl)*sin(jl*pi/180);
            %x(jl) = xl;
            %y(jl) = yl;
            if xl < 0 && abs(yl) < .16 && xl > -.45
                detected = 1;
                if jl < 212
                    xline((640*(211-jl)/(211-149)), 'LineWidth', 2, 'Color', 'g');
                    %detection_angles(jl) = jl;
                end
                set(gca,'Color','r')
                if min_point_left(1) > filtered_scan_data(i).Ranges(jl)
                    min_point_left = [filtered_scan_data(i).Ranges(jl), jl, xl, yl, 0];
                end
                detections_left = detections_left + 1;

            end
        end
        %all_detection_angles = detection_angles(detection_angles ~= 0);
        
        
        if detected == 0
            set(gca,'Color','w')
        else
            if detections_left > detections_right
                detections = detections_left;
                if detections_right == 0
                    min_point = min_point_left;
                else
                    min_point = min_point_right;
                end
            else
                detections = detections_right;
                if detections_left == 0
                    min_point = min_point_right;
                else
                    min_point = min_point_left;
                end
            end
        end
    end
    
end