function [C_fore_mask] = foreground_mask(I)
%% Foreground detection function, detects the foreground based on the
% largest blob appearing in the image as found by bwboundaries with
% noholes.
% Input:
%       C               1 channel grayscale image
% Outputs: 
%       C_fore_mask     image mask with foreground as 1's

    BW = imbinarize(I); % Threshold image according to otsu's method
    [B, L] = bwboundaries(BW, 'noholes'); 
    [max_sz, index] = maxk(cellfun('length', B),3);
    C_fore_mask = zeros(size(BW),'uint8'); 
    if max(max(B{index(1)}(:,1))) == size(BW, 1)
        rng = [min(B{index(1)}(:,1)), max(B{index(1)}(:,1)); min(B{index(1)}(:,2)),max(B{index(1)}(:,2))];
    elseif max(max(B{index(2)}(:,1)))max(B{index(2)},1) == size(BW, 1)
        rng = [min(B{index(2)}(:,1)), max(B{index(2)}(:,1)); min(B{index(2)}(:,2)),max(B{index(2)}(:,2))];
    elseif max(max(B{index(3)}(:,1))) == size(BW, 1)
        rng = [min(B{index(3)}(:,1)), max(B{index(3)}(:,1)); min(B{index(3)}(:,2)),max(B{index(3)}(:,2))];
    else
        return 
    end
    C_fore_mask(rng(1,1):rng(1,2), rng(2,1):rng(2,2)) = 1; 
    
end