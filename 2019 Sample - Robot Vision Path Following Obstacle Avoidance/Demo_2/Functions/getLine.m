function [lines, lineM, lineC, specular_mask, fore_mask] = getLine(C, enableLineCheck, Adapt_sense, doVert)
% getLine does white line identification inside IAVS
% Inputs:
%   C                   RGB image from turtlebot
%   enableLineCheck     flag taking 0 or 1, 0 disabling the calling of the
%                       line check function, 1 enabling the checking for
%                       identified lines that begin in dubious parts of the
%                       image
%   Adapt_sense         Sensitivity value used for imbinarize that reduces
%                       nonspecular parts of the image typical values are
%                       .55 or .60 depending on light conditions
%   doVert              flag taking value 0 or 1, 0 enabling detection of
%                       lines not along the turtlebot's forward axis. 1
%                       enabling detection of such "vertical" lines.
%  Output:
%   lines               Line found in the ground plane, extended to total
%                       length
%   lineM               Slope of the lines
%   lineC               offset of the line

%                       
%% Do preprocessing/noise reduction
    %subplot(2,2,m);
    %C = histeq(C);
    I = rgb2gray(C);
    I = histeq(I);
    [fore_mask, rng] = foreground_mask(I); 
%     if sum(sum(fore_mask)) < 
%     
%     end
    fore_mask = uint8(fore_mask);
    J = histeq(I);
    J = J.*fore_mask;
    J = histeq(J);
% Second round of reduction
    %imshowpair(I,J,'montage')
    BW = imbinarize(J, .85); 
    specular_mask = imgaussfilt(uint8(BW), 5);
    specular_mask = imfill(logical(specular_mask),'holes');

    M = J.*uint8(specular_mask);
    %imshowpair(J,M,'montage')

    N = histeq(M);%.*specular_mask.*fore_mask;
    %imshowpair(M,N,'montage')

% Third round    
    %BW = imbinarize(N, [0.9, 0.97]); 
    sumB4 = sum(sum(N)); 
    sumNow =  sum(sum(N)); 
    dec = 0;
    BWadapt = imbinarize(N, 'adaptive','Sensitivity',Adapt_sense+dec);
    Q = N.*uint8(BWadapt);
    target =  sum(sum(N.*Q))/sumB4/25;
    P = N;
    while (sumNow/sumB4 > target) && (dec> -Adapt_sense)
        BWadapt = imbinarize(N, 'adaptive','Sensitivity',Adapt_sense+dec);
        dec = dec - 0.01;
        %BWadapt =imgaussfilt(uint8(BWadapt), 5);
        P = N.*uint8(BWadapt);
        sumNow = sum(sum(P));
        %P = histeq(P);
        %BW = imbinarize(P, .9); 
        %imshow(BWadapt)
        %imshow(P)
    end
   
 %% Detected boundaries for reduced image with only specular objects   
 BW_filt = logical(uint8(medfilt2(BWadapt)).*uint8(specular_mask).*uint8(fore_mask));   
 %imshow(BW_filt)
 [B,L] = bwboundaries(BW_filt,'noholes');
    %imshow(B)
    %title('Labeled Image')
    %imshow(label2rgb(L, @jet, [.5 .5 .5]))
    
% Turn boundaries into masks and calc selection metrics
    masks = nan(size(BW,1),size(BW,2), length(B)); 
    edges = nan(size(BW,1),size(BW,2), length(B));
    sums = nan(length(B),1);
    size_B = cellfun('length',B);
    num_elts = 5;%ceil(length(B)*1/3) % hopefully reduce comp time. 
    [bigBs, idx_B] = maxk(size_B, num_elts);
    B = B(idx_B); 
    if length(B) == 1
        my_mask = zeros(size(BW));
        inds = sub2ind(size(my_mask), B{1}(:,1), B{1}(:,2));
        my_mask(inds) = 1; 
        edges =cat(3, my_mask, edges(:,:, 1:end-1));
        BW = imfill(my_mask,'holes');
        masks = my_mask; 
        sums = sum(sum(BW));

    else 
        for k = 1:length(B)
            my_mask = zeros(size(BW));
            inds = sub2ind(size(my_mask), B{k}(:,1), B{k}(:,2));
            my_mask(inds) = 1; 
            edges =cat(3, my_mask, edges(:,:, 1:end-1));
            BW = imfill(my_mask,'holes');
            masks =cat(3, BW, masks(:,:, 1:end-1));
            sums =cat(1, sum(sum(BW)), sums(1:end-1));
        end
    end
    
%% Select "best" mask by size - future by size and degree of specularness
    [sorted_sums, I_sum] = sort(sums); 
    
    % Just choose the biggest
    [~, candidate_1] = max(sums); 
%     figure(1);
%     subplot(1,1,1)
%     imshow(edges(:,:,candidate_1));
%% Detect lines from calcuated boundaries (no canny needed) 
    [lines, all_lines] = findEdgesThick(edges(:,:,candidate_1),enableLineCheck, 1);
       
    if ~isempty(lines) 
        if enableLineCheck
            lines = checkLines(lines);
        end
    end
    lineM = []; 
    lineC = []; 
    if ~isempty(lines)
        if ~doVert
            [long_line, lineM, lineC]  = elongateLines(lines, rng, 1);
            lines = long_line;
        else 
            [long_line, lineM, lineC]  = elongateLinesVert(lines, rng, 1);
            lines = long_line;
        end
    end
end