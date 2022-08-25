%close all
clear all
close all
addpath('Functions')
addpath('WorkspacesDemo2'); 
addpath('WorkspacesDemo1'); 

load('file_good.mat');
R_rgb = double(R_rgb);
nR_rgb = double(nR_rgb);
%nR_rgb = nR_rgb(1:900:end,:); % Downsample

%% Step 1 compute the d-dimensional mean vectors - d = 3
mu_rgb_R = mean(R_rgb, 1); 
mu_rgb_nR = mean(nR_rgb, 1); 

%% Step 2 Computing the scatter (covariance)
% Step 2.1 Computing the within clas scatter matricies Sw1 Sw2
C_rgb_R = cov(R_rgb,1);
C_rgb_nR = cov(nR_rgb,1);

% Step 2.2 Computing the between class scatter matrix Sb
rgb = cat(1, R_rgb, nR_rgb);% combine date
C_rgb = cov(rgb,1);

%% Step 3 Solve gneralized eiganvalue problem for Sw^-1 * Sb
A = C_rgb_R^-1*C_rgb;

%% Step 4 Selecting linear discriminants for new feature subspace
% 4.1 Sorting the eigenvectors by decreasing eigenvalues
[ev_A_rgb,lv_A_rgb]=eig(A);
[lv_A_rgb_sort,I_A] = sort(lv_A_rgb); 

% 4,2 Choosing k eigenvectors with largest eigenvalues to for W
% a k x d dimensional matrix
largest_2 = I_A(3,:) == 1 |I_A(3,:) == 2;
W = ev_A_rgb(:, largest_2); 

%% Transforming samples into a new subspace
y_nR = nR_rgb*W; 
y_R = R_rgb*W; 
figure; 
plot(y_nR(:,1),y_nR(:,2), '.g'); 
hold on; 
plot(y_R(:,1),y_R(:,2), '.r');
xlabel('LD1'); 
ylabel('LD2'); 
title('LDA rgb space');

% looked at plot 
thresh_LD2 = -30; % Red is < -50 than not red
% LD1 is not significant
thresh_LD1 = []; 
% Classification trial 
%% 
filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_backward_triangles.mat'; 
load(filename); 
% subplot(221); imshow(imageCircBuff(:,:,:,1));
% subplot(222); imshow(imageCircBuff(:,:,:,3));
% subplot(223); imshow(imageCircBuff(:,:,:,4));
figure;
C = imageCircBuff(:,:,:,5); 
imshow(C);
%subplot(224); imshow(imageCircBuff(:,:,:,5));

% Compute HSV C
row_C = reshape(C, [], 3); 

C_lda = double(row_C)*W; 
figure 
title('LDA outcomes spread'); 
plot(C_lda(:,1), C_lda(:,2), '.b');
R_mask = C_lda(:,2) < thresh_LD2; 
R_mask = reshape(R_mask, size(C,1), size(C,2)); 
figure(86)
title('Image (Right), Detected Red Triangle Left');
subplot(2,1,1);
imshowpair(C, R_mask, 'montage')


filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_backward_triangles.mat';
load(filename)
C = imageCircBuff(:,:,:,4);
row_C = reshape(C, [], 3); 

C_lda = double(row_C)*W; 
R_mask = C_lda(:,2) < thresh_LD2; 
R_mask = reshape(R_mask, size(C,1), size(C,2)); 
title('Image (Right), Detected Red Triangle Left');
subplot(2,1,2);
imshowpair(C, R_mask, 'montage')

filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_forward_triangle_facing_yellow.mat';
load(filename)
C = imageCircBuff(:,:,:,1);
row_C = reshape(C, [], 3); 

C_lda = double(row_C)*W; 
R_mask = C_lda(:,2) < thresh_LD2; 
R_mask = reshape(R_mask, size(C,1), size(C,2)); 
title('Image (Right), Detected Red Triangle Left');
figure(87)
subplot(2,1,1);
imshowpair(C, R_mask, 'montage');

filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_wall_2.mat';
load(filename)
C = imageCircBuff(:,:,:,4);
row_C = reshape(C, [], 3); 

C_lda = double(row_C)*W; 
R_mask = C_lda(:,2) < thresh_LD2; 
R_mask = reshape(R_mask, size(C,1), size(C,2)); 
title('Image (Right), Detected Red Triangle Left');
subplot(2,1,2);
imshowpair(C, R_mask, 'montage');

save('lda_rgb.mat', 'W', 'thresh_LD2', 'thresh_LD1'); 


function [R_mask] = rbg_LDA(W, thresh_LD2, C);
    row_C = reshape(C, [], 3); 
    C_lda = double(row_C)*W; 
    R_mask = C_lda(:,2) < thresh_LD2; 
    R_mask = reshape(R_mask, size(C,1), size(C,2)); 
    
end 
