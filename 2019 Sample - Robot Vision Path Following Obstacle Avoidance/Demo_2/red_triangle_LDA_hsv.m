%close all
clear all
close all
addpath('Functions')
addpath('WorkspacesDemo2'); 
addpath('WorkspacesDemo1'); 

load('agglomerated_data.mat');
R_hsv = double(R_hsv);
nR_hsv = double(nR_hsv);
%nR_hsv = nR_hsv(1:900:end,:); % Downsample

%% Step 1 compute the d-dimensional mean vectors - d = 3
mu_hsv_R = mean(R_hsv, 1); 
mu_hsv_nR = mean(nR_hsv, 1); 

%% Step 2 Computing the scatter (covariance)
% Step 2.1 Computing the within clas scatter matricies Sw1 Sw2
C_hsv_R = cov(R_hsv,1);
C_hsv_nR = cov(nR_hsv,1);

% Step 2.2 Computing the between class scatter matrix Sb
hsv = cat(1, R_hsv, nR_hsv);% combine date
C_hsv = cov(hsv,1);

%% Step 3 Solve gneralized eiganvalue problem for Sw^-1 * Sb
A = C_hsv_R^-1*C_hsv;

%% Step 4 Selecting linear discriminants for new feature subspace
% 4.1 Sorting the eigenvectors by decreasing eigenvalues
[ev_A_hsv,lv_A_hsv]=eig(A);
[lv_A_hsv_sort,I_A] = sort(lv_A_hsv); 

% 4,2 Choosing k eigenvectors with largest eigenvalues to for W
% a k x d dimensional matrix
largest_2 = I_A(3,:) == 1 |I_A(3,:) == 2;
W = ev_A_hsv(:, largest_2); 

%% Transforming samples into a new subspace
y_nR = nR_hsv*W; 
y_R = R_hsv*W; 
figure; 
plot(y_nR(:,1),y_nR(:,2), '.g'); 
hold on; 
plot(y_R(:,1),y_R(:,2), '.r');
xlabel('LD1'); 
ylabel('LD2'); 
title('LDA HSV space');

% looked at plot 
thresh_LD1 = max(y_R(:,1)); % Red is < 1 LD1 
thresh_LD2 = min(y_R(:,2)); % Red is > -1 LD1 
% LD2 is not significant

% Classification trial 
%% 
filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_backward_triangles.mat'; 
load(filename); 
% subplot(221); imshow(imageCircBuff(:,:,:,1));
% subplot(222); imshow(imageCircBuff(:,:,:,3));
% subplot(223); imshow(imageCircBuff(:,:,:,4));
figure;
C = imageCircBuff(:,:,:,4); 
imshow(C);
%subplot(224); imshow(imageCircBuff(:,:,:,5));

% Compute HSV C
hsv_C = rgb2hsv(C); 
hsv_C = reshape(hsv_C, [], 3); 

C_lda = hsv_C*W; 
figure 
plot(C_lda(:,1), C_lda(:,2), '.b');
figure
R_mask = (C_lda(:,1) < thresh_LD1) & (C_lda(:,2) > thresh_LD2); 
R_mask = reshape(R_mask, size(C,1), size(C,2)); 
title('Image (Right), Detected Red Triangle Left');
imshowpair(C, R_mask, 'montage')

filename = 'WorkspacesDemo2/WorkspacesDemo2_along_line_complex_backward_triangles.mat';
load(filename)
C = imshow(imageCircBuff(:,:,:,4));



