clc
close all
clear all
files = dir('*file*.mat'); 


R_rgb_agglo = []; 
nR_rgb_agglo = []; 
R_hsv_agglo = []; 
nR_hsv_agglo = []; 


for k = 2:length(files)
    load(files(k).name); 
    R_rgb_agglo = cat(1, R_rgb, R_rgb_agglo); 
    nR_rgb_agglo = cat(1, nR_rgb, nR_rgb_agglo); 
    R_hsv_agglo = cat(1, R_hsv, R_hsv_agglo); 
    nR_hsv_agglo = cat(1, nR_hsv, nR_hsv_agglo); 
end

R_rgb = R_rgb_agglo;
nR_rgb = nR_rgb_agglo;
R_hsv = R_hsv_agglo;
nR_hsv = nR_hsv_agglo; 
save('agglomerated_data.mat', 'R_rgb', 'nR_rgb', 'R_hsv', 'nR_hsv'); 

