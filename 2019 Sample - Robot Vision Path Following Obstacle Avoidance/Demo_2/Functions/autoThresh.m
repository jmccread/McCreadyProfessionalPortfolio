function [auto] = autoThresh(I, lvl, verbose)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
[counts,binLocations] = imhist(I);
intensity = binLocations/255;
Ctot = trapz(counts);
Ccum = cumtrapz(counts);
Cper = Ccum./Ctot; 
if verbose == 1
    figure; 
    plot(intensity, Cper);
    title('Cumulative pixel count at until pixel value(Normalized'); 
    xlabel('Pixel value (Normalized)');
    xlim([0,1]);
end 
thresh = intensity(Cper>lvl);
auto = thresh(1);
end

