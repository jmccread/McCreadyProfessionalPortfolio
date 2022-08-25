listing = dir('*.png'); 


buffSize = 5;
imageCircBuff1 = nan(480, 640,3, buffSize);
for i = 1:buffSize
     C = imread(listing(i).name);
    imageCircBuff1 =cat(4, C, imageCircBuff(:,:,:, 1:end-1));

end 


imageCircBuff2 = nan(480, 640,3, buffSize);
for i = 1:buffSize
     C = imread(listing(i).name);
    imageCircBuff2 =cat(4, C, imageCircBuff(:,:,:, 1:end-1));

end 