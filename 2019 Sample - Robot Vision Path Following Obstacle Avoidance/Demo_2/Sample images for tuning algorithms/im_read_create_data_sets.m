listing = dir('*.png'); 


buffSize = 4;
imageCircBuff1 = nan(480, 640,3, buffSize);
for i = 1:4
     C = imread(listing(i).name);
    imageCircBuff1 =cat(4, C, imageCircBuff(:,:,:, 1:end-1));

end 


imageCircBuff2 = nan(480, 640,3, buffSize);
for i = 5:8
     C = imread(listing(i).name);
    imageCircBuff2 =cat(4, C, imageCircBuff(:,:,:, 1:end-1));

end 