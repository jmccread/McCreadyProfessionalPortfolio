function [C] = TurtleBot3_Image(image_sub)
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    C = readImage(image_compressed); 
end
