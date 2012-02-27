function overlay(img1, img2 )
figure, imshow(img1), hold on
hImg = imshow(img2); set(hImg, 'AlphaData', 0.3);


% Example of use:  
% overlay(imshow(uint8(reshape(csvread('/home/gavin/Desktop/tmp.csv'),480,640,3))), imread('/home/gavin/source/RGBDemov5/build/bin/6.png'));