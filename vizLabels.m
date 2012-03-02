function [Image]=vizLabels(Image)
%Gives a grayscale image an extra dimension
%in order to use color within it
[m n]=size(Image);
rgb=zeros(m,n,3);
known = Image;
known(known == -1 ) = 0;
known(known == 1 ) = 255;

unknown = Image;
unknown(unknown == 1 ) = 0;
unknown(unknown == -1 ) = 255;

rgb(:,:,1)=unknown;
rgb(:,:,2)=known;
rgb(:,:,3)=zeros(m,n,1);
Image=rgb/255;
end