function [imgOut] = mask2RWBMask( oneNegOneMask )

%Gives a grayscale image an extra dimension
%in order to use color within it
[m n]=size(oneNegOneMask);
rgb=zeros(m,n,3);
rgb(:,:,1)=oneNegOneMask;
rgb(oneNegOneMask == -1) = 255;
rgb(:,:,2)=oneNegOneMask;
rgb(:,:,3)=oneNegOneMask;
rgb(rgb == 1) = 255;
rgb(rgb == -1 ) = 0;

rtn = oneNegOneMask;
rtn(oneNegOneMask == -1) = 2;

imgOut=rtn;
