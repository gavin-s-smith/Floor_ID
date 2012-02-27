function [ overlapAsPercentOfGT, overSegAsPercentOfGT, overlapScore, floor3DScore ] = evalSegGT( segMaskFilename, GTFilename )
% Over seg is the percent of pixels that were incorrectly marked as true
% which were not as a percent of those that were actually correct

% dirIn = '~/Dropbox/indoorWayfinding/code/scripts'
% segMaskFilename = strcat(dirIn,'/','0459.flr.mat')
% GTFilename = strcat(dirIn,'/','0459GT.bmp')

i = customImRead( GTFilename );
v = load( segMaskFilename );

idFloor = load( segMaskFilename(1:length(segMaskFilename)-4) );

if size(v,1) == 1,
    v = v.v;
end

i = (i==0); % invert as black was the mask and black is encoded as 0
v = (v==255);

gt = i;

tp = gt & v;
intersect = sum(sum(gt & v));
union = sum(sum(gt | v));



% remove the bottom lines as they are not so usefull (were black bars)
%gt = gt(1:473,:);
%tp = tp(1:473,:);

overlapAsPercentOfGT = sum(sum(tp)) / sum(sum(gt));
overSegAsPercentOfGT = ( sum(sum(v)) - sum(sum(tp)) ) / sum(sum(gt));
overlapScore = 100 * (intersect / union);

floor3DScore = 100 * ( sum(sum(gt & (idFloor==1))) / sum(sum(gt | (idFloor==1)))) ;
end

