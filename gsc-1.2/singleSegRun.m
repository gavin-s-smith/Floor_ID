 function [ segmentation posterior fgPosterior bgPosterior ] = singleSegRun( inImg, inLabels, extraImages, extraMasks, gcGamma, customPosterior )
% Performs a singe gscSeq segmentation. 
% inLables are labels with -1 bg, 1 fg, 0 unknown.  These are generated by
% the software (FreenectIndoors).
% inImg is the loaded RGB image.

%inverse labelling for kicks
%inLabels(inLabels==1)=9;
%inLabels(inLabels==-1)=1;
%inLabels(inLabels==9)=-1;

segType = 0; % 0 = gscSeg, 1 = random walker

inLabels=robotUser.mask2RWBMask( inLabels );
for i = 1:length(extraMasks),
    extraMasks{i} = robotUser.mask2RWBMask( extraMasks{i} );
end

inGT = inLabels;
inGT = 1; % Not used in the first pass, which is all we do


opts.brushStyle='dotMiddle'; % Not used in the first pass, which is all we do
opts.numStrokes=1;% Define that we are only doing the first pass.
opts.brushRad=8; % Not used in the first pass, which is all we do


segOpts=gscSeq.segOpts();
if length(extraImages) ~= 0,
    segOpts.extraImages = extraImages;
    segOpts.extraImageMasks = extraMasks;
    segOpts.featureSpace = 'moreImages';
end

segOpts.gcGamma = gcGamma;

segH=gscSeq.segEngine(0,segOpts); % Debug level is the first arg.


% Transform the image to be somewhat invarient to illumination changes as
% per the paper: "A Perception-based Color Space for Illumination-invariant
% Image Processing" by Chong, Gorler & Zickler
%imageToSend = im2double(inImg);
imageToSend = inImg;
%C = makecform('srgb2cmyk'); %First convert to the xyz colour space (paper converts from this space)
%imageToSend = applycform(imageToSend,C);

% 13 Jan 2011
%imageToSend = illumInvColourSpace(imageToSend);

imageToSend = im2double(imageToSend); % Image must be of format double to process correctly

segH.preProcess(imageToSend);
 startOk=segH.start(inLabels, customPosterior);
 segmentation=segH.seg;
 posterior = segH.posteriorImage;
fgPosterior = segH.fgPosterior;
 bgPosterior = segH.bgPosterior;
 %imshow(segH.posteriorImage);

%[annoSeq,segSeq]=robotUser.run(segH,opts,imageToSend,inGT,inLabels);
delete(segH);

% --- Show the segmentation and brush sequence ---
%figure;
%for i=1:opts.numStrokes
%  subplot(1,opts.numStrokes,i);
%  imshow(segSeq(:,:,i));
%  title(sprintf('Segmentation after\nstroke %d',i-1));
%end

%subplot(2,2,1), imshow(inImg); title('Image');
%subplot(2,2,2), imshow(inLabels,[]); title('Seeds');

%subplot(2,2,3), imshow(inImg);
%hold on;
%contour(segSeq(:,:,1),[0 0],'g','linewidth',4);
%contour(segSeq(:,:,1),'k','linewidth',2);
%hold off;
%title('Output');

%subplot(2,2,4), imshow(segSeq(:,:,1));
%title('Binary Output');
               
                if min( min( posterior )) < 0,
                    fprintf('Bad vizPosterior: %f',min( min( posterior )));
                    throw err;
                end
