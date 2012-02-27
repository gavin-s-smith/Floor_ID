function [ posteriorImage fgPosterior bgPosterior ]=getPosteriorImage(features,labelImg,segOpts)
% Function to compute posteriors, by learning color models
% from the user strokes in labelImg
% 
% features is a D x N array , which contains the features at each
% pixel (of the h x w x nFrames video)
% labelImg is uint8 h x w x nFrames array
% segOpts is a segmentation options structure documeted
% in videoOptions.m

switch(segOpts.posteriorMethod)
  case 'gmm_bs'
    posteriorImage=do_gmmBS_posteriorImage(features,labelImg,segOpts);
  case 'gmm_bs_mixtured'
    [posteriorImage fgPosterior bgPosterior]=do_gmmBS_posteriorImage_mixtured(features,labelImg,segOpts);
  case 'none'
    posteriorImage=0.5*ones(size(labelImg));
  otherwise
    error('Unsupported posteriorMethod: %s\n',segOpts.posteriorMethod);
end

function [posteriors fgPosterior bgPosterior ]=do_gmmBS_posteriorImage_mixtured(features,labelImg,segOpts)
import gscSeq.gmm.*;
% --- learn the gmm first --------
%fprintf('\nCORRECT METHOD CALLED!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');
% Serializes the labels from multiple images for training.
% For the actual images this has already been done in preProcess.m
% and placed in the trainFeatures slot of the segOpts
% TODO: Unify this coding.

%Sanity check
if sum(size(segOpts.extraImages) == size(segOpts.extraImageMasks)) ~= 2,
   % 2) Construct an MException object to represent the error.
    err = MException('Must have the same number of extra images as extra image masks.', ...
        '');
    throw(err)
end

labelImgUse = labelImg(:);
  switch(segOpts.featureSpace)
      case 'moreImages'
        
        for i = 1:length(segOpts.extraImageMasks),
             toAdd = segOpts.extraImageMasks{i};
             toAdd = toAdd(:);
             %size(toAdd)
             labelImgUse = [ labelImgUse ; toAdd ];
             %size(labelImgUse)
        end
  end
  
  %Alter the extraImageMasks and extraImages to be only unique colours
  %[A m n] = unique(segOpts.trainFeatures','rows');
  
  %segOpts.trainFeatures = A';
  
  %labelImgUse = labelImgUse';
  %labelImgUse = labelImgUse(:,m)';
  if size(segOpts.trainFeatures,2) <= (640*480),
      fprintf('88888888888888888888888888  NOT ENOUGH IMAGES!!!!');
  end
  %Done altering
  
 % fprintf('\nFeatures\n');
 %size(features)
 %  fprintf('\nImage Labels\n');
% size(labelImgUse)
% ---- Train the GMMs ------
fgFeatures=segOpts.trainFeatures(:,labelImgUse==1);
fgGmm=init_gmmBS(fgFeatures,segOpts.gmmNmix_fg);
%sum(labelImgUse==2)
%max(labelImgUse)
%size(labelImgUse)
%size(segOpts.trainFeatures)
bgFeatures=segOpts.trainFeatures(:,labelImgUse==2);
bgGmm=init_gmmBS(bgFeatures,segOpts.gmmNmix_bg);

% --- Now compute posteriors ------
% But ONLY on the image in question.  Hence we use features, not
% trainFeatures.
[posteriors fgPosterior bgPosterior ] =compute_gmmPosteriors_mixtured(features,fgGmm,bgGmm,segOpts.gmmLikeli_gamma,segOpts.gmmUni_value);
sTmp = size(labelImg);
posteriors=reshape(posteriors(1:(sTmp(1)*sTmp(2))),size(labelImg));
fgPosterior=reshape(fgPosterior(1:(sTmp(1)*sTmp(2))),size(labelImg));
bgPosterior=reshape(bgPosterior(1:(sTmp(1)*sTmp(2))),size(labelImg));
%imshow(mat2gray(posteriors))

function posteriors=do_gmmBS_posteriorImage(features,labelImg,segOpts)

import gscSeq.gmm.*;
% --- learn the gmm first --------
fgFeatures=features(:,labelImg(:)==1);
fgGmm=init_gmmBS(fgFeatures,segOpts.gmmNmix_fg);

bgFeatures=features(:,labelImg(:)==2);
bgGmm=init_gmmBS(bgFeatures,segOpts.gmmNmix_bg);

% --- Now compute posteriors ------
posteriors=compute_gmmPosteriors(features,fgGmm,bgGmm);
posteriors=reshape(posteriors,size(labelImg));
