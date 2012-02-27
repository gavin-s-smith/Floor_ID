% Copyright (C) 2009-10 Varun Gulshan
% This class defines the options for Boykov Jolly segmentation
classdef segOpts
  properties (SetAccess=public, GetAccess=public)
    gcGamma  % gamma of graph cuts
    gcScale  % scaling to convert to integers
    gcNbrType % 'nbr4' or 'nbr8'
    gcSigma_c % 'auto' or a numeric value

    gmmNmix_fg % number of gmm mixtures for fg, usually 2-3
    gmmNmix_bg % number of gmm mixtures for bg, usually 2-3
    gmmUni_value % uniform likelihood mixing, see constructor for default
    gmmLikeli_gamma % strenght of uniform likliehood mixing

    featureSpace % rgb or rgb_xy or moreImages
    posteriorMethod % gmm_bs, gmm_bs_mixtured, none

    % star shape options
    starNbrhood_size % 4 or 8
    starMethod % 'imgGrad' or 'likeliGrad'
    geoGamma 
    extraImages
    extraImageMasks
    trainFeatures
  end

  methods
    function obj=segOpts()
      % Set the default options
      obj.gcGamma=150;
      obj.gcScale=50;
      obj.gcNbrType='colourGradient8';
      obj.gcSigma_c='auto';
        %fg, bg was both = 5
      obj.gmmNmix_fg=5;
      obj.gmmNmix_bg=5;
      obj.gmmUni_value=1.0; % assuming features in [0,1]
      obj.gmmLikeli_gamma=0.05; %orig 0.05
      obj.posteriorMethod='gmm_bs_mixtured';
      obj.featureSpace='rgb';
      obj.extraImages = [];
      obj.extraImageMasks=[];
      obj.trainFeatures=[];
      obj.starNbrhood_size=8;
      obj.starMethod='likeliGrad';
      %obj.starMethod='imgGrad';
      obj.geoGamma=0.3;
    end
  end

end % of classdef
