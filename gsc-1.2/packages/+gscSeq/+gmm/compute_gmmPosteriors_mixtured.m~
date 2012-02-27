function [ posteriors fgPosterior bgPosterior] =compute_gmmPosteriors_mixtured(features,fgGmm,bgGmm,gamma,uniform_value)
% Function to compute posteriors of each feature
% given the gmm
% Inputs:
%  features: a D x N array of features (D=dimensionality)
%  fgGmm, bgGmm: gmm structures for the foreground and background
%  gmm
%  gamma in [0,1] ,gamma=0 means totally gmm likelihood,
%  gamma = 1 means totally uniform likelihood
%  uniform_value -> the value a uniform distribution takes
import gscSeq.gmm.*;

% Create a gammaFG that increases the uniform probability fanned out over
% the image
perRowColInc = 0.00001; % pretty good
%perRowColInc = 0.000001; % pretty good
gammaFGGrad =zeros(480,640,1);

for r = 1:480,
    for c = 1:640,
       
        gammaFGGrad(r,c) = min( mod(r-1,240),mod(c-1,320) ) * perRowColInc;
   
    end
end

perRowColInc = 0.003; % pretty good
%perRowColInc = 0.000001; % pretty good
gammaBGGrad =zeros(480,640,1);

for r = 1:480,
    for c = 1:640,
       
        gammaBGGrad(r,c) = min( mod(r-1,240),mod(c-1,320) ) * perRowColInc;
   
    end
end


%Enable these two to disable variable colour uniform priors
gammaFGGrad =zeros(480,640,1);% + gamma;

gammaBGGrad = zeros(480,640,1);% + gamma;

gammaFGGrad =gscSeq.extractPixels(gammaFGGrad);

gammaBGGrad =gscSeq.extractPixels(gammaBGGrad);



fgLikeli=computeGmm_likelihood(features,fgGmm);
bgLikeli=computeGmm_likelihood(features,bgGmm);




%fgLikeli(fgLikeli>1)=1;
%bgLikeli(bgLikeli>1)=1;
% Scale to be between zero and one
%fgLikeli = fgLikeli - min(fgLikeli);
%fgLikeli = fgLikeli / max(fgLikeli);
%bgLikeli = bgLikeli - min(bgLikeli);
%bgLikeli = bgLikeli / max(bgLikeli);

%fgLikeli=gammaFG*uniform_value+(1-gammaFG)*fgLikeli; 

fgLikeli=gammaFGGrad*uniform_value+(1-gammaFGGrad).*fgLikeli; 

%bgLikeli=gammaBG*uniform_value+(1-gammaBG)*bgLikeli;

bgLikeli=(1-gammaBGGrad)+(gammaBGGrad).*bgLikeli;

divByZero=(fgLikeli==0 & bgLikeli==0);
fgLikeli(divByZero)=1;
bgLikeli(divByZero)=1;

%fgLikeli = fgLikeli/max(fgLikeli);
%bgLikeli = bgLikeli/max(bgLikeli);

fgPosterior = fgLikeli;
bgPosterior = bgLikeli;

posteriors=fgLikeli./(fgLikeli+bgLikeli);
%posteriors=fgLikeli/max(fgLikeli);



%function likeli=computeGmm_likelihood(features,gmm)

%likeli=zeros(1,size(features,2));
%for i = 1:length( gmm.pi )
% likeli=likeli+vag_normPdf(features,gmm.mu(:,i),gmm.sigma(:,:,i),gmm.pi(i));
%end
