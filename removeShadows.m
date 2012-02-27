function [ bestAngle rtnImg ] = removeShadows( rgbImage, floorMask, useAngle )

% First apply Gaussian smoothing.  See Intrinsic Images by Entropy
% Mimization, pg 591.

%myfilter = fspecial('gaussian',[3 3], 0.5);
%rgbImage = imfilter(rgbImage, myfilter, 'replicate');
cImage = getChromaticity( rgbImage );

if useAngle < 0,
    

minEntropy = 999999999999999;
bestAngle = -1;
fprintf('\n');
for theta = 1:180 
    I = calcI( cImage, theta, floorMask );
    entropyVal = calcMinEntropy( I );
    if entropyVal < minEntropy,
        minEntropy = entropyVal;
        bestAngle = theta;
    end
    fprintf('.');
end
fprintf('\n');
else

    bestAngle = useAngle;
end

%bestAngle = 20;

rtnImg = calcI(cImage, bestAngle, floorMask);
%rtnImg = mat2gray(rtnImg); %normalizes the image to be between 0 and 1
%imshow(rtnImg);
%imwrite(rtnImg, '/home/gavin/Desktop/t1.png','png');
end

function [ rtn ] = calcI( cImage, theta, floorMask )
theta_rad = theta*pi/180;

sz = size(cImage);
rtn = zeros(sz(2), sz(3));

toPlot = zeros(sum(sum(floorMask == 1)), 2);
toPlotCt = 1;

U = calcU;
toPlot = [];
for row = 1:sz(2),
    for col = 1:sz(3),
       % if row == 100,
       %     fprintf('\npause point');
       % end
        cImage(:,row, col);
        X = U*cImage(:,row, col);
       % if floorMask(row,col) == 1,
       %     %fprintf('\n%f,%f',X(1),X(2));
       %     toPlot(toPlotCt,:) = [X(1) X(2)];
       %     toPlotCt = toPlotCt + 1;
       % end
        rtn( row, col ) = X(1)*cos(theta_rad) + X(2)*sin(theta_rad);
    end
end


end



function [ rtn ] = getChromaticity( rgbImage )
rgbImage = im2double(rgbImage);
sz = size(rgbImage);
rtn = zeros(3, sz(1), sz(2));

for row = 1:sz(1),
    for col = 1:sz(2),
        r = rgbImage( row, col, 1);
        g = rgbImage( row, col, 2);
        b = rgbImage( row, col, 3);
        denom = ( (r * g * b )^(1/3) );
        if denom == 0,
            c = [r g b] / (denom + 0.0000001);
        else
            c = [r g b] / denom;
        end
        rtn(:,row,col) = c;
    end
end
end

function [ U ] = calcU()

u = (1/sqrt(3)) *[1 1 1]';                                                                                                    
   UtU = eye(3) - u*u';
   [V,D] = eig(UtU);
   U = V(:,2:3)';

end



function [ entropyVal ] = calcMinEntropy( I )
                                  
  
 %  ******APPLY SCOTT'S RULE******
 
   bins_width = 3.5 * std(I(:)) * length(I(:))^(-1/3);
   bins_vector = min(I(:)):bins_width:max(I(:));
       
   [N,x] = hist(I(:),bins_vector);                 
   sum_bin_counts = sum(N);                        
   p_i = N./sum_bin_counts;  
                   
   %ind = find(p_i ~= 0);
   
   p_i = p_i(p_i ~= 0);    % speed up summation by not summing zeros                         
    
 %  ******AND NOW GET ENTROPY*****
   entropyVal = -sum((p_i .* (log2(p_i))));          
   
  
end
