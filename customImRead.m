function [ image ] = customImRead( filename )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
omitEdgePixels = 0;%10;

% Hack to ensure that if the colour map in a bmp is wrong there is no effect

if strcmp(filename(length(filename)-3:length(filename)), '.bmp') == 1,

    filenamePBM = strcat(filename(1:(length(filename)-4) ),'.PBM'); 

    system( ['convert ' filename ' ' filenamePBM] );
    
    filename = filenamePBM;
end
    
image = imread( filename );

%image = image(omitEdgePixels:(480-omitEdgePixels), omitEdgePixels:(640-omitEdgePixels),:);

end


