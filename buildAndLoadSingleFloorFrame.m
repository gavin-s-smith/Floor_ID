function [ rtn ] = buildAndLoadSingleFloorFrame( dirIn, dataDir, filenumber, maxDepth, reprocess,restrictNumPoints, keepCtBasedOnFloorPts,keepMaxNPts )
%UNTITLED2 Summary of this function goes here
%   dir in the imgs directory, dataDir is the parent of the viewXXXX
%   directories

% Build the .flr and .van via the c++ program
thisDir = pwd;
cd(dirIn);
cmd = strcat(thisDir,'/cpp/build/bin/SingleFrameExtraction -f',' ', dataDir, '/view', sprintf('%04i',str2num(filenumber)), ' -m', ' ', num2str(maxDepth), ' -r', ' ', reprocess, ' -p', ' ', restrictNumPoints, ' -b', ' ', keepCtBasedOnFloorPts, ' -k', ' ', num2str(keepMaxNPts) )
system( cmd );
cd(thisDir);
% Load the .flr
doesExist = exist(strcat( dirIn, '/', filenumber, '.flr'));
doesExistVan = exist(strcat( dirIn, '/', filenumber, '.van'));
if doesExist == 2 && doesExistVan == 2,
    rtn = load( strcat( dirIn, '/', filenumber, '.flr') );
else
   rtn = -1; 
end

end

