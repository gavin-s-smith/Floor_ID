function [ numberPart ] = demo( dataViewDir, dirOut )

% Set up the paths
cd 'gsc-1.2'
setup
cd ..

% Set the input directory
%dataDir = strcat(pwd,'/samples/rgbd-demo-data');

% Create the output directory
%dirOut = strcat(pwd,'/samples/output');
mkdir(dirOut);

% Clear the previous image directory
system(['rm -r ',dirOut,'/imgs']);

% Create the images directory (TODO: make this step unnecessary)
imgDir = strcat(dirOut,'/imgs');
mkdir(imgDir)

% Move image files and name correctly via external script (Linux)
% TODO: make this step unnecessary
% Create the script
% fid = fopen( strcat(pwd,'/samples/script.sh'), 'w' );
% fprintf(fid, 'r=`pwd`\n');
% fprintf(fid, strcat('viewDir="',dataDir,'"\n'));
% fprintf(fid, strcat('outputDir="',imgDir,'"\n'));
% fprintf(fid, 'cd $viewDir\n');
% fprintf(fid, 'for file in *; do\n');
% fprintf(fid, '   if [ -d $file ]; then\n');
% fprintf(fid, '      fileShort=${file:(-4)}\n');
% fprintf(fid, '      cp $viewDir/$file/raw/color.png $outputDir/${fileShort:`expr match "$fileShort" ''[0]*''`}.png\n');
% fprintf(fid, '   fi\n');
% fprintf(fid, 'done\n');
% fprintf(fid, 'cd $r\n');
% fclose(fid);

% Since we are only processing one view directory but the scripts are setup
% to process multiple at once do a little hack

parent = dataViewDir(1:length(dataViewDir)-8);
viewPart = dataViewDir(length(dataViewDir)-7:length(dataViewDir));
numberPart = str2num(dataViewDir(length(dataViewDir)-3:length(dataViewDir)));
system(['cp ', dataViewDir,'/raw/color.png ',dirOut,'/imgs/',num2str(numberPart),'.png']); 

% Parameters
paperMethod = 'AMCI';
gcGamma = 500;
usePostCapturedImages = 0;
keepMaxNPts = 20000;

buildFullFloorAlt2( parent, imgDir, 'none', usePostCapturedImages, paperMethod, gcGamma, keepMaxNPts );