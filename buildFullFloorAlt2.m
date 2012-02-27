function buildFullFloorAlt2( dataDir, dirIn, dirGT, usePostCapturedImages, paperMethod, gcGamma, keepMaxNPts )


% dataDir = parent containing the viewXXXX files
intoFuture = 0;
numIntoFutureOrPast = 5;
skipMultiplier = 5;
initalAngleInDegs = 90;
%initalAngleInDegs = -1; % To automatically determine the angle in degree.
%Should be fixed for each camera in theory.
angleSet = [];% [40 172 46 62 68 84 76 72 57 83 30 60 68 70 69 148 1 12 108 24];
normalizeInvarient = 1; % false seems better here


reprocess = 'TRUE'; % Reindentify floor and vanishing point if .flr and .van files exist?
restrictNumPoints = 'TRUE';
keepCtBasedOnFloorPts = 'TRUE';
maxDepth = 1000;

% Load the list of files that will be evaluated.  Store in variable
% moveList
gtFiles = dir( strcat(dirGT, '/', '*.bmp'  ) );
moveList = zeros(length(gtFiles));
for idx49 = 1:length(gtFiles),
    filebase = gtFiles(idx49).name(1:(length(gtFiles(idx49).name)-6));
    moveList(idx49) = str2double(filebase);
end


% Load the image sequence and sort correctly.
% Image sequence is stored in variable 
% filebaseList
allImgs = dir( strcat(dirIn, '/', '*.png'  ) );
allImgsN = zeros(size(allImgs,1));
for i = 1:size(allImgs,1),
    filebase = allImgs(i).name(1:(length(allImgs(i).name)-4));
    allImgsN(i) = str2double(filebase); 
end
allImgsN = sort(allImgsN);
filebaseList = allImgsN;

% ---------------------------------------------------------------
% ---------- Process each individual image in moveList ----------
% --- (The images for which we have ground truth information) ---
% ---------------------------------------------------------------

for idx49 = 1:length(moveList),
    
    % Set the angle for calculating Invar(I) (fixed or from a set)
    angleInDegs = initalAngleInDegs;
    if length(angleSet) == length(moveList),
        angleInDegs = angleSet(idx49);
    end
    
    
    filebase = num2str( moveList(idx49) ); % The file being processed.
    
    if strcmp( reprocess, 'TRUE'),
        delete(strcat( dirIn, '/', filebase, '.flr'));
        delete(strcat( dirIn, '/', filebase, '.flr.mat'));
        delete(strcat( dirIn, '/', filebase, '.van'));
    end



    filebase = num2str( moveList(idx49) );
    inImg = customImRead( strcat(dirIn,'/',filebase,'.png') );

    % Get the floor/non-floor mask
    inLabels = buildAndLoadSingleFloorFrame( dirIn, dataDir, filebase, maxDepth, reprocess,restrictNumPoints, keepCtBasedOnFloorPts,keepMaxNPts  );

    if length(inLabels) == 1,
        fprintf('\nSkipping this frame.  No .flr file has been made.\n');
        continue;
    end
            
    % Print the file being processed to the console    
    fprintf('\nIdentifying floor in image: %s',strcat(dirIn,'/',filebase,'.png'));

    % Load the vanishing point and then merge the information with the
    % labels.  This is not done in the c++ code so that a constrast can
    % be made here. The requirement is to add -1 (known not floor
    % points) to all rows above the vpRow.
    strcat(dirIn,'/',filebase,'.van');
    [ ~ , ~ , ~ , ~, ve ] = processVPFile( strcat(dirIn,'/',filebase,'.van') );

    vpRow = round( ve(1) ); % E is in row, col format
    inLabels(1:vpRow,:) = -1;


    % Add the images that will train the gmms and identify their
    % correpsonding floor and non-floor regions.
    moreImages = {};
    moreImageMasks = {};
    moreImagesInv = {};
    moreImageMasksInv = {};

    % If we need to add a whole lot of images to t
    if usePostCapturedImages == 1,
        maxIdx = numIntoFutureOrPast;

        for idxAt = 1:maxIdx,
           % numbase = str2double(filebase);
            if intoFuture == 1,
                %numbase = moves(str2double( ( find(filebaseList == str2double(filebase)) + (idxAt*skipMultiplier) )) );
                 numbase = filebaseList( ( find(filebaseList == str2double(filebase)) + (idxAt*skipMultiplier) ) );
            else
              tIdx = find(filebaseList == str2double(filebase)) - (idxAt*skipMultiplier);  

              %tIdx = str2double(filebase) - (idxAt*skipMultiplier);
              if tIdx <=0,
                  numbase = tIdx;
              else
                numbase =  filebaseList( tIdx );
              end
            end

            if numbase >0 && length(dir(strcat(dirIn,'/',num2str(numbase),'.png') ) ) == 1,
                filebaseNext = num2str(numbase);
                inImgNext = customImRead( strcat(dirIn,'/',filebaseNext,'.png') );
                %inLabelsNext = customLoad( strcat(dirIn,'/',filebaseNext,'.flr') );
                inLabelsNext = buildAndLoadSingleFloorFrame( dirIn, dataDir, filebaseNext, maxDepth, reprocess,restrictNumPoints, keepCtBasedOnFloorPts,keepMaxNPts  );

                if length(inLabelsNext) ~= 1,
                    strcat(dirIn,'/',filebaseNext,'.van');
                    vanFileNext = textscan(fopen( strcat(dirIn,'/',filebaseNext,'.van') ), '%s');
                    vpRowNext = round( str2double(cell2mat( vanFileNext{1}(24))) );
                    inLabelsNext(1:vpRowNext,:) = -1;

                    if sum(size(inImgNext) == [480 640 3]) ~= 3 || sum(size(inLabelsNext) == [480 640]) ~= 2,
                        fprintf( '\nSkipped file (bad image or mask size): %s', strcat(filebaseNext,'.flr/png') );
                    else
                        moreImages{end+1} = inImgNext;
                        moreImageMasks{end+1} = inLabelsNext;
                        moreImageMasksInv{end+1} = inLabelsNext;
                        [ angleInDegs invarientImg ] = removeShadows(inImg,inLabels ,angleInDegs);
                        moreImagesInv{end+1} = invarientImg;
                    end
                else 
                     fprintf( '\nSkipped file (could not find floor): %s', strcat(filebaseNext,'.flr/png') );
                end
            else
                fprintf('\nSkipped file (did not exist): %s\n', num2str(numbase, '%4.4i'));
            end
        end
    end

    % The filename to write the indentified floor to
    nameOut = strcat(dirIn,'/',filebase,'.flr', '.mat');


    % calculate the information for all segmentation model types in all
    % case for now to prevent copy paste errors in the codebase

    % Calcluate Invar( I )
    [ angleInDegs invarientImg ] = removeShadows(inImg, inLabels,angleInDegs);
    fprintf('\nAngle In Degrees: %d', angleInDegs);
    if normalizeInvarient == 1,
        invarientImg = mat2gray(invarientImg);
    end
    % Calc P( I | Invar( I ) )
    [ ~, invarPosterior, ~, ~] = singleSegRun( invarientImg, inLabels, {}, {}, gcGamma, -1);

    % Calc P( I | I.x )
    [ ~, vizPosterior, ~, ~ ] = singleSegRun( inImg, inLabels, moreImages, moreImageMasks, gcGamma, -1 );


   if strcmp( paperMethod, 'JMJIU'),
           pixelModel = max(vizPosterior,invarPosterior);
           contrastImage = max(vizPosterior,invarPosterior);
   elseif strcmp( paperMethod, 'CMCIU' ),
           pixelModel = vizPosterior;
           contrastImage = inImg;
   elseif strcmp( paperMethod, 'RMRIU' ),
           pixelModel = invarPosterior;
           contrastImage = invarientImg;
    elseif strcmp( paperMethod, 'CMVIU' ),
           pixelModel = vizPosterior;
           contrastImage = vizPosterior;
    elseif strcmp( paperMethod, 'JAMJAIU' ),
           pixelModel = ( vizPosterior + invarPosterior ) / 2;
           contrastImage = ( vizPosterior + invarPosterior ) / 2;
     elseif strcmp( paperMethod, 'ZMVIU' ),
           pixelModel = zeros(480,640)+0.95;
           contrastImage = vizPosterior;
     elseif strcmp( paperMethod, 'ZMCIU' ),
           pixelModel = zeros(480,640)+0.05;
           contrastImage = inImg;   
           
     elseif strcmp( paperMethod, 'IMCIU' ),
           pixelModel = invarPosterior;
           contrastImage = inImg; 
           
     elseif strcmp( paperMethod, 'AMCI' ),
           pixelModel = ( vizPosterior + invarPosterior ) / 2;
           contrastImage = inImg; 
           
   end

    v = singleSegRun(contrastImage , inLabels, {}, {}, gcGamma, pixelModel);

    save(nameOut,'v');
end
    

end


