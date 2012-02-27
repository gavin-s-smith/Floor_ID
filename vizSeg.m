 function vizSeg( dirIn, imgNum )
 
        usePostCapturedImages = 1;
    

        filebase = imgNum;
        inImg = imread( strcat(dirIn,'/',filebase,'.png') );
        inLabels = load( strcat(dirIn,'/',filebase,'.flr') );
        
        
        
        % Load the vanishing point and then merge the information with the
        % labels.  This is not done in the c++ code so that a constrast can
        % be made here. The requirement is to add -1 (known not floor
        % points) to all rows above the vpRow.
        strcat(dirIn,'/',filebase,'.van')
        vanFile = textscan(fopen( strcat(dirIn,'/',filebase,'.van') ), '%s');
        vpRow = round( str2double(cell2mat( vanFile{1}(24))) );
        inLabels(1:vpRow,:) = -1;
        
        moreImages = {};
        moreImageMasks = {};
        
        if usePostCapturedImages == 1,
            maxIdx = 10;
           
            for idxAt = 1:maxIdx,
                numbase = str2double(filebase);
                numbase = numbase + idxAt;
                
                if length(dir(strcat(dirIn,'/',num2str(numbase, '%4.4i'),'.van') ) ) == 1,
                    filebaseNext = num2str(numbase, '%4.4i');
                    inImgNext = imread( strcat(dirIn,'/',filebaseNext,'.png') );
                    inLabelsNext = load( strcat(dirIn,'/',filebaseNext,'.flr') );
                    strcat(dirIn,'/',filebaseNext,'.van')
                    vanFileNext = textscan(fopen( strcat(dirIn,'/',filebaseNext,'.van') ), '%s');
                    vpRowNext = round( str2double(cell2mat( vanFileNext{1}(24))) );
                    inLabelsNext(1:vpRowNext,:) = -1;
                    moreImages{end+1} = inImgNext;
                    moreImageMasks{end+1} = inLabelsNext;
                else
                    fprintf('\nSkipped file: %s\n', num2str(numbase, '%4.4i'));
                end
            end
        end
        
        
        nameOut = strcat(dirIn,'/',filebase,'.flr', '.mat');
        
        % Write out an image that visualizes the segmentation process
        nameOutDbg = strcat(dirIn,'/',filebase, '.dbg');
        labelImg = vizLabels( inLabels );
        
        overlay(inImg,labelImg);
        
        
        v=singleSegRun( inImg, inLabels, moreImages, moreImageMasks );
        
        overlay(inImg,v);
        
        
        
      %  save(nameOut,'v');
      