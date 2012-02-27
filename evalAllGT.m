% Model type = [ 'J' 'C' 'R' ]
% Contrast image = [ 'J' 'C' 'R' ]
function [ output_args ] = evalAllGT( dirIn, dirOut, dirGT, usePostCapturedImages, paperMethod, gcGamma, keepMaxNPts )

useFloor = 1;

outputbase = strcat(paperMethod, '_',num2str(gcGamma),'_',num2str(keepMaxNPts) );
if usePostCapturedImages == 1,
    outputbase = strcat(outputbase, 'P');
end

gtFiles = dir( strcat(dirGT, '/', '*.bmp'  ) );
overlapTotal = [];
overSegTotal = [];
overlapScoreTotal = [];
floor3DScoreTotal = [];
fileNameSet = [];
    for idx49 = 1:length(gtFiles),
         filebase = gtFiles(idx49).name(1:(length(gtFiles(idx49).name)-6));
          if useFloor == 1,
                segMaskFilename = strcat( dirIn,'/',filebase, '.flr.mat' ); % What is created by the segmentation (only for those in the trajectory)
                if ~exist(segMaskFilename), 
                    continue;
                end
          else
                % we need to use the mask from the xyz image
                segMaskFilename = strcat( dirIn,'/',filebase, '.xyz.flr' );
          end
          GTFilename = strcat(dirGT,'/',gtFiles(idx49).name);
          %fprintf('\nRunning:evalSegGT(''%s'',''%s'') \n', segMaskFilename, GTFilename);
          
          [ overlapAsPercentOfGT, overSegAsPercentOfGT overlapScore floor3DScore ] = evalSegGT( segMaskFilename, GTFilename ); %simply write the result to the terminal
          overlapTotal = [overlapTotal overlapAsPercentOfGT];
          overSegTotal = [overSegTotal overSegAsPercentOfGT];
          overlapScoreTotal = [overlapScoreTotal overlapScore];
          fileNameSet = [fileNameSet str2double(filebase) ];
          floor3DScoreTotal = [floor3DScoreTotal floor3DScore];
          
    end
    
    
    if isempty(fileNameSet),
        overlapTotal =  0; 
        overSegTotal = 0;
        overlapScoreTotal = 0;
        fileNameSet = -1;
        floor3DScoreTotal = 0;
    end
        

    outNameCSV = strcat( dirOut, '/',outputbase, '.csv');

    fidCSV = fopen( outNameCSV, 'w' ); 
    
    for idx = 1:length( fileNameSet ),
        if idx <= length(fileNameSet),
            fprintf(fidCSV, '%4.0d,%f,%f,%f,%f\n',fileNameSet(idx), overlapTotal(idx), overSegTotal(idx),overlapScoreTotal(idx),floor3DScoreTotal(idx) ); 
    
        end
    end

    
    fprintf( '\n %s eval completed.\n', outNameCSV );
    
end

