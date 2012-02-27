 function buildFullFloorMasks( dirIn )
    imgFiles = dir( strcat(dirIn, '/', '*.png'  ) );
    labelFiles = dir( strcat(dirIn, '/', '*.flr'  ) );
    
    for i = 1:len(dirIn),
        inImg = imread( strcat(dirIn,'/',imgFiles(i).name) );
        inLabels = load( strcat(dirIn,'/',labelFiles(i).name) );
        nameOut = strcat(dirIn,'/',labelFiles(i).name, '.msk');
        save(nameOut,singleSegRun( inImg, inLabels ));
    end