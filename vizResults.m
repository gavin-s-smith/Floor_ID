 function vizResults( dirIn, imgNum )
     
    

        inImg = customImRead( strcat(dirIn,'/',imgNum,'.png') );
        load( strcat(dirIn,'/',imgNum,'.flr.mat') );
        inLabels = load( strcat(dirIn,'/',imgNum,'.flr') );
        labelImg = vizLabels( inLabels );
       
      %  inImg = inImg(1:475,:,:);
      %  v = v(1:475,:);
      %  labelImg = labelImg(1:475,:);
        
        overlay3(inImg,v, labelImg);
        figure
        overlay(inImg,v);
        figure
        overlay3(inImg,v,v);
        
      
