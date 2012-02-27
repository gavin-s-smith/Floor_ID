function buildFullFloor( dirIn, usePostCapturedImages, intoFuture, numIntoFutureOrPast )
%  usePostCapturedImages = 1;
%  intoFuture = 0;
%  numIntoFutureOrPast = 4;
vanishPoints = dir( strcat(dirIn, '/', '*bmp'  ) );


for idx49 = 1:length(vanishPoints),
    for iteration = [ 1 2 ],
        filebase = vanishPoints(idx49).name(1:(length(vanishPoints(idx49).name)-6));
        inImg = imread( strcat(dirIn,'/',filebase,'.png') );
        
        if iteration == 1,
            inLabels = load( strcat(dirIn,'/',filebase,'.flr') );
        else
            inLabels = load( strcat(dirIn,'/',filebase,'.flr2') );
        end
        
        fprintf('\n%s',strcat(dirIn,'/',filebase,'.png'));
        
        % Load the vanishing point and then merge the information with the
        % labels.  This is not done in the c++ code so that a constrast can
        % be made here. The requirement is to add -1 (known not floor
        % points) to all rows above the vpRow.
        strcat(dirIn,'/',filebase,'.van');
        [ va vb vc vd ve ] = processVPFile( strcat(dirIn,'/',filebase,'.van') );
        
        vpRow = round( ve(1) ); % E is in row, col format
        inLabels(1:vpRow,:) = -1;
        
        % Now use the line angles that determined the vanishing point
        % to identify lines that can NOT be on the floor.  Mark these
        % as not floor.
        % Step 1: Determine which line angles can not be floor
        v1 = [ 0 1 ]'; %x, y
        v2 = va - vb;
        v2 = [ v2(2) v2(1) ]'; %va vb were in row, col.  Convert to column x,y
        v3 = vc - vd;
        v3 = [ v3(2) v3(1) ]'; %va vb were in row, col.  Convert to column x,y
        angleRad = ( max(abs(atan2(abs(det([v1,v2])),dot(v1,v2))), abs(atan2(abs(det([v1,v3])),dot(v1,v3)) ) ) );
        angle = angleRad/pi*180;
        
        % Step 2: Detect the line segments
        I = rgb2gray(inImg);
        
        rotI = I;%imrotate(I,33,'crop');
        BW = edge(rotI,'canny',[0,0.35]);
        % imshow(BW)
        [H,T,R] = hough(BW);
        %imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
        %xlabel('\theta'), ylabel('\rho');
        %axis on, axis normal, hold on;
        P  = houghpeaks(H,40,'threshold',ceil(0.03*max(H(:))));
        %x = T(P(:,2)); y = R(P(:,1));
        %plot(x,y,'s','color','white');
        % Find lines and plot them
        lines = houghlines(BW,T,R,P,'FillGap',10,'MinLength',17);
        
        % Step 3: Eval and threshold the line segements
        [ lines floorLines ] = filterEdgesBasedOnSlope( lines, angle );
        
        
        % Step 4: Mark the lines are not part of the floor
        % negLines = zeros( size(inLabels ) );
        if 1 == 0,
        for line = lines,
            % fprintf('here');
            lineIdxs = drawline([line.point1(2) line.point1(1)], [line.point2(2) line.point2(1)], size(inLabels) );
            %  inLabels( lineIdxs ) = -1;
            [xi yi]=ind2sub( size(inLabels), lineIdxs );
            inLabels( sub2ind( size(inLabels), xi,yi) ) = -1;
            inLabels( sub2ind( size(inLabels), xi-1,yi-1) ) = -1;
            inLabels( sub2ind( size(inLabels), xi-1,yi) ) = -1;
            inLabels( sub2ind( size(inLabels), xi,yi-1) ) = -1;
            
            inLabels( sub2ind( size(inLabels), xi+1,yi+1) ) = -1;
            inLabels( sub2ind( size(inLabels), xi+1,yi) ) = -1;
            inLabels( sub2ind( size(inLabels), xi,yi+1) ) = -1;
            
            %  negLines( lineIdxs ) = 1;
        end
        end
        [ corners markedImg ] = corner(I,[],[],[],0.2);
        for line = floorLines,
            lineIdxs = drawline([line.point1(2) line.point1(1)], [line.point2(2) line.point2(1)], size(markedImg));
            markedImg(lineIdxs) = 255;
        end
        imshow(markedImg);
        
        % Add the extra known floor areas from VP corner processing
        if 1 == 0,
            [ corners markedImg ] = corner(I,[],[],[],0.2);
            lineIdxs = drawline([vpRow 0], [vpRow 640], size(markedImg) );
            % lineIdxs = drawline([line.point1(2) line.point1(1)], [line.point2(2) line.point2(1)], size(markedImg));
            markedImg(lineIdxs) = 255;
            
            
            
            for line = floorLines,
                
                %for desk checking
                lineIdxs = drawline([line.point1(2) line.point1(1)], [line.point2(2) line.point2(1)], size(markedImg) );
                %  inLabels( lineIdxs ) = -1;
                [xi yi]=ind2sub( size(markedImg), lineIdxs );
                markedImg( sub2ind( size(markedImg), xi,yi) ) = 1;
                markedImg( sub2ind( size(markedImg), xi-1,yi-1) ) = 1;
                markedImg( sub2ind( size(markedImg), xi-1,yi) ) = 1;
                markedImg( sub2ind( size(markedImg), xi,yi-1) ) = 1;
                
                markedImg( sub2ind( size(markedImg), xi+1,yi+1) ) = 1;
                markedImg( sub2ind( size(markedImg), xi+1,yi) ) = 1;
                markedImg( sub2ind( size(markedImg), xi,yi+1) ) = 1;
                imshow(markedImg);
                % End for desk checking
                
                if line.point1(2) > vpRow && line.point2(2) > vpRow,
                    bb = buildBoundingBox( line );
                    
                    % bb Output: [ lowerLeftRowY, lowerLeftColX, upperRightRowY, upperRightColX]
                    
                    lineIdxs = drawline([bb(1) bb(2)], [bb(3) bb(4)], size(markedImg) );
                    % lineIdxs = drawline([line.point1(2) line.point1(1)], [line.point2(2) line.point2(1)], size(markedImg));
                    markedImg(lineIdxs) = 255;
                    imshow(markedImg);
                    
                    
                    % Check to ensure line is ok.
                    %  fprintf('debug');
                    
                    
                    if getFurtherDepthSupport( vpRow, corners, bb, BW ),
                        
                        % fprintf('here');
                        lineIdxs = drawline([line.point1(2) line.point1(1)], [line.point2(2) line.point2(1)], size(inLabels) );
                        %  inLabels( lineIdxs ) = -1;
                        [xi yi]=ind2sub( size(inLabels), lineIdxs );
                        inLabels( sub2ind( size(inLabels), xi,yi) ) = 1;
                        inLabels( sub2ind( size(inLabels), xi-1,yi-1) ) = 1;
                        inLabels( sub2ind( size(inLabels), xi-1,yi) ) = 1;
                        inLabels( sub2ind( size(inLabels), xi,yi-1) ) = 1;
                        
                        inLabels( sub2ind( size(inLabels), xi+1,yi+1) ) = 1;
                        inLabels( sub2ind( size(inLabels), xi+1,yi) ) = 1;
                        inLabels( sub2ind( size(inLabels), xi,yi+1) ) = 1;
                        
                    end
                    
                end
            end
        end
         overlay(inImg,inLabels);
         fprintf('test');
        
        %   [edgelist, labelededgeim] = edgelink(negLines, 10);
        %   drawedgelist(edgelist, size(negLines), 1, 'rand', 2); axis off
        %     figure, imshow(rotI), hold on
        %     for k = 1:length(lines)
        %        xy = [lines(k).point1; lines(k).point2];
        %        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        
        % Plot beginnings and ends of lines
        %        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        %        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        %     end
        
        
        moreImages = {};
        moreImageMasks = {};
        
        if usePostCapturedImages == 1,
            maxIdx = numIntoFutureOrPast;
            
            for idxAt = 1:maxIdx,
                numbase = str2double(filebase);
                if intoFuture == 1,
                    numbase = numbase + (idxAt*5);
                else
                    numbase = numbase - (idxAt*5);
                end
                
                if length(dir(strcat(dirIn,'/',num2str(numbase, '%4.4i'),'.van') ) ) == 1,
                    filebaseNext = num2str(numbase, '%4.4i');
                    inImgNext = imread( strcat(dirIn,'/',filebaseNext,'.png') );
                    inLabelsNext = load( strcat(dirIn,'/',filebaseNext,'.flr') );
                    strcat(dirIn,'/',filebaseNext,'.van');
                    vanFileNext = textscan(fopen( strcat(dirIn,'/',filebaseNext,'.van') ), '%s');
                    vpRowNext = round( str2double(cell2mat( vanFileNext{1}(24))) );
                    inLabelsNext(1:vpRowNext,:) = -1;
                    
                    
                    %Sanity check
                    %   if sum(size(inImgNext) == [480 640 3]) ~= 3,
                    %       err = MException('BadImage:WrongSize', ...
                    %           'Input image had the wrong size. Bad image was %s', strcat(dirIn,'/',filebaseNext,'.flr') );
                    %       throw(err)
                    %   end
                    %   if sum(size(inLabelsNext) == [480 640]) ~= 2,
                    %       err = MException('BadImage:WrongSize', ...
                    %           'Input mask had the wrong size. Bad image was %s', strcat(dirIn,'/',filebaseNext,'.flr') );
                    %
                    %       throw(err)
                    %   end
                    
                    if sum(size(inImgNext) == [480 640 3]) ~= 3 || sum(size(inLabelsNext) == [480 640]) ~= 2,
                        fprintf( '\nSkipped file (bad image or mask size): %s', strcat(filebaseNext,'.flr/png') );
                    else
                        moreImages{end+1} = inImgNext;
                        moreImageMasks{end+1} = inLabelsNext;
                    end
                else
                    fprintf('\nSkipped file (did not exist): %s\n', num2str(numbase, '%4.4i'));
                end
            end
        end
        
        
        nameOut = strcat(dirIn,'/',filebase,'.flr', '.mat');
        
        % Write out an image that visualizes the segmentation process
        %    nameOutDbg = strcat(dirIn,'/',filebase, '.dbg');
        %    labelImg = vizLabels( inLabels );
        
        % overlay(inImg,labelImg);
        
        
        v=singleSegRun( inImg, inLabels, moreImages, moreImageMasks );
        
        % overlay(inImg,v);
        
        % hold off;
        % Below enables recusive calling - FOR TESTING ONLY at this stage
        t = v;
        inLabels( t == 255 ) = 1;
        csvwrite(strcat(dirIn,'/',filebase,'.flr2'),inLabels)
        
        save(nameOut,'v');
    end
    
end
end

function [bb]=buildBoundingBox( line )
% line is of the form line.point1, line.point2 where a point is
% [x,y]
% bb must be of the form [ lowerLeftRowY, lowerLeftColX, upperRightRowY,
% upperRightColX]

if line.point1(1) < line.point2(1),
    pointLeft = line.point1;
    pointRight = line.point2;
else
    pointLeft = line.point2;
    pointRight = line.point1;
end

bb = [ pointLeft(2) pointLeft(1) 0 pointRight(1) ];


end


