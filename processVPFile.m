function [ a b c d e ] = processVPFile( vpFile )

vpFile
 vanFile = textscan(fopen( vpFile ), '%s');
 
 a = [ 0 0 ];
 b = [ 0 0 ];
 c = [ 0 0 ];
 d = [ 0 0 ];
 e = [ 0 0 ];
 
 for i = [7 11 15 19 23],
     
     if strcmp(cell2mat(vanFile{1}(i)),  'a=['),
         a = [ str2double(cell2mat(vanFile{1}(i+1))) str2double(cell2mat(vanFile{1}(i+2))) ];
     end
     
     if strcmp(cell2mat(vanFile{1}(i)), 'b=['),
         b = [ str2double(cell2mat(vanFile{1}(i+1))) str2double(cell2mat(vanFile{1}(i+2))) ];
     end
     
     if strcmp(cell2mat(vanFile{1}(i)), 'c=['),
         c = [ str2double(cell2mat(vanFile{1}(i+1))) str2double(cell2mat(vanFile{1}(i+2))) ];
     end
     
     if strcmp(cell2mat(vanFile{1}(i)), 'd=['),
         d = [ str2double(cell2mat(vanFile{1}(i+1))) str2double(cell2mat(vanFile{1}(i+2))) ];
     end
     
     if strcmp(cell2mat(vanFile{1}(i)), 'e=['),
         e = [ str2double(cell2mat(vanFile{1}(i+1))) str2double(cell2mat(vanFile{1}(i+2))) ];
     end


 end


