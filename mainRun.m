%matlabpool local 4
for useFloor = 1,
    for keepMaxNPts = 20000,%;[8000 10000 15000 25000],%[20000 30000 40000 50000],% 60000 100000],%2:10,
        parfor i = 1:14,%1:6,%1:3,

            if i == 1,

                % --------------------------
                % ------- CS A Run ---------
                % --------------------------
                
                dirOut = '/home/gavin/data/cs_a_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/cs_a_output/imgs';
                dirGT = '/home/gavin/data/cs_a_output/GTAll';
                dataDir = '/home/gavin/data/CS_24Nov2011A';
               % moveFile = '/home/gavin/data/cs_a_output/base/moveFile.csv';

            elseif i == 2,
                % --------------------------
                % ------- CS B Run ---------
                % --------------------------

                dirOut = '/home/gavin/data/cs_b_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/cs_b_output/imgs';
                dirGT = '/home/gavin/data/cs_b_output/GTAll';
                dataDir = '/home/gavin/data/CS_24Nov2011B';
              %  moveFile = '/home/gavin/data/cs_b_output/base/moveFile.csv';

            elseif i == 10,
                % --------------------------
                % ------- NGB Run ----------
                % --------------------------

                dirOut = '/home/gavin/data/ngb_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/ngb_output/imgs';
                dirGT = '/home/gavin/data/ngb_output/GTAll';
                dataDir = '/home/gavin/data/NGB_C_FLoor_24Nov2011';
              %  moveFile = '/home/gavin/data/ngb_output/base/moveFile.csv';
              
            elseif i == 8,
               % --------------------------
                % ------- SCC Run ----------
                % --------------------------

                dirOut = '/home/gavin/data/scc_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/scc_output/imgs';
                dirGT = '/home/gavin/data/scc_output/GTAll';
                dataDir = '/home/gavin/data/grabSCC';
                
            elseif i == 5,
               % --------------------------
                % ------- NGB downstairs Run ----------
                % --------------------------

                dirOut = '/home/gavin/data/ngbe_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/ngbe_output/imgs';
                dirGT = '/home/gavin/data/ngbe_output/GTAll';
                dataDir = '/home/gavin/data/grabNGB';
                
           elseif i == 6,
               % --------------------------
                % ------- CS natural light Run ----------
                % --------------------------

                dirOut = '/home/gavin/data/cs_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/cs_output/imgs';
                dirGT = '/home/gavin/data/cs_output/GTAll';
                dataDir = '/home/gavin/data/grabCS';
                
           elseif i == 15,
               % --------------------------
                % ------- texk ----------
                % --------------------------

                dirOut = '/home/gavin/data/texk_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/texk_output/imgs';
                dirGT = '/home/gavin/data/texk_output/GTAll';
                dataDir = '/home/gavin/data/texk';
                
          elseif i == 4,
               % --------------------------
                % ------- texs ----------
                % --------------------------

                dirOut = '/home/gavin/data/texs_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/texs_output/imgs';
                dirGT = '/home/gavin/data/texs_output/GTAll';
                dataDir = '/home/gavin/data/texs';
                
                
         elseif i == 16,
               % --------------------------
                % ------- ngb7k ----------
                % --------------------------

                dirOut = '/home/gavin/data/ngb7k_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/ngb7k_output/imgs';
                dirGT = '/home/gavin/data/ngb7k_output/GTAll';
                dataDir = '/home/gavin/data/ngb7k';
                
         elseif i == 3,
               % --------------------------
                % ------- ngb7s ----------
                % --------------------------

                dirOut = '/home/gavin/data/ngb7s_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/ngb7s_output/imgs';
                dirGT = '/home/gavin/data/ngb7s_output/GTAll';
                dataDir = '/home/gavin/data/ngb7s';
              
                
         elseif i == 11,
               % --------------------------
                % ------- ngb4 ----------
                % --------------------------

                dirOut = '/home/gavin/data/ngb4_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/ngb4_output/imgs';
                dirGT = '/home/gavin/data/ngb4_output/GTAll';
                dataDir = '/home/gavin/data/ngb4';
                
                
        elseif i == 12,
               % --------------------------
                % ------- ngb5 ----------
                % --------------------------

                dirOut = '/home/gavin/data/ngb5_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/ngb5_output/imgs';
                dirGT = '/home/gavin/data/ngb5_output/GTAll';
                dataDir = '/home/gavin/data/ngb5';
           
                
        elseif i == 13,
               % --------------------------
                % ------- scc2 ----------
                % --------------------------

                dirOut = '/home/gavin/data/scc2_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/scc2_output/imgs';
                dirGT = '/home/gavin/data/scc2_output/GTAll';
                dataDir = '/home/gavin/data/scc2';
                
                
         elseif i == 14,
               % --------------------------
                % ------- sccc1 ----------
                % --------------------------

                dirOut = '/home/gavin/data/sccc1_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/sccc1_output/imgs';
                dirGT = '/home/gavin/data/sccc1_output/GTAll';
                dataDir = '/home/gavin/data/sccc1';
                
           elseif i == 7,
               % --------------------------
                % ------- sccc2 ----------
                % --------------------------

                dirOut = '/home/gavin/data/sccc2_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/sccc2_output/imgs';
                dirGT = '/home/gavin/data/sccc2_output/GTAll';
                dataDir = '/home/gavin/data/sccc2';     
                
                
          elseif i == 9,
               % --------------------------
                % ------- sccc3 ----------
                % --------------------------

                dirOut = '/home/gavin/data/sccc3_output/output3';
                mkdir(dirOut);
                dirIn = '/home/gavin/data/sccc3_output/imgs';
                dirGT = '/home/gavin/data/sccc3_output/GTAll';
                dataDir = '/home/gavin/data/sccc3';  
                
            end


                for usePostCapturedImages = 0,% 0:1, %5 with a 5 skip
                    allMethods ={ 'AMCI'; 'CMCIU'; 'JAMJAIU';   'IMCIU'; 'JAMJAIU';'ZMVIU';'ZMCIU' ;'RMRIU'}; 
                    for paperMethodIdx = 1,
                        paperMethod = allMethods{paperMethodIdx};
                        for gcGamma = 500,% [500 150],

                            buildFullFloorAlt2( dataDir, dirIn, dirGT, usePostCapturedImages, paperMethod, gcGamma, keepMaxNPts );
                            evalAllGT( dirIn, dirOut, dirGT, usePostCapturedImages, paperMethod, gcGamma, keepMaxNPts );
                        end
                    end
                end
           
            
            
          

        end
    end

end
matlabpool close