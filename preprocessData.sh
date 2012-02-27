r=`pwd`
viewDir="/home/gavin/data/ngb7s"
outputDir="/home/gavin/data/ngb7s_output/imgs"
mkdir /home/gavin/data/ngb7s_output/output
mkdir -p $outputDir
cd $viewDir
for file in *; do
   if [ -d $file ]; then
      fileShort=${file:(-4)}
      cp $viewDir/$file/raw/color.png $outputDir/${fileShort:`expr match "$fileShort" '[0]*'`}.png
   fi
done
cd $r
