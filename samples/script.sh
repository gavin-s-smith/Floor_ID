r=`pwd`
viewDir="/home/gavin/Floor_ID/samples/rgbd-demo-data"
outputDir="/home/gavin/Floor_ID/samples/output/imgs"
cd $viewDir
for file in *; do
   if [ -d $file ]; then
      fileShort=${file:(-4)}
      cp $viewDir/$file/raw/color.png $outputDir/${fileShort:`expr match "$fileShort" '[0]*'`}.png
   fi
done
cd $r
