/**
 * This file uses parts of the rgbdemo project by Nicolas Burrus <nicolas.burrus@uc3m.es>.
 *
 * Author: Gavin Smith <gavin.smith@nottingham.ac.uk>
 */

#include <ntk/ntk.h>
#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>
#include <ntk/stats/histogram.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/eigen_utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>
#include <ntk/mesh/pcl_utils.h>
#include <ntk/geometry/pose_3d.h>

#include <ntk/mesh/pcl_utils.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <math.h>
#include <fstream>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <iomanip>
#include <stdexcept>

#include "/usr/local/include/opencv2/core/types_c.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "CommonFunctions.cpp"
#include "XYZFloorIDInc.cpp"
#include "IDVanPoint.cpp"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;

using namespace pcl;
using cv::Vec3f;
using cv::Point3f;

using namespace std;
using namespace boost;
using namespace ntk;

/**
 * A structure representing a point cloud and the pose it was aquired at
 */
struct dataset{

	PointCloud<PointXYZRGB> cloud;
	Pose3D pose;

};

/**
 * Load a point cloud from a image directory as recorded by the RGBD-Demo application
 * 
 * calibrationFile The Kinect calibration as created by the RGBD-Demo application
 * imageDir The image directory containing the data as created by the RGBD-Demo application
 * maxDepth The maximum depth, after which points are not considered
 * 
 * return A struct containing a point cloud and the pose it was aquired at
 */
dataset loadData( std::string calibrationFile, std::string imageDir, float maxDepth ) {


	//set up the RGBD processor so to process the individual colour and depth streams
	ntk::RGBDProcessor* processor = 0;

	processor = new ntk::OpenniRGBDProcessor();
	//processor = new ntk::FreenectRGBDProcessor();

	ntk::RGBDCalibration* calib_data = 0;

	calib_data = new ntk::RGBDCalibration();
	// calib_data->loadFromFile("/home/gavin/Desktop/run2/calibration.yml");
	calib_data->loadFromFile( calibrationFile.c_str() );


	//load the images
	RGBDImage image;
	image.loadFromDir( imageDir, calib_data);


	//post-process the images to fill in the rgb and depth channels 
	processor->processImage(image);


	Pose3D pose = *image.calibration()->rgb_pose;

	PointCloud<PointXYZRGB> input_cloud;

	input_cloud = rgbdImageToPointCloud(image, pose, maxDepth );

	dataset rtn;
	rtn.cloud = input_cloud;
	rtn.pose = pose;
	return rtn;

}


float convertToFloat(std::string const& s)
{
	std::istringstream i(s);
	float x;
	if (!(i >> x)){
		cout << "convertToFloat(\"" + s + "\") Exiting.";
		exit(-1);
	}
	return x;
} 


//For arguments see in code though Boost options declaration
int main (int argc, char** argv)
{
	bool showDebug = false;
	// Hard coded calibration file location.  TODO: Make
	std::string calib_file = "/home/gavin/SpiderOak/git/rgbdemo/build/calibration.yml";

	// Hard coded default, will override with commandline options
	std::string dataFolder = "/home/gavin/source/floorIdentifier/build/bin/grabNGB/view0545";

	// Hard coded paramters
	float max_dist_and_still_part_of_floor = 0.05; // Currently not used.  TODO: remove
	float min_dist_before_considered_not_part_of_floor = 0.1; //Currently not used. TODO: remove

	float maxDepth = 10.0; // Default, able to be changed via the commandline options
	// Defines the maximum depth for which to consider 3D points

	bool reprocess = FALSE; // Default.  Able to be changed via the commandline options.
	// If the processing seems to be done (label file exists)
	// then do not process (FALSE) or process anyway (TRUE)

	bool restrictNumPoints = true; // Default.  Able to be changed via the commandline options.
	// restrictNumPoints Should we restrict the labeled points to a fixed number
	// of pixels?

	bool keepCtBasedOnFloorPts = false; // Default.  Able to be changed via the commandline options.
	// keepCtBasedOnFloorPts Valid only when restrictNumPoints == True.  Should 
	// we restrict the labeled points based on a count over
	// all points, or just floor points?

	size_t keepMaxNPts = 1000000; // Default.  Able to be changed via the commandline options.
	// Valid only when restrictNumPoints == True.  The number of points
	// to restrict the labelling to.

	//declare the commandline options
	po::options_description od("Available options"); // 1.
	od.add_options()
    				("help,h", "help message") // 2.
    				("file,f", po::value<string>(), "input file")
    				("directory,d", po::value<string>(), "input directory")
    				("maxDepth,m", po::value<string>(), "Maximum depth value. Default = 10.0")
    				("restrictNumPoints,p", po::value<string>(), "")
    				("keepCtBasedOnFloorPts,b", po::value<string>(), "")
    				("keepMaxNPts,k", po::value<int>(), "")
    				("reprocess,r", po::value<string>(), "Boolean (TRUE/FALSE) indicating whether to redo the floor identification and vanishing point detection if the .flr and .van files exist. Default = FALSE"); // 3.



	//process the commandline options
	po::variables_map vm;
	try
	{
		po::store(po::parse_command_line(argc, argv, od), vm);
	}
	catch(const std::exception& ex)
	{
		std::cout << "Error checking program options: " << ex.what() << std::endl;
		std::cout << od << std::endl;
		return 1;
	}
	po::notify(vm);


	//check for certain options
	if(vm.count("help"))
	{
		std::cout << od << std::endl;
		return 0;
	}


	//fill the vector to process
	std::vector<std::string> dirs = std::vector<std::string>();


	if(vm.count("file"))
	{

		//note in this case the filename is actually a folder
		std::string filename = boost::filesystem::path( vm["file"].as<string>() ).stem().string();
		dataFolder = boost::filesystem::path( vm["file"].as<string>() ).parent_path().string();//.stem().string(); // .stem to remove extention (before .string())

		if( showDebug ) printf( "String was: %s", filename.c_str() );
		dirs.push_back( filename );

	} else if (vm.count("directory")){
		dataFolder = boost::filesystem::path( vm["file"].as<string>() ).parent_path().string();//.stem().string(); // .stem to remove extention (before .string())

		getdir( vm["directory"].as<string>(), dirs );
	}

	if(vm.count("maxDepth")){
		maxDepth = convertToFloat(vm["maxDepth"].as<string>());
		if( showDebug ) printf("\nSetting max depth to %f\n", maxDepth);
	}

	if(vm.count("keepMaxNPts")){
		keepMaxNPts = vm["keepMaxNPts"].as<int>();
		if( showDebug ) printf("\nSetting keepMaxNPts to %i\n", (int)keepMaxNPts);
	}

	if(vm.count("reprocess")){
		if (vm["reprocess"].as<string>().compare("TRUE") != 0){
			reprocess = false;
			if( showDebug ) printf("\nReprocessing? FALSE");
		}else{
			reprocess = true;
			if( showDebug ) printf("\nReprocessing? TRUE");
		}



	}

	if(vm.count("restrictNumPoints")){
		if (vm["restrictNumPoints"].as<string>().compare("TRUE") != 0){
			restrictNumPoints = false;
			if( showDebug ) printf("\nrestrictNumPoints? FALSE");
		}else{
			restrictNumPoints = true;
			if( showDebug ) printf("\nrestrictNumPoints? TRUE");
		}



	}

	if(vm.count("keepCtBasedOnFloorPts")){
		if (vm["keepCtBasedOnFloorPts"].as<string>().compare("TRUE") != 0){
			keepCtBasedOnFloorPts = false;
			if( showDebug ) printf("\nkeepCtBasedOnFloorPts? FALSE");
		}else{
			keepCtBasedOnFloorPts = true;
			if( showDebug ) printf("\nkeepCtBasedOnFloorPts? TRUE");
		}



	}

	//Processing commandline options is now complete.  Start real code....

	int ii;

// For each frame create the (non)floor label file.  This can be done in parallel
#pragma omp parallel for
	for(ii=0; ii < dirs.size(); ii++)
	{

		size_t found;
		std::string  dataDir = dirs[ii];	
		found=dataDir.find("view");
		if (found!=std::string::npos){

			if( showDebug )	cout << "\nData directory :" << dataDir;
			//Determine the filename as a number (the dir less the word "view")
			std::stringstream ss;//create a stringstream
			ss << atoi( dataDir.substr( 4, dataDir.length() ).c_str() );//add number to the stream
			std::string filenameBase = ss.str();


			std::stringstream ssA;//create a stringstream
			ssA << dataFolder << "/" << dataDir;//add number to the stream
			std::string fullDataPath = ssA.str();   

			if( showDebug ) cout << "\nBase Filename: " << filenameBase << " Full path: " << fullDataPath << "\n";



			std::stringstream ss3;//create a stringstream
			ss3 << filenameBase << ".flr";//add to the floor.  This means the extention is .xyz.flr indicating the floor mask of the xyz image
			std::string filePathOut = ss3.str();


			if( !reprocess && fexists(filePathOut.c_str() ) ) {
				printf( "\nSkipping file %s as already processed.\n", filePathOut.c_str() );
				continue;
			}

			// load the frame (generate the PCL point cloud)
			dataset data = loadData( calib_file, fullDataPath, maxDepth );

			// create the floor mask and write it out to disk
			// return value indicates success or failure
			bool correct = genFloorMask( data.cloud, filePathOut, max_dist_and_still_part_of_floor, min_dist_before_considered_not_part_of_floor, maxDepth, false, restrictNumPoints,  keepCtBasedOnFloorPts,  keepMaxNPts);

			// Determine the vanishing point.  Used to mark anything above this point
			// in the image as non-floor.
			determineVanishingPoint( data.pose, filePathOut, data.cloud, filenameBase );

			//clean up the memory that is part of the pointers in the struct dataset
			//delete data.pose;
			//delete data.cloud;
			//delete data;

			if( !correct ) {
				printf( "\n\n*****************************************\nError.  Program terminated incorrectly. Deleting any files created for this image.\n*****************************************\n\n");
				std::string filenameVan = cv::format("%s.van", filenameBase.c_str() );
				//std::string filename2 = cv::format("%s.png", filenameBase.c_str() );
				std::string filenameFlr = cv::format("%s.flr", filenameBase.c_str() );
				remove( filenameVan.c_str() );
				//remove( filename2.c_str() );
				remove( filenameFlr.c_str() );
			}
		}
	}

}














