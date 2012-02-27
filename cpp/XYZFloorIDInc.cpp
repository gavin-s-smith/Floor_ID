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
# include <ntk/mesh/pcl_utils.h>
#include <ntk/geometry/pose_3d.h>

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

#include <iostream>
#include <algorithm>
#include <vector>

#include "/usr/local/include/opencv2/core/types_c.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

# include <pcl/registration/icp.h>
# include <pcl/filters/voxel_grid.h>
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

using namespace pcl;
using namespace ntk;
using cv::Vec3f;
using cv::Point3f;

using namespace std;
using namespace boost;
using namespace ntk;

struct GridCell {
	int row;
	int col;
	float zValue;
} ;

struct ordering {
	bool operator ()(GridCell const& a, GridCell const& b) {
		return a.zValue > b.zValue; //descending order
	}
};





/*
 * 
 * Given an input cloud (must be organised) and a set of parameters labels the pixels in
 * the cloud as either floor, non-floor or unknown.  The labels are written to a file
 * (filePathOut), 1 == floor, -1 == non-floor, 0 == unknown.  File type is a csv file
 * in a single line.  Designed for use with known image dimentions and the reshape 
 * function within Matlab.
 * 
 * @param input_cloud A organised point cloud to provide labels for
 * @param filePathOut The path to write the file containing the labels to
 * @param min_dist_to_be_considered_a_plane_inlier CURRENTLY NOT USED. TODO: Remove
 * @param planeDistThreshold CURRENTLY NOT USED. TODO: Remove
 * @param maxDepth CURRENTLY NOT USED. TODO: Remove
 * @param visualDebug Used for debugging purposes only.  CURRENTLY NOT USED.
 * @param restrictNumPoints Should we restrict the labeled points to a fixed number
 *                          of pixels?
 * @param keepCtBasedOnFloorPts Valid only when restrictNumPoints == True.  Should 
 *                              we restrict the labeled points based on a count over
 *                              all points, or just floor points?
 * @param keepMaxNPts Valid only when restrictNumPoints == True.  The number of points
 *                    to restrict the labelling to.
 * 
 * @return true if point cloud was sucessfully labeled, false otherwise
 */


bool genFloorMask( PointCloud<PointXYZRGB> input_cloud, std::string filePathOut, 
		float 	min_dist_to_be_considered_a_plane_inlier, float planeDistThreshold, 
		float maxDepth, bool visualDebug, bool restrictNumPoints, bool keepCtBasedOnFloorPts, size_t keepMaxNPts  ) {
	
	printf( "\nInput cloud size: %i\n", (int)input_cloud.size() );
	if( input_cloud.size() <=0 ){
		printf("\nEmpty input cloud!! Exiting.");
		exit(-1);
	}


	//---------------------
	//Hard-coded Parameters
	//----------------------
	int minClusterSize = 10; // Minimum number of points for a cluster to be considered
	                         // valid.  Used when clustering in 2D image space.
	
	// Below are all part of old code, currently UNUSED. TODO: Remove
	int minNumPtsForPlaneToBeConsideredValid = 10; // Part of old code. Now unused  
	                                               // TODO: Remove
	float threshold_radius = 0.2; // Part of old code. Now unused.  TODO: Remove
	float maxDistFromPlane = 0.3; // Part of old code. Now unused.  TODO: Remove
	int findXPlanes = 1; // Part of old code. Now unused.  TODO: Remove
	int planesFound = 0; // Part of old code. Now unused.  TODO: Remove
	//-------------------------
	//End hard-coded paramters
	//-------------------------

	
	//Constants
	float rad60degs =  1.04719755;
	float rad30degs =  0.523598776;
	float rad40degs = 0.698131701;
	float rad15degs = 0.261799388;
	float rad20degs = 0.34906585;
	float planeNormalX = 0;
	float planeNormalY = 1;
	float planeNormalZ = 0;
	
	//Vars
	pcl::PointCloud<PointXYZRGB>::ConstPtr cloud_ptr = input_cloud.makeShared();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDWriter writer;
	cv::Vec4f rtn_plane;

	//Allocate the label matrix and initialise it to all zeros
	int** rtnMatrix = new int*[input_cloud.height];
	for (int i = 0 ; i < input_cloud.height ; i++){
		rtnMatrix[i] = new int[input_cloud.width];
		for (int j = 0 ; j < input_cloud.width ; j++){
			rtnMatrix[i][j] = 0;
		}
	}

	printf("\nStarting floor identification...\n");


	printf("\nStarting to determine point normals...");

	// Estimate normals via PCL
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal> normals;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud_ptr);
	ne.compute(normals);
	printf(" Done determining point normals.\n");

	std::pair<int,int> tmpPt;

	// Assume that the camera is held correctly and that the Y axis 
	// points to the ceiling filter based on point normals being within
	// a fixed number of degrees.  
	// NOTE: This seemed to work better than assuming that the floor 
	// would be the dominant plane identified by PCL.
	planeNormalX = 0;
	planeNormalY = 1;
	planeNormalZ = 0;
	int ct=0;
	for ( int i = 0; i < input_cloud.points.size(); i++ ) {
		//printf("\nDist to plane was: %f",distToPlane( input_cloud.points[i].x,input_cloud.points[i].y,input_cloud.points[i].z, rtn_plane));
		tmpPt = convertToXY( i, input_cloud.width );
		if( //Angle was within the specified angle and point should be considered as floor

				(	
						calcAngle(normals.points[i].normal_x,normals.points[i].normal_y,normals.points[i].normal_z,
								planeNormalX,planeNormalY,planeNormalZ) < rad40degs 
								|| 
								calcAngle(normals.points[i].normal_x,normals.points[i].normal_y,normals.points[i].normal_z,
										planeNormalX,-planeNormalY,planeNormalZ) < rad40degs  
				)

		){

			// Use the colour fields to mark the point
			//rtnMatrix[tmpPt.second][tmpPt.first] = 1;
			input_cloud.points[i].r = 0;
			input_cloud.points[i].g = 0;
			input_cloud.points[i].b = 255;
			ct++;
		} else if(  //Angle was outside specified (potenitally different) angle
                    // and point should not be considered part of the floor
				(	
						calcAngle(normals.points[i].normal_x,normals.points[i].normal_y,normals.points[i].normal_z,
								planeNormalX,planeNormalY,planeNormalZ) > rad60degs 
								|| 
								calcAngle(normals.points[i].normal_x,normals.points[i].normal_y,normals.points[i].normal_z,
										planeNormalX,-planeNormalY,planeNormalZ) > rad60degs  
				)

		){
			tmpPt = convertToXY( i, input_cloud.width );			
			rtnMatrix[tmpPt.second][tmpPt.first] = -1;
			input_cloud.points[i].r = 255;
			input_cloud.points[i].g = 0;
			input_cloud.points[i].b = 0;

		} else {

			input_cloud.points[i].r = 255;
			input_cloud.points[i].g = 0;
			input_cloud.points[i].b = 0;
		}
	}


	//density based filter (points must be surrounded by 60% floor points to be a point)
	//NOTE: must use a percent as the density varies along the z-axis (as it came from an image)
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	int K = 1;
	kdtree.setInputCloud (input_cloud.makeShared());
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 0.1;
	for ( int r = 0; r < input_cloud.height; r++ ) {
		for ( int c = 0; c < input_cloud.width; c++ ) {
			
			//if point was preliminarly labeled as part of the floor
			if( input_cloud(c,r).b > 200 ) {
				kdtree.radiusSearch (input_cloud(c,r), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
				float yCt = 0; float total = 0;
				for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){	
					if( input_cloud[ pointIdxRadiusSearch[i] ].b > 200 ) { //point was ID'd as part of the floor			
						yCt++;
					}
					total++;
				}
				
				// If point does not have enough neighbours then re-mark it as non-floor
				if( ( yCt / total ) < 0.4 ) {
					//rtnMatrix[r][c] = -1;				
					input_cloud(c,r).r = 255;
					input_cloud(c,r).g = 0;
					input_cloud(c,r).b = 0;
				}
			}
		}
	}

	// Now cluster the points.  The 
	//now extract the floor part (as image indexes). 
	// TODO: remove hack of using the 3d clustering algorithm!
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (size_t i = 0; i < input_cloud.points.size (); ++i)
	{
		if( input_cloud.points[i].b > 200 ) { //point was ID'd as part of the floor
			pcl::PointXYZRGB tmp;	
			tmpPt = convertToXY( i, input_cloud.width );	
			tmp.x = tmpPt.first;
			tmp.y = tmpPt.second;
			tmp.z = 0;
			tmp.r = 0;
			tmp.g = 255;
			tmp.b = 0;
			tmp_cloud->points.push_back( tmp );

		}
	}

	tmp_cloud->width = tmp_cloud->points.size ();
	tmp_cloud->height = 1;
	tmp_cloud->is_dense = true;
	std::cout << "PointCloud representing the Cluster: " << tmp_cloud->points.size () << " data points." << std::endl;



	if( tmp_cloud->points.size () < minClusterSize ){

		printf( "\nCould not find the floor.  Giving up on this frame.\n" );
		return false;

	}


	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud ( tmp_cloud );
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (10); 
	ec.setMinClusterSize (minClusterSize);
	ec.setMaxClusterSize (2500000);
	ec.setSearchMethod (tree);
	ec.setInputCloud( tmp_cloud );
	ec.extract (cluster_indices);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	//This is the final filter.  Only this one can mark the input cloud with green and fill in the return matrix!
	if( cluster_indices.size() == 0 ){

		printf( "\nCould not find the floor.  Giving up on this frame.\n" );
		return false;

	}

	pcl:PointIndices pts_cluster0 = cluster_indices[0];
	for( int i = 0; i < pts_cluster0.indices.size(); i++ ){
		//pcl::PointXYZRGB tmp;		
		int c = floor(tmp_cloud->points[pts_cluster0.indices[i]].x + 0.5);
		int r = floor(tmp_cloud->points[pts_cluster0.indices[i]].y + 0.5);
		input_cloud(c,r).r = 0;
		input_cloud(c,r).g = 255;
		input_cloud(c,r).b = 0;
		rtnMatrix[r][c] = 1; //otherwise the return matrix should not be updated with green!!!!

	}


	printf("\nCloud processing complete.\n");

	// The remaining code cleans up the labels by performing some post processing
	// Specifically: 
	// (1) Fills in small holes in the labeled area.
	// (2) Removing a small amount from the edges of the labeled (floor and non-floor)
	//     region edges
	// (3) Restricts the number of points labeled (resets them to unknown) based
	//     on user setting (via the function parameter)
	
	//Fill in single pixel holes
	bool n1 = false;
	bool n2 = false;
	bool n3 = false;
	bool n4 = false;
	bool n5 = false;
	bool now = false;
	for ( int r = 0; r < input_cloud.height; r++ ) {
		n1  = false;
		n2 = false;
		n3 = false;
		n4 = false;
		n5 = false;
		for ( int c = 0; c < input_cloud.width; c++ ) {
			now = (rtnMatrix[r][c] == 1 );
			if( now && ( n1 || n2 || n3 || n4 || n5 ) ){
				rtnMatrix[r][c-1] = 1;
				rtnMatrix[r][c-2] = 1;
				rtnMatrix[r][c-3] = 1;
				rtnMatrix[r][c-4] = 1;
				rtnMatrix[r][c-5] = 1;
				n1 = n2 = n3 = n4 = n5 = true;
			}
			n5 = n4;
			n4 = n3;
			n3 = n2;
			n2 = n1;
			n1 = now;			

		}
	}

	for ( int c = 0; c < input_cloud.width; c++ ) {
		n1  = false;
		n2 = false;
		n3 = false;
		n4 = false;
		n5 = false;
		for ( int r = 0; r < input_cloud.height; r++ ) {

			now = (rtnMatrix[r][c] == 1 );
			if( now && ( n1 || n2 || n3 || n4 || n5 ) ){
				rtnMatrix[r][c-1] = 1;
				rtnMatrix[r][c-2] = 1;
				rtnMatrix[r][c-3] = 1;
				rtnMatrix[r][c-4] = 1;
				rtnMatrix[r][c-5] = 1;
				n1 = n2 = n3 = n4 = n5 = true;
			}
			n5 = n4;
			n4 = n3;
			n3 = n2;
			n2 = n1;
			n1 = now;			

		}
	}
	
	//done filling in holes

	//Now trim

	int numToRemovePerSideFloor = 10;
	int numToRemovePerSideNonFloor = 10;
	int resetAfterNZeros = 10;
	//REMOVE FLOOR BITS

	//Remove from the left
	for ( int ii = 0; ii< numToRemovePerSideFloor; ii++) {
		int lastZeroCt = 0;
		for ( int r = 0; r < input_cloud.height; r++ ) {
			for ( int c = 0; c < input_cloud.width; c++ ) {

				if (  rtnMatrix[r][c] == 1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}

	//Remove from the right
	for ( int ii = 0; ii< numToRemovePerSideFloor; ii++) {
		int lastZeroCt = 0;
		for ( int r = 0; r < input_cloud.height; r++ ) {
			for ( int c = input_cloud.width-1; c >=0 ; c-- ) {

				if (  rtnMatrix[r][c] == 1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}

	//Remove from the bottom

	for ( int ii = 0; ii< numToRemovePerSideFloor; ii++) {
		int lastZeroCt = 0;
		for ( int c = 0; c < input_cloud.width; c++ ) {
			for ( int r = input_cloud.height-1; r >=0 ; r-- ) {

				if (  rtnMatrix[r][c] == 1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}


	//Remove from the top
	for ( int ii = 0; ii< numToRemovePerSideFloor; ii++) {
		int lastZeroCt = 0;
		for ( int c = 0; c < input_cloud.width; c++ ) {
			for ( int r = 0; r < input_cloud.height ; r++ ) {

				if (  rtnMatrix[r][c] == 1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}



	//REMOVE NON-FLOOR BITS
	//Remove from the left
	for ( int ii = 0; ii< numToRemovePerSideNonFloor; ii++) {
		int lastZeroCt = 0;
		for ( int r = 0; r < input_cloud.height; r++ ) {
			for ( int c = 0; c < input_cloud.width; c++ ) {

				if (  rtnMatrix[r][c] == -1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}

	//Remove from the right
	for ( int ii = 0; ii< numToRemovePerSideNonFloor; ii++) {
		int lastZeroCt = 0;
		for ( int r = 0; r < input_cloud.height; r++ ) {
			for ( int c = input_cloud.width-1; c >=0 ; c-- ) {

				if (  rtnMatrix[r][c] == -1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}

	//Remove from the bottom
	for ( int ii = 0; ii< numToRemovePerSideNonFloor; ii++) {
		int lastZeroCt = 0;
		for ( int c = 0; c < input_cloud.width; c++ ) {
			for ( int r = input_cloud.height-1; r >=0 ; r-- ) {

				if (  rtnMatrix[r][c] == -1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}

	//Remove from the top

	for ( int ii = 0; ii< numToRemovePerSideNonFloor; ii++) {
		int lastZeroCt = 0;
		for ( int c = 0; c < input_cloud.width; c++ ) {
			for ( int r = 0; r < input_cloud.height ; r++ ) {

				if (  rtnMatrix[r][c] == -1 ) {
					if ( lastZeroCt > resetAfterNZeros ) {

						rtnMatrix[r][c] = 0;

					}
					lastZeroCt = 0;
				} else {
					lastZeroCt++;			
				}

			}
		}
	}


	//done trimming



	//------------------------------------------------------------------------------
	//---------------------- RESTRICT NUM POINTS CODE ------------------------------
	//------------------------------------------------------------------------------



	//Allocate the return matrix and initialise it to all zeros (in all cases so we can delete in all cases)
	int** blackList = new int*[input_cloud.height];
	for (int i = 0 ; i < input_cloud.height ; i++){
		blackList[i] = new int[input_cloud.width];
		for (int j = 0 ; j < input_cloud.width ; j++){
			blackList[i][j] = 0;
		}
	}

	if( restrictNumPoints ){

		vector< GridCell > listOfPts;
		//order all points (of interest) by distance on the z axis
		size_t singleIdx = 0;
		for ( int r = 0; r < input_cloud.height; r++ ) {
			for ( int c = 0; c < input_cloud.width; c++ ) {
				if( 
						( !keepCtBasedOnFloorPts && isfinite(input_cloud(c,r).z) )
						||
						( keepCtBasedOnFloorPts && isfinite(input_cloud(c,r).z) && rtnMatrix[r][c] == 1 )
				){		
					GridCell a;
					a.row = r;
					a.col = c;
					a.zValue = input_cloud(c,r).z;
					listOfPts.push_back( a );
					singleIdx++;
				}
			}
		}

		sort(listOfPts.begin(), listOfPts.end(), ordering());

		//points are now sorted in desc/asc order
		cout << "\n";
		float lastZ = 99999;
		for( size_t i = 0; i < listOfPts.size() && i < keepMaxNPts; ++i){
			//cout << ", " << listOfPts[ i ].zValue;
			blackList[ listOfPts[ i ].row ][ listOfPts[ i ].col ] = 1; //these are the points we keep.  Otherwise throw away.
			lastZ = listOfPts[ i ].zValue;	
		}
		printf("\nLast z: %f\n", lastZ);

		if( keepCtBasedOnFloorPts ) { //we must add the non-floor points

			//first we must order all points
			vector< GridCell > listOfAllPts;
			singleIdx = 0;
			for ( int r = 0; r < input_cloud.height; r++ ) {
				for ( int c = 0; c < input_cloud.width; c++ ) {
					if( 
							(  isfinite(input_cloud(c,r).z) )
					){		
						GridCell a;
						a.row = r;
						a.col = c;
						a.zValue = input_cloud(c,r).z;
						listOfAllPts.push_back( a );
						singleIdx++;
					}
				}
			}

			sort(listOfAllPts.begin(), listOfAllPts.end(), ordering());

			for( size_t i = 0; i < listOfAllPts.size(); ++i){
				if( isfinite(input_cloud(listOfAllPts[ i ].col,listOfAllPts[ i ].row).z) && isfinite(input_cloud(listOfAllPts[ i ].col,listOfAllPts[ i ].row).z) && input_cloud(listOfAllPts[ i ].col,listOfAllPts[ i ].row).z > lastZ ) { //recall z becomes negative the futher away it is
					blackList[ listOfAllPts[ i ].row ][ listOfAllPts[ i ].col ] = 1; //these are the points we keep.  Otherwise throw away.
				}
			}
		}


		int numPts = 0;
		//use the blackList against the return matrix
		for ( int r = 0; r < input_cloud.height; r++ ) {
			for ( int c = 0; c < input_cloud.width; c++ ) {
				if( blackList[r][c] != 1 ){
					rtnMatrix[r][c] = 0;
				} else {
					numPts++;
				}
			}
		}


	}


	//write out the floor file. 
	//count number of identified floor points and non floor points
	int floorPtCt = 0;
	int nonFloorPtCt = 0;
	ofstream myfile;
	printf("Writing file %s", filePathOut.c_str());
	myfile.open ( filePathOut.c_str() );
	for ( int r = 0; r < input_cloud.height; r++ ) {
		for ( int c = 0; c < input_cloud.width; c++ ) {
			//if(c < 50 ) { myfile << "1,"; } else { myfile << "0,";}
			if (rtnMatrix[r][c] == 1 ) {
				myfile << "1,";
				floorPtCt++;
			} else if ( rtnMatrix[r][c] == -1 ) {
				myfile << "-1,";
				nonFloorPtCt++;
			} else {
				myfile  << "0,";
			}

		}
		myfile << "\n";
	}
	myfile.close();


	// Make sure enough points have been labeled point and non-points to
	// consitute floor identification.
	if( floorPtCt < 50 || nonFloorPtCt < 50 ) return false; 


	printf( "\nDone determining the floor plane.\n");


	//clean up
	for( int j = 0; j < input_cloud.height; ++j ){
		delete [] rtnMatrix[ j ];		
	}
	delete [] rtnMatrix;

	for( int j = 0; j < input_cloud.height; ++j ){
		delete [] blackList[ j ];		
	}
	delete [] blackList;

	return true;

}





