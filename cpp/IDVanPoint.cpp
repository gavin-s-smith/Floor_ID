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


/**
 * Determines the vanishing point within an image.  Currently only considers a single
 * vanishing point.  Writes the found vanishing point to a file.
 * 
 * Requires a file with the (non)floor labels to already exist.
 * 
 * Note hardcoded image size of 640x480
 * 
 * TODO: Detect more than one VP.
 * TODO: Implement a success/failure return value or return the value rather than writing it to a file.
 * 
 * @param pose The pose of the camera at the point the point cloud was captured
 * @param floorMaskFilename The filename of the file with the floor/non-floor labels
 * @param input_cloud The PCL point cloud of the image
 * @param filenameBase The filename to write the vanishing point out to
 */
void determineVanishingPoint( Pose3D pose, std::string floorMaskFilename, pcl::PointCloud<pcl::PointXYZRGB> input_cloud, std::string filenameBase ){

	// Extract cloud based on mask.  Note the cloud loses its organisation.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFloor (new pcl::PointCloud<pcl::PointXYZRGB>);

	int** floorMask = readFloorMask( floorMaskFilename, 640, 480 );
	printf("\nDetermining vanishing point... input cloud size: %i\n", (int)input_cloud.size());
	for ( int r = 0; r < input_cloud.height; r++ ) {
		for ( int c = 0; c < input_cloud.width; c++ ) {

			if( floorMask[r][c] == 1 ) { //point was ID'd as part of the floor
				pcl::PointXYZRGB tmp;		
				tmp.x = input_cloud(c,r).x;
				tmp.y = input_cloud(c,r).y;
				tmp.z = input_cloud(c,r).z;
				tmp.r = 0;
				tmp.g = 255;
				tmp.b = 0;
				cloudFloor->points.push_back( tmp );
				//printf("\nShould have added point to floor.\n");
			}
		}
	}

	cloudFloor->width = cloudFloor->points.size ();
	cloudFloor->height = 1;
	cloudFloor->is_dense = true;

	// Determine the floor plane. 
	// Required as we will extend lines parallel to each other along this plane
	// away from the camera in 3D space mapping them back to the image space.  
	// Where these two lines cross we have the vanishing point.
	// Destroys the cloudFloor (known floor point cloud) object in the process

	int findXPlanes = 1; //if the floor "bends" a lot from noise in the sensor
	                     // it may be beneficial to construct the floor from multiple
	                 	 // planes.  Currently implemented but not used.
	int planesFound = 0;
	cv::Vec4f rtn_plane;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold ( 0.005 ); 

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	int i = 0, nr_points = (int) cloudFloor->points.size ();
	// While 30% of the original cloud is still there
	printf("Finding floor plane (to detect the vanishing point) from cloud of size: %i",(int)cloudFloor->points.size ());
	while (cloudFloor->points.size () > 0.01 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloudFloor);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloudFloor);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		// A plane is only considered as part of the floor if the angle is valid 
		// (i.e. the plane normal is within 30 degrees of UP)
		// Note this makes an assumption of how the camera is held as discussed in the
		// paper.
		if( angleValid( coefficients->values[0],coefficients->values[1],coefficients->values[2] ) ){

			rtn_plane[0] = coefficients->values[0];
			rtn_plane[1] =  coefficients->values[1];
			rtn_plane[2] = coefficients->values[2];
			rtn_plane[3] = coefficients->values[3];

			if ( approxEquals(rtn_plane[0],0) && approxEquals(rtn_plane[1],0) && approxEquals(rtn_plane[2],0) && approxEquals(rtn_plane[3],0) ) {
				printf("\nPlane was zero!\nSkipping.  Please investigate this potential bug.");
				//bug means some images may be skipped when they could be used.
				exit(-1);
			} else {


				//ensure normal vector is always facing "up"
				if ( rtn_plane[1] < 0 ) {
					rtn_plane = rtn_plane *-1;
				}

				if( ++planesFound > findXPlanes ) break; //found the (or the specified number of) floor plane(s).
			}

		}


		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloudFloor);

		i++;
	}


	// check to ensure we found a plane correctly 
	// TODO: find a better way rather than this hack
	if ( approxEquals(rtn_plane[0],0) && approxEquals(rtn_plane[1],0) && approxEquals(rtn_plane[2],0) && approxEquals(rtn_plane[3],0) ) {
		printf("\nPlane was zero!\nSkipping whole image.");
		//clean up before exiting
		//clean up
		for( int j = 0; j < input_cloud.height; ++j ){
			delete [] floorMask[ j ];		
		}
		delete [] floorMask;

		return; //skip is via not writting a .van file.  This will lead to the incomplete image set and the images eventual removal
	} else {

		//printf("\n\n******************Made IT********************\n\n");
	}

	// The identified floor plane is now stored in rtn_plane
	// Now start the vanishing point identification.
	float xMin = 999999;
	float xMax = -999999;

	//determine the minimum and maximum floor pixel index along the image x-axis
	for ( int r = 0; r < input_cloud.height; r++ ) {
		for ( int c = 0; c < input_cloud.width; c++ ) {
			if (floorMask[r][c] == 1 ) {
				if( c < xMin ) xMin = c;
				if( c > xMax ) xMax = c;
			}
		}  
	}

	
	std::vector<Point3f> tmpG;

	bool foundFirst = false;
	bool foundSecond = false;
	bool calcVanishingPt = false;
	ofstream sampledPts;
	std::string filenameVan = cv::format("%s.van", filenameBase.c_str() );

	sampledPts.open( filenameVan.c_str() );	

	// Determine the direction that the camera is looking
	// If camera is at (0,0,0) this would be (0,0,-1) so transform that
	// coord into the correct coord given the camera pose.
	Point3f lookAt = pose.invCameraTransform( Point3f( 0,0,-1 ) );
	// get the camera position from the pose
	Vec3f lfv = pose.cvTranslation(); 
	Point3f lookFrom = Point3f( lfv[0], lfv[1], lfv[2] );
	//project both onto the plane.
	lookAt = projectPointToPlane( rtn_plane, lookAt );
	lookFrom = projectPointToPlane( rtn_plane, lookFrom );
	//dir = lookAt - lookFrom
	Point3f lookDir = lookAt - lookFrom;
	
	//add the lookDir to the orig point on the plane to get the next point 

	sampledPts << "LookAt0 = " << lookAt.x << "," << lookAt.y << "," << lookAt.z << "\n";
	//lookAt = Point3f( 0,0,-1 ) ;
	//we now need to project the lookAt point onto the floor plane as we
	//want the direction of the lookAt vector but along the plane.

	float distD = distToPlane( lookAt.x, lookAt.y, lookAt.z, rtn_plane );
	if( fabs(distD) > 0.00001 ){
		cout << "\nDistD: " << distD;
		exit(-1);
	}
	sampledPts << "LookAtP = " << lookAt.x << "," << lookAt.y << "," << lookAt.z << "\n";

	//lookAt = Point3f(0,0,1);

	float x1 = -99999.9;
	float y1 = -99999.9;
	float x2 = -99999.9;
	float y2 = -99999.9;
	float x3 = -99999.9;
	float y3 = -99999.9;
	float x4 = -99999.9;
	float y4 = -99999.9;//prevent the compiler complaining.  TODO: check after if statements to ensure they are initalized.
	float floorCenterX = xMin + ( ( xMax - xMin ) / 2 );
	for ( int r = 0; r < input_cloud.height; r++ ) {
		for ( int c = 0; c < input_cloud.width; c++ ) {
			//if the point is a floor point and it is in the point cloud
			// (due to filling in small holes the point may not be in the point cloud
			if (floorMask[r][c] == 1 && isfinite( input_cloud(c,r).x ) ) { 

				Point3f tmpPt( input_cloud(c,r).x, input_cloud(c,r).y, input_cloud(c,r).z );

				Point3f tPt(c,r,-1);

				if( !foundFirst && c > 20 && tPt.x < floorCenterX ){ //offset a little from the first

					Point3f projTmpPt = projectPointToPlane( rtn_plane, tmpPt );
					Point3f projTPt = pose.projectToImage( projTmpPt );
					printf("\nPoint (a) before project [%f, %f, %f ]\n", tmpPt.x,tmpPt.y,tmpPt.z);
					float distD = distToPlane( projTmpPt.x, projTmpPt.y, projTmpPt.z, rtn_plane );
					if( fabs(distD) > 0.2 ){
						cout << "\nDist to tmpPt: " << distD;
						exit(-1);
					}
					//generate the second point in the line
					Point3f nPt = Point3f( projTmpPt.x + lookDir.x, projTmpPt.y + lookDir.y, projTmpPt.z + lookDir.z );
					//sanity check, point must lie on floor plane
					distD = distToPlane( nPt.x, nPt.y, nPt.z, rtn_plane );
					if( fabs(distD) > 0.2 ){
						cout << "\nDist to first Found: " << distD;
						exit(-1);
					}
					
					Point3f pt2 = pose.projectToImage( nPt );
					sampledPts << "a=[ "<< projTPt.y << " " << projTPt.x << " ]\n"; //The first point on line 1
					sampledPts << "b=[ "<< pt2.y << " " << pt2.x << " ]\n"; //The second point on line 1
					x1 = projTPt.x; y1 = projTPt.y; x2 = pt2.x; y2 = pt2.y;
					
					tmpG.push_back(projTPt);
					tmpG.push_back(pt2);
					foundFirst = true;
				}

				if( !foundSecond && c > 20 && tPt.x > floorCenterX ){
					
					Point3f projTmpPt = projectPointToPlane( rtn_plane, tmpPt );
					Point3f projTPt = pose.projectToImage( projTmpPt );
					
					//generate the second point in the line
					Point3f nPt = Point3f( projTmpPt.x + lookDir.x, projTmpPt.y + lookDir.y, projTmpPt.z + lookDir.z );
					
					//sanity check, point must lie on floor plane
					float distD = distToPlane( nPt.x, nPt.y, nPt.z, rtn_plane );
					if( fabs(distD) > 0.2 ){
						cout << "\nDist to second found: " << distD;
						exit(-1);
					}
					
					Point3f pt2 = pose.projectToImage( nPt );
					sampledPts << "c=[ "<< projTPt.y << " " << projTPt.x << " ]\n";
					sampledPts << "d=[ "<< pt2.y << " " << pt2.x << " ]\n";
					x3 = projTPt.x; y3 = projTPt.y; x4 = pt2.x; y4 = pt2.y;
					tmpG.push_back(projTPt);
					tmpG.push_back(pt2);

					foundSecond = true;
				}

				if( !calcVanishingPt && foundFirst && foundSecond ){
					//determine the intesect (vanishing point) in image coords
					float vX = ( (x1*y1-y1*x2) * (x3-x4) - (x1-x2) * (x3*y4-y3*x4) ) / ( (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4) );
					float vY = ( (x1*y2-y1*x2) * (y3-y4) - (y1-y2) * (x3*y4-y3*x4) ) / ( (x1-x2) * (y3-y4) - (y1 - y2) * (x3-x4) );
					sampledPts << "e=[ "<< vY << " " << vX << " ]\n";
					calcVanishingPt = true;
				}
			}}

	}
	if( !foundFirst || !foundSecond ){
		printf( "Could not determine the vanishing points! Exiting. Code: 048tugv.  main.cpp");	
		printf("\nFloor center X: %f, min: %f, max %f, pose.imageCenterX(): %f\n", floorCenterX,xMin,xMax,pose.imageCenterX());
		exit(-1);
	}
	sampledPts.close();

	//DONE CALC VANISHING POINTS


	//clean up
	for( int j = 0; j < input_cloud.height; ++j ){
		delete [] floorMask[ j ];		
	}
	delete [] floorMask;

}





