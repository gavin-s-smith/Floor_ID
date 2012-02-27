
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
 * Calculates the L2 length of a vector with three components
 */
float calcVecLen( float x, float y, float z ){
	return sqrt( x*x + y*y + z*z );
}


/**
 * Calculates the angle bewteen two 3D vectors
 */
float calcAngle( float x1, float y1, float z1, float x2, float y2, float z2){

	float len1 = calcVecLen(x1,y1,z1);
	float len2 = calcVecLen(x2,y2,z2);

	return acos( (x1*x2 + y1*y2 + z1*z2) / ( len1 * len2 ) );

}

/**
 * Checks a 3D vector to ensure has an angle with 30 degrees of the line along the
 * y-axis.
 * 
 * @param x x-component of the 3D vector
 * @param y y-component of the 3D vector
 * @param z z-component of the 3D vector
 * 
 * @return boolean indicating whether the vector was within 30 degrees of the y-axis
 */
bool angleValid( float x, float y, float z){
	float rad30degs =  0.523598776;
	float rad40degs = 0.698131701;
	float rad15degs = 0.261799388;
	float rad20degs = 0.34906585;
	return calcAngle(x,y,z,0,1,0) < rad30degs || calcAngle(x,y,z,0,-1,0) < rad30degs;
}


/**
 * Function to perform equals to within a fixed precision.
 */
bool approxEquals( float in, float in2 ){
	float precision = 0.0000001;

	if( ( in - in2 ) >= -precision && ( in - in2 ) <= precision ) return true;

	return false;
}

/**
 *  Returns a list of files in a vector given a directory
 *  @param dir path to the directory
 *  @param return parameter, a vector of strings with the filenames.
 */
int getdir (std::string dir, std::vector<std::string> &files)
{
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir(dir.c_str())) == NULL) {
		cout << "Error(" << errno << ") opening " << dir << endl;
		return errno;
	}

	while ((dirp = readdir(dp)) != NULL) {
		std::stringstream ss;
		ss << dirp->d_name;
		files.push_back(ss.str() );
	}
	closedir(dp);
	return 0;
}


/*
 * Determines the distance from a point to a plane.
 * Pt: first three args, [ x, y, z ]
 * Plane: Given by a cv::Vec4f of a, b, c, d where plane: a b c d, ax+by+cz+d = 0
 */

float distToPlane( float x, float y, float z, cv::Vec4f plane ) {
	if( approxEquals(plane[0],0) && approxEquals(plane[1],0) && approxEquals(plane[2],0) && approxEquals(plane[3],0) ){

		printf("\n\n-------------------------\n------Plane was all zeros!------\n-------------------------\n\n");	
		//exit(-1);
	}

	return fabs(x * plane[0] + y * plane[1] + z * plane[2] + plane[3]);

}

/*
 * Projects a point onto a plane.
 * Plane: Given by a cv::Vec4f of a, b, c, d where plane: a b c d, ax+by+cz+d = 0
 * Pt: Given by a Point3f
 */
cv::Point3f projectPointToPlane( cv::Vec4f plane, Point3f pt ){

	float A = plane[0];
	float B = plane[1];
	float C = plane[2];
	float D = plane[3];
	float a = pt.x;
	float b = pt.y;
	float c = pt.z;
	float alpha;

	alpha = (-D-a*A-b*B-c*C) / (A*A+B*B+C*C);
	return cv::Point3f( a + alpha * A, b + alpha * B, c + alpha * C );

}

/*
 * Reads an XYZ matrix as written by the XYZ program included as part of this set of tools.
 * I.e. a continuous set of csv written sequentially for each col, for each row, write x,y,z.
 * Returns a (PCL) point cloud (organised)
 */
pcl::PointCloud<pcl::PointXYZRGB> readXYZMatrix( std::string filenameXYZ, int imgWidthInPixels, int imgHeightInPixels ){

	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	// Fill in the cloud data
	cloud.width    = imgWidthInPixels;
	cloud.height   = imgHeightInPixels;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);


	std::ifstream  data( filenameXYZ.c_str() );
	int j = 0;
	int i = 0;
	int k = 0;
	std::string line;
	while(std::getline(data,line))
	{
		std::stringstream  lineStream(line);
		std::string        cell;
		while(std::getline(lineStream,cell,','))
		{

			float val = atof( cell.c_str() );
			//-999999 reflects a NaN in my XYZ file format, therefore assign a NaN (needed for PCL processing).  
			if( val < -99999 ) val =  std::numeric_limits<float>::quiet_NaN();	
			if( k == 0 ) cloud.points[j].x  = val ;    	
			if( k == 1 ) cloud.points[j].y  = val;   
			if( k == 2 ) cloud.points[j].z  = val;   	
			k++;
			if( k == 3 ){
				k = 0;
				j++;

			}
		}

	}

	data.close();

	return cloud;
}

/**
 * Converts vector index to XY index
 */
std::pair<int,int> convertToXY( int idx, int width ){
	int v = idx / width;
	int u = idx - v* width;
	return std::make_pair(u,v);
}

/**
 * Checks if a file exists or not
 */
bool fexists(const char *filename)
{
	ifstream ifile(filename);
	return ifile;
}

/*
 * Read a floor mask as written by the program XYZFloorID
 */
int** readFloorMask( std::string filenameXYZ, int imgWidthInPixels, int imgHeightInPixels ){
	printf("\nImage width: %i height: %i \n", imgWidthInPixels,  imgHeightInPixels);
	//Allocate the return matrix and initialise it to all zeros
	int** rtnMatrix = new int*[imgHeightInPixels];
	for (int r = 0 ; r < imgHeightInPixels ; r++){
		rtnMatrix[r] = new int[imgWidthInPixels];
		for (int c = 0 ; c < imgWidthInPixels ; c++){
			rtnMatrix[r][c] = 0;
		}
	}

	std::ifstream  data( filenameXYZ.c_str() );
	int r = 0;
	int c = 0;
	int k = 0;
	int val = -1;
	std::string line;
	while(std::getline(data,line,'\n'))
	{
		//printf("\n%s\n", line.c_str());
		std::stringstream  lineStream(line);
		std::string        cell;
		c = 0;
		while(std::getline(lineStream,cell,','))
		{
			val = atoi( cell.c_str() );
			if( val == 1 ){		
				rtnMatrix[r][c] = 1;
			} else {
				rtnMatrix[r][c] = 0;
			}
			c++;
		}
		r++;

	}

	return rtnMatrix;
}


//own function similar to that of RGBDemo, except this one generates an organised point cloud
pcl::PointCloud<pcl::PointXYZRGB> rgbdImageToPointCloud(
		const RGBDImage& image,
		const Pose3D& poseRGB,
		float maxDepth
)
{

	int subsampling_factor = 1;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	// Fill in the cloud data
	cloud.width    = image.depth().cols;
	cloud.height   = image.depth().rows;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	//we will be using the depth image to generate the points.  Therefore we must convert the
	//rgb pose to a depth pose and use this.
	Pose3D pose = poseRGB;

	cv::Point3f tmp =  poseRGB.cvRodriguesRotation();
	printf("\nRGB Rotation: [%f, %f, %f]\n",  tmp.x, tmp.y, tmp.z );

	pose.toLeftCamera(image.calibration()->depth_intrinsics, image.calibration()->R, image.calibration()->T);



	tmp =  pose.cvRodriguesRotation();
	printf("\nProjecting via depth.  Rotation when unprojecting: [%f, %f, %f]\n",  tmp.x, tmp.y, tmp.z );
	printf("\nOnly keeping points within a depth range of [0.0, %f]", maxDepth);
	for (int r = 0; r < image.depth().rows; r += subsampling_factor)
		for (int c = 0; c < image.depth().cols; c += subsampling_factor)
		{
			float d = image.depth()(r,c);
			bool mask_ok = !image.depthMask().data || image.depthMask()(r,c);
			//printf("\nWill throw away point as it is greater than maxDepth, %f > %f",d,maxDepth);
			//   if( d > maxDepth ) printf("\nWill throw away point as it is greater than maxDepth, %f > %f",d,maxDepth);
			if (d < 1e-5 || d > maxDepth || !mask_ok){
				//write a NaN
				cloud(c,r).x = std::numeric_limits<float>::quiet_NaN();
				cloud(c,r).y = std::numeric_limits<float>::quiet_NaN();
				cloud(c,r).z = std::numeric_limits<float>::quiet_NaN();
			} else {
				cv::Point3f p = pose.unprojectFromImage(cv::Point2f(c,r),d);
				cloud(c,r).x = p.x;
				cloud(c,r).y = p.y;
				cloud(c,r).z = p.z;
			}




		}


	return cloud;

}


