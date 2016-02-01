#include <QtGui>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointTT;
typedef pcl::PointCloud<PointTT> CloudT;
typedef CloudT::Ptr CloudTPtr;
typedef CloudT::ConstPtr CloudTConstPtr;
