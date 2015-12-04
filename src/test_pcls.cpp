/*
 * main.cpp
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

/*
 * SegmentationLib
 */
#include "segmentation.h"
#include "visualisation.h"

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "atom.h"

#include "atom_graphs.h"
#include "GraphsMatch.h"
#include "objects/ObjectDetector.h"

#include "utils.h"

using namespace boost;
using namespace videoseg;
//using namespace std;

//finds the bounding rectangle of the slc
void find_slc_bounding_box(Mat& detections, Rect& rect){
	Mat pointsMat,mask;
	cvtColor(detections,mask,CV_RGB2GRAY);
	mask = mask > 0;
	cv::findNonZero(mask,pointsMat);
	rect=boundingRect(pointsMat);
}

int main(int argc, char** argv) {


	int scale_for_propagation = 2;
	int starting_scale = 2;
	Utils utils;
	int scales = 3;
	int gpu = 0;
	double threshold = 0.01; //0.05;
	unsigned int starting_frame = 15;
	string input_img_path, output_path, svm_path,input_clouds_path;

	std::vector<std::string> clouds_list;


	utils.parse_args(argc, argv, threshold, scales, starting_scale,
				scale_for_propagation, gpu, input_img_path, output_path,
				svm_path, input_clouds_path);
	cout <<"svm_path="<<svm_path<<endl;

	std::ifstream file_pcls(input_clouds_path);
		std::string content_clouds((std::istreambuf_iterator<char>(file_pcls)),
				std::istreambuf_iterator<char>());
		boost::split(clouds_list, content_clouds, boost::is_any_of("\t \n"));



	/*
	 * open image files
	 */

	std::vector<std::string> images_list;

	std::ifstream file_colours(input_img_path);

	std::string content_colour((std::istreambuf_iterator<char>(file_colours)),
			std::istreambuf_iterator<char>());

	boost::split(images_list, content_colour, boost::is_any_of("\t \n"));


	cout << " reading input..." << argv[1] << endl;
	vector<Vec3b> colours;
	unsigned int images = images_list.size() - 1; //starting_frame+20


	for (unsigned int i = starting_frame; i < images; i++) {

		cv::Mat img_1, img_2,depth, depth_float, contours_mat, gradient, grayGradient;

		cout <<"> testing "<<images_list[i]<<endl;
		img_1 = cv::imread(images_list[i], -1);
		depth = cv::imread(clouds_list[i], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
		depth.convertTo(depth_float,CV_32FC1);
		depth_float *= 0.001f;





		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr normals;
		string time_text("image to pcl: ");
		utils.tick();
		utils.image_to_pcl(img_1,depth_float,pcl_cloud);
		utils.tock(time_text);
		//utils.remove_outliers(pcl_cloud,pcl_cloud);
		time_text = "time for normal computation";
		utils.tick();
		utils.compute_integral_normals(pcl_cloud, normals);
		utils.tock(time_text);
		string text("cloud");
		//utils.compute_normals(pcl_cloud)
		//utils.display_cloud(pcl_cloud, text);

		Segmentation segmentation_1(img_1, gpu, scales, starting_scale);
		segmentation_1.segment_pyramid(threshold);

		vector<Segment*> current_segs = segmentation_1.getSegmentsPyramid()[scale_for_propagation];
		segmentation_1.map_segments(scale_for_propagation);
		for(Segment *seg : current_segs){
			imshow("seg",seg->getBinaryMat());
			waitKey(0);
			seg->addPcl(img_1, depth_float);


			seg->add_precomputed_pcl(pcl_cloud, normals);
			//seg->addPcl(img_1, depth_float);
			seg->computeFeatures();
		}




	}


	return 0;
}

