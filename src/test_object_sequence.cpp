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



int main(int argc, char** argv) {


	int scale_for_propagation = 2;
	int starting_scale = 2;
	Utils utils;
	int scales = 3;
	int gpu = 0;
	double threshold = 0.01; //0.05;
	unsigned int starting_frame = 0;
	string input_img_path, output_path, svm_path,input_clouds_path;
	utils.parse_args(argc, argv, threshold, scales, starting_scale,
			scale_for_propagation, gpu, input_img_path, output_path,svm_path,input_clouds_path);
	cout <<"svm_path="<<svm_path<<endl;
	ObjectDetector slc(ObjectDetector::TEST_MODE,svm_path);



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

		cv::Mat img_1, img_2, contours_mat, gradient, grayGradient;


		img_1 = cv::imread(images_list[i], -1);
		string prefix_1 = utils.get_file_name(images_list[i]);
		prefix_1 = utils.remove_extension(prefix_1);

		/*
		 * process img_1
		 */
		Segmentation segmentation_1(img_1, gpu, scales, starting_scale);
		segmentation_1.segment_pyramid(threshold);

		vector<Segment*> current_segs = segmentation_1.getSegmentsPyramid()[scale_for_propagation];
		for(Segment *seg : current_segs){
			seg->computeFeatures();
		}
		slc.test_data(current_segs);
		Mat& ref = segmentation_1.getOutputSegmentsPyramid()[scale_for_propagation];
		Mat detections = Mat::zeros(ref.rows,ref.cols,CV_8UC3);
		for(Segment *seg : current_segs){

			if(seg->getClassLabel() > 0.3)
				detections(seg->getBoundRect()) += seg->getRandomColourMat();
		}
		imshow("img",img_1);
		imshow("segmentation",ref);
		imshow("prediction",detections);
		waitKey(1);


	}


	return 0;
}

