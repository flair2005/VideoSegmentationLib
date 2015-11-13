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
#include "VideoSegmentation.h"

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
	string input_img_path,output_path;
	utils.parse_args(argc,argv,threshold,scales,starting_scale,scale_for_propagation,gpu,input_img_path,output_path);


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
	unsigned int images = images_list.size() - 1;
	VideoSegmentation videoSegmenter;
	for (unsigned int i = 0, j = i + 1; i < images-1; i++, j++) {

		cv::Mat img_1, img_2, contours_mat, gradient, grayGradient;


		cout <<images_list[i]<<" <--> "<<images_list[j]<<endl;

		img_1 = cv::imread(images_list[i], -1);
		img_2 = cv::imread(images_list[j], -1);

		string prefix_1 = utils.get_file_name(images_list[i]);
		prefix_1 = utils.remove_extension(prefix_1);
		string prefix_2 = utils.get_file_name(images_list[j]);
		prefix_2 = utils.remove_extension(prefix_2);
		cout << "outputPath=" << output_path << endl << " prefix_1=" << prefix_1
				<< endl << " prefix_2=" << prefix_2 << endl;


		Mat dst;
		videoSegmenter.addImage(img_1,dst);
		//imshow("dst",dst);
		//waitKey(0);
		cout <<" ----- iteration finished -------"<<endl;

	}

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

