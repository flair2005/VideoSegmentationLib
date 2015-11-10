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


		/*
		 * process img_1
		 */
		Segmentation segmentation_1(img_1, gpu, scales, starting_scale);
		segmentation_1.segment_pyramid(threshold);
		//scale_for_propagation = segmentation_1.getSegmentsPyramid().size()-1;
		if(i>0)
			segmentation_1.reset_colours(scale_for_propagation,colours);
		const vector<Segment*>& segments_1 =
				segmentation_1.getSegmentsPyramid()[scale_for_propagation];
		vector<Atom*> atoms_1;
		for (Segment* seg : segments_1) {
			Atom * atom = new Atom(seg);
			atoms_1.push_back(atom);
		}
		cout << "> constructing visual representation 1" << endl;

		VisualRepresentation representation_1(atoms_1, prefix_1);
		imwrite("segmentation1.png",
				segmentation_1.getOutputSegmentsPyramid()[scale_for_propagation]);

		/*
		 * process img_2
		 */
		Atom::static_id = 0;
		Segmentation segmentation_2(img_2, gpu, scales, starting_scale);
		segmentation_2.segment_pyramid(threshold);
		const vector<Segment*>& segments_2 =
				segmentation_2.getSegmentsPyramid()[scale_for_propagation];
		vector<Atom*> atoms_2;
		for (Segment* seg : segments_2) {
			Atom * atom = new Atom(seg);
			atoms_2.push_back(atom);
		}
		cout << "> constructing visual representation 2" << endl;
		VisualRepresentation representation_2(atoms_2, prefix_2);
		imwrite("segmentation2.png",
				segmentation_2.getOutputSegmentsPyramid()[scale_for_propagation]);
		//	representation_1.compare_to(representation_2);

		/*
		 * match the two representations
		 */
		GraphsMatch graph_match(representation_1, representation_2);
		graph_match.find_match();
		Mat seg1,seg2;
		graph_match.getMats(seg1,seg2);
		string name1 = output_path+"/out"+to_string(i)+to_string(j)+"a.png";
		string name2 = output_path+"/out"+to_string(i)+to_string(j)+"b.png";
		imwrite(name1,seg1);
		imwrite(name2,seg2);
		Atom::static_id = 0;

		//save the colours of representation 1
		segmentation_2.save_colours(scale_for_propagation,colours);
		cout <<" ----- iteration finished -------"<<endl;

	}

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

