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

using namespace boost;
using namespace videoseg;
//using namespace std;

string getFileName(const string& s) {

	char sep = '/';

	size_t i = s.rfind(sep, s.length());
	if (i != string::npos) {
		return (s.substr(i + 1, s.length() - i));
	}

	return ("");
}

int main(int argc, char** argv) {

	int scale = 2;
	int starting_scale = 0;
	if (argc < 4) {
		std::cout
				<< "Usage: $> ./segmenter <images.txt>  <output path> <scales> [<use gpu, default 0=false>]"
				<< std::endl;
		return 0;
	}
	/*
	 * open image files
	 */

	std::vector<std::string> images_list;

	std::ifstream file_colours(argv[1]);

	std::string content_colour((std::istreambuf_iterator<char>(file_colours)),
			std::istreambuf_iterator<char>());

	boost::split(images_list, content_colour, boost::is_any_of("\t \n"));


	cout << " reading input..." << argv[1] << endl;
	vector<Vec3b> colours;
	for (int i = 0, j = i + 1; i < images_list.size() - 1; i++, j++) {

		cv::Mat img_1, img_2, contours_mat, gradient, grayGradient;
		double threshold = 0.01; //0.05;

		cout <<images_list[i]<<" <--> "<<images_list[j]<<endl;
		img_1 = cv::imread(images_list[i], -1);
		img_2 = cv::imread(images_list[j], -1);
		std::string outputPath(argv[2]);
		string prefix_1 = getFileName(images_list[i]);
		string prefix_2 = getFileName(images_list[j]);
		cout << "outputPath=" << outputPath << endl << " prefix_1=" << prefix_1
				<< endl << " prefix_2=" << prefix_2 << endl;

		int scales = atoi(argv[3]);
		int gpu = 0;
		if (argc == 5) {
			gpu = atoi(argv[4]);
		} else if (argc == 6) {
			gpu = atoi(argv[4]);
			threshold = atof(argv[5]);
		}

		bool use_gpu = gpu > 0 ? true : false;

		/*
		 * process img_1
		 */
		Segmentation segmentation_1(img_1, gpu, scales, starting_scale);
		segmentation_1.segment_pyramid(threshold);
		if(i>0)
			segmentation_1.reset_colours(scale,colours);
		const vector<Segment*>& segments_1 =
				segmentation_1.getSegmentsPyramid()[scale];
		vector<Atom*> atoms_1;
		for (Segment* seg : segments_1) {
			Atom * atom = new Atom(seg);
			atoms_1.push_back(atom);
		}
		cout << "> constructing visual representation 1" << endl;

		VisualRepresentation representation_1(atoms_1, prefix_1);
		imwrite("segmentation1.png",
				segmentation_1.getOutputSegmentsPyramid()[scale]);

		/*
		 * process img_2
		 */
		Atom::static_id = 0;
		Segmentation segmentation_2(img_2, gpu, scales, starting_scale);
		segmentation_2.segment_pyramid(threshold);
		const vector<Segment*>& segments_2 =
				segmentation_2.getSegmentsPyramid()[scale];
		vector<Atom*> atoms_2;
		for (Segment* seg : segments_2) {
			Atom * atom = new Atom(seg);
			atoms_2.push_back(atom);
		}
		cout << "> constructing visual representation 2" << endl;
		VisualRepresentation representation_2(atoms_2, prefix_2);
		imwrite("segmentation2.png",
				segmentation_2.getOutputSegmentsPyramid()[scale]);
		//	representation_1.compare_to(representation_2);

		GraphsMatch graph_match(representation_1, representation_2);
		graph_match.find_match();
		Mat seg1,seg2;
		graph_match.getMats(seg1,seg2);
		string name1 = outputPath+"/out"+to_string(i)+to_string(j)+"a.png";
		string name2 = outputPath+"/out"+to_string(i)+to_string(j)+"b.png";
		imwrite(name1,seg1);
		imwrite(name2,seg2);
		Atom::static_id = 0;

		//save the colours of representation 1
		segmentation_2.save_colours(scale,colours);

	}

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

