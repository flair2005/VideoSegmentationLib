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
      return(s.substr(i+1, s.length() - i));
   }

   return("");
}

int main(int argc, char** argv) {
	cv::Mat img_1, img_2, contours_mat, gradient, grayGradient;
	double threshold = 0.01; //0.05;
	int starting_scale = 0;

	if (argc < 5) {
		std::cout
				<< "Usage: $> ./segmenter <input img 1> <input img 2> <output path> <scales> [<use gpu, default 0=false>]"
				<< std::endl;
		return 0;
	}

	img_1 = cv::imread(argv[1], -1);
	img_2 = cv::imread(argv[2], -1);
	std::string outputPath(argv[3]);
	string prefix_1 = getFileName(argv[1]);
	string prefix_2 = getFileName(argv[2]);
	cout <<"outputPath="<<outputPath<< endl<< " prefix_1="<<prefix_1<<endl<< " prefix_2="<<prefix_2<<endl;

	int scales = atoi(argv[4]);
	int gpu = 0;
	if (argc == 6) {
		gpu = atoi(argv[5]);
	} else if (argc == 7) {
		gpu = atoi(argv[5]);
		threshold = atof(argv[6]);
	}

	bool use_gpu = gpu > 0 ? true : false;

	/*
	 * process img_1
	 */
	Segmentation segmentation_1(img_1, gpu, scales, starting_scale);
	segmentation_1.segment_pyramid(threshold);
	const vector<Segment*>& segments_1 = segmentation_1.getSegmentsPyramid()[2];
	vector<Atom*> atoms_1;
	for (Segment* seg : segments_1) {
		Atom * atom = new Atom(seg);
		atoms_1.push_back(atom);
	}
	cout << "> constructing visual representation 1" << endl;

	VisualRepresentation representation_1(atoms_1, prefix_1);
	imwrite("segmentation1.png", segmentation_1.getOutputSegmentsPyramid()[2]);

	/*
	 * process img_2
	 */
	Atom::static_id = 0;
	Segmentation segmentation_2(img_2, gpu, scales, starting_scale);
	segmentation_2.segment_pyramid(threshold);
	const vector<Segment*>& segments_2 = segmentation_2.getSegmentsPyramid()[2];
	vector<Atom*> atoms_2;
	for (Segment* seg : segments_2) {
		Atom * atom = new Atom(seg);
		atoms_2.push_back(atom);
	}
	cout << "> constructing visual representation 2" << endl;
	VisualRepresentation representation_2(atoms_2, prefix_2);
	imwrite("segmentation2.png", segmentation_2.getOutputSegmentsPyramid()[2]);
//	representation_1.compare_to(representation_2);

	GraphsMatch graph_match(representation_1,representation_2);
	graph_match.find_match();

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

