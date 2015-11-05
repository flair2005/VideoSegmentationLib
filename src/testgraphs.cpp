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



using namespace boost;
using namespace videoseg;
//using namespace std;



void print_adjacent_nodes(const MyAtomGraph& g, MyAtomGraph::Vertex& node) {
	MyAtomGraph::adjacency_iter i, end;
	for (tie(i, end) = g.getAdjacentVertices(node); i != end; ++i) {
		auto neighbour = *i;
		cout << g.properties(node).id <<" -> " << g.properties(neighbour).id << ";" <<endl;
	}
}



int _main(int argc, char** argv) {

	MyAtomGraph g;
	Atom atom1, atom2;
	MyAtomGraph::Vertex u = g.AddVertex(atom1);
	MyAtomGraph::Vertex v = g.AddVertex(atom2);

	std::cout << " id of node u=" << g.properties(u).id << endl;
	std::cout << " id of node v=" << g.properties(v).id << endl;

	EdgeProperties edge_props1, edge_props2;
	g.AddEdge(u, v, edge_props1, edge_props2);

	std::cout << "# of vertices=" << g.getVertexCount() << endl;

	std::cout << "vertices(g) = ";

	//iterate over the nodes
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = g.getVertices(); i != end; ++i) {
		MyAtomGraph::Vertex node = *i;
		std::cout << "> " << g.properties(node).id << " adjacent nodes= ";
		//get adjacent nodes
		print_adjacent_nodes(g, node);
		std::cout << std::endl;

	}
	std::cout << std::endl;
	Visualisation vis;

	return 0;

}

int main(int argc, char** argv) {
	cv::Mat original_img,contours_mat, gradient,grayGradient;
	double threshold = 0.01;//0.05;
	int starting_scale = 0;

	if(argc < 4){
		std::cout <<"Usage: $> ./segmenter <input img> <output path> <scales> [<use gpu, default 0=false>]"<< std::endl;
		return 0;
	}

	original_img = cv::imread(argv[1], -1);
	std::string outputPath(argv[2]);
	std::string prefix(argv[3]);
	int scales = atoi(argv[4]);
	int gpu = 0;
	if(argc == 6){
		gpu = atoi(argv[5]);
	}
	else if(argc == 7){
		gpu = atoi(argv[5]);
		threshold = atof(argv[6]);
	}

	bool use_gpu = gpu > 0 ? true: false;
	Segmentation segmentation(original_img,gpu,scales,starting_scale);
	segmentation.segment_pyramid(threshold);
	const vector<Segment*>& segments = segmentation.getSegmentsPyramid()[2];
	vector<Atom*> atoms;
	for(Segment* seg : segments){
		Atom * atom = new Atom(seg);
		atoms.push_back(atom);
	}
	cout <<"> constructing visual representation"<<endl;
	VisualRepresentation representation(atoms,prefix);
	imwrite("segmentation.png",segmentation.getOutputSegmentsPyramid()[2]);

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

