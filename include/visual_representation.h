/*
 * visual_representation.h
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

#ifndef VISUAL_REPRESENTATION_H_
#define VISUAL_REPRESENTATION_H_

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "atom.h"
#include "atom_graphs.h"
#include <memory>

using namespace boost;
using namespace videoseg;
using namespace std;

struct EdgeProperties {

	EdgeProperties():source(-1),dest(-1),angle(0.){};
	EdgeProperties(int source,int dest):
		source(source),dest(dest),angle(0.){};
	EdgeProperties(int source,int dest, double angle):
			source(source),dest(dest),angle(angle){};

	int source;
	int dest;
	double angle;

	bool operator>(const EdgeProperties &e2) const
	{
		//cout << "operator trick"<<endl;
		return angle > e2.angle;
	}
};



typedef Graph<videoseg::Atom, EdgeProperties> MyAtomGraph;
typedef Graph<std::unique_ptr <videoseg::Atom>, EdgeProperties> MyUniqueAtomGraph;



struct VertexProperties {
	int i;
};
typedef Graph<VertexProperties, EdgeProperties> MyGraph;



namespace videoseg {

class VisualRepresentation {


public:
	VisualRepresentation();
	VisualRepresentation(vector<Atom*>& atoms, std::string prefix);
	virtual ~VisualRepresentation();

	void compare_to(VisualRepresentation& visual_rep_2);



	void match(VisualRepresentation& visual_rep_2);

	int getSegments() const {
		return segments;
	}

	const vector<MyAtomGraph::Vertex*>& getVertices() const {
		return vertices_;
	}

	const MyAtomGraph& getG() const {
		return g_;
	}

	const vector<Atom*>& getAtoms() const {
		return atoms_;
	}

	void get_segmentation_mat(Mat& seg);

private:
	bool attached_to(Atom& atom1, Atom& atom2);
	void print_adjacent_nodes(MyAtomGraph::Vertex& node);
	void iterate_nodes();
	void iterate_edges();
protected:
	MyAtomGraph g_;
	vector<MyAtomGraph::Vertex*> vertices_;
	vector<Atom*> atoms_;
	int segments;

};

} /* namespace imgseg */

#endif /* VISUAL_REPRESENTATION_H_ */
