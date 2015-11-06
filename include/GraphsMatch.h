/*
 * GraphsMatch.h
 *
 *  Created on: 2 Nov 2015
 *      Author: martin
 */

#ifndef GRAPHSMATCH_H_
#define GRAPHSMATCH_H_

#include "visual_representation.h"
#include <iostream>
#include "atom.h"
#include "atom_graphs.h"


using namespace boost;
using namespace videoseg;
using namespace std;

namespace videoseg {

class GraphsMatch {
public:
	GraphsMatch(VisualRepresentation& visual_repr_1,VisualRepresentation& visual_repr_2);

	void precompute_scores();

	void find_match();

	void getMats(Mat& seg1,Mat& seg2);

	void structural();

	virtual ~GraphsMatch();

private:
	VisualRepresentation visual_repr_1_,visual_repr_2_;

	vector< vector <double > > appearance_scores_;
	vector< vector <double > > structural_scores_;

	void print_vector(vector<int>& vect);

	void generate_permutations(vector< vector<int> >& permutations,vector<int>& adj_vector);

	inline const Atom& g1_get_atom(MyAtomGraph::vertex_iter i);

	inline const Atom& g2_get_atom(MyAtomGraph::vertex_iter j);

	void get_neighbours(VisualRepresentation& visual_repr,MyAtomGraph::Vertex& node, vector<int>& ids);

	double probability(double similarity);

	double structural_similarity(vector<int>& ids_g1,vector<int>& ids_g2);

	const int p_err = 0.4;



};

} /* namespace videoseg */

#endif /* GRAPHSMATCH_H_ */
