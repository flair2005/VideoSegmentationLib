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


//using namespace boost;
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
	VisualRepresentation& visual_repr_1_,visual_repr_2_;

	vector< vector <double > > appearance_scores_;
	vector< vector <double > > structural_scores_;

	void print_vector(vector<int>& vect);

	double solve_assignment(vector<int>& adj_vector_u1,vector<int>& adj_vector_v2);

	void generate_factorial_permutations(vector<vector<int> >& permutations,vector<int>& adj_vector);

	void generate_permutations(vector< vector<int> >& permutations,vector<int>& adj_vector);

	inline const AtomRef& g1_get_atom(MyUniqueAtomGraph::vertex_iter i);

	inline const AtomRef& g2_get_atom(MyUniqueAtomGraph::vertex_iter j);

	void get_neighbours(VisualRepresentation& visual_repr,MyUniqueAtomGraph::Vertex& node, vector<int>& ids);

	double probability(double similarity);

	double structural_similarity(vector<int>& ids_g1,vector<int>& ids_g2);

	const double p_err = 0.5;
	const double p_structural_err = 0.1;



};

} /* namespace videoseg */

#endif /* GRAPHSMATCH_H_ */
