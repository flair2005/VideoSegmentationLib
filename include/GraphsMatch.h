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

	virtual ~GraphsMatch();

private:
	VisualRepresentation visual_repr_1_,visual_repr_2_;
	vector< vector <double > > scores_;



};

} /* namespace videoseg */

#endif /* GRAPHSMATCH_H_ */
