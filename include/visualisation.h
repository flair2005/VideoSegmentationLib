/*
 * visualisation.h
 *
 *  Created on: 30 Oct 2015
 *      Author: martin
 */

#ifndef VISUALISATION_H_
#define VISUALISATION_H_

#include "visual_representation.h"
#include "atom_graphs.h"

#define REAL_COLOURS false

using namespace videoseg;

namespace videoseg {

class Visualisation {
public:
	Visualisation();
	virtual ~Visualisation();

	void write_gv_file(MyAtomGraph& g,std::string winname, string prefix);

private:
	void print_adjacent_nodes(std::ofstream& dotFile, const MyAtomGraph& g, MyAtomGraph::Vertex& node);

	void print_mat_adjacent_nodes(std::ofstream& dotFile, const MyAtomGraph& g, MyAtomGraph::Vertex& node);

	void writeNodes(MyAtomGraph& g, std::string prefix);

	void writeDot(MyAtomGraph& g,string fileName, string prefix);

	void writeUDot(MyAtomGraph& g, std::string fileName);

	bool display_real_colours;
};

} /* namespace imgseg */

#endif /* VISUALISATION_H_ */
