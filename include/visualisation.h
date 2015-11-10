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

	void write_gv_file(MyUniqueAtomGraph& g,std::string winname, string prefix);

private:
	void print_adjacent_nodes(std::ofstream& dotFile, const MyUniqueAtomGraph& g, MyUniqueAtomGraph::Vertex& node);

	void print_mat_adjacent_nodes(std::ofstream& dotFile, const MyUniqueAtomGraph& g, MyUniqueAtomGraph::Vertex& node);

	void writeNodes(MyUniqueAtomGraph& g, std::string prefix);

	void writeDot(MyUniqueAtomGraph& g,string fileName, string prefix);

	void writeUDot(MyUniqueAtomGraph& g, std::string fileName);

	bool display_real_colours;
};

} /* namespace imgseg */

#endif /* VISUALISATION_H_ */
