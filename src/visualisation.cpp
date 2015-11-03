/*
 * visualisation.cpp
 *
 *  Created on: 30 Oct 2015
 *      Author: martin
 */

#include "visualisation.h"
#include "visual_representation.h"
#include "atom_graphs.h"
#include <iostream>
#include <graphviz/gvc.h>

#include <iostream>                  // for std::cout
#include <fstream>

namespace videoseg {

Visualisation::Visualisation():display_real_colours(REAL_COLOURS) {
	// TODO Auto-generated constructor stub

}

Visualisation::~Visualisation() {
	// TODO Auto-generated destructor stub
}

void Visualisation::writeNodes(MyAtomGraph& g, std::string prefix) {


	//iterate over the nodes
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = g.getVertices(); i != end; ++i) {
		MyAtomGraph::Vertex node = *i;
		Atom& atom = g.properties(node);

		if(display_real_colours)
			imwrite("nodemats/"+prefix+"node" + to_string(atom.id) + ".png",
				atom.segment_->getMatOriginalColour());
		else
			imwrite("nodemats/"+prefix+"node" + to_string(atom.id) + ".png",
							atom.segment_->getRandomColourMat());
	}

}

void Visualisation::print_adjacent_nodes(std::ofstream& dotFile,
		const MyAtomGraph& g, MyAtomGraph::Vertex& node) {
	MyAtomGraph::adjacency_iter i, end;
	for (tie(i, end) = g.getAdjacentVertices(node); i != end; ++i) {
		auto neighbour = *i;
		dotFile << g.properties(node).id << " -> " << g.properties(neighbour).id
				<< ";" << std::endl;
	}
}

void Visualisation::print_mat_adjacent_nodes(std::ofstream& dotFile,
		const MyAtomGraph& g, MyAtomGraph::Vertex& node) {
	MyAtomGraph::adjacency_iter i, end;
	for (tie(i, end) = g.getAdjacentVertices(node); i != end; ++i) {
		auto neighbour = *i;
		dotFile << g.properties(node).id << " -> " << g.properties(neighbour).id
				<< ";" << std::endl;
	}
}

void Visualisation::writeDot(MyAtomGraph& g, string fileName, string prefix) {

	ofstream dotFile;
	fileName = fileName + ".gv";
	dotFile.open(fileName.c_str());
	dotFile << "graph g {\n";

	//iterate over the nodes
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = g.getVertices(); i != end; ++i) {

		MyAtomGraph::Vertex node = *i;
		Atom& atom = g.properties(node);

		string element =
				to_string(atom.id)
						+ " [margin=0 shape=circle, style=bold, label=<<TABLE border='0' cellborder='0'><TR><TD><IMG SRC='/home/martin/workspace/VideoSegmentationLib/build/nodemats/"+prefix+ "node"
						+ to_string(atom.id) + ".png'/></TD><TD>"
						+ to_string(atom.id) + "</TD></TR></TABLE>>]";
		dotFile << element << endl;

		//get adjacent nodes
		//print_mat_adjacent_nodes(dotFile, g, node);
	}
	//iterate over the edges and write them
	MyAtomGraph::edge_iter e, e_end;
		for (tie(e, e_end) = g.getEdges(); e != e_end; ++e) {
			MyAtomGraph::Edge edge = *e;
			dotFile << g.properties(edge).source << " -- " << g.properties(edge).dest

					<< ";"<< endl;
		}

//	for (tie(i, end) = g.getVertices(); i != end; ++i) {
//		MyAtomGraph::Vertex node = *i;
//		Atom& atom = g.properties(node);
//		MyAtomGraph::adjacency_iter j ,end_j;
//		for (tie(j, end_j) = g.getAdjacentVertices(node); j != end_j; ++j) {
//
//			MyAtomGraph::Vertex neigh_node = *j;
//			Atom& neigh_atom = g.properties(neigh_node);
//
//			dotFile << atom.id << " -> " << neigh_atom.id
//					//<< "[label=\"" << 0 << "\"][color=\""
//					//<< 1				<< "\"];"
//					<< ";"<< endl;
//
//		}
//
//	}



	dotFile << "}";
	dotFile.close();

}

void Visualisation::writeUDot(MyAtomGraph& g, std::string fileName) {

	std::ofstream dotFile;
	fileName = fileName + ".gv";
	dotFile.open(fileName.c_str());
	dotFile << "graph {\n";

	//iterate over the nodes
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = g.getVertices(); i != end; ++i) {
		MyAtomGraph::Vertex node = *i;
		//get adjacent nodes
		print_adjacent_nodes(dotFile, g, node);
	}

	dotFile << "}";
	dotFile.close();

}

void Visualisation::write_gv_file(MyAtomGraph& g, std::string winname, string prefix) {

	//write the node png files so that they can be referenced in the next function call
	writeNodes(g, prefix);
	//write the DOT file
	writeDot(g, winname, prefix);

//	static GVC_t *gvc = NULL;
//	if (!gvc) {
//		// 	cout <<"calling gvContext()"<<endl;
//		gvc = gvContext();
//	}
//
//	std::string dot("dot ");
//	std::string tpng("-Tpng ");
//	std::string fileName = winname + ".gv ";
//	std::string outputFileName = "-o" + winname + ".png ";
//	std::string command("dot -Tpng " + winname + ".gv -o " + winname + ".png");
//	std::system(command.c_str());

}

} /* namespace imgseg */
