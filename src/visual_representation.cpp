/*
 * visual_representation.cpp
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

#include "visual_representation.h"
#include "visualisation.h"

using namespace std;

namespace videoseg {

VisualRepresentation::VisualRepresentation() {
	// TODO Auto-generated constructor stub

}

VisualRepresentation::VisualRepresentation(vector<Atom*>& atoms, std::string prefix):
		atoms_(atoms),segments(atoms.size()){

	vector<MyAtomGraph::Vertex> vertices(atoms.size());
	for (Atom* atom : atoms) {
		MyAtomGraph::Vertex u = g_.AddVertex(*atom);
		//cout <<"atom->id="<<atom->id <<" vertices.size()="<<vertices.size()<<endl;
		vertices[atom->id] = u;

	}

	for (int i = 0; i < atoms.size(); i++) {
		for (int j = i + 1; j < atoms.size(); j++) {
			if (i != j) {
				Atom* atom1 = atoms[i];
				Atom* atom2 = atoms[j];

				if (attached_to(*atom1, *atom2)) {
					const Mat& seg1_mat = atom1->segment_->getRandomColourMat();
					const Mat& seg2_mat = atom2->segment_->getRandomColourMat();

					EdgeProperties edge_props(atom1->id,atom2->id);
					g_.AddEdge(vertices[atom1->id], vertices[atom2->id],
							edge_props);

				}
			}
		}
	}
	//DEBUG
	//iterate_nodes();
	//iterate_edges();
	Visualisation vis;
	cout <<"prefix="<<prefix<<endl;
	vis.write_gv_file(g_,prefix,prefix);

}

VisualRepresentation::~VisualRepresentation() {
// TODO Auto-generated destructor stub
}

void VisualRepresentation::compare_to(VisualRepresentation& visual_rep_2){
	//iterate this' nodes and compare one by one to those in visual_rep_2

	for(Atom* atom_u1 : atoms_){
		double max = 0.;
		Atom* max_atom;
		for(Atom* atom_v2 : visual_rep_2.atoms_){

			double score=atom_u1->similarity(*atom_v2);
			//cout <<"score="<<score<<endl;
			if(score>max){
				max = score;
				max_atom = atom_v2;
			}

		}
		imshow("atom_u1",atom_u1->segment_->getMatOriginalColour());
		imshow("max atom_v2",max_atom->segment_->getMatOriginalColour());
		atom_u1->similarity(*max_atom);

		waitKey(0);

	}


}



void VisualRepresentation::match(VisualRepresentation& visual_rep_2){



}


void VisualRepresentation::print_adjacent_nodes(MyAtomGraph::Vertex& node) {
	MyAtomGraph::adjacency_iter i, end;
	for (tie(i, end) = g_.getAdjacentVertices(node); i != end; ++i) {
		auto neighbour = *i;
		cout << g_.properties(node).id <<" -> " << g_.properties(neighbour).id << ";" <<endl;
	}
}

void VisualRepresentation::iterate_nodes() {

	//iterate over the nodes
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = g_.getVertices(); i != end; ++i) {
		MyAtomGraph::Vertex node = *i;
		std::cout << "> " << g_.properties(node).id << " adjacent nodes= ";
		//get adjacent nodes
		print_adjacent_nodes(node);
		std::cout << std::endl;

	}
}

void VisualRepresentation::iterate_edges() {

	MyAtomGraph::edge_iter e, end;
	for (tie(e, end) = g_.getEdges(); e != end; ++e) {
		MyAtomGraph::Edge edge = *e;
		std::cout <<" edge: "<<g_.properties(edge).source<<"->"<<g_.properties(edge).dest<<std::endl;
	}

}

/*
 * returns true if the segment of atom1 shares a common border
 * with that of atom2
 */
bool VisualRepresentation::attached_to(Atom& atom1, Atom& atom2) {

	Mat intersection;
	const Mat& seg1_mat = atom1.segment_->getBinaryMat();
	const Mat& seg2_mat = atom2.segment_->getBinaryMat();

	cv::bitwise_and(seg1_mat, seg2_mat, intersection);

	Scalar pixels_inter = countNonZero(intersection);
	int size_a = atom1.segment_->getSegmentSize();
	int size_b = atom2.segment_->getSegmentSize();

//	if((atom1.id == 8 && atom2.id == 35)||atom2.id == 8 && atom1.id == 35){
//		imshow("seg1_mat",seg1_mat);
//		imshow("seg2_mat",seg2_mat);
//		imshow("intersection",intersection);
//		cout <<"pixels_inter="<<pixels_inter[0]<<endl;
//		waitKey(0);
//
//	}

	double ratio =
			size_a > size_b ?
					pixels_inter[0] / size_a : pixels_inter[0] / size_b;

	if (pixels_inter[0] > 0)
		return true;

//
//
//  if (sum(intersection)[0] > 0 && pixels_inter[0] > 1) {
//
////      Point2d gradient = orientation(intersection);
////      std::cout << "orientation of contour: " << gradient.x << " " << gradient.y
////          << std::endl;
//
//    // cout <<" ratio of pixels "<<ratio<<endl;
//    return true;
//  }
	return false;
}

void VisualRepresentation::get_segmentation_mat(Mat& dst){

	dst = Mat::zeros(atoms_[0]->segment_->getBinaryMat().rows,atoms_[0]->segment_->getBinaryMat().cols,CV_8UC3);
	for(Atom* atom: atoms_){
		Segment* seg = atom->segment_;
		seg->getRandomColourMat().copyTo(dst(seg->getBoundRect()),seg->getMask());
	}

}


} /* namespace videoseg */
