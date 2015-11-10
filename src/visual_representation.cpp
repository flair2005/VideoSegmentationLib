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
	cout <<"VisualRepresentation::VisualRepresentation()"<<endl;
}

VisualRepresentation::VisualRepresentation(vector<std::shared_ptr<Atom> >& atoms, std::string prefix):
		segments(atoms.size()){

	cout <<"VisualRepresentation::VisualRepresentation(vector<AtomRef>& atoms, std::string prefix)"<<this <<endl;
	vector<MyUniqueAtomGraph::Vertex> vertices(atoms.size());
	for (std::shared_ptr<Atom>& atom : atoms) {
		AtomRef atom_ref;
		atom_ref.atom_ptr = atom;
		MyUniqueAtomGraph::Vertex u = g_.AddVertex(atom_ref);
		atoms_.push_back(atom_ref);
		//cout <<"atom->id="<<atom->id <<" vertices.size()="<<vertices.size()<<endl;
		vertices[atom->id] = u;

	}


	for (unsigned int i = 0; i < atoms_.size(); i++) {
		for (unsigned int j = i + 1; j < atoms_.size(); j++) {
			if (i != j) {
				AtomRef& atom1 = atoms_[i];
				AtomRef& atom2 = atoms_[j];

				if (attached_to(atom1.atom_ptr, atom2.atom_ptr)) {
					const Mat& seg1_mat = atom1.atom_ptr->segment_->getRandomColourMat();
					const Mat& seg2_mat = atom2.atom_ptr->segment_->getRandomColourMat();

					double angle = atan2(atom2.atom_ptr->segment_->getCenter().x - atom1.atom_ptr->segment_->getCenter().x,
							atom2.atom_ptr->segment_->getCenter().y - atom1.atom_ptr->segment_->getCenter().y);
					EdgeProperties edge_props(atom1.atom_ptr->id,atom2.atom_ptr->id,angle);
					g_.AddEdge(vertices[atom1.atom_ptr->id], vertices[atom2.atom_ptr->id],
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
//	cout <<"~VisualRepresentation() :<<"<< this <<"atoms_.size()="<<atoms_.size()<<endl;
//	for(AtomRef atom: atoms_)
//		delete atom;

}

void VisualRepresentation::compare_to(VisualRepresentation& visual_rep_2){
	//iterate this' nodes and compare one by one to those in visual_rep_2

	for(AtomRef& atom_u1 : atoms_){
		double max = 0.;
		AtomRef max_atom;
		for(AtomRef atom_v2 : visual_rep_2.atoms_){

			double score=atom_u1.atom_ptr->similarity(atom_v2.atom_ptr.get());
			//cout <<"score="<<score<<endl;
			if(score>max){
				max = score;
				max_atom = atom_v2;
			}

		}
		imshow("atom_u1",atom_u1.atom_ptr->segment_->getMatOriginalColour());
		imshow("max atom_v2",max_atom.atom_ptr->segment_->getMatOriginalColour());
		atom_u1.atom_ptr->similarity(max_atom.atom_ptr.get());

		waitKey(0);

	}


}



void VisualRepresentation::match(VisualRepresentation& visual_rep_2){



}


void VisualRepresentation::print_adjacent_nodes(MyUniqueAtomGraph::Vertex& node) {
	MyUniqueAtomGraph::adjacency_iter i, end;
	for (tie(i, end) = g_.getAdjacentVertices(node); i != end; ++i) {
		auto neighbour = *i;
		cout << g_.properties(node).atom_ptr->id <<" -> " << g_.properties(neighbour).atom_ptr->id << ";" <<endl;
	}
}

void VisualRepresentation::iterate_nodes() {

	//iterate over the nodes
	MyUniqueAtomGraph::vertex_iter i, end;
	for (tie(i, end) = g_.getVertices(); i != end; ++i) {
		MyUniqueAtomGraph::Vertex node = *i;
		std::cout << "> " << g_.properties(node).atom_ptr->id << " adjacent nodes= ";
		//get adjacent nodes
		print_adjacent_nodes(node);
		std::cout << std::endl;

	}
}

void VisualRepresentation::iterate_edges() {

	MyUniqueAtomGraph::edge_iter e, end;
	for (tie(e, end) = g_.getEdges(); e != end; ++e) {
		MyUniqueAtomGraph::Edge edge = *e;
		std::cout <<" edge: "<<g_.properties(edge).source<<"->"<<g_.properties(edge).dest<<std::endl;
	}

}

/*
 * returns true if the segment of atom1 shares a common border
 * with that of atom2
 */
bool VisualRepresentation::attached_to(std::shared_ptr<Atom> atom1, std::shared_ptr<Atom> atom2) {

	Mat intersection;
	const Mat& seg1_mat = atom1->segment_->getBinaryMat();
	const Mat& seg2_mat = atom2->segment_->getBinaryMat();

	cv::bitwise_and(seg1_mat, seg2_mat, intersection);

	Scalar pixels_inter = countNonZero(intersection);
	int size_a = atom1->segment_->getSegmentSize();
	int size_b = atom2->segment_->getSegmentSize();

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

	dst = Mat::zeros(atoms_[0].atom_ptr->segment_->getBinaryMat().rows,atoms_[0].atom_ptr->segment_->getBinaryMat().cols,CV_8UC3);
	for(AtomRef& atom: atoms_){
		Segment* seg = atom.atom_ptr->segment_;
		seg->getRandomColourMat().copyTo(dst(seg->getBoundRect()),seg->getMask());
	}

}


} /* namespace videoseg */
