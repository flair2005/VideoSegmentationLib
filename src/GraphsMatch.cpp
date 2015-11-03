/*
 * GraphsMatch.cpp
 *
 *  Created on: 2 Nov 2015
 *      Author: martin
 */

#include "GraphsMatch.h"

namespace videoseg {

GraphsMatch::GraphsMatch(VisualRepresentation& visual_repr_1,VisualRepresentation& visual_repr_2):
		visual_repr_1_(visual_repr_1),visual_repr_2_(visual_repr_2){
	// TODO Auto-generated constructor stub
	precompute_scores();

}

void GraphsMatch::precompute_scores(){

	//allocate memory: +0 since the atom ids start with 0
	cout <<"scores resized for "<<visual_repr_1_.getSegments()<<endl;
	scores_.resize(visual_repr_1_.getSegments());
	for(int index = 0;index<visual_repr_1_.getSegments();index++)
		scores_[index].resize(visual_repr_2_.getSegments());

	//iterate over the nodes of visual_rep_1
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = visual_repr_1_.getG().getVertices(); i != end; ++i) {
		MyAtomGraph::Vertex node_u1 = *i;
		const Atom& atom_u1=visual_repr_1_.getG().properties(node_u1);
		//cout << "> u1[" << visual_repr_1_.getG().properties(node_u1).id<<"]" <<endl;

		//iterate over the nodes of visual_rep_2
		MyAtomGraph::vertex_iter j, jend;
		for (tie(j, jend) = visual_repr_2_.getG().getVertices(); j != jend; ++j) {
			MyAtomGraph::Vertex node_v2 = *j;
			const Atom& atom_v2=visual_repr_2_.getG().properties(node_v2);
			double score = atom_u1.similarity(atom_v2);
			scores_[atom_u1.id][atom_v2.id] = score;
			cout <<"scores_["<<atom_u1.id<<"]["<<atom_v2.id<<"] ="<<score<<endl;
		}
//		std::cout << "> " << g_.properties(node).id << " adjacent nodes= ";
//		//get adjacent nodes
//		print_adjacent_nodes(node);
//		std::cout << std::endl;

	}
}

void GraphsMatch::find_match(){
	for(int i=0;i<visual_repr_1_.getSegments();i++){
		double max = 0.;
		int max_index_v2 = -1;
		for(int j=0;j<visual_repr_2_.getSegments();j++){
			if(scores_[i][j] > max){
				max = scores_[i][j];
				max_index_v2 = j;
			}
		}
		visual_repr_1_.getAtoms()[i]->id_matched_to=max_index_v2;
	}


	for(int i=0;i<visual_repr_1_.getSegments();i++){
		int match = visual_repr_1_.getAtoms()[i]->id_matched_to;
		Segment* seg1 = visual_repr_1_.getAtoms()[i]->segment_;
		Segment* seg2 = visual_repr_2_.getAtoms()[match]->segment_;

		seg2->re_colour(seg1->getRandomColour());
		cout <<"node U1 "<<visual_repr_1_.getAtoms()[i]->id <<" matched to "<<match<<endl;
		cout <<"colours set as: "<<seg1->getRandomColour()<<" "<<seg2->getRandomColour()<<endl;

	}



}

void GraphsMatch::getMats(Mat& seg1,Mat& seg2){
	//print the segmentations

	visual_repr_1_.get_segmentation_mat(seg1);
	visual_repr_2_.get_segmentation_mat(seg2);
//	imshow("seg1",seg1);
//	imshow("seg2",seg2);
//	waitKey(0);

}

GraphsMatch::~GraphsMatch() {
	// TODO Auto-generated destructor stub
}

} /* namespace imgseg */
