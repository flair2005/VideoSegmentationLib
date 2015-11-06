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

const Atom& GraphsMatch::g1_get_atom(MyAtomGraph::vertex_iter i){
	return visual_repr_1_.getG().properties(*i);
}

const Atom& GraphsMatch::g2_get_atom(MyAtomGraph::vertex_iter j){
	return visual_repr_2_.getG().properties(*j);
}

void GraphsMatch::get_neighbours(VisualRepresentation& visual_repr, MyAtomGraph::Vertex& node, vector<int>& ids){

	MyAtomGraph::adjacency_iter i, end;
	for (tie(i, end) = visual_repr.getG().getAdjacentVertices(node); i != end; ++i) {
		auto neighbour = *i;
		//cout << visual_repr.getG().properties(node).id <<" -> " << visual_repr.getG().properties(neighbour).id << ";" <<endl;
		ids.push_back(visual_repr.getG().properties(neighbour).id);
	}

}
void GraphsMatch::generate_permutations(vector< vector<int> >& permutations,vector<int>& adj_vector){

	int perms  = adj_vector.size();
	for (int i = 0; i < perms; i++) {
			int pos = i;
			vector<int> permutation;
			//permutation.push_back(center);
			for (int count = 0; count < perms; count++) {
				pos = (pos + 1) % perms;
				permutation.push_back(adj_vector[pos]);
			}
			permutations.push_back(permutation);
		}
}

void GraphsMatch::print_vector(vector<int>& vect){
	for(int i=0;i<vect.size();i++){
		cout << vect[i]<< " ";
	}
	cout << endl;
}

double GraphsMatch::probability(double similarity){

	double distance = 1- similarity;
	double dist_square = distance*distance;
	double lambda = 3.0;
	return exp(-dist_square * lambda);

}
/*
 * return the structural similarity between two permutations of neighbour nodes
 */
double GraphsMatch::structural_similarity(vector<int>& ids_g1,vector<int>& ids_g2){
	double acc_score = 1.;
	unsigned int diff_size = 0;
	if(ids_g1.size() <= ids_g2.size()){
		for(int i = 0;i<ids_g1.size();i++){
			acc_score *= appearance_scores_[ids_g1[i]][ids_g2[i]];
		}
		diff_size = ids_g2.size() - ids_g1.size();
	}
	else{
		for(int i = 0;i<ids_g2.size();i++){
			acc_score *= appearance_scores_[ids_g1[i]][ids_g2[i]];
		}
		diff_size = ids_g1.size() - ids_g2.size();
	}

	for(unsigned int i=0;i<diff_size;i++)
		acc_score *= p_err;
	return acc_score;
}

void GraphsMatch::structural(){

	//iterate over the nodes of graph 1
	MyAtomGraph::vertex_iter i, end;
	for (tie(i, end) = visual_repr_1_.getG().getVertices(); i != end; ++i) {
		//get the node
		MyAtomGraph::Vertex node_u1 = *i;
		vector<int> neighbours_u1;
		get_neighbours(visual_repr_1_,node_u1,neighbours_u1);
		const Atom&  atom_u1 = g1_get_atom(i);
		//get its neighbours' indexes
		cout <<"Node u1: "<<atom_u1.id <<" neighbours: ";
		print_vector(neighbours_u1);

		double best_candidate_score = -0.1;
		int id_best_match = -1;
		//iterate over the nodes of graph 2
		MyAtomGraph::vertex_iter j, jend;
		for (tie(j, jend) = visual_repr_2_.getG().getVertices(); j != jend; ++j) {
			MyAtomGraph::Vertex node_v2 = *j;
			vector<int> neighbours_v2;
			get_neighbours(visual_repr_2_,node_v2,neighbours_v2);
			const Atom&  atom_v2 = g2_get_atom(j);
			//cout <<"Node v2: "<<atom_v2.id <<" neighbours: ";
			//print_vector(neighbours_v2);

			//generate permutations for the neighbours_u1 of node u1
			vector< vector<int> > permutations;
			generate_permutations(permutations,neighbours_u1);
			double max_score = -0.1;
			int best_permutation = -1;
			for(unsigned int k=0;k<permutations.size();k++){

				double struct_score = structural_similarity(permutations[k],neighbours_v2);
				if(struct_score > max_score){
					max_score = struct_score;
					best_permutation = k;
				}
				//cout <<"permutation #"<<k<<" with score= "<<struct_score << endl;
				//print_vector(permutations[k]);
			}
			structural_scores_[atom_u1.id][atom_v2.id] = probability(max_score);
			//cout <<"best permutation #"<<best_permutation<<" with score= "<<max_score<<endl;
			//print_vector(permutations[best_permutation]);
			if(max_score>best_candidate_score){
				best_candidate_score = max_score;
				id_best_match = atom_v2.id;
			}

		}
		cout <<"Best match node v2: "<<id_best_match <<" score= "<<best_candidate_score<<endl;


	}

}

/*
 *  It pre-computes the scores array of similarity
 *  between nodes from graphs G1 and G2.
 *
 *	scores is a 2-D array of doubles that contains
 *	the similarity scores between nodes
 *
 *        v2.id
 *        ----------
 * u1.id |	|	|	|
 *       |	|	|	|
 *       |	|	|	|
 * 		 ------------
 */
void GraphsMatch::precompute_scores(){

	//allocate memory: +0 since the atom ids start with 0
	cout <<"scores resized for "<<visual_repr_1_.getSegments()<<endl;
	structural_scores_.resize(visual_repr_1_.getSegments());
	appearance_scores_.resize(visual_repr_1_.getSegments());
	for(int index = 0;index<visual_repr_1_.getSegments();index++){
		appearance_scores_[index].resize(visual_repr_2_.getSegments());
		structural_scores_[index].resize(visual_repr_2_.getSegments());
	}

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
			double score = (atom_u1.similarity(atom_v2));
			appearance_scores_[atom_u1.id][atom_v2.id] = probability(score);

			//cout <<"appearance_scores_["<<atom_u1.id<<"]["<<atom_v2.id<<"] ="<<score<<" prob="<<probability(score)<<endl;
		}
	}
	//structural similarities
	structural();
}



void GraphsMatch::find_match(){
	for(int i=0;i<visual_repr_1_.getSegments();i++){
		double max = 0.;
		int max_index_v2 = -1;
		for(int j=0;j<visual_repr_2_.getSegments();j++){
			double score = structural_scores_[i][j]* appearance_scores_[i][j];
			if(score > max){
				max = score;
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
		//cout <<"node U1 "<<visual_repr_1_.getAtoms()[i]->id <<" matched to "<<match<<endl;
		//cout <<"colours set as: "<<seg1->getRandomColour()<<" "<<seg2->getRandomColour()<<endl;

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
