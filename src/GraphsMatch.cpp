/*
 * GraphsMatch.cpp
 *
 *  Created on: 2 Nov 2015
 *      Author: martin
 */

#include <iostream>     // std::cout
#include <algorithm>    // std::next_permutation, std::sort
#include "GraphsMatch.h"
#include "hungarian/hungarian.h"
// next_permutation example

namespace videoseg {

GraphsMatch::GraphsMatch(VisualRepresentation& visual_repr_1,
		VisualRepresentation& visual_repr_2) :
		visual_repr_1_(visual_repr_1), visual_repr_2_(visual_repr_2) {
	// TODO Auto-generated constructor stub
	precompute_scores();

}

const AtomRef& GraphsMatch::g1_get_atom(MyUniqueAtomGraph::vertex_iter i) {
	return visual_repr_1_.getG().properties(*i);
}

const AtomRef& GraphsMatch::g2_get_atom(MyUniqueAtomGraph::vertex_iter j) {
	return visual_repr_2_.getG().properties(*j);
}

struct atom_struct {

	atom_struct(int id, double angle) :
			id(id), angle(angle) {
	}
	;
	int id;
	double angle;
};

bool c1_ptr_less(const atom_struct &s1, const atom_struct &s2) {
	return (s1.angle < s2.angle);
}

void GraphsMatch::get_neighbours(VisualRepresentation& visual_repr,
		MyUniqueAtomGraph::Vertex& node, vector<int>& ids) {

	MyUniqueAtomGraph::adjacency_iter i, end;
	vector<atom_struct> atoms;
	const AtomRef& atom1 = visual_repr.getG().properties(node);

	//cout <<"Neighbours of u1="<<atom1->id<<" ";
	for (tie(i, end) = visual_repr.getG().getAdjacentVertices(node); i != end;
			++i) {
		auto neighbour = *i;

		//see the angle

		const AtomRef& atom2 = visual_repr.getG().properties(neighbour);

		double angle = atan2(
				atom2.atom_ptr->segment_->getCenter().x - atom1.atom_ptr->segment_->getCenter().x,
				atom2.atom_ptr->segment_->getCenter().y - atom1.atom_ptr->segment_->getCenter().y);
		atom_struct a(atom2.atom_ptr->id, angle);

		//cout << visual_repr.getG().properties(node)->id <<" -> " << visual_repr.getG().properties(neighbour)->id << "; angle= "<<visual_repr.getG().properties(neighbour).angle <<endl;
		atoms.push_back(a);

	}
	std::sort(atoms.begin(), atoms.end(), &c1_ptr_less);
	for (unsigned int k = 0; k < atoms.size(); k++) {
		//cout <<" id ="<<atoms[k]->id<<" angle="<<atoms[k].angle<<endl;
		ids.push_back(atoms[k].id);
	}
	//cout <<" ----"<<endl;

}

double GraphsMatch::solve_assignment(vector<int>& adj_vector_u1,
		vector<int>& adj_vector_v2) {
	hungarian_problem_t p;

	unsigned int cost_size = 50;//adj_vector_u1.size();
	if(adj_vector_u1.size() == 0 || adj_vector_v2.size()== 0)
		return p_structural_err;
	int **cost_matrix = new int*[cost_size];

	for (unsigned int i = 0; i < cost_size; i++)
		cost_matrix[i] = new int[cost_size];
//	int cost_matrix[500][500];

	for (unsigned int i = 0; i < adj_vector_u1.size(); i++) {
		for (unsigned int j = 0; j < adj_vector_v2.size(); j++) {
			cost_matrix[i][j] = 1000
					* appearance_scores_[adj_vector_u1[i]][adj_vector_v2[j]];
		}
	}

	/* initialize the gungarian_problem using the cost matrix*/
	int matrix_size = hungarian_init(&p, (cost_matrix), adj_vector_u1.size(),
			adj_vector_v2.size(), HUNGARIAN_MODE_MAXIMIZE_UTIL);

//	fprintf(stderr,
//			"assignement matrix has a now a size %d rows and %d columns.\n\n",
//			matrix_size, matrix_size);

	/* some output */
//	fprintf(stderr, "cost-matrix:");
//	hungarian_print_costmatrix(&p);

	/* solve the assignement problem */
	hungarian_solve(&p);
	/* some output */
//	fprintf(stderr, "assignment:");
//	hungarian_print_assignment(&p);


	//print the mapping
	double total_cost = 1.;
	for (unsigned int i = 0; i < adj_vector_u1.size(); i++){
		//cout << "Node U1: "<<adj_vector_u1[i]<<" mapped to V2: ";
		for (unsigned int j = 0; j < adj_vector_v2.size(); j++){
			if(p.assignment[i][j] == 1){
				//cout << adj_vector_v2[j]<<endl;
				total_cost+= (0.001*cost_matrix[i][j]);
			}
		}
	}
	if(adj_vector_v2.size() > adj_vector_u1.size())
		total_cost /= adj_vector_v2.size();
	else
		total_cost /= adj_vector_u1.size();

	for (unsigned int i = 0; i < cost_size; i++){
		//cout <<"deleting "<<i <<" out of "<<(int)cost_size<<endl;
		delete [] cost_matrix[i];
	}
	delete [] cost_matrix;
	hungarian_free(&p);

	return  total_cost;

}

void GraphsMatch::generate_factorial_permutations(
		vector<vector<int> >& permutations, vector<int>& adj_vector) {

	//cout <<"Factorial permutations:"<<endl;
	do {
		//print_vector(adj_vector);
		//cout << endl;
		permutations.push_back(adj_vector);
	} while (std::next_permutation(adj_vector.begin(), adj_vector.end()));
	//cout <<" ---------"<<endl;

}

void GraphsMatch::generate_permutations(vector<vector<int> >& permutations,
		vector<int>& adj_vector) {

	//generate_factorial_permutations(permutations,adj_vector);

	int perms = adj_vector.size();
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

void GraphsMatch::print_vector(vector<int>& vect) {
	for (unsigned int i = 0; i < vect.size(); i++) {
		cout << vect[i] << " ";
	}
	cout << endl;
}

double GraphsMatch::probability(double similarity) {

	double distance = 1 - similarity;
	double dist_square = distance * distance;
	double lambda = 3.0;
	return exp(-dist_square * lambda);

}
/*
 * return the structural similarity between two permutations of neighbour nodes
 */
double GraphsMatch::structural_similarity(vector<int>& ids_g1,
		vector<int>& ids_g2) {
	double acc_score = 1.;
	unsigned int diff_size = 0;
	if (ids_g1.size() <= ids_g2.size()) {
		for (unsigned int i = 0; i < ids_g1.size(); i++) {
			acc_score *= appearance_scores_[ids_g1[i]][ids_g2[i]];
		}
		diff_size = ids_g2.size() - ids_g1.size();
	} else {
		for (unsigned int i = 0; i < ids_g2.size(); i++) {
			acc_score *= appearance_scores_[ids_g1[i]][ids_g2[i]];
		}
		diff_size = ids_g1.size() - ids_g2.size();
	}

	for (unsigned int i = 0; i < diff_size; i++)
		acc_score *= p_err;
	return acc_score;
}

void GraphsMatch::structural() {

	//iterate over the nodes of graph 1
	MyUniqueAtomGraph::vertex_iter i, end;
	for (tie(i, end) = visual_repr_1_.getG().getVertices(); i != end; ++i) {
		//get the node
		MyUniqueAtomGraph::Vertex node_u1 = *i;
		vector<int> neighbours_u1;
		get_neighbours(visual_repr_1_, node_u1, neighbours_u1);
		const AtomRef& atom_u1 = g1_get_atom(i);
		//get its neighbours' indexes
		//cout << "Node u1: " << atom_u1.atom_ptr->id << " neighbours: ";
		//print_vector(neighbours_u1);

		double best_candidate_score = -0.1;
		int id_best_match = -1;
		//iterate over the nodes of graph 2
		MyUniqueAtomGraph::vertex_iter j, jend;
		for (tie(j, jend) = visual_repr_2_.getG().getVertices(); j != jend;
				++j) {
			MyUniqueAtomGraph::Vertex node_v2 = *j;
			vector<int> neighbours_v2;
			get_neighbours(visual_repr_2_, node_v2, neighbours_v2);
			const AtomRef& atom_v2 = g2_get_atom(j);
			//cout <<"Node v2: "<<atom_v2->id <<" neighbours: ";
			//if (atom_u1->id == 24 && atom_v2->id == 27) {
			//	print_vector(neighbours_v2);

			double structural_score = solve_assignment(neighbours_u1, neighbours_v2);
			structural_scores_[atom_u1.atom_ptr->id][atom_v2.atom_ptr->id] = structural_score;
			//}

			//generate permutations for the neighbours_u1 of node u1
			if (false) {
				vector<vector<int> > permutations;
				generate_permutations(permutations, neighbours_u1);
				double max_score = -0.1;
				int best_permutation = -1;
				for (unsigned int k = 0; k < permutations.size(); k++) {

					double struct_score = structural_similarity(permutations[k],
							neighbours_v2);
					if (struct_score > max_score) {
						max_score = struct_score;
						best_permutation = k;
					}
					if (atom_u1.atom_ptr->id == 21 && atom_v2.atom_ptr->id == 25) {
						cout << "permutation #" << k << " with score= "
								<< struct_score << endl;
						print_vector(permutations[k]);
						cout << "neighbours of V2:";
						print_vector(neighbours_v2);
					}

				}
				structural_scores_[atom_u1.atom_ptr->id][atom_v2.atom_ptr->id] = max_score;

				//cout <<"best permutation #"<<best_permutation<<" with score= "<<max_score<<endl;
				//print_vector(permutations[best_permutation]);
				if (max_score > best_candidate_score) {
					best_candidate_score = max_score;
					id_best_match = atom_v2.atom_ptr->id;
				}
			}

		}
//		cout << "Best structural match node v2: " << id_best_match << " score= "
//									<< best_candidate_score << endl;

	}

}

/*
 *  It pre-computes the scores array of similarity
 *  between nodes from graphs G1 and G2.
 *
 *	scores is a 2-D array of doubles that contains
 *	the similarity scores between nodes
 *
 *        v2->id
 *        ----------
 * u1->id |	|	|	|
 *       |	|	|	|
 *       |	|	|	|
 * 		 ------------
 */
void GraphsMatch::precompute_scores() {

	//allocate memory: +0 since the atom ids start with 0
	//cout << "scores resized for " << visual_repr_1_.getSegments() << endl;
	structural_scores_.resize(visual_repr_1_.getSegments());
	appearance_scores_.resize(visual_repr_1_.getSegments());
	for (int index = 0; index < visual_repr_1_.getSegments(); index++) {
		appearance_scores_[index].resize(visual_repr_2_.getSegments());
		structural_scores_[index].resize(visual_repr_2_.getSegments());
	}

	//iterate over the nodes of visual_rep_1
	MyUniqueAtomGraph::vertex_iter i, end;
	for (tie(i, end) = visual_repr_1_.getG().getVertices(); i != end; ++i) {
		MyUniqueAtomGraph::Vertex node_u1 = *i;
		const AtomRef& atom_u1 = visual_repr_1_.getG().properties(node_u1);
		//cout << "> u1[" << visual_repr_1_.getG().properties(node_u1)->id<<"]" <<endl;

		//iterate over the nodes of visual_rep_2
		MyUniqueAtomGraph::vertex_iter j, jend;
		for (tie(j, jend) = visual_repr_2_.getG().getVertices(); j != jend;
				++j) {
			MyUniqueAtomGraph::Vertex node_v2 = *j;
			const AtomRef& atom_v2 = visual_repr_2_.getG().properties(node_v2);
			double score = (atom_u1.atom_ptr->similarity(atom_v2.atom_ptr.get()));
			appearance_scores_[atom_u1.atom_ptr->id][atom_v2.atom_ptr->id] = probability(score);

			//cout <<"appearance_scores_["<<atom_u1->id<<"]["<<atom_v2->id<<"] ="<<score<<" prob="<<probability(score)<<endl;
		}
	}
	//structural similarities
	structural();
}

void GraphsMatch::find_match() {
	for (int i = 0; i < visual_repr_1_.getSegments(); i++) {
		double max = 0.;
		int max_index_v2 = -1;
		for (int j = 0; j < visual_repr_2_.getSegments(); j++) {
			double score = structural_scores_[i][j] * appearance_scores_[i][j];
			if (score > max) {
				max = score;
				max_index_v2 = j;
			}
//			if( i == 21){
//				cout <<" Node U1: "<<i<<" V2:"<<j<<  " appearance score="<<appearance_scores_[i][j]<<" structural_score="<<structural_scores_[i][j] <<endl;
//			}
		}
		visual_repr_1_.getAtoms()[i].atom_ptr->id_matched_to = max_index_v2;
	}

	for (int i = 0; i < visual_repr_1_.getSegments(); i++) {
		int match = visual_repr_1_.getAtoms()[i].atom_ptr->id_matched_to;
		if (match < 0)
			continue;
		Segment* seg1 = visual_repr_1_.getAtoms()[i].atom_ptr->segment_;
		Segment* seg2 = visual_repr_2_.getAtoms()[match].atom_ptr->segment_;

		seg2->re_colour(seg1->getRandomColour());
		//cout <<"node U1 "<<visual_repr_1_.getAtoms()[i]->id <<" matched to "<<match<<endl;
		//cout <<"colours set as: "<<seg1->getRandomColour()<<" "<<seg2->getRandomColour()<<endl;

	}

}

void GraphsMatch::getMats(Mat& seg1, Mat& seg2) {
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
