/*
 * VideoSegmentation.cpp
 *
 *  Created on: 11 Nov 2015
 *      Author: martin
 */

#include "VideoSegmentation.h"
#include "GraphsMatch.h"
#include "atom.h"
#include <memory>

using namespace boost;
using namespace videoseg;

namespace videoseg {

VideoSegmentation::VideoSegmentation() :
		scale_for_propagation_(0), starting_scale_(2), scales_(3), gpu_(0), threshold_(
				0.06), segmentation_1(nullptr), segmentation_2(nullptr), representation_1(
				nullptr), representation_2(nullptr), prefix_1_("./nodemats"), prefix_2_(
				"./nodemats") {
	// TODO Auto-generated constructor stub

}

VideoSegmentation::VideoSegmentation(int scale_for_propagation,
		int starting_scale, int scales, int gpu, double thres) :
		scale_for_propagation_(scale_for_propagation), starting_scale_(
				starting_scale), scales_(scales), gpu_(gpu), threshold_(thres), segmentation_1(
				nullptr), segmentation_2(nullptr), representation_1(nullptr), representation_2(
				nullptr), prefix_1_("./nodemats"), prefix_2_("./nodemats") {
	// TODO Auto-generated constructor stub

}

void VideoSegmentation::init(int scale_for_propagation, int starting_scale,
		int scales, int gpu, double thres) {
	scale_for_propagation_ = scale_for_propagation;
	starting_scale_ = starting_scale;
	scales_ = scales;
	gpu_ = gpu;
	threshold_ = thres;
}

VideoSegmentation::~VideoSegmentation() {
	// TODO Auto-generated destructor stub
}

Segment* VideoSegmentation::get_segment_at(int row, int col) {
	if (segmentation_2 != nullptr)
		return segmentation_2->get_segment_at_fast(row, col);
	else
		return segmentation_1->get_segment_at_fast(row, col);
}

vector<Segment*> VideoSegmentation::get_segments() {
	if (segmentation_2 == nullptr)
		return segmentation_1->getSegmentsPyramid()[scale_for_propagation_];
	else
		return segmentation_2->getSegmentsPyramid()[scale_for_propagation_];
}

bool VideoSegmentation::rect_contained(const Rect& rect1,const Rect& rect2) {
	return ((rect1.x >= rect2.x) && (rect1.y >= rect1.y)
			&& (rect1.x + rect1.width < rect2.x + rect2.width)
			&& (rect1.y + rect1.height < rect2.y + rect2.height));
}

void VideoSegmentation::get_segments(Rect& object_rect,
		vector<Segment*>& output_segments) {

	if (segmentation_2 == nullptr) {
		const vector<Segment*>& all_segments =
				segmentation_1->getSegmentsPyramid()[scale_for_propagation_];
		for (Segment *seg : all_segments) {
			const Rect& candidate_rect = seg->getBoundRect();
			if (rect_contained(candidate_rect, object_rect))
				output_segments.push_back(seg);
		}
	} else {
		const vector<Segment*>& all_segments =
				segmentation_2->getSegmentsPyramid()[scale_for_propagation_];
		for (Segment *seg : all_segments) {
			const Rect& candidate_rect = seg->getBoundRect();
			if (rect_contained(candidate_rect, object_rect))
				output_segments.push_back(seg);
		}

	}

}

void VideoSegmentation::addImage(Mat& src, Mat& dst) {

	/*
	 * in case we have the first image of the stream
	 */
	if (representation_1 == nullptr) {
		segmentation_1 = new Segmentation(src, gpu_, scales_, starting_scale_);
		segmentation_1->segment_pyramid(threshold_);

		const vector<Segment*>& segments_1 =
				segmentation_1->getSegmentsPyramid()[scale_for_propagation_];

		//imshow("outputseg",segmentation_1->getOutputSegmentsPyramid()[scale_for_propagation_]);
		//waitKey(0);
		vector<std::shared_ptr<Atom>> atoms_1;
		for (Segment* seg : segments_1) {
			std::shared_ptr<Atom> atom = std::make_shared < Atom > (seg);
			atoms_1.push_back(atom);
		}
		representation_1 = new VisualRepresentation(atoms_1, prefix_1_);
		dst =
				segmentation_1->getOutputSegmentsPyramid()[scale_for_propagation_];
		segmentation_1->map_segments(scale_for_propagation_);
		Atom::static_id = 0;
	} else {

		segmentation_2 = new Segmentation(src, gpu_, scales_, starting_scale_);
		segmentation_2->segment_pyramid(threshold_);

		const vector<Segment*>& segments_2 =
				segmentation_2->getSegmentsPyramid()[scale_for_propagation_];

		//imshow("outputseg",segmentation_1->getOutputSegmentsPyramid()[scale_for_propagation_]);
		//waitKey(0);
		vector<std::shared_ptr<Atom>> atoms_2;
		for (Segment* seg : segments_2) {
			std::shared_ptr<Atom> atom = std::make_shared < Atom > (seg);
			atoms_2.push_back(atom);
		}
		representation_2 = new VisualRepresentation(atoms_2, prefix_2_);

		/*
		 * match the two representations
		 */
		GraphsMatch graph_match(*representation_1, *representation_2);
		graph_match.find_match();
		Mat seg1, seg2;
		graph_match.getMats(seg1, dst);

		//caches the segments at a given level of the pyramid
		//so that they can be retrieved with the get_segment_at
		//method
		segmentation_2->map_segments(scale_for_propagation_);
		//reset the atom id; otherwise: kaboom!
		Atom::static_id = 0;

		//free memory
		delete segmentation_1;
		delete representation_1;

		//re-assign pointers
		segmentation_1 = segmentation_2;
		representation_1 = representation_2;

	}
}

} /* namespace videoseg */
