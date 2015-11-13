/*
 * VideoSegmentation.h
 *
 *  Created on: 11 Nov 2015
 *      Author: martin
 */

#ifndef VIDEOSEGMENTATION_H_
#define VIDEOSEGMENTATION_H_

#include <opencv2/core/core.hpp>
#include "utils.h"
#include "segmentation.h"
#include "visual_representation.h"
using namespace cv;

namespace videoseg {

class VideoSegmentation {
public:
	VideoSegmentation();
	VideoSegmentation(int scale_for_propagation, int starting_scale, int scales, int gpu,
	 double thres);
	void init(int scale_for_propagation, int starting_scale, int scales, int gpu,
		 double thres);
	virtual ~VideoSegmentation();

	void addImage(Mat& src, Mat& dst);

	//these two methods interface with the image segmentation
	Segment* get_segment_at(int row, int col);
	vector<Segment*> get_segments();

private:
	int scale_for_propagation_ = 2;
	int starting_scale_ = 2;
	Utils utils_;
	int scales_ = 3;
	int gpu_ = 0;
	double threshold_ = 0.01;

	Segmentation *segmentation_1,*segmentation_2;
	VisualRepresentation *representation_1,*representation_2;

	string prefix_1_, prefix_2_;

};

} /* namespace videoseg */

#endif /* VIDEOSEGMENTATION_H_ */
