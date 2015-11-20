/*
 * main.cpp
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

/*
 * SegmentationLib
 */
#include "segmentation.h"
#include "visualisation.h"

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "atom.h"

#include "atom_graphs.h"
#include "GraphsMatch.h"
#include "VideoSegmentation.h"
#include "objects/ObjectDetector.h"

#include "utils.h"

using namespace boost;
using namespace videoseg;
//using namespace std;

Rect object_rect;

const string WINDOW_INTERACT = "Train Object Detector";
bool drag, drop, object_selected_, select_bg_segments =false;
Mat display, tmp;

vector<Segment*> tmp_bg_segments;
vector<Segment> fg_segments;
//the class selected
int segment_class = 1; //Foreground

VideoSegmentation videoSegmenter;

static void doMouseCallback(int event, int x, int y, int flags, void* param) {
	if(segment_class == 1){
		switch (event) {
			case CV_EVENT_LBUTTONDOWN:		//left button press
				drag = true;
				drop = true;
				object_rect.x = x;
				object_rect.y = y;
				object_rect.width = 10;
				object_rect.height = 10;
				//cout << "left button: x,y=" << x << " " << y << endl;
				object_selected_ = false;
				break;

			case CV_EVENT_LBUTTONUP:	//left mouse button release
			{
				drag = false;
				drop = true;
				int width = x - object_rect.x;
				int height = y - object_rect.y;
				object_rect.width = width;
				object_rect.height = height;
				//cout << "left button release: x,y=" << x << " " << y << endl;
				object_selected_ = true;
				display = tmp.clone();
				rectangle(display, object_rect, Scalar(0, 0, 255), 4);
				imshow(WINDOW_INTERACT, display);
				waitKey(1);
				break;
			}

			case CV_EVENT_MOUSEMOVE: {
				if (drag == true) {
					int width = x - object_rect.x;
					int height = y - object_rect.y;
					object_rect.width = width;
					object_rect.height = height;
					object_selected_ = false;
					//cout << "dragging: x,y=" << x << " " << y << endl;
					display = tmp.clone();
					rectangle(display, object_rect, Scalar(0, 0, 255), 4);
					imshow(WINDOW_INTERACT, display);
					waitKey(1);

				}

				break;
			}

			}
	}
	else if (segment_class == 0){
		if( event == CV_EVENT_LBUTTONDOWN){
			Segment* seg=videoSegmenter.get_segment_at(y,x);
			tmp_bg_segments.push_back(seg);
			imshow("segment",seg->getRandomColourMat());
			waitKey(0);
		}


	}


}

//the callback function for the trackbar
static void on_trackbar(int event, void* param) {
	if(segment_class == 0){
		cout <<"> click to select background segments "<<endl;
	}
	else if(segment_class == 1){
		cout <<"> drag and drop to select foreground segments "<<endl;
	}

}

void show_segments(vector<Segment*>& segs){
	Mat debugMat = Mat::zeros(tmp.rows,tmp.cols,CV_8UC3);
	for(Segment& seg: fg_segments){
		cout <<" debug.size="<<debugMat(seg.getBoundRect()).size()<<" seg.size="<<seg.getRandomColourMat().size()<<endl;
		debugMat(seg.getBoundRect()) += seg.getRandomColourMat();
		imshow("segment",debugMat);

	}
	waitKey(1);
}

void show_segments(vector<Segment>& segs){
	Mat debugMat = Mat::zeros(tmp.rows,tmp.cols,CV_8UC3);
	for(Segment& seg: fg_segments){
		cout <<" debug.size="<<debugMat(seg.getBoundRect()).size()<<" seg.size="<<seg.getRandomColourMat().size()<<endl;
		debugMat(seg.getBoundRect()) += seg.getRandomColourMat();
		imshow("segment",debugMat);

	}
	waitKey(1);
}

int main(int argc, char** argv) {

	ObjectDetector slc;
	int scale_for_propagation = 2;
	int starting_scale = 2;
	Utils utils;
	int scales = 3;
	int gpu = 0;
	double threshold = 0.01; //0.05;
	string input_img_path, output_path;
	utils.parse_args(argc, argv, threshold, scales, starting_scale,
			scale_for_propagation, gpu, input_img_path, output_path);

	videoSegmenter.init(scale_for_propagation, starting_scale, scales,
			gpu, threshold);

	//add mouse callback function for specifying the rectangular region
	cv::namedWindow(WINDOW_INTERACT,CV_WINDOW_AUTOSIZE);
	setMouseCallback(WINDOW_INTERACT, doMouseCallback, 0);

	//add trackback function for selecting either foreground or background segments
	char trackbarName[50];
	int slider_class_max = 1;
	sprintf(trackbarName, "Class (BG= 0, FG=1)= %d", slider_class_max);
	createTrackbar(trackbarName, WINDOW_INTERACT, &segment_class,
			slider_class_max, on_trackbar);

	/*
	 * open image files
	 */

	std::vector<std::string> images_list;

	std::ifstream file_colours(input_img_path);

	std::string content_colour((std::istreambuf_iterator<char>(file_colours)),
			std::istreambuf_iterator<char>());

	boost::split(images_list, content_colour, boost::is_any_of("\t \n"));

	cout << " reading input..." << argv[1] << endl;
	vector<Vec3b> colours;
	unsigned int images = images_list.size() - 1;



	for (unsigned int i = 0; i < images; i++) {

		cv::Mat img, outputMat,  contours_mat, gradient, grayGradient;

		cout << images_list[i] <<  endl;

		img = cv::imread(images_list[i], -1);



		string prefix_1 = utils.get_file_name(images_list[i]);
		prefix_1 = utils.remove_extension(prefix_1);

		videoSegmenter.addImage(img, outputMat);
		imshow("video segmentation",outputMat);


		//if it is the first image
		if (i == 0) {
			display = outputMat.clone();
			tmp = display.clone();
			imshow(WINDOW_INTERACT, display);
			waitKey(0);
			if(object_selected_){

				vector<Segment*>tmp_segments;
				videoSegmenter.get_segments(object_rect, tmp_segments);
				for(Segment*seg:tmp_segments)
					fg_segments.push_back(*seg);
				cout <<">output_segments.size()="<<fg_segments.size()<<endl;
				show_segments(fg_segments);
				slc.add_training_data(tmp_segments,tmp_bg_segments);
			}
		}
		//else need to propagate the labels of the segments
		else{

			vector<Segment*> propagated_segs;
			for(Segment& initial_seg : fg_segments){
				Segment* propagated = videoSegmenter.get_segment_by_label(initial_seg.getLabel());
				if(propagated ==nullptr)
					cout <<"propagated segment is null, no match found for label:"<<initial_seg.getLabel()<<endl;
				else
					propagated_segs.push_back(propagated);
			}
			cout <<" propagated segments="<<propagated_segs.size()<<endl;

			show_segments(propagated_segs);
		}



	}

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

