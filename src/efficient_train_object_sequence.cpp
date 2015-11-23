/*
 * main.cpp
 *
 * It reads a sequence of images and asks the user to interactively mark
 * the segments that correspond to the object class to train
 * and those that correspond to the background
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

vector<Segment*> initial_bg_segments(0), bg_segments(0);
vector<Segment*> initial_fg_segments(0), fg_segments(0);
//the class selected
int segment_class = 1; //Foreground
int start_training = 0; //1 = starts training

VideoSegmentation videoSegmenter;

int current_frame =0;

//a vector of training examples
vector<TrainingExample> fg_training_examples,bg_training_examples;
string seq_name;

static void doMouseCallback(int event, int x, int y, int flags, void* param) {
	if(segment_class == 1){
//		switch (event) {
//			case CV_EVENT_LBUTTONDOWN:		//left button press
//				drag = true;
//				drop = true;
//				object_rect.x = x;
//				object_rect.y = y;
//				object_rect.width = 10;
//				object_rect.height = 10;
//				//cout << "left button: x,y=" << x << " " << y << endl;
//				object_selected_ = false;
//				break;
//
//			case CV_EVENT_LBUTTONUP:	//left mouse button release
//			{
//				drag = false;
//				drop = true;
//				int width = x - object_rect.x;
//				int height = y - object_rect.y;
//				object_rect.width = width;
//				object_rect.height = height;
//				//cout << "left button release: x,y=" << x << " " << y << endl;
//				object_selected_ = true;
//				display = tmp.clone();
//				rectangle(display, object_rect, Scalar(0, 0, 255), 4);
//				imshow(WINDOW_INTERACT, display);
//				waitKey(1);
//				break;
//			}
//
//			case CV_EVENT_MOUSEMOVE: {
//				if (drag == true) {
//					int width = x - object_rect.x;
//					int height = y - object_rect.y;
//					object_rect.width = width;
//					object_rect.height = height;
//					object_selected_ = false;
//					//cout << "dragging: x,y=" << x << " " << y << endl;
//					display = tmp.clone();
//					rectangle(display, object_rect, Scalar(0, 0, 255), 4);
//					imshow(WINDOW_INTERACT, display);
//					waitKey(1);
//
//				}
//
//				break;
//			}
//
//			}
		if( event == CV_EVENT_LBUTTONDOWN){
			Segment* seg=videoSegmenter.get_segment_at(y,x);

			TrainingExample example(seg->getCenter(),current_frame,seq_name,*seg);
			fg_training_examples.push_back(example);
			imshow("fg segment",seg->getRandomColourMat());
			waitKey(1);


		}
	}
	else if (segment_class == 0){
		if( event == CV_EVENT_LBUTTONDOWN){
			Segment* seg=videoSegmenter.get_segment_at(y,x);
			TrainingExample example(seg->getCenter(),current_frame,seq_name,*seg);
			bg_training_examples.push_back(example);
			imshow("bg segment",seg->getRandomColourMat());
			waitKey(1);
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

void show_segments(string& text, vector<Segment*>& segs){
	Mat debugMat = Mat::zeros(tmp.rows,tmp.cols,CV_8UC3);
	for(Segment* seg: segs){
		cout <<" debug.size="<<debugMat(seg->getBoundRect()).size()<<" seg.size="<<seg->getRandomColourMat().size()<<endl;
		debugMat(seg->getBoundRect()) += seg->getRandomColourMat();
		imshow(text,debugMat);

	}
	waitKey(1);
}

void show_segments(string& text,vector<Segment>& segs){
	Mat debugMat = Mat::zeros(tmp.rows,tmp.cols,CV_8UC3);
	for(Segment& seg: segs){
		cout <<" debug.size="<<debugMat(seg.getBoundRect()).size()<<" seg.size="<<seg.getRandomColourMat().size()<<endl;
		debugMat(seg.getBoundRect()) += seg.getRandomColourMat();
		imshow(text,debugMat);

	}
	waitKey(1);
}

void user_interaction_select_object(Mat& outputMat){
	display = outputMat.clone();
	tmp = display.clone();
	imshow(WINDOW_INTERACT, display);

	int cmd = 0;
	while( (cmd = waitKey(0)) != 1048608);
	cout <<"fg_training_examples.size()="<<fg_training_examples.size()<<endl;
	cout <<"bg_training_examples.size()="<<bg_training_examples.size()<<endl;
//
//
//	cout <<">output_segments.size()="<<initial_fg_segments.size()<<endl;
//	string text = "segments";
//	show_segments(text,initial_fg_segments);


}



int main(int argc, char** argv) {


	int scale_for_propagation = 2;
	int starting_scale = 2;
	Utils utils;
	int scales = 3;
	int gpu = 0;
	double threshold = 0.01; //0.05;
	unsigned int starting_frame = 0;
	unsigned int frame_interval = 30;
	string input_img_path, output_path;
	string examples_path ;
	utils.parse_args(argc, argv, threshold, scales, starting_scale,
			scale_for_propagation, gpu, input_img_path, output_path,examples_path);


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
	sprintf(trackbarName, "Start training= %d", slider_class_max);
		createTrackbar(trackbarName, WINDOW_INTERACT, &start_training,
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
	unsigned int images = images_list.size() - 1;//starting_frame+20;//



	for (unsigned int i = starting_frame; i < images; i+=frame_interval) {

		current_frame = i;
		cv::Mat img, outputMat,  contours_mat, gradient, grayGradient;

		cout << images_list[i] <<  endl;

		img = cv::imread(images_list[i], -1);



		string prefix_1 = utils.get_file_name(images_list[i]);
		prefix_1 = utils.remove_extension(prefix_1);

		videoSegmenter.addImage(img, outputMat);
		imshow("video segmentation",outputMat);



		user_interaction_select_object(outputMat);




	}



	return 0;
}

