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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "objects/ObjectDetector.h"

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

#include "utils.h"


//using namespace boost;
using namespace videoseg;
//using namespace std;

Rect object_rect;

const string WINDOW_INTERACT = "Train Object Detector";
bool drag, drop, object_selected_ = true, select_bg_segments = false;
Mat display, tmp;

vector<Segment*> all_segments(0),initial_bg_segments(0), bg_segments(0);
vector<Segment*> initial_fg_segments(0), fg_segments(0);
//the class selected
int segment_class = 1; //Foreground
int start_training = 0; //1 = starts training
int re_configure = 1;

VideoSegmentation videoSegmenter;

static void doMouseCallback(int event, int x, int y, int flags, void* param) {
	if (segment_class == 1) {

		if (event == CV_EVENT_LBUTTONDOWN) {
			Segment* seg = videoSegmenter.get_segment_at(y, x);

			if(seg != nullptr){
				initial_fg_segments.push_back(new Segment(*seg));
				//remove this one from all_segments
				all_segments.erase(std::remove(all_segments.begin(), all_segments.end(), seg), all_segments.end());
				imshow("segment", seg->getRandomColourMat());
				waitKey(5);
			}


		}
	} else if (segment_class == 0) {
		if (event == CV_EVENT_LBUTTONDOWN) {
			Segment* seg = videoSegmenter.get_segment_at(y, x);
			if(seg != nullptr){
				initial_bg_segments.push_back(new Segment(*seg));
				imshow("segment", seg->getRandomColourMat());
				waitKey(5);

			}

		}

	}

}

//the callback function for the trackbar
static void on_trackbar(int event, void* param) {
	if (segment_class == 0) {
		cout << "> click to select background segments " << endl;
	} else if (segment_class == 1) {
		cout << "> drag and drop to select foreground segments " << endl;
	}

}

void show_segments(string& text, vector<Segment*>& segs) {
	Mat debugMat = Mat::zeros(tmp.rows, tmp.cols, CV_8UC3);
	for (Segment* seg : segs) {
		cout << " debug.size=" << debugMat(seg->getBoundRect()).size()
				<< " seg.size=" << seg->getRandomColourMat().size() << endl;
		debugMat(seg->getBoundRect()) += seg->getRandomColourMat();
		imshow(text, debugMat);

	}
	waitKey(1);
}

void show_segments(string& text, vector<Segment>& segs) {
	Mat debugMat = Mat::zeros(tmp.rows, tmp.cols, CV_8UC3);
	for (Segment& seg : segs) {
		cout << " debug.size=" << debugMat(seg.getBoundRect()).size()
				<< " seg.size=" << seg.getRandomColourMat().size() << endl;
		debugMat(seg.getBoundRect()) += seg.getRandomColourMat();
		imshow(text, debugMat);

	}
	waitKey(1);
}

void user_interaction_select_object(Mat& outputMat, ObjectDetector& slc) {
	display = outputMat.clone();
	tmp = display.clone();
	imshow(WINDOW_INTERACT, display);
	waitKey(0);
	initial_bg_segments=all_segments;
	if (object_selected_) {

//		vector<Segment*>tmp_segments;
//		videoSegmenter.get_segments(object_rect, tmp_segments);
//		for(Segment*seg:tmp_segments)
//			initial_fg_segments.push_back(*seg);
		cout << ">output_segments.size()=" << initial_fg_segments.size()
				<< endl;
		string text = "segments";
		show_segments(text, initial_fg_segments);

	}

}

void propagate_segments(vector<Segment*>& initial_segments,
		vector<Segment*>& propagated_segments) {
	for (Segment* initial_seg : initial_segments) {
		Segment* propagated = videoSegmenter.get_segment_by_label(
				initial_seg->getLabel());
		if (propagated == nullptr)
			cout << "propagated segment is null, no match found for label:"
					<< initial_seg->getLabel() << endl;
		else {

			propagated_segments.push_back(propagated);
		}

	}
}

void cropped_pcl_from_segments(Mat& img, Mat& depth,vector<Segment*>&segments,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,Mat& tmp_img,Mat& tmp_depth){

	Size size_segments = segments[0]->getBinaryMat().size();
	cout <<" target size="<<size_segments<<endl;
	cout <<" current size="<<img.size()<<endl;
//	pyrDown(img,img2,cv::Size(img.cols / 2, img.rows / 2));
//	pyrDown(depth,depth2,cv::Size(img.cols / 2, img.rows / 2));

	//depth2 *= 0.001;
	Mat cropped_img,cropped_depth;
	tmp_img = Mat::zeros(img.rows, img.cols, CV_8UC3);
	tmp_depth = Mat::zeros(img.rows, img.cols, CV_32FC1);

	Mat mask = Mat::zeros(size_segments, CV_8UC1);
	for (Segment * seg : initial_fg_segments) {
		//mask(seg->getBoundRect()) += seg->getRandomColourMat();
		//imshow("seg->getRandomColourMat()",seg->getRandomColourMat());
		//imshow("seg->getBinaryMat()",seg->getBinaryMat());
		//waitKey(0);
		//cout <<"mask.size()="<<mask.size()<<" seg->getBinaryMat().size()="<<seg->getBinaryMat().size()<<endl;
		mask += seg->getBinaryMat();
		//imshow("seg",seg->getMatOriginalColour());
	}
	resize(mask,mask,img.size());


	Mat pointsMat;
	cv::findNonZero(mask,pointsMat);
	Rect minRect=boundingRect(pointsMat);
//	imshow("mask", mask);
//	imshow("mask cropped", mask(minRect));
//	waitKey(0);
	//cout <<"mask.size()="<<mask.size()<<" img2.size()="<<img2.size()<<endl;
	//cvtColor(mask,mask,CV_BGR2GRAY);

	img.copyTo(tmp_img, mask);
	depth.copyTo(tmp_depth, mask);



	Utils utils;
	cropped_img = tmp_img(minRect);
	cropped_depth = tmp_depth(minRect);
	//imshow("mask", mask);
	//imshow("cropped_img", cropped_img);
	//waitKey(0);
	int cx = img.cols/2;
	int cy = img.rows/2;
	utils.image_to_pcl(cropped_img,cropped_depth , cloud,cx,cy,minRect);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pruned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	utils.prune_pcl(cloud, pruned_cloud);
//	cout <<"original cloud ->points.size()="<<cloud->points.size()<<endl;
//	cout <<"pruned_cloud->points.size()="<<pruned_cloud->points.size()<<endl;
//	pcl::visualization::PCLVisualizer viewer("3d Viewer");
//	viewer.setBackgroundColor(0, 0, 0);
//	viewer.addPointCloud < pcl::PointXYZRGB > (pruned_cloud, "sample cloud");
//	viewer.spin();
}



void add_selected_segments(Mat&img, Mat& depth_float,vector<Segment*>& fg_segments,vector<Segment*>& bg_segments, ObjectDetector& slc){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals;

	Utils utils;
	utils.image_to_pcl(img,depth_float,pcl_cloud);
	utils.compute_integral_normals(pcl_cloud, normals);


	for(Segment* seg: fg_segments){
		seg->add_precomputed_pcl(pcl_cloud, normals);
		seg->computeFeatures();
	}
	for(Segment* seg: bg_segments){
		seg->add_precomputed_pcl(pcl_cloud, normals);
		seg->computeFeatures();
	}

	slc.add_training_data(fg_segments, bg_segments,pcl_cloud,img,depth_float);
}

int main(int argc, char** argv) {

	int scale_for_propagation = 2;
	int starting_scale = 2;
	Utils utils;
	int scales = 3;
	int gpu = 0;
	double threshold = 0.01; //0.05;
	unsigned int starting_frame = 0;
	string input_img_path, input_clouds_path, output_path;
	string svm_model_path;
	utils.parse_args(argc, argv, threshold, scales, starting_scale,
			scale_for_propagation, gpu, input_img_path, output_path,
			svm_model_path, input_clouds_path);

	videoSegmenter.init(scale_for_propagation, starting_scale, scales, gpu,
			threshold);

	ObjectDetector slc(ObjectDetector::TRAIN_MODE, svm_model_path);

	//add mouse callback function for specifying the rectangular region
	cv::namedWindow(WINDOW_INTERACT, CV_WINDOW_AUTOSIZE);
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

	sprintf(trackbarName, "Reconfigure= %d", slider_class_max);
	createTrackbar(trackbarName, WINDOW_INTERACT, &re_configure,
			slider_class_max, on_trackbar);

	/*
	 * open image files
	 */

	std::vector<std::string> images_list;
	std::ifstream file_colours(input_img_path);
	std::string content_colour((std::istreambuf_iterator<char>(file_colours)),
			std::istreambuf_iterator<char>());
	boost::split(images_list, content_colour, boost::is_any_of("\t \n"));

	std::vector<std::string> clouds_list;
	std::ifstream file_pcls(input_clouds_path);
	std::string content_clouds((std::istreambuf_iterator<char>(file_pcls)),
			std::istreambuf_iterator<char>());
	boost::split(clouds_list, content_clouds, boost::is_any_of("\t \n"));

	vector<Vec3b> colours;
	unsigned int images = images_list.size() - 1; //starting_frame+20;//

	for (unsigned int i = starting_frame; i < images; i++) {

		cv::Mat img, depth, depth_float, outputMat, contours_mat, gradient, grayGradient;

		cout << images_list[i] << endl;

		img = cv::imread(images_list[i], -1);
		cout << "> opening depth map: " << clouds_list[i] << endl;
		depth = cv::imread(clouds_list[i], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
		depth.convertTo(depth_float,CV_32FC1);
		depth_float *= 0.001f;

		cout <<" depth.type()="<<depth.type()<<endl;




		string prefix_1 = utils.get_file_name(images_list[i]);
		prefix_1 = utils.remove_extension(prefix_1);


		videoSegmenter.addImage(img, outputMat);
		all_segments = videoSegmenter.get_segments();
		cout <<"videoSegmenter added"<<endl;
		imshow("video segmentation", outputMat);
		waitKey(1);

		//if it is the first image
		if (i == starting_frame) {
			user_interaction_select_object(outputMat, slc);
			add_selected_segments(img,depth_float,initial_fg_segments,initial_bg_segments,slc);
			//display the cropped point cloud
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//			Mat cropped_img;
//			Mat cropped_depth;
//			for(Segment* seg: initial_fg_segments){
//				seg->addPcl(img,depth_float);
//				seg->computeFeatures();
//			}
//			for(Segment* seg: initial_bg_segments){
//				seg->addPcl(img,depth_float);
//				seg->computeFeatures();
//			}
//
//
//			cropped_pcl_from_segments(img,depth,initial_fg_segments,cloud,cropped_img,cropped_depth);
//			slc.add_training_data(initial_fg_segments, initial_bg_segments,cloud,cropped_img,cropped_depth);




		} else if (re_configure) {
			user_interaction_select_object(outputMat, slc);
			add_selected_segments(img,depth_float,initial_fg_segments,initial_bg_segments,slc);
			//display the cropped point cloud

//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//			Mat cropped_img;
//			Mat cropped_depth;
//			for(Segment* seg: initial_fg_segments){
//				seg->addPcl(img,depth_float);
//				seg->computeFeatures();
//			}
//			for(Segment* seg: initial_bg_segments){
//				seg->addPcl(img,depth_float);
//				seg->computeFeatures();
//			}
//			cropped_pcl_from_segments(img,depth,initial_fg_segments,cloud,cropped_img,cropped_depth);
//			slc.add_training_data(initial_fg_segments, initial_bg_segments,cloud,cropped_img,cropped_depth);
		}
		//else need to propagate the labels of the segments
		else {

			vector<Segment*> propagated_fg_segs, propagated_bg_segs;
			//propagate the foreground segments
			propagate_segments(initial_fg_segments, propagated_fg_segs);
			//propagate the background segments
			propagate_segments(initial_bg_segments, propagated_bg_segs);
			//if the succesfully propagated segments are not the same we had in the model

			// ask the user for interaction
			if (initial_fg_segments.size() != propagated_fg_segs.size()) {
				initial_fg_segments.clear();
				initial_bg_segments.clear();
				user_interaction_select_object(outputMat, slc);
				add_selected_segments(img,depth_float,initial_fg_segments,initial_bg_segments,slc);
				//display the cropped point cloud
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//				Mat cropped_img;
//				Mat cropped_depth;
//				for(Segment* seg: initial_fg_segments){
//					seg->addPcl(img,depth_float);
//					seg->computeFeatures();
//				}
//				for(Segment* seg: initial_bg_segments){
//					seg->addPcl(img,depth_float);
//					seg->computeFeatures();
//				}
//				cropped_pcl_from_segments(img,depth,initial_fg_segments,cloud,cropped_img,cropped_depth);
//				slc.add_training_data(initial_fg_segments, initial_bg_segments,cloud,cropped_img,cropped_depth);

//				pcl::visualization::PCLVisualizer viewer("3d Viewer");
//								viewer.setBackgroundColor(0, 0, 0);
//								viewer.addPointCloud < pcl::PointXYZRGB > (cloud, "sample cloud");
//								//viewer.addCoordinateSystem(0.0);
//								viewer.spin();

			}

			//otherwise append the propagated segments to the fg_segments vector
			else {

				fg_segments.insert(fg_segments.end(),
						propagated_fg_segs.begin(), propagated_fg_segs.end());
				bg_segments.insert(bg_segments.end(),
						propagated_bg_segs.begin(), propagated_bg_segs.end());
				cout << " fg_segments appended : fg_segments.size()="
						<< fg_segments.size()
						<< " bg_segments appended : bg_segments.size()="
						<< bg_segments.size() << endl;

				add_selected_segments(img,depth_float,propagated_fg_segs,propagated_bg_segs,slc);
				//display the cropped point cloud
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//				Mat cropped_img;
//				Mat cropped_depth;
//
//				for(Segment* seg: propagated_fg_segs){
//					seg->addPcl(img,depth_float);
//					seg->computeFeatures();
//				}
//				for(Segment* seg: propagated_bg_segs){
//					seg->addPcl(img,depth_float);
//					seg->computeFeatures();
//				}
//				cropped_pcl_from_segments(img,depth,propagated_fg_segs,cloud,cropped_img,cropped_depth);
//				slc.add_training_data(propagated_fg_segs, propagated_bg_segs,cloud,cropped_img,cropped_depth);

			}
			cout << " propagated segments=" << propagated_fg_segs.size()
					<< endl;
			string text = "fg segments";
			show_segments(text, propagated_fg_segs);
			text = "bg segments";
			show_segments(text, propagated_bg_segs);





		}
		if (start_training) {
			//slc.run_kinfu(1.5);
			slc.train();
			return 0;
		}

	}
	//slc.add_training_data(fg_segments,bg_segments);
	//slc.run_kinfu(1.5);
	slc.train();

//
//	vector<cv::Mat> outputPyr = segmentation.getOutputSegmentsPyramid();
	return 0;
}

