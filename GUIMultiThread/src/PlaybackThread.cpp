/************************************************************************/
/* qt-opencv-multithreaded:                                             */
/* A multithreaded OpenCV application using the Qt framework.           */
/*                                                                      */
/* PlaybackThread.cpp                                                   */
/* Based on the class from:                                             */
/* Nick D'Ademo <nickdademo@gmail.com>                                  */
/*                                                                      */
/* Copyright (c) 2012-2015 Nick D'Ademo                                 */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files (the "Software"), to deal in the Software without restriction, */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#include "PlaybackThread.h"
#include "MatToQImage.h"

#include "SharedImageBuffer.h"
#include "Config.h"


#include <QDebug>


PlaybackThread::PlaybackThread(SharedImageBuffer *sharedImageBuffer, SegmentationParameters* seg_params) :
    QThread(),
    m_sharedImageBuffer(sharedImageBuffer), m_seg_params(seg_params),m_object_entity(nullptr)
{

    video_segmentation = false;

}

PlaybackThread::PlaybackThread(SharedImageBuffer *sharedImageBuffer, SegmentationParameters* seg_params, bool videoSegmentation) :
    QThread(),
    m_sharedImageBuffer(sharedImageBuffer), m_seg_params(seg_params),m_object_entity(nullptr),
    video_segmentation(videoSegmentation)
{

    if(video_segmentation)
        videoSegmenter.init(m_seg_params->getScaleForPropagation(),  m_seg_params->getStartingScale(),  m_seg_params->getScales(),  false,
                            m_seg_params->getThreshold());

}

void PlaybackThread::activateVideoSegmentation(){
    video_segmentation = true;
    videoSegmenter.init(m_seg_params->getScaleForPropagation(),  m_seg_params->getStartingScale(),  m_seg_params->getScales(),  false,
                         m_seg_params->getThreshold());
}

SegmentationParameters* PlaybackThread::getSegmentationParameters(){
    return m_seg_params;
}

void PlaybackThread::segment(Segmentation& segmentation,cv::Mat& src,cv::Mat& dst,std::vector<Segment*>& segments){
    segmentation.init(src, gpu, m_seg_params->getScales(), m_seg_params->getStartingScale());
    std::cout <<" segmenting with threshold="<<m_seg_params->getThreshold()<<endl;
    segmentation.segment_pyramid(m_seg_params->getThreshold());
    segmentation.map_segments(m_seg_params->getScaleForPropagation());
    dst = segmentation.getOutputSegmentsPyramid()[m_seg_params->getScaleForPropagation()];
    segments = segmentation.getSegmentsPyramid()[m_seg_params->getScaleForPropagation()];
}

void PlaybackThread::activateTestMode(ObjectEntity* object_entity){
    test_mode = true;
    m_object_entity = object_entity;
}

void PlaybackThread::video_segmentation_run(){
    for(std::string imagePath : m_imagePaths){
        std::cout << imagePath<<std::endl;

        cv::Mat src = cv::imread(imagePath,-1);
        Mat video_segmented_mat;
        videoSegmenter.addImage(src, video_segmented_mat);

        m_frame = MatToQImage(src);
        m_segmentation = MatToQImage(video_segmented_mat);

        // Inform GUI thread of new frame (QImage)
        emit newFrame(m_frame);
        emit newVideoSegmentation(m_segmentation);
    }
}

void PlaybackThread::image_segmentation_run(){
    int i=0;
    for(std::string imagePath : m_imagePaths){
    //std::string imagePath = imagePaths[(imagePaths.size()-1)/2];
        std::cout << imagePath<<std::endl;
        string depthPath =m_depthPaths[i];
        cout <<depthPath<<endl;
        i++;
        cv::Mat src = cv::imread(imagePath,-1);
        cv::Mat depth_float,depth = cv::imread(depthPath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        depth.convertTo(depth_float,CV_32FC1);
        depth_float *= 0.001f;
        //segment it
        m_frame = MatToQImage(src);
        Mat segMat, debug;
        std::vector<Segment*> segments;
        Segmentation segmentation;
        Mat src_clone = src.clone();
        segment(segmentation,src_clone,segMat,segments);
        m_segmentation = MatToQImage(segMat);


        //check if we should test a model
        if(test_mode){
            //m_object_entity->m_detector->
            vector<Mat> masks;
            cout <<"testing the object model. Size of the image="<<src.size()<<endl;
            m_object_entity->m_detector_->test_data(segments,
                                          src, depth_float, masks, debug );
            m_frame = MatToQImage(debug);
        }

        // Inform GUI thread of new frame (QImage)
        emit newFrame(m_frame);
        emit newSegmentation(m_segmentation);


    }
}

void PlaybackThread::run()
{

    qDebug() << "Starting capture thread...";

    if(video_segmentation)
        video_segmentation_run();

    else
       image_segmentation_run();



    qDebug() << "Stopping capture thread...";
}

void PlaybackThread::setPath(std::vector<std::string>& imagePaths,std::vector<std::string>& depthPaths)
{
    m_imagePaths = imagePaths;
    m_depthPaths =depthPaths;

}


void PlaybackThread::updateFPS(int timeElapsed)
{

}

void PlaybackThread::stop()
{

    m_doStop = true;
}



int PlaybackThread::getInputSourceWidth()
{
    return 0;

}

int PlaybackThread::getInputSourceHeight()
{
    return 0;

}
