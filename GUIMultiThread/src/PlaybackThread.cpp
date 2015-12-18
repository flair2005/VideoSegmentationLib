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
    m_sharedImageBuffer(sharedImageBuffer), m_seg_params(seg_params)
{

}

void PlaybackThread::segment(cv::Mat& src,cv::Mat& dst){
    Segmentation segmentation(src, gpu, scales, starting_scale);
    std::cout <<" segmenting with threshold="<<m_seg_params->getThreshold()<<endl;
    segmentation.segment_pyramid(m_seg_params->getThreshold());
    dst = segmentation.getOutputSegmentsPyramid()[scale_for_propagation];
}

void PlaybackThread::run()
{

    qDebug() << "Starting capture thread...";

    for(std::string imagePath : m_imagePaths){
    //std::string imagePath = imagePaths[(imagePaths.size()-1)/2];
        std::cout << imagePath<<std::endl;
        cv::Mat src = cv::imread(imagePath,-1);

        //segment it


        m_frame = MatToQImage(src);
        Mat segMat;
        segment(src,segMat);
        m_segmentation = MatToQImage(segMat);

        // Inform GUI thread of new frame (QImage)
        emit newFrame(m_frame);
        emit newSegmentation(m_segmentation);


    }

    qDebug() << "Stopping capture thread...";
}

void PlaybackThread::setPath(std::vector<std::string>& imagePaths)
{
    m_imagePaths = imagePaths;

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
