/************************************************************************/
/* qt-opencv-multithreaded:                                             */
/* A multithreaded OpenCV application using the Qt framework.           */
/*                                                                      */
/* PlaybackThread.h                                                      */
/*                                                                      */
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

#ifndef PlaybackThread_H
#define PlaybackThread_H
#include "objectentity.h"
#include <QMutex>
#include <QThread>
#include <QTime>
#include <QQueue>
#include <QImage>

#include <opencv2/opencv.hpp>

#include "Structures.h"
#include "segmentation.h"
#include "Parameters.h"
#include "VideoSegmentation.h"



class SharedImageBuffer;


class PlaybackThread : public QThread
{
    Q_OBJECT

    public:
        PlaybackThread(SharedImageBuffer *sharedImageBuffer,SegmentationParameters* seg_params);
        PlaybackThread(SharedImageBuffer *sharedImageBuffer,SegmentationParameters* seg_params, bool videoSegmentation);
        void init();
        void stop();
        void setPath(std::vector<std::string>& imagePaths,std::vector<std::string>& depthPaths);
        void activateTestMode(ObjectEntity* object_entity);

        int getInputSourceWidth();
        int getInputSourceHeight();
        void activateVideoSegmentation();
        SegmentationParameters* getSegmentationParameters();


signals:
    void newFrame(const QImage& frame);
    void newSegmentation(const QImage& seg);
    void newVideoSegmentation(const QImage& seg);

    private:
        void updateFPS(int);
        SharedImageBuffer *m_sharedImageBuffer;
        cv::VideoCapture m_cap;
        cv::Mat m_grabbedFrame;
        QTime m_t;
        QMutex m_doStopMutex;
        QQueue<int> m_fps;
        ThreadStatisticsData m_statsData;
        volatile bool m_doStop;
        int m_captureTime;
        int m_sampleNumber;
        int m_fpsSum;
        bool m_dropFrameIfBufferFull;
        int m_deviceNumber;
        int m_width;
        int m_height;


        //new member attributes
        std::vector<std::string> m_imagePaths,m_depthPaths;
        QImage m_frame;
        QImage m_segmentation;

        //parameters for segmentation
        SegmentationParameters* m_seg_params;        
        //Utils utils;

        int gpu = 0;
        double threshold = 0.05; //0.05;

        //test object model mode
        bool test_mode = false;
        ObjectEntity* m_object_entity;

        //video segmentation attributes
        bool video_segmentation;
        VideoSegmentation videoSegmenter;

        void segment(Segmentation& segmentation,cv::Mat& src,cv::Mat& dst,std::vector<Segment*>& segments);

        void video_segmentation_run();
        void image_segmentation_run();
    protected:
        void run();


};

#endif // PlaybackThread_H
