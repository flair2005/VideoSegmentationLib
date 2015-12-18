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
#include <QMutex>
#include <QThread>
#include <QTime>
#include <QQueue>
#include <QImage>

#include <opencv2/opencv.hpp>

#include "Structures.h"
#include "segmentation.h"
#include "Parameters.h"



class SharedImageBuffer;


class PlaybackThread : public QThread
{
    Q_OBJECT

    public:
        PlaybackThread(SharedImageBuffer *sharedImageBuffer,SegmentationParameters* seg_params);
        void init();
        void stop();
        void setPath(std::vector<std::string>& imagePaths);

        int getInputSourceWidth();
        int getInputSourceHeight();

signals:
    void newFrame(const QImage& frame);
    void newSegmentation(const QImage& seg);

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
        std::vector<std::string> m_imagePaths;
        QImage m_frame;
        QImage m_segmentation;

        //parameters for segmentation
        SegmentationParameters* m_seg_params;
        int scale_for_propagation = 0;
        int starting_scale = 2;
        //Utils utils;
        int scales = 3;
        int gpu = 0;
        double threshold = 0.05; //0.05;

        void segment(cv::Mat& src,cv::Mat& dst);

    protected:
        void run();


};

#endif // PlaybackThread_H
