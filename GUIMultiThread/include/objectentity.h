#ifndef OBJECTENTITY_H
#define OBJECTENTITY_H

#include <QString>
#include "objects/ObjectDetector.h"

struct FrameData {
    vector<Segment*> fg_segments,bg_segments;
    Mat img;
    Mat depth_float;
};

class ObjectEntity
{
public:
    ObjectEntity(QString name);

    ObjectEntity(QString name,QString dir_path,QString svm_path, bool train);

    void save_current_data();

    virtual ~ObjectEntity();


    QString dir_path_;
    QString m_name_;
    QString m_svm_path_;
    ObjectDetector* m_detector_;
    FrameData frameData_;

};

#endif // OBJECTENTITY_H
