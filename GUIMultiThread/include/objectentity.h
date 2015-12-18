#ifndef OBJECTENTITY_H
#define OBJECTENTITY_H

#include <QString>
#include "objects/ObjectDetector.h"

class ObjectEntity
{
public:
    ObjectEntity(QString name);

    ObjectEntity(QString name,QString svm_path, bool train);


    virtual ~ObjectEntity();


    QString m_name;
    QString m_svm_path;
    ObjectDetector* m_detector;
};

#endif // OBJECTENTITY_H
