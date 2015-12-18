#include "objectentity.h"

ObjectEntity::ObjectEntity(QString name):
    m_name(name), m_detector(nullptr)
{
}

ObjectEntity::ObjectEntity(QString name,QString svm_path):
    m_name(name),m_svm_path(svm_path)
{
    std::string svm_model_path = svm_path.toUtf8().constData();
    m_detector = new ObjectDetector(ObjectDetector::TRAIN_MODE, svm_model_path);
}


ObjectEntity::~ObjectEntity(){
    delete m_detector;
}
