#include "objectentity.h"
#include <QDirIterator>

ObjectEntity::ObjectEntity(QString name):
    m_name_(name), m_detector_(nullptr)
{
}

ObjectEntity::ObjectEntity(QString name,QString dir_path,QString svm_path, bool train):
    dir_path_(dir_path),m_name_(name),m_svm_path_(svm_path)
{

    if(train){
        std::string model_name = m_name_.toUtf8().constData();
        std::string svm_model_path = m_svm_path_.toUtf8().constData();
        svm_model_path +="/"+model_name;
        std::string svm_file_full_path = svm_model_path;
        svm_file_full_path += "/"+model_name+".xml";
        std::string dir_path_string = dir_path_.toUtf8().constData();
        dir_path_string += "/"+model_name;
        m_detector_ = new ObjectDetector(ObjectDetector::TRAIN_MODE,svm_file_full_path,dir_path_string ,model_name);
    }

    else{
        std::string model_name = m_name_.toUtf8().constData();
        std::string svm_model_path = m_svm_path_.toUtf8().constData();


        QDir dir = QFileInfo(m_svm_path_).absoluteDir();
        QString folder_path =  dir.absolutePath();//folder.absoluteFilePath(m_svm_path_);

        std::string dir_path_string = folder_path.toUtf8().constData();

        std::cout << "opening svm file for testing model="<<svm_model_path<<endl;
        m_detector_ = new ObjectDetector(ObjectDetector::TEST_MODE,svm_model_path,dir_path_string ,model_name.substr(0,model_name.size()-4));
    }
}


void ObjectEntity::save_current_data(){
    m_detector_->save_current_data();
}

ObjectEntity::~ObjectEntity(){
    delete m_detector_;
}
