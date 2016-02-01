#include "PlaybackThread.h"
#include "mainwindowtrain.h"
#include "ui_mainwindowtrain.h"
#include "MatToQImage.h"
#include <QObject>
#include <QFileDialog>
#include <QDirIterator>
#include <QDebug>
#include <QThread>

#include "SharedImageBuffer.h"





MainWindowTrain::MainWindowTrain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindowTrain),
    segmentation(nullptr),
    m_current_segment(nullptr),
    test_object(nullptr),
    train_object(nullptr)
{
    ui->setupUi(this);
    geo_panel << 0, 0, 382, 382;
    geo_viewer << 0, 0, 750, 500;


    // Create SharedImageBuffer object
    m_sharedImageBuffer = new SharedImageBuffer();

    //create processing thread
    // Create capture thread
    m_processingThread = new PlaybackThread(m_sharedImageBuffer, &seg_params);

    // Setup signal/slot connections

    //Qt5
    //connect(m_processingThread, &PlaybackThread::newFrame, this, &MainWindowTrain::updateFrame);
    //connect(m_processingThread, &PlaybackThread::newSegmentation, this, &MainWindowTrain::updateSegmentation);
    //Qt4
    connect(m_processingThread, SIGNAL(newFrame(const QImage&)), this, SLOT(updateFrame(const QImage&)));
    connect(m_processingThread, SIGNAL(newSegmentation(const QImage&)), this, SLOT(updateSegmentation(const QImage&)));
    connect(m_processingThread, SIGNAL(newVideoSegmentation(const QImage&)), this, SLOT(updateVideoSegmentation(const QImage&)));


    connect(ui->frameLabel, SIGNAL(newMouseData(MouseData)), this, SLOT(updateMouseData(const MouseData& )));
    connect(ui->segmentationLabel, SIGNAL(newMouseData(MouseData)), this, SLOT(updateMouseData(const MouseData& )));

    ui->frameLabel->setStyleSheet("QLabel {border: 0px solid gray;border-radius: 0px;background-color: black;padding: 0px 0px 0px 0px;}");
    ui->frameLabel->setMargin(0);
    ui->frameLabel->setContentsMargins(0,0,0,0);
    ui->segmentationLabel->setContentsMargins(0,0,0,0);
    ui->segmentationLabel->setMargin(0);
    ui->segmentationLabel->setStyleSheet("QLabel {border: 0px solid gray;border-radius: 0px;background-color: black;padding: 0px 0px 0px 0px;}");

    ui->frameLabelVideo->setStyleSheet("QLabel {border: 0px solid gray;border-radius: 0px;background-color: black;padding: 0px 0px 0px 0px;}");
    ui->frameLabelVideoSegmentation->setStyleSheet("QLabel {border: 0px solid gray;border-radius: 0px;background-color: black;padding: 0px 0px 0px 0px;}");
    //set the segment slider to 1
    ui->currSegmentsHorizontalSlider->setMaximum(0);

    ui->frameIndexHorizontalScrollBar->setMaximum(0);


    //add PCL viewer
    pclViewer_raw = new PCLViewer();
    CloudTPtr cloud(new CloudT);
    pclViewer_raw->setPC(cloud);
    pclViewer_raw->setWindowTitle("PCS raw");
    pclViewer_raw->setGeometry(geo_viewer[0]+geo_panel[2], geo_viewer[1], geo_viewer[2], geo_viewer[3]);
    //pclViewer_raw->viewer->setCameraPosition(campos[0], campos[1], campos[2], camposup[0], camposup[1], camposup[2]);
    //pclViewer_raw->viewer->setCameraClipDistances(camclip[0], camclip[1]);
    pclViewer_raw->show();


    //set the scale selector values
    ui->propScaleSpinBox->setValue(seg_params.getScaleForPropagation());
    int allowed_interval = seg_params.getScales()-seg_params.getStartingScale()-1;
    ui->propScaleSpinBox->setMaximum(allowed_interval);

}

void MainWindowTrain::updateMouseData(const MouseData& mouseData){
    cout <<"row,col: "<<mouseData.row<<" "<<mouseData.col<<endl;

    if(segmentation){
        cv::Size size = segmentation->getSize(seg_params.getScaleForPropagation());
        int row = (mouseData.row*1.f/ui->frameLabel->height())*size.height;
        int col = (mouseData.col*1.f/ui->frameLabel->width())*size.width;
        m_current_segment = segmentation->get_segment_at_fast(row,col);
        if(m_current_segment){
            //add to ui->segmentsFrameLabel
            QImage frame = MatToQImage(m_current_segment->getMatOriginalColour());
            ui->segmentsFrameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->segmentsFrameLabel->width(), ui->segmentsFrameLabel->height(),Qt::KeepAspectRatio));
        }
        else{
            cout <<" no segment at "<<mouseData.row<<" "<<mouseData.col<<endl;
        }

    }

}

void MainWindowTrain::updateFrame(const QImage &frame){
    // Display frame
    ui->frameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));
    ui->frameLabelVideo->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

}

void MainWindowTrain::updateSegmentation(const QImage &frame){
    // Display frame
    ui->segmentationLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->segmentationLabel->width(), ui->segmentationLabel->height(),Qt::KeepAspectRatio));

}

void MainWindowTrain::updateVideoSegmentation(const QImage &frame){
    // Display frame
    ui->frameLabelVideoSegmentation->setPixmap(QPixmap::fromImage(frame).scaled(ui->segmentationLabel->width(), ui->segmentationLabel->height(),Qt::KeepAspectRatio));
    ui->segmentationLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->segmentationLabel->width(), ui->segmentationLabel->height(),Qt::KeepAspectRatio));

}

void MainWindowTrain::createActions()
{

    /*actionOpen_folder = new QAction(tr("&Open"), this);
    actionOpen_folder->setShortcuts(QKeySequence::Open);
    actionOpen_folder->setStatusTip(tr("Open a new folder"));
    connect(actionOpen_folder, SIGNAL(triggered()), this, SLOT(open()));*/
}


void MainWindowTrain::open(){

}

MainWindowTrain::~MainWindowTrain()
{
    delete ui;
    for(auto Object : trained_objects) {
        delete Object;
    }
}

/*
 * comparison function to sort filenames in a natural order
 * e.g.:
 *  1.txt
 *  2.txt
 *  10.txt
 *  20.txt
 *
 */
bool compareNat(const std::string& a, const std::string& b)
{
    if (a.empty())
        return true;
    if (b.empty())
        return false;
    if (std::isdigit(a[0]) && !std::isdigit(b[0]))
        return true;
    if (!std::isdigit(a[0]) && std::isdigit(b[0]))
        return false;
    if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
    {
        if (std::toupper(a[0]) == std::toupper(b[0]))
            return compareNat(a.substr(1), b.substr(1));
        return (std::toupper(a[0]) < std::toupper(b[0]));
    }

    // Both strings begin with digit --> parse both numbers
    std::istringstream issa(a);
    std::istringstream issb(b);
    int ia, ib;
    issa >> ia;
    issb >> ib;
    if (ia != ib)
        return ia < ib;

    // Numbers are the same --> remove numbers and recurse
    std::string anew, bnew;
    std::getline(issa, anew);
    std::getline(issb, bnew);
    return (compareNat(anew, bnew));
}



void MainWindowTrain::on_actionOpen_folder_triggered()
{

    trainingDir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    "/home/martin/bagfiles",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    if(trainingDir==nullptr)
        return;
    //open the img folder

    {
        QString dir(trainingDir+"/img/");
        //qDebug() << dir;
        QDir folder(dir);
        folder.setNameFilters( QStringList() << "*.png" <<  "*.jpg" );
        QStringList fileList = folder.entryList();

        //add the paths to the images
        for(QString file : fileList){
            QString fullPathFile = dir+"/"+file;
            //qDebug() <<fullPathFile;
            std::string utf8_path = fullPathFile.toUtf8().constData();
            imagePaths.push_back(utf8_path);

        }

        //sort them naturally!
        std::sort(imagePaths.begin(), imagePaths.end(), compareNat);
         //open the first image
        if(imagePaths.size()>0){
            m_current_frame = cv::imread(imagePaths[0],-1);
            QImage qSrc = MatToQImage(m_current_frame);

            // Display frame
            ui->frameLabel->setPixmap(QPixmap::fromImage(qSrc).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));
            ui->frameLabelVideo->setPixmap(QPixmap::fromImage(qSrc).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

        }
        //set the size of the scroll bar
        ui->frameIndexHorizontalScrollBar->setMaximum(imagePaths.size()-1);
    }

    //open the depth folder
    {
        QString dir(trainingDir+"/depth/");
        QDir folder(dir);
        folder.setNameFilters( QStringList() << "*.png" );
        QStringList fileList = folder.entryList();

        //add the paths to the images
        for(QString file : fileList){
            QString fullPathFile = dir+"/"+file;
            //qDebug() <<fullPathFile;
            std::string utf8_path = fullPathFile.toUtf8().constData();

            depthPaths.push_back(utf8_path);

        }

        if(depthPaths.size()>0){
            cv::Mat depth = cv::imread(depthPaths[0], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
            depth.convertTo(m_current_depth_float,CV_32FC1);
            m_current_depth_float *= 0.001f;
            cv::medianBlur(m_current_depth_float, m_current_depth_float, 3);
            cv::medianBlur(m_current_depth_float, m_current_depth_float, 5);
//            Mat tmp(m_current_depth_float.size(),CV_32FC1);
//            cv::bilateralFilter(m_current_depth_float, tmp, -1, 15, 7);
//            m_current_depth_float = tmp;

        }


        //sort them naturally!
        std::sort(depthPaths.begin(), depthPaths.end(), compareNat);

    }
    //display the point cloud
    display_cur_point_cloud();

}

void MainWindowTrain::display_cur_point_cloud(){
    //set the point cloud viewer

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pclViewer_raw->updatePC(pcl_cloud);
    Utils utils;
    utils.image_to_pcl(m_current_frame,m_current_depth_float,pcl_cloud);
    //utils.remove_outliers(pcl_cloud,pcl_cloud);
    pclViewer_raw->updatePC(pcl_cloud);
    //pclViewer_raw->setPC(pcl_cloud);
    pclViewer_raw->show();
}

void MainWindowTrain::display_segment_point_cloud(){
    //set the point cloud viewer

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pclViewer_raw->updatePC(pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat tmp_img, tmp_depth;
    std::vector<Segment*> fg_segments({m_current_segment});
    Utils utils;
    utils.cropped_pcl_from_segments(m_current_frame, m_current_depth_float,fg_segments, cropped_cloud, tmp_img, tmp_depth);


    //downsample
    //utils.sub_sample(cropped_cloud,cropped_cloud);

    //utils.image_to_pcl(m_current_frame,m_current_depth_float,pcl_cloud);
    pclViewer_raw->updatePC(cropped_cloud);
    //pclViewer_raw->setPC(pcl_cloud);
    pclViewer_raw->show();
}

void MainWindowTrain::on_playPushButton_clicked()
{

    // Start the processing thread
    m_processingThread->setPath(imagePaths,depthPaths);
    m_processingThread->start((QThread::Priority)1);
}



void MainWindowTrain::on_thresholdDoubleSpinBox_valueChanged(double arg1)
{
    seg_params.setThreshold(arg1);
}

void MainWindowTrain::on_comboBox_activated(int index)
{



}

/*
 *
 * button to load an .xml file with the model of an object detector
 *
 */
void MainWindowTrain::on_openSVMDirButton_pressed()
{
    svm_path = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                 "/home/martin/bagfiles",
                                                 QFileDialog::ShowDirsOnly
                                                 | QFileDialog::DontResolveSymlinks);

    if(svm_path!= nullptr)
        ui->labelPathModel->setText(svm_path);

}


/*
 *
 * a new action has been chosen: either
 *  - create a new object model
 *  - open a current one
 *
 *
 */
void MainWindowTrain::on_objectsComboBox_currentIndexChanged(int index)
{
    //new object
    if(index == 0){
        ui->labelPathModel->setText(QString(""));
        ui->objectNameLineEdit->setText(QString(""));

    }

    //new temporal object
    else if(index == 1){

       std::cout <<" opening temporal object"<<std::endl;
       std::cout <<" opening temporal object"<<std::endl;
       QString temp_object_folder =  QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                        "/home/martin/bagfiles/MODELS",
                                                        QFileDialog::ShowDirsOnly
                                                        | QFileDialog::DontResolveSymlinks);


       if(temp_object_folder==nullptr)
           return;

       //gets the name of the folder
       QFileInfo fi(temp_object_folder);
       QString folder_name = fi.baseName();
       //goes up and gets the name of the parent directory
       QDir dir_object(temp_object_folder);
       dir_object.cdUp();
       QString dir_name = dir_object.path();
       qDebug()<<dir_name;
       ui->objectNameLineEdit->setText(folder_name);
       ui->labelPathModel->setText(dir_name);
       object_name = folder_name;
       train_object = new ObjectEntity(object_name,dir_name,dir_name,true);

    }


    //open object
    else if(index == 2){
        QString svm_file_full_path = QFileDialog::getOpenFileName(this,
                                                               tr("Open XML File 1"), "/home/martin/bagfiles/MODELS", tr("XML Files (*.xml)"));

        if(svm_file_full_path==nullptr)
            return;
        QDir folder(svm_file_full_path);
        QString folder_path = folder.absoluteFilePath(svm_file_full_path);
        QFileInfo fi(svm_file_full_path);
        QString folder_name = fi.baseName();

        QString file_name = fi.fileName();

        ui->objectNameLineEdit->setText(folder_name);
        ui->labelPathModel->setText(file_name);

        object_name = folder_name;

        test_object = new ObjectEntity(object_name,folder_path,svm_file_full_path,false);


    }
    else{
        int offset_index = index -2;
        ui->objectNameLineEdit->setText(trained_objects[offset_index]->m_name_);
        ui->labelPathModel->setText(trained_objects[offset_index]->m_svm_path_);

    }

}

/*
 * saves the results of a new object detector
 *
 *
 *
 */
void MainWindowTrain::on_saveNewObjectButton_pressed()
{

    QString saveText("Save");
    //we want to save the current state of the detector
    if(train_object&& ui->saveNewObjectButton->text()==saveText){
        train_object->save_current_data();
    }
    else{
        object_name = ui->objectNameLineEdit->text();
        object_directory_path=ui->labelPathModel->text();
        ui->objectsComboBox->addItem(object_name);

        //create new directory at the path with the name of the object
        QString path(object_directory_path+"/"+object_name);
        QString path_clouds(object_directory_path+"/"+object_name+"/clouds");
        qDebug() << path;
        QDir dir = QDir::root();
        dir.mkpath(path);
        //dir.mkpath(path_clouds);
        ObjectEntity* object = new ObjectEntity(object_name,object_directory_path, svm_path,true);
        train_object = object;
        trained_objects.push_back(object);
        ui->saveNewObjectButton->setText(saveText);
    }




}

/*
 * runs the segmentation on the whole sequence
 * and displays the results
 *
 */

void MainWindowTrain::on_testPushButton_released()
{
    //test the loaded model
    if(!test_object){
        return;
    }
    else{
        // Start the processing thread
        m_processingThread->setPath(imagePaths,depthPaths);
        m_processingThread->activateTestMode(test_object);
        m_processingThread->start((QThread::Priority)1);
    }
}

/*
 * segments the current frame stored in m_current_frame
 * and displays the result.
 * The segmentation pointer is destroyed and created anew.
 *
 */

void MainWindowTrain::on_segFramePushButton_clicked()
{

   Mat tmp = m_current_frame.clone();
   QImage frame = MatToQImage(tmp);
   ui->frameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));
   if(segmentation)
       delete segmentation;
   segmentation = new Segmentation(tmp, false, seg_params.getScales(), seg_params.getStartingScale());

   segmentation->segment_pyramid(seg_params.getThreshold());
   segmentation->map_segments(seg_params.getScaleForPropagation());
   Mat mat_segmentation = segmentation->getOutputSegmentsPyramid()[seg_params.getScaleForPropagation()];

   frame = MatToQImage(mat_segmentation);
   ui->segmentationLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->segmentationLabel->width(), ui->segmentationLabel->height(),Qt::KeepAspectRatio));
   //segments = seg.getSegmentsPyramid()[seg_params.getScaleForPropagation()];

}

/*
 * add the current segment to the object model
 */

void MainWindowTrain::on_addSegmentPushButton_pressed()
{
    if(train_object && m_current_segment){
        if(train_object->frameData_.fg_segments.size()==0){
            train_object->frameData_.img = m_current_frame.clone();
            train_object->frameData_.depth_float = m_current_depth_float.clone();
        }
        Segment* segment_copy = new Segment(*m_current_segment);
        train_object->frameData_.fg_segments.push_back(segment_copy);
        int number_fg_segms = train_object->frameData_.fg_segments.size();
        cout <<"number_fg_segms="<<number_fg_segms<<endl;
        ui->currSegmentsHorizontalSlider->setMaximum(number_fg_segms);
        ui->currSegmentsHorizontalSlider->setValue(number_fg_segms);
        display_segment_point_cloud();        
        double value = ui->fgLcdNumber->value()+1;
        ui->fgLcdNumber->display(value);
    }

    m_current_segment  = nullptr;

}

void MainWindowTrain::on_addBGpushButton_pressed()
{
    if(train_object&& m_current_segment){
        Segment* segment_copy = new Segment(*m_current_segment);
        train_object->frameData_.bg_segments.push_back(segment_copy);
        double value = ui->bgLcdNumber->value()+1;
        ui->bgLcdNumber->display(value);
    }
    m_current_segment  = nullptr;
}

/*
 * the slider that selects foreground segments has been moved
 *
 */
void MainWindowTrain::on_currSegmentsHorizontalSlider_valueChanged(int value)
{


}

void MainWindowTrain::on_currSegmentsHorizontalSlider_sliderReleased()
{

}

/*
 * the slider that controls the segments
 * being displayed has moved
 *
 */

void MainWindowTrain::on_currSegmentsHorizontalSlider_sliderMoved(int position)
{
    unsigned int value = position;//ui->currSegmentsHorizontalSlider->value();
    cout <<"slider value="<<value<<endl;

    if(train_object){
        cout <<" frameData_.fg_segments.size()+1="<<train_object->frameData_.fg_segments.size()+1<<endl;

        if(value ==train_object->frameData_.fg_segments.size() && m_current_segment)
        {

            QImage frame = MatToQImage(m_current_segment->getMatOriginalColour());
            ui->segmentsFrameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

        }
        else if (value <train_object->frameData_.fg_segments.size()){
          Segment *seg = train_object->frameData_.fg_segments[value];
          Mat tmp = seg->getMatOriginalColour().clone();
          QImage frame = MatToQImage(tmp);
          ui->segmentsFrameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

       }

    }
}

/*
 * the slider that controls the frames has moved
 *
 *
 */
void MainWindowTrain::on_delayHorizontalScrollBar_sliderMoved(int position)
{


}

/*
 * the slider to selects segments moved
 *
 */
void MainWindowTrain::on_frameIndexHorizontalScrollBar_sliderMoved(int position)
{
    cout <<" slider moved to: "<<position<<endl;

    m_current_frame = cv::imread(imagePaths[position],-1);
    cv::Mat depth = cv::imread(depthPaths[position], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    depth.convertTo(m_current_depth_float,CV_32FC1);
    m_current_depth_float *= 0.001f;
    cv::medianBlur(m_current_depth_float, m_current_depth_float, 3);
    cv::medianBlur(m_current_depth_float, m_current_depth_float, 5);
//    Mat tmp(m_current_depth_float.size(),CV_32FC1);
//    cv::bilateralFilter(m_current_depth_float, tmp, -1, 15, 7);
//    m_current_depth_float = tmp;

    QImage qSrc = MatToQImage(m_current_frame);
    // Display frame
    ui->frameLabel->setPixmap(QPixmap::fromImage(qSrc).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

    display_cur_point_cloud();

}

/*
 * train button pushed
 *
 */
void MainWindowTrain::on_pushButton_pressed()
{
    if(train_object)
        train_object->m_detector_->train();

}
/*
 * the frame slider has been pressed: good moment
 * for adding training data to the object detector
 *
 *
 */
void MainWindowTrain::on_frameIndexHorizontalScrollBar_sliderPressed()
{
    //if we have data available
    if(train_object && train_object->frameData_.fg_segments.size()>0
            && train_object->frameData_.bg_segments.size()>0){

        train_object->m_detector_->add_selected_segments(m_current_frame,
          m_current_depth_float, train_object->frameData_.fg_segments,
            train_object->frameData_.bg_segments );

        train_object->frameData_.fg_segments.clear();
        train_object->frameData_.bg_segments.clear();
    }
}

void MainWindowTrain::on_startVideoSegPushButton_2_pressed()
{
    // Start the processing thread
    m_processingThread->activateVideoSegmentation();
    m_processingThread->setPath(imagePaths,depthPaths);
    m_processingThread->start((QThread::Priority)1);

}

/*
 * runs the video segmentation method on the whole
 * sequence
 */
void MainWindowTrain::on_vidSegPushButton_pressed()
{

    // Start the processing thread
    m_processingThread->activateVideoSegmentation();
    m_processingThread->setPath(imagePaths,depthPaths);
    m_processingThread->start((QThread::Priority)1);

}

/*
 * tests whether the currently added segments
 * match the trained model
 *
 *
 */
void MainWindowTrain::on_testSegmentPushButton_pressed()
{

    //if we have data available
    if(train_object  && train_object->frameData_.fg_segments.size()>0){

        train_object->m_detector_->test_pcl_segments(m_current_frame,
          m_current_depth_float, train_object->frameData_.fg_segments);

        train_object->frameData_.fg_segments.clear();
        train_object->frameData_.bg_segments.clear();
    }
}

/*
 * the scale selector changed the parameter for the
 * scale to be segmented
 *
 */
void MainWindowTrain::on_propScaleSpinBox_valueChanged(int value)
{
    seg_params.setScaleForPropagation(value);

}
