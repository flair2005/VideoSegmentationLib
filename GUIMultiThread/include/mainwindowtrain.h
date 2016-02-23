#ifndef MAINWINDOWTRAIN_H
#define MAINWINDOWTRAIN_H
//#include "objectentity.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include "segmentation.h"
#include "Parameters.h"
#include "Structures.h"
#include "pclviewer.h"

#include "utils.h"
#include "pcl_segmentation.h"
#include "ms_segmentation.h"

namespace Ui {
class MainWindowTrain;
}

class SharedImageBuffer;
class CameraView;
class PlaybackThread;
class ObjectEntity;

class MainWindowTrain : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindowTrain(QWidget *parent = 0);
    ~MainWindowTrain();

private:
    Ui::MainWindowTrain *ui;
    QGraphicsScene *scene;

    //action to open a folder
    QAction *openAct;
    //action to play the scene
    QAction *playAct;
    //the path to the directory where the training images are located
    QString trainingDir;
    std::vector<std::string> imagePaths, depthPaths;


    SharedImageBuffer *m_sharedImageBuffer;

    //the thread that will do the work
    PlaybackThread *m_processingThread;
    Segmentation* segmentation;
    MSSegmentation* ms_segmentation;
    PCLSegmentation *pcl_segmentation;
    SegmentationParameters seg_params;
    Mat m_current_frame,m_current_depth_float;
    Segment* m_current_segment;
    //stores the added segments for when we want to add the remaining ones
    //as background
    vector<Segment*> added_foreground_segments;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_pcl_cloud_rgba;

    std::vector<ObjectEntity*> trained_objects;
    //temporary members for newly created objects
    QString svm_path;
    QString object_directory_path;
    QString object_name;

    ObjectEntity* test_object;
    //a pointer to the current object model to be trained
    ObjectEntity* train_object;

    PCLViewer *pclViewer_raw,*pclViewer_detections,*pclViewer_models;
    PCS *pcs_raw;
    Eigen::Vector4d geo_panel, geo_viewer;

    Utils utils_;

// functions    
    void display_cur_point_cloud();
    void display_segment_point_cloud();

//Qt4
public slots:
    void updateMouseData(const MouseData& mouseData);
    void updateFrame(const QImage &frame);
    void updateSegmentation(const QImage &frame);
    void updateVideoSegmentation(const QImage &frame);


protected:

    //void contextMenuEvent(QContextMenuEvent *event) Q_DECL_OVERRIDE;
    void createActions();
    void add_training_data();

private slots:
    void open();
    void on_actionOpen_folder_triggered();    
    void on_playPushButton_clicked();    
    void on_thresholdDoubleSpinBox_valueChanged(double arg1);
    void on_comboBox_activated(int index);
    void on_openSVMDirButton_pressed();

    void on_objectsComboBox_currentIndexChanged(int index);
    void on_saveNewObjectButton_pressed();
    void on_testPushButton_released();
    void on_segFramePushButton_clicked();
    void on_addSegmentPushButton_pressed();
    void on_addBGpushButton_pressed();
    void on_currSegmentsHorizontalSlider_valueChanged(int value);
    void on_currSegmentsHorizontalSlider_sliderReleased();
    void on_currSegmentsHorizontalSlider_sliderMoved(int position);
    void on_delayHorizontalScrollBar_sliderMoved(int position);
    void on_frameIndexHorizontalScrollBar_sliderMoved(int position);
    void on_pushButton_pressed();
    void on_frameIndexHorizontalScrollBar_sliderPressed();
    void on_startVideoSegPushButton_2_pressed();
    void on_vidSegPushButton_pressed();
    void on_testSegmentPushButton_pressed();
    void on_propScaleSpinBox_valueChanged(int value);
    void on_addTrainingDataPushButton_pressed();
    void on_trainingExamplesHorizontalSlider_sliderMoved(int position);
    void on_testFramePushButton_pressed();
    void on_scharrRadioButton_clicked(bool checked);
    void on_startScaleSpinBox_valueChanged(int arg1);
    void on_scalesSpinBox_valueChanged(int arg1);
    void on_actionEqualize_cameras_triggered();
    void on_addRemainingPushButton_pressed();

    void on_spSpinBox_valueChanged(int arg1);
    void on_srSpinBox_valueChanged(int arg1);
    void on_msizeSpinBox_valueChanged(int arg1);
    void on_lowSamplingCheckBox_clicked(bool checked);
};

#endif // MAINWINDOWTRAIN_H
