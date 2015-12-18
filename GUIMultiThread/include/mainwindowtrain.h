#ifndef MAINWINDOWTRAIN_H
#define MAINWINDOWTRAIN_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include "segmentation.h"
#include "Parameters.h"

namespace Ui {
class MainWindowTrain;
}

class SharedImageBuffer;
class CameraView;
class PlaybackThread;

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
    std::vector<std::string> imagePaths;


    SharedImageBuffer *m_sharedImageBuffer;

    //the thread that will do the work
    PlaybackThread *m_processingThread;
    Segmentation segmentation;
    SegmentationParameters seg_params;

//Qt4
public slots:
    void updateFrame(const QImage &frame);
    void updateSegmentation(const QImage &frame);


protected:

    //void contextMenuEvent(QContextMenuEvent *event) Q_DECL_OVERRIDE;
    void createActions();

private slots:
    void open();
    void on_actionOpen_folder_triggered();
    void on_playPushButton_clicked();    
    void on_thresholdDoubleSpinBox_valueChanged(double arg1);
};

#endif // MAINWINDOWTRAIN_H