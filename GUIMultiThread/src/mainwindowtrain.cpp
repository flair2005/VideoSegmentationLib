#include "mainwindowtrain.h"
#include "ui_mainwindowtrain.h"
#include "MatToQImage.h"
#include <QObject>
#include <QFileDialog>
#include <QDirIterator>
#include <QDebug>
#include <QThread>

#include "SharedImageBuffer.h"
#include "PlaybackThread.h"



MainWindowTrain::MainWindowTrain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindowTrain)
{
    ui->setupUi(this);


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


}

void MainWindowTrain::updateFrame(const QImage &frame){
    // Display frame
    ui->frameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

}

void MainWindowTrain::updateSegmentation(const QImage &frame){
    // Display frame
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


    QDir folder(trainingDir);
    folder.setNameFilters( QStringList() << "*.png" <<  "*.jpg" );
    QStringList fileList = folder.entryList();

    //add the paths to the images
    for(QString file : fileList){
        QString fullPathFile = trainingDir+"/"+file;
        //qDebug() <<fullPathFile;
        std::string utf8_path = fullPathFile.toUtf8().constData();
        imagePaths.push_back(utf8_path);

    }
    //sort them naturally!
    std::sort(imagePaths.begin(), imagePaths.end(), compareNat);
     //open the first image
    if(imagePaths.size()>0){
        cv::Mat src = cv::imread(imagePaths[0],-1);
        QImage qSrc = MatToQImage(src);

        // Display frame
        ui->frameLabel->setPixmap(QPixmap::fromImage(qSrc).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));

    }
}

void MainWindowTrain::on_playPushButton_clicked()
{

    // Start the processing thread
    m_processingThread->setPath(imagePaths);
    m_processingThread->start((QThread::Priority)1);
}



void MainWindowTrain::on_thresholdDoubleSpinBox_valueChanged(double arg1)
{
    seg_params.setThreshold(arg1);
}
