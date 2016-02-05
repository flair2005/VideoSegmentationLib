#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <QFileDialog>
#include <QMessageBox>

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));    
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    // Connect point size slider
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    //set the point size default value to 3
    pSliderValueChanged (3);
    viewer->resetCamera ();
    viewer->addCoordinateSystem(0.1);
    ui->qvtkWidget->update ();



    pcs = new PCS();
}

void PCLViewer::setPC(CloudTPtr pc)
{
    viewer->addPointCloud (pc, "viewer");
    ui->qvtkWidget->update ();
}

void PCLViewer::setPCS(PCS* p)
{
    pcs = p;
    viewer->addPointCloud (pcs->pcs.at(0), "viewer");
    ui->qvtkWidget->update ();
}

void PCLViewer::updatePC(CloudTPtr pc)
{
    viewer->updatePointCloud(pc, "viewer");
    ui->qvtkWidget->update ();
}

void PCLViewer::pSliderValueChanged (int value)
{
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "viewer");
    ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
    delete ui;
}
