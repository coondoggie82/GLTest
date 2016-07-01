#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <pcl/io/ply_io.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("GLTest");
    glViewer = new GLViewer(ui->glWidget);
    openMeshFile();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openMeshFile(){
    pcl::PolygonMesh mesh = pcl::PolygonMesh();
    pcl::io::loadPLYFile("./scans/testScan.ply", mesh);
    glViewer->showPolygonMesh(mesh, "mesh1");
}
