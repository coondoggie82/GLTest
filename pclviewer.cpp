#include "pclviewer.h"
#include <vtkRenderWindow.h>


PCLViewer::PCLViewer(QWidget *parent) :
    QVTKWidget(parent)
{
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));
    SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(GetInteractor(), GetRenderWindow());
    viewer->setShowFPS(false);
    viewer->addCoordinateSystem(0.05);
    update();
}

QSize PCLViewer::sizeHint() const{
    return QSize(400,400);
}

void PCLViewer::showPolygonMesh(pcl::PolygonMesh mesh, QString meshName){
    viewer->addPolygonMesh(mesh, meshName.toLatin1().data());
}

