#include "glviewer.h"


GLViewer::GLViewer(QWidget *parent) :
    QOpenGLWidget(parent)
{
    //Setup OpenGL Widget

}

QSize GLViewer::sizeHint() const{
    return QSize(400,400);
}

void GLViewer::showPolygonMesh(pcl::PolygonMesh mesh, QString meshName){
    //Do your magic to display the mesh here :)
}

