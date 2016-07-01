#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <QOpenGLWidget>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


class GLViewer : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit GLViewer(QWidget *parent = 0);
    void showPolygonMesh(pcl::PolygonMesh, QString meshName);
    QSize sizeHint() const;

private:

};

#endif // PCLVIEWER_H
