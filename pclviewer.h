#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QVTKWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


class PCLViewer : public QVTKWidget
{
    Q_OBJECT
public:
    explicit PCLViewer(QWidget *parent = 0);
    void showPolygonMesh(pcl::PolygonMesh, QString meshName);
    QSize sizeHint() const;

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;

};

#endif // PCLVIEWER_H
