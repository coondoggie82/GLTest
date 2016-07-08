#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <QGLWidget>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


class GLViewer : public QGLWidget
{
    Q_OBJECT

public:
    explicit GLViewer(QWidget *parent = 0);
    void showPolygonMesh(pcl::PolygonMesh, QString meshName);
    QSize sizeHint() const;

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void getDiffs();

private:
    int m_nxRot;
    int m_nyRot;
    int m_nzRot;
    double m_lfDx;
    double m_lfDy;
    double m_lfDz;
    double m_lfxTrans;
    double m_lfyTrans;
    double m_lfzTrans;
    int m_nshift;
    QPoint m_QPlastPos;
    pcl::PolygonMesh m_PMmesh;

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setXTranslation(double dist);
    void setYTranslation(double dist);
    void setZTranslation(double dist);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
    void xTranslationChanged(double dist);
    void yTranslationChanged(double dist);
    void zTranslationChanged(double dist);
};

#endif // PCLVIEWER_H
