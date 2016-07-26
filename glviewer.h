#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <QGLWidget>
#include <QTouchEvent>
#include <QGraphicsItem>
#include <qgesture.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


class GLViewer : public QGLWidget
{
    Q_OBJECT

public:
    class SmallDimension { };       // exception class
    class LargeDimension { };       // exception class
    class NameExists     { };       // exception class
    class NameMissing    { };       // exception class

    explicit GLViewer(QWidget *parent = 0);
    void showPolygonMesh(pcl::PolygonMesh mesh, QString meshName);
    void showPolygonMesh(QString meshName);
    void hidePolygonMesh(QString meshName);
    void showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
                   QString cloudName);
    void showCloud(QString cloudName);
    void hideCloud(QString cloudName);
    QSize sizeHint() const;
    std::vector<double> getCameraLocation();
    void setCameraLocation(std::vector<double> vecLoc);
    std::vector<double> getCameraLineOfSight();
    void setCameraLineOfSight(std::vector<double> vecLOS);
    std::vector<double> getCameraUp();
    void setCameraUp(std::vector<double> vecUp);
    void pointPicker();
    std::vector<double>getCloudPoint();
    int getCloudIndex();
    int getMeshPolygonIndex();
    std::string getIndexName();

protected:
    void checkCameraDim(size_t uDim);
    bool checkShowMeshes(size_t i);
    bool checkShowClouds(size_t i);
    void checkMeshNameExists(QString meshName);
    void checkMeshNameMissing(QString meshName, bool bShow);
    void checkCloudNameExists(QString cloudName);
    void checkCloudNameMissing(QString cloudName, bool bShow);
    void initializeGL();
    void drawGLTriangles(std::vector<pcl::PolygonMesh>::iterator it);
    void paintGL();
    void resizeGL(int width, int height);
    bool event(QEvent *event);
    void swipeTriggered(QSwipeGesture *gesture);
    void panTriggered(QPanGesture *gesture);
    void pinchTriggered(QPinchGesture *gesture);
    bool gestureEvent(QGestureEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void getDiffs(pcl::PolygonMesh mesh);
    void getDiffs(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    void addMesh(pcl::PolygonMesh mesh);
    void addCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    std::vector<double> normalizeUP();
    std::vector<double> normalizeLOS();
    std::vector<double> normalizeCross(std::vector<double> fVec, 
                                       std::vector<double> upVec);
    std::vector<double> getPointCoords(int mouseX, int mouseY);
    void getPointIndex();
    void findIndex(int idx);

private:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    int m_nxRot;
    int m_nyRot;
    int m_nzRot;
    double m_lfDx;
    double m_lfDy;
    double m_lfDz;
    double m_lfxTrans;
    double m_lfyTrans;
    double m_lfzTrans;
    double m_lfxLOS;
    double m_lfyLOS;
    double m_lfzLOS;
    double m_lfxUp;
    double m_lfyUp;
    double m_lfzUp;

    double m_lfXMin;
    double m_lfXMax;
    double m_lfYMin;
    double m_lfYMax;
    double m_lfZMin;
    double m_lfZMax;
    double m_lfXAvg;
    double m_lfYAvg;
    double m_lfZAvg;
    double m_lfXTot;
    double m_lfYTot;
    double m_lfZTot;
    int    m_nPTot;

    bool m_bDrawAxis;

    bool m_bshift;
    bool m_bMouse;
    bool m_bRotate;
    QPoint m_QPlastPos;
    qreal m_qrTotalScaleFactor;

    std::vector<double> m_vlfPoint;
    int m_nIndex;
    int m_nPolygonIndex;
    std::string m_sIndex;
    bool m_bPoint;
    std::vector<QString> m_vQSCloudNames;
    std::vector<bool> m_vbShowClouds;
    std::vector<QString> m_vQSMeshNames;
    std::vector<bool> m_vbShowMeshes;
    std::vector<pcl::PolygonMesh> m_vMeshes;
    std::vector< PointCloudT::Ptr, 
                 Eigen::aligned_allocator <PointCloudT::Ptr > > m_vClouds;

    // MIP These are for testing purposes
    bool m_bCKey;
    bool m_bHKey;
    void splitPolygonMesh(pcl::PolygonMesh mesh);

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
