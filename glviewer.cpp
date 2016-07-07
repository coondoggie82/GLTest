#include "glviewer.h"
#include <QMouseEvent>
#include <QCoreApplication>
#include <QGLShaderProgram>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include "glut.h"

GLViewer::GLViewer(QWidget *parent) :
    QGLWidget(parent)
{
    //Setup OpenGL Widget
    m_nxRot = 0;
    m_nyRot = 0;
    m_nzRot = 0;
    m_lfDx = 0.01;
    m_lfDy = 0.01;
    m_lfDz = 0.01;
    m_lfxTrans = 0;
    m_lfyTrans = 0;
    m_lfzTrans = 0;
    m_nshift = 0;

    m_qtGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
    m_qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);

    setAutoBufferSwap(true);
    setFocusPolicy(Qt::StrongFocus);
}

void GLViewer::getDiffs()
{
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    PointCloudT* meshCloud = new PointCloudT;
    pcl::fromPCLPointCloud2(m_PMmesh.cloud, *meshCloud);

    double xMin = 1e12;
    double xMax = 1e-12;
    double yMin = 1e12;
    double yMax = 1e-12;
    double zMin = 1e12;
    double zMax = 1e-12;
    double xAvg = 0.0;
    double yAvg = 0.0;
    double zAvg = 0.0;
    for(size_t i=0; i<meshCloud->points.size(); ++i)
    {        
        int v1, v2, v3;
        PointT p1 = meshCloud->points[i];
        if(p1.x < xMin)
        {
            xMin = p1.x;
        }
        if(xMax < p1.x)
        {
            xMax = p1.x;
        }
        if(p1.y < yMin)
        {
            yMin = p1.y;
        }
        if(yMax < p1.y)
        {
            yMax = p1.y;
        }
        if(p1.z < zMin)
        {
            zMin = p1.z;
        }
        if(zMax < p1.z)
        {
            zMax = p1.z;
        }
        xAvg += p1.x;
        yAvg += p1.y;
        zAvg += p1.z;
    }
    m_lfDx = (xMax-xMin)/250.0;
    m_lfDy = (yMax-yMin)/250.0;
    m_lfDz = (yMax-yMin)/20.0;

    xAvg = xAvg/double(meshCloud->points.size());
    yAvg = yAvg/double(meshCloud->points.size());
    zAvg = zAvg/double(meshCloud->points.size());
    
    m_lfxTrans = -xAvg;
    m_lfyTrans = -yAvg;
    m_lfzTrans = -zAvg - 1.5*(yMax-yMin);
}

QSize GLViewer::sizeHint() const{
    return QSize(400,400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void GLViewer::showPolygonMesh(pcl::PolygonMesh mesh, QString meshName){
    //Do your magic to display the mesh here :)    //paintGL();
    m_PMmesh = mesh;
    getDiffs();
    paintGL();
    initializeGL();
}

void GLViewer::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_nxRot) {
        m_nxRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void GLViewer::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_nyRot) {
        m_nyRot = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void GLViewer::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_nzRot) {
        m_nzRot = angle;
        emit zRotationChanged(angle);
        update();
    }
}

void GLViewer::setXTranslation(double dist)
{
    if (dist != m_lfxTrans) {
        m_lfxTrans = dist;
        emit xTranslationChanged(dist);
        update();
    }
}

void GLViewer::setYTranslation(double dist)
{
    if (dist != m_lfyTrans) {
        m_lfyTrans = dist;
        emit yTranslationChanged(dist);
        update();
    }
}

void GLViewer::setZTranslation(double dist)
{
    if (dist != m_lfzTrans) {
        m_lfzTrans = dist;
        cout << "zoomed to " << m_lfzTrans << endl;
        emit zTranslationChanged(dist);
        update();
    }
}

void GLViewer::initializeGL()
{
    
    // glClearColor(0.0, 0.0, 0.0, 0.0);    //black background

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);
    glDisable(GL_FOG);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_1D);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_3D);
    glShadeModel(GL_FLAT);
    // glCullFace(GL_FRONT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0, 1.0, 0.01f, 350.0f); 
    // glShadeModel(GL_SMOOTH);
    // glEnable(GL_LIGHTING);
    // glEnable(GL_LIGHT0);
    glEnable(GL_MULTISAMPLE);
    // static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    // glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void GLViewer::resizeGL(int w, int h)
{
    glLoadIdentity();
    gluPerspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

void GLViewer::mousePressEvent(QMouseEvent *event)
{
    setFocusPolicy(Qt::StrongFocus);
    m_QPlastPos = event->pos();
}

void GLViewer::mouseMoveEvent(QMouseEvent *event)
{
    setFocusPolicy(Qt::StrongFocus);
    int dx = event->x() - m_QPlastPos.x();
    int dy = event->y() - m_QPlastPos.y();

    if (!m_nshift && event->buttons() & Qt::LeftButton) 
    {
        // cout << "rotate x and y " << m_nshift << endl;
        setXRotation(m_nxRot + 8 * dy);
        setYRotation(m_nyRot + 8 * dx);
    } 
    else if (!m_nshift && event->buttons() & Qt::RightButton) 
    {
        // cout << "zoom in/out " << m_nshift << endl;
        setZTranslation(m_lfzTrans - m_lfDz * dy);
    } 
    else if (m_nshift && event->buttons() & Qt::RightButton) 
    {
        // cout << "rotate x and z " << m_nshift << endl;
        setXRotation(m_nxRot + 8 * dy);
        setZRotation(m_nzRot + 8 * dx);
    } 
    else if (m_nshift && event->buttons() & Qt::LeftButton) 
    {
        // cout << "translate x and y " << m_nshift << endl;
        setXTranslation(m_lfxTrans + m_lfDx * dx);
        setYTranslation(m_lfyTrans - m_lfDy * dy);
    }

    m_QPlastPos = event->pos();
}

void GLViewer::keyPressEvent(QKeyEvent *event)
{
    // cout << "key pressed: ";
    std::string str;
    str = event->text().toUtf8().constData();
    if (event->modifiers() & Qt::ShiftModifier)
    {
        m_nshift = 1;
        // cout << "SHIFT" << endl;
    }
    else
    {
        // cout << str << endl;
    }
}

void GLViewer::keyReleaseEvent(QKeyEvent *event)
{
    m_nshift = 0;
}

void GLViewer::paintGL()
{
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    PointCloudT* meshCloud = new PointCloudT;
    pcl::fromPCLPointCloud2(m_PMmesh.cloud, *meshCloud);

    QSize viewport_size = size();
    glViewport(0, 0, viewport_size.width(), viewport_size.height());

    glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // gluPerspective(45.0, 1.0, 0.0, 2.0);
    // glFrustum(-1, 1, -1, 1, 8, 12); // near and far match your triangle Z distance
    // glLoadIdentity();
    // gluLookAt(0.0, 0.0, -8.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // glEnable(GL_LIGHTING);
    glTranslatef(m_lfxTrans, m_lfyTrans, m_lfzTrans);

    glRotatef(m_nxRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(m_nyRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(m_nzRot / 16.0, 0.0, 0.0, 1.0);

    size_t nPolygons = m_PMmesh.polygons.size();
    // nPolygons = 3;

    // glPushMatrix();

    // ofstream outDebug;
    // outDebug.open("/home/mike/Documents/dbug.txt",ios::app);
    for(size_t i=0; i<nPolygons; ++i)
    {
        int v1, v2, v3;
        v1 = m_PMmesh.polygons[i].vertices[0];
        v2 = m_PMmesh.polygons[i].vertices[1];
        v3 = m_PMmesh.polygons[i].vertices[2];
        PointT p1 = meshCloud->points[v1];
        PointT p2 = meshCloud->points[v2];
        PointT p3 = meshCloud->points[v3];
        // glColor3f(1.0, 1.0, 1.0);
        
        float rAvg = float(int(p1.r)+int(p2.r)+int(p3.r))/3.0f/255.0f;
        float gAvg = float(int(p1.g)+int(p2.g)+int(p3.g))/3.0f/255.0f;
        float bAvg = float(int(p1.b)+int(p2.b)+int(p3.b))/3.0f/255.0f;
        float aAvg = float(int(p1.a)+int(p2.a)+int(p3.a))/3.0f/255.0f;
        glColor4f(rAvg, gAvg, bAvg, aAvg);
        glBegin(GL_TRIANGLES);
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glEnd();    
    }

    delete meshCloud;
}

