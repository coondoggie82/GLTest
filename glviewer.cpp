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
    m_nxRot = 0;    // Tracks rotation of camera about the x-axis
    m_nyRot = 0;    // Tracks rotation of camera about the y-axis
    m_nzRot = 0;    // Tracks rotation of camera about the z-axis
    m_lfDx = 0.01;  // Tracks camera translation change in the x-direction
    m_lfDy = 0.01;  // Tracks camera translation change in the y-direction
    m_lfDz = 0.01;  // Tracks camera translation change in the z-direction
    m_lfxTrans = 0; // Tracks total camera translation change in the x-direction
    m_lfyTrans = 0; // Tracks total camera translation change in the y-direction
    m_lfzTrans = 0; // Tracks total camera translation change in the z-direction
    m_nshift = 0;   // Tracks whether the Shift key is held down

    // Refresh rendering after camera position/rotation changes
    setAutoBufferSwap(true);

    // Makes sure glviewer is in focus for keypress events
    setFocusPolicy(Qt::StrongFocus);   
}

// Determine the best translation changes based on the object size
void GLViewer::getDiffs()
{
    // typedefs used to make cloud point type easier to define
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    // Create normal point cloud
    PointCloudT* meshCloud = new PointCloudT;

    // Transfer pointcloud2 data to point cloud data
    pcl::fromPCLPointCloud2(m_PMmesh.cloud, *meshCloud);

    // used to store maximum/minimum x, y, and z-values
    double xMin = 1e12;
    double xMax = -1e12;
    double yMin = 1e12;
    double yMax = -1e12;
    double zMin = 1e12;
    double zMax = -1e12;

    // used to find the center of the object
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
    // Determined through trial and error for visualization translation
    m_lfDx = (xMax-xMin)/250.0;
    m_lfDy = (yMax-yMin)/250.0;
    m_lfDz = (yMax-yMin)/20.0;

    xAvg = xAvg/double(meshCloud->points.size());
    yAvg = yAvg/double(meshCloud->points.size());
    zAvg = zAvg/double(meshCloud->points.size());

    // Initial translation of the camera to place object in center view    
    m_lfxTrans = -xAvg;
    m_lfyTrans = -yAvg;
    m_lfzTrans = -zAvg - 1.5*(yMax-yMin);

    delete meshCloud;    // Cleans up memory
}

// GL window size
QSize GLViewer::sizeHint() const{
    return QSize(400,400);
}

// Makes sure the rotation angle of the camera is always between 0 and 360 (deg)
static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

// The main function to pass in point cloud polygon mesh data for the viewer
void GLViewer::showPolygonMesh(pcl::PolygonMesh mesh, QString meshName){
    m_PMmesh = mesh;    // Main mesh for drawing the point cloud polygon mesh
    getDiffs();         // Determining object size for translation steps
    paintGL();          // Draws the mesh in the glviewer
    initializeGL();     // Resets GL values for rendering
}

// Update the x-rotation based on mouse/touch events
void GLViewer::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_nxRot) {
        m_nxRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

// Update the y-rotation based on mouse/touch events
void GLViewer::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_nyRot) {
        m_nyRot = angle;
        emit yRotationChanged(angle);
        update();
    }
}

// Update the z-rotation based on mouse/touch events
void GLViewer::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_nzRot) {
        m_nzRot = angle;
        emit zRotationChanged(angle);
        update();
    }
}

// Update the x-translation based on mouse/touch events
void GLViewer::setXTranslation(double dist)
{
    if (dist != m_lfxTrans) {
        m_lfxTrans = dist;
        emit xTranslationChanged(dist);
        update();
    }
}

// Update the y-translation based on mouse/touch events
void GLViewer::setYTranslation(double dist)
{
    if (dist != m_lfyTrans) {
        m_lfyTrans = dist;
        emit yTranslationChanged(dist);
        update();
    }
}

// Update the z-translation based on mouse/touch events
void GLViewer::setZTranslation(double dist)
{
    if (dist != m_lfzTrans) {
        m_lfzTrans = dist;
        emit zTranslationChanged(dist);
        update();
    }
}

// Initialize GL environment settings for lighting, rendering, and depth testing
void GLViewer::initializeGL()
{
    
    // glClearColor(0.0, 0.0, 0.0, 0.0);    // black background

    glEnable(GL_DEPTH_TEST);     // Occludes objects behind closer objects
    glDisable(GL_CULL_FACE);     // Makes sure triangle normals render correctly

    // The following were disabled in hopes of better color mapping
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);
    glDisable(GL_FOG);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_1D);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_3D);
    glShadeModel(GL_FLAT);
    // glCullFace(GL_FRONT);
    glMatrixMode(GL_PROJECTION); // Matrix mode for camera coordinates
    glLoadIdentity();            // Resets camera transformation matrix

    // Sets camera fov, aspect ratio, and near/far values
    gluPerspective(45.0, 1.0, 0.01f, 350.0f); 
    glEnable(GL_MULTISAMPLE);    // Antialiasing

    // These may be helpful later? MIP
    // glShadeModel(GL_SMOOTH);
    // glEnable(GL_LIGHTING);
    // glEnable(GL_LIGHT0);
    // static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    // glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

// Allows the glviewer window to be resized if needed
void GLViewer::resizeGL(int w, int h)
{
    glLoadIdentity();                                     // Reset camera xform
    gluPerspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f); // Set camera values
}

// Captures when a mouse key is pressed
void GLViewer::mousePressEvent(QMouseEvent *event)
{
    // Makes sure glviewer is in focus for keypress events
    setFocusPolicy(Qt::StrongFocus);
    m_QPlastPos = event->pos();      // Stores previous mouse position
}

void GLViewer::mouseMoveEvent(QMouseEvent *event)
{
    // Makes sure glviewer is in focus for keypress events
    setFocusPolicy(Qt::StrongFocus);
    int dx = event->x() - m_QPlastPos.x(); // Mouse change in x-direction
    int dy = event->y() - m_QPlastPos.y(); // Mouse change in y-direction
    QSize windowSize = size();             // View window height/width size

    // Left mouse button held down (without Shift key) rotates camera 
    //     around object
    if (!m_nshift && event->buttons() & Qt::LeftButton) 
    {
        // rotate about x-axis only when mouse is fairly close to object center
        if(event->x() < int(2.0f/3.0f * float(windowSize.width())) && 
           event->x() > int(1.0f/3.0f * float(windowSize.width())))
        {
            setXRotation(m_nxRot + 8 * dy);
        }
        // rotate about z-axis when on the edge of object
        else if(event->x() > int(2.0f/3.0f * float(windowSize.width())))
        {
            if(m_nyRot <= 90*16 && m_nyRot >= -90*16)
            {
            	setZRotation(m_nzRot - 8 * dy);
            }
            else if(m_nyRot >= 270*16 && m_nyRot <= 360*16)
            {
            	setZRotation(m_nzRot - 8 * dy);
            }
            else if(m_nyRot > 90*16 && m_nyRot < 270*16)
            {
                setZRotation(m_nzRot + 8 * dy);
            }
        }
        // rotate opposite about z-axis when on the opposite edge of object
        else if(event->x() < int(1.0f/3.0f * float(windowSize.width())))
        {
            if(m_nyRot <= 90*16 && m_nyRot >= -90*16)
            {
            	setZRotation(m_nzRot + 8 * dy);
            }
            else if(m_nyRot >= 270*16 && m_nyRot <= 360*16)
            {
            	setZRotation(m_nzRot + 8 * dy);
            }
            else if(m_nyRot > 90*16 && m_nyRot < 270*16)
            {
                setZRotation(m_nzRot - 8 * dy);
            }
        }
        // rotate about y-axis only when mouse is fairly close to object center
        if(event->y() < int(2.0f/3.0f * float(windowSize.height())) && 
           event->y() > int(1.0f/3.0f * float(windowSize.height())))
        {
            setYRotation(m_nyRot + 8 * dx);
        }
        // rotate about z-axis when on the edge of object
        else if(event->y() > int(2.0f/3.0f * float(windowSize.height())))
        {
            if(m_nyRot <= 90*16 && m_nyRot >= -90*16)
            {
            	setZRotation(m_nzRot + 8 * dx);
            }
            else if(m_nyRot >= 270*16 && m_nyRot <= 360*16)
            {
            	setZRotation(m_nzRot + 8 * dx);
            }
            else if(m_nyRot > 90*16 && m_nyRot < 270*16)
            {
                setZRotation(m_nzRot - 8 * dx);
            }
        }
        // rotate opposite about z-axis when on the opposite edge of object
        else if(event->y() < int(1.0f/3.0f * float(windowSize.height())))
        {
            if(m_nyRot <= 90*16 && m_nyRot >= -90*16)
            {
            	setZRotation(m_nzRot - 8 * dx);
            }
            else if(m_nyRot >= 270*16 && m_nyRot <= 360*16)
            {
            	setZRotation(m_nzRot - 8 * dx);
            }
            else if(m_nyRot > 90*16 && m_nyRot < 270*16)
            {
                setZRotation(m_nzRot + 8 * dx);
            }
        }
    }
    // Right button held down zooms in/out on object
    else if (event->buttons() & Qt::RightButton) 
    {
        setZTranslation(m_lfzTrans - m_lfDz * dy);
    } 
    // Shift and left button held down translates object in xy-coordinate space
    else if (m_nshift && event->buttons() & Qt::LeftButton) 
    {
        setXTranslation(m_lfxTrans + m_lfDx * dx);
        setYTranslation(m_lfyTrans - m_lfDy * dy);
    }
    
    m_QPlastPos = event->pos();    // Saves mouse position after movement ends
}

// Track when a key is pressed with glviewer window in focus
void GLViewer::keyPressEvent(QKeyEvent *event)
{
    // Checks for when the Shift key is held down
    if (event->modifiers() & Qt::ShiftModifier)
    {
        m_nshift = 1;
    }
    else
    {
    }
}

// Checks for when a key is released
void GLViewer::keyReleaseEvent(QKeyEvent *event)
{
    m_nshift = 0;    // Stores when the Shift key is released
}

void GLViewer::paintGL()
{
    // typedefs used to make cloud point type easier to define
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    // Create normal point cloud
    PointCloudT* meshCloud = new PointCloudT;

    // Transfer pointcloud2 data to point cloud data
    pcl::fromPCLPointCloud2(m_PMmesh.cloud, *meshCloud);

    QSize viewport_size = size();    // View window height/width size

    // Activates GL window; necessary for any rendering to occur
    glViewport(0, 0, viewport_size.width(), viewport_size.height());

    // Sets camera transformations
    glMatrixMode(GL_PROJECTION);

    // Clears buffers for drawing
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);    // Sets object transformations
    glLoadIdentity();              // Resets object transformations

    glTranslatef(m_lfxTrans, m_lfyTrans, m_lfzTrans);  // Translates object
    glRotatef(m_nxRot / 16.0, 1.0, 0.0, 0.0);          // Rotates object
    glRotatef(m_nyRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(m_nzRot / 16.0, 0.0, 0.0, 1.0);

    // Gets number of polygons in polygon mesh
    size_t nPolygons = m_PMmesh.polygons.size();

    // Draws polygons as GL_TRIANGLES
    for(size_t i=0; i<nPolygons; ++i)
    {
        int v1, v2, v3;                         // Polygon vertices map
        v1 = m_PMmesh.polygons[i].vertices[0];
        v2 = m_PMmesh.polygons[i].vertices[1];
        v3 = m_PMmesh.polygons[i].vertices[2];
        PointT p1 = meshCloud->points[v1];      // Cloud points for vertices
        PointT p2 = meshCloud->points[v2];
        PointT p3 = meshCloud->points[v3];

        // Average color for all three vertices        
        float rAvg = float(int(p1.r)+int(p2.r)+int(p3.r))/3.0f/255.0f;
        float gAvg = float(int(p1.g)+int(p2.g)+int(p3.g))/3.0f/255.0f;
        float bAvg = float(int(p1.b)+int(p2.b)+int(p3.b))/3.0f/255.0f;
        float aAvg = float(int(p1.a)+int(p2.a)+int(p3.a))/3.0f/255.0f;

        // Sets triangle color
        glColor4f(rAvg, gAvg, bAvg, aAvg);

        // Draws triangle
        glBegin(GL_TRIANGLES);
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glEnd();    
    }

    delete meshCloud;    // Cleans up memory
}

