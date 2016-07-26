#include "glviewer.h"
#include <QMouseEvent>
#include <QCoreApplication>
#include <QGLShaderProgram>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include "glut.h"

using  std::vector;
using  std::cout;
using  std::string;
using  std::allocator;
using  pcl::Vertices;

GLViewer::GLViewer(QWidget *parent) :
    QGLWidget(parent)
{
    //Setup OpenGL Widget
    m_nxRot = 0;      // Total rotation of camera about the x-axis
    m_nyRot = 0;      // Total rotation of camera about the y-axis
    m_nzRot = 0;      // Total rotation of camera about the z-axis
    m_lfDx = 0.01;    // Total camera translation change in the x-direction
    m_lfDy = 0.01;    // Total camera translation change in the y-direction
    m_lfDz = 0.01;    // Total camera translation change in the z-direction
    m_lfxLOS =  0.0;  // Camera line of sight x-direction initialization
    m_lfyLOS =  0.0;  // Camera line of sight y-direction initialization
    m_lfzLOS = -1.0;  // Camera line of sight z-direction initialization
    m_lfxUp =  0.0;   // Camera line of sight x-direction initialization
    m_lfyUp =  1.0;   // Camera line of sight y-direction initialization
    m_lfzUp =  0.0;   // Camera line of sight z-direction initialization
    m_lfxTrans = 0;   // Total camera translation change in the x-direction
    m_lfyTrans = 0;   // Total camera translation change in the y-direction
    m_lfzTrans = 0;   // Total camera translation change in the z-direction
    m_bshift = false; // Whether the Shift key is held down
    m_bPoint = false; // Whether to grab point info or not

    m_lfXMin = 1e12;  // Minimum x-value of objects;
    m_lfYMin = 1e12;  // Minimum y-value of objects;
    m_lfZMin = 1e12;  // Minimum z-value of objects;
    m_lfXMax = -1e12; // Maximum x-value of objects;
    m_lfYMax = -1e12; // Maximum y-value of objects;
    m_lfZMax = -1e12; // Maximum z-value of objects;
    m_lfXAvg = 0.0;   // Average x-value of objects;
    m_lfYAvg = 0.0;   // Average y-value of objects;
    m_lfZAvg = 0.0;   // Average z-value of objects;
    m_lfXTot = 0.0;   // Total x-value of object points;
    m_lfYTot = 0.0;   // Total y-value of object points;
    m_lfZTot = 0.0;   // Total z-value of object points;
    m_nPTot = 0;      // Total number of object points;

    m_bDrawAxis = true; // Whether to draw the x, y, z axis

    // MIP testing only
    m_bCKey = false;
    m_bHKey = false;

    // Refresh rendering after camera position/rotation changes
    setAutoBufferSwap(true);

    // Makes sure glviewer is in focus for keypress events
    setFocusPolicy(Qt::StrongFocus);
}

// The main function to pass in point cloud polygon mesh data for the viewer
void GLViewer::showPolygonMesh(pcl::PolygonMesh mesh, QString meshName){
    try
    {
        checkMeshNameExists(meshName);
    }
    catch(GLViewer::NameExists)
    {
        cout << "\033[1;31mException: " << meshName.toUtf8().constData() 
             << " already exists." << "\033[0m" << endl;
        return;
    }
    getDiffs(mesh);     // Determining object size for translation steps
    m_vQSMeshNames.push_back(meshName);
    m_vbShowMeshes.push_back(true);
    addMesh(mesh);      // Adds mesh to m_VTriangles
    initializeGL();     // Resets GL values for rendering
    paintGL();          // Draws the meshes and clouds in the glviewer
}

// Shows polygon mesh that was previously hidden
void GLViewer::showPolygonMesh(QString meshName){
    try
    {
        checkMeshNameMissing(meshName, true);
    }
    catch(GLViewer::NameMissing)
    {
        cout << "\033[1;31mException: " << meshName.toUtf8().constData() 
             << " does not exist." << "\033[0m" << endl;
        return;
    }
    initializeGL();     // Resets GL values for rendering
    paintGL();          // Draws the meshes and clouds in the glviewer
}

// Hides polygon mesh that was previously seen
void GLViewer::hidePolygonMesh(QString meshName){
    try
    {
        checkMeshNameMissing(meshName, false);
    }
    catch(GLViewer::NameMissing)
    {
        cout << "\033[1;31mException: " << meshName.toUtf8().constData() 
             << " does not exist." << "\033[0m" << endl;
        return;
    }
    initializeGL();     // Resets GL values for rendering
    paintGL();          // Draws the meshes and clouds in the glviewer
}

void GLViewer::showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
                         QString cloudName)
{
    try
    {
        checkCloudNameExists(cloudName);
    }
    catch(GLViewer::NameExists)
    {
        cout << "\033[1;31mException: " << cloudName.toUtf8().constData() 
             << " already exists." << "\033[0m" << endl;
        return;
    }
    getDiffs(cloud);     // Determining object size for translation steps
    m_vQSCloudNames.push_back(cloudName);
    m_vbShowClouds.push_back(true);
    addCloud(cloud);      // Adds mesh to m_VTriangles
    initializeGL();     // Resets GL values for rendering
    paintGL();          // Draws the meshes and clouds in the glviewer
}

void GLViewer::showCloud(QString cloudName)
{
    try
    {
        checkCloudNameMissing(cloudName, true);
    }
    catch(GLViewer::NameMissing)
    {
        cout << "\033[1;31mException: " << cloudName.toUtf8().constData() 
             << " does not exist." << "\033[0m" << endl;
        return;
    }
    initializeGL();     // Resets GL values for rendering
    paintGL();          // Draws the meshes and clouds in the glviewer
}

void GLViewer::hideCloud(QString cloudName)
{
    try
    {
        checkCloudNameMissing(cloudName, false);
    }
    catch(GLViewer::NameMissing)
    {
        cout << "\033[1;31mException: " << cloudName.toUtf8().constData() 
             << " does not exist." << "\033[0m" << endl;
        return;
    }
    initializeGL();     // Resets GL values for rendering
    paintGL();          // Draws the meshes and clouds in the glviewer
}

// Throw an exception if the name already exists
void GLViewer::checkMeshNameExists(QString meshName)
{
    for(size_t i=0; i<m_vQSMeshNames.size(); ++i)
    {
        if(m_vQSMeshNames[i] == meshName)
        {
            throw NameExists();
        }
    }
}

// Throw an exception if the name is missing to show/hide this mesh
//   also includes a boolean for whether to show/hide mesh to reduce operations
void GLViewer::checkMeshNameMissing(QString meshName, bool bShow)
{
    bool bFound = false;
    for(size_t i=0; i<m_vQSMeshNames.size(); ++i)
    {
        if(m_vQSMeshNames[i] == meshName)
        {
            bFound = true;
            bool bCheck = true;
            try
            {
                bCheck = checkShowMeshes(i);
                bCheck = true;
            }
            catch(GLViewer::SmallDimension)
            {
                cout << "\033[1;31mException: Mesh[" << i << "] does not exist" 
                 << "\033[0m" << endl;
                bCheck = false;
            }
            if(bCheck)
            {
                cout << "show = " << bShow << " for " 
                     << meshName.toUtf8().constData() << endl;
                m_vbShowMeshes[i] = bShow;
            }
        }
    }
    if(!bFound)
    {
        throw NameMissing();
    }
}

// Throw an exception if the name already exists
void GLViewer::checkCloudNameExists(QString cloudName)
{
    for(size_t i=0; i<m_vQSCloudNames.size(); ++i)
    {
        if(m_vQSCloudNames[i] == cloudName)
        {
            throw NameExists();
        }
    }
}

// Throw an exception if the name is missing to show/hide this cloud
//   also includes a boolean for whether to show/hide cloud to reduce operations
void GLViewer::checkCloudNameMissing(QString cloudName, bool bShow)
{
    bool bFound = false;
    for(size_t i=0; i<m_vQSCloudNames.size(); ++i)
    { 
        if(m_vQSCloudNames[i] == cloudName)
        {
            bFound = true;
            bool bCheck = true;
            try
            {
                bCheck = checkShowClouds(i);
                bCheck = true;
            }
            catch(GLViewer::SmallDimension)
            {
                cout << "\033[1;31mException: Cloud[" << i << "] does not exist"
                 << "\033[0m" << endl;
                bCheck = false;
            }
            if(bCheck)
            {
                cout << "show = " << bShow << " for " 
                     << cloudName.toUtf8().constData() << endl;
                m_vbShowClouds[i] = bShow;
            }
        }
    }
    if(!bFound)
    {
        throw NameMissing();
    }
}

// Determine the best translation changes based on the object size
void GLViewer::getDiffs(pcl::PolygonMesh mesh)
{
    // typedefs used to make cloud point type easier to define
    // typedef pcl::PointXYZRGBA PointT;
    // typedef pcl::PointCloud<PointT> PointCloudT;

    // Create normal point cloud
    PointCloudT* meshCloud = new PointCloudT;

    // Transfer pointcloud2 data to point cloud data
    pcl::fromPCLPointCloud2(mesh.cloud, *meshCloud);

    for(size_t i=0; i<meshCloud->points.size(); ++i)
    {        
        PointT p1 = meshCloud->points[i];
        if(p1.x < m_lfXMin)
        {
            m_lfXMin = p1.x;
        }
        if(m_lfXMax < p1.x)
        {
            m_lfXMax = p1.x;
        }
        if(p1.y < m_lfYMin)
        {
            m_lfYMin = p1.y;
        }
        if(m_lfYMax < p1.y)
        {
            m_lfYMax = p1.y;
        }
        if(p1.z < m_lfZMin)
        {
            m_lfZMin = p1.z;
        }
        if(m_lfZMax < p1.z)
        {
            m_lfZMax = p1.z;
        }
        m_lfXTot += p1.x;
        m_lfYTot += p1.y;
        m_lfZTot += p1.z;
    }
    // Determined through trial and error for visualization translation
    m_lfDx = (m_lfXMax-m_lfXMin)/250.0;
    m_lfDy = (m_lfYMax-m_lfYMin)/250.0;
    m_lfDz = (m_lfZMax-m_lfZMin)/20.0;

    m_nPTot += int(meshCloud->points.size());
    m_lfXAvg = m_lfXTot/double(m_nPTot);
    m_lfYAvg = m_lfYTot/double(m_nPTot);
    m_lfZAvg = m_lfZTot/double(m_nPTot);

    // Finds the maximum distance between object sides
    double maxDist = m_lfXMax - m_lfXMin;
    if(maxDist < m_lfYMax - m_lfYMin)
    {
        maxDist = m_lfYMax - m_lfYMin;
    }
    if(maxDist < m_lfZMax - m_lfZMin)
    {
        maxDist = m_lfZMax - m_lfZMin;
    }

    // Initial translation of the camera to place object in center view    
    m_lfxTrans = -m_lfXAvg;
    m_lfyTrans = -m_lfYAvg;
    m_lfzTrans = -fabs(maxDist);

    delete meshCloud;    // Cleans up memory
}

// Determine the best translation changes based on the object size
void GLViewer::getDiffs(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    for(size_t i=0; i<cloud->points.size(); ++i)
    {        
        PointT p1 = cloud->points[i];
        if(p1.x < m_lfXMin)
        {
            m_lfXMin = p1.x;
        }
        if(m_lfXMax < p1.x)
        {
            m_lfXMax = p1.x;
        }
        if(p1.y < m_lfYMin)
        {
            m_lfYMin = p1.y;
        }
        if(m_lfYMax < p1.y)
        {
            m_lfYMax = p1.y;
        }
        if(p1.z < m_lfZMin)
        {
            m_lfZMin = p1.z;
        }
        if(m_lfZMax < p1.z)
        {
            m_lfZMax = p1.z;
        }
        m_lfXTot += p1.x;
        m_lfYTot += p1.y;
        m_lfZTot += p1.z;
    }    
    // Determined through trial and error for visualization translation
    m_lfDx = (m_lfXMax-m_lfXMin)/250.0;
    m_lfDy = (m_lfYMax-m_lfYMin)/250.0;
    m_lfDz = (m_lfZMax-m_lfZMin)/20.0;

    m_nPTot += int(cloud->points.size());
    m_lfXAvg = m_lfXTot/double(m_nPTot);
    m_lfYAvg = m_lfYTot/double(m_nPTot);
    m_lfZAvg = m_lfZTot/double(m_nPTot);

    // Finds the maximum distance between object sides
    double maxDist = m_lfXMax - m_lfXMin;
    if(maxDist < m_lfYMax - m_lfYMin)
    {
        maxDist = m_lfYMax - m_lfYMin;
    }
    if(maxDist < m_lfZMax - m_lfZMin)
    {
        maxDist = m_lfZMax - m_lfZMin;
    }

    // Initial translation of the camera to place object in center view    
    m_lfxTrans = -m_lfXAvg;
    m_lfyTrans = -m_lfYAvg;
    m_lfzTrans = -fabs(maxDist);
}

// Adds a cloud to m_vClouds
void GLViewer::addCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourceCloud(cloud);
    m_vClouds.push_back(sourceCloud);
} 

// Adds a mesh to m_vMeshes
void GLViewer::addMesh(pcl::PolygonMesh mesh)
{
    // Create normal point cloud
    PointCloudT::Ptr meshCloud(new PointCloudT);

    // Transfer pointcloud2 data to point cloud data
    pcl::fromPCLPointCloud2(mesh.cloud, *meshCloud);

    m_vMeshes.push_back(mesh);
    m_vbShowClouds.push_back(false); // Already showing mesh. Do not show cloud
    showCloud(meshCloud, "meshCloud1"); // Add cloud for future use
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
//     glDisable(GL_TEXTURE_3D);
    glShadeModel(GL_FLAT);
    
    // glCullFace(GL_FRONT);
    glMatrixMode(GL_PROJECTION); // Matrix mode for camera coordinates
    glLoadIdentity();            // Resets camera transformation matrix

    // Sets camera fov, aspect ratio, and near/far values
    gluPerspective(45.0, 1.0, 0.01f, 350.0f); 
//     glEnable(GL_MULTISAMPLE);    // Antialiasing

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
    if(m_bPoint && event->buttons() & Qt::LeftButton)
    {
        m_vlfPoint = getPointCoords(event->x(), event->y());
        getPointIndex();
        cout << "Point: (" << m_vlfPoint[0] << ", " << m_vlfPoint[1]
             << ", " << m_vlfPoint[2] << ")" << endl;
        cout << "Index for " << m_sIndex << ": " << m_nIndex << endl;
        if(m_nPolygonIndex > -1)
        {
            cout << m_sIndex << " polygon: " << m_nPolygonIndex << endl;
        }
    }
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
    if(!m_bshift && !m_bPoint && event->buttons() & Qt::LeftButton) 
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
    else if (m_bshift && event->buttons() & Qt::LeftButton) 
    {
        setXTranslation(m_lfxTrans + m_lfDx * dx);
        setYTranslation(m_lfyTrans - m_lfDy * dy);
    }
    
    m_QPlastPos = event->pos();    // Saves mouse position after movement ends
}

// Track when a key is pressed with glviewer window in focus
void GLViewer::keyPressEvent(QKeyEvent *event)
{
    // cout << event->text().toUtf8().constData() << endl;
    string sKey = event->text().toUtf8().constData();
    // MIP Testing purposes with keys
    // Checks for when the "P" or "p" key is pressed
    if (sKey == "p" || sKey == "P")
    {
        pointPicker();
    }
    if (sKey == "c" || sKey == "C")
    {
        if(m_bCKey)
        {
           m_bCKey = false;
           hideCloud("meshCloud1");
           showPolygonMesh("mesh1");
           update();
        }
        else
        {
           m_bCKey = true;
           hidePolygonMesh("mesh1");
           showCloud("meshCloud1");
           update();
        }
    }
    if (sKey == "h" || sKey == "H")
    {
        if(m_bHKey)
        {
           m_bHKey = false;
        }
        else
        {
           m_bHKey = true;
        }
    }
    // Checks for when the Shift key is held down
    if (event->modifiers() & Qt::ShiftModifier)
    {
        m_bshift = 1;
    }
    else
    {
    }
}

// Checks for when a key is released
void GLViewer::keyReleaseEvent(QKeyEvent *event)
{
    m_bshift = 0;    // Stores when the Shift key is released
}

vector<double> GLViewer::normalizeUP()
{
   vector<double> normalizedUp;
   // UP/||UP||
   normalizedUp.push_back(m_lfxUp/sqrt(m_lfxUp * m_lfxUp + m_lfyUp * m_lfyUp + 
                                       m_lfzUp * m_lfzUp));
   normalizedUp.push_back(m_lfyUp/sqrt(m_lfxUp * m_lfxUp + m_lfyUp * m_lfyUp + 
                                       m_lfzUp * m_lfzUp));
   normalizedUp.push_back(m_lfzUp/sqrt(m_lfxUp * m_lfxUp + m_lfyUp * m_lfyUp + 
                                       m_lfzUp * m_lfzUp));
   return normalizedUp;
}

vector<double> GLViewer::normalizeLOS()
{
   // F/||F||
   vector<double> normalizedLOS;
   normalizedLOS.push_back(m_lfxLOS/sqrt(m_lfxLOS * m_lfxLOS + 
                                         m_lfyLOS * m_lfyLOS +
                                         m_lfzLOS * m_lfzLOS));
   normalizedLOS.push_back(m_lfyLOS/sqrt(m_lfxLOS * m_lfxLOS + 
                                         m_lfyLOS * m_lfyLOS +
                                         m_lfzLOS * m_lfzLOS));
   normalizedLOS.push_back(m_lfzLOS/sqrt(m_lfxLOS * m_lfxLOS + 
                                         m_lfyLOS * m_lfyLOS +
                                         m_lfzLOS * m_lfzLOS));
   return normalizedLOS;
}

vector<double> GLViewer::normalizeCross(std::vector<double> fVec, 
                                        std::vector<double> upVec)
{
    vector<double>normalizedCross;
    // i = j x k - k x j
    normalizedCross.push_back(fVec[1] * upVec[2] - fVec[2] * upVec[1]);
    // j = k x i - i x k
    normalizedCross.push_back(fVec[2] * upVec[0] - fVec[0] * upVec[2]);
    // k = i x j - j x i
    normalizedCross.push_back(fVec[0] * upVec[1] - fVec[1] * upVec[0]);
    return normalizedCross;
}

// Draws triangles
void GLViewer::drawGLTriangles(vector<pcl::PolygonMesh>::iterator it)
{
    PointCloudT *meshCloud = new PointCloudT;
    pcl::fromPCLPointCloud2(it->cloud, *meshCloud);
    size_t nPolygons = it->polygons.size();
    for(size_t i=0; i < nPolygons; ++i)
    {
        int v1, v2, v3;                         // Polygon vertices map
        v1 = it->polygons[i].vertices[0];
        v2 = it->polygons[i].vertices[1];
        v3 = it->polygons[i].vertices[2];
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
    delete meshCloud; // Clean up memory
}

// Throw an exception if the cloud does not exist
bool GLViewer::checkShowClouds(size_t i)
{
    if(m_vbShowClouds.size() <= i)
    {
        throw SmallDimension();
    }
    else
    {
        return m_vbShowClouds[i];
    }
    return false;
}

// Throw an exception if the mesh does not exist
bool GLViewer::checkShowMeshes(size_t i)
{
    if(m_vbShowMeshes.size() <= i)
    {
        throw SmallDimension();
    }
    else
    {
        return m_vbShowMeshes[i];
    }
    return false;
}

// Renders the scene
void GLViewer::paintGL()
{
    QSize viewport_size = size();    // View window height/width size

    // Activates GL window; necessary for any rendering to occur
    glViewport(0, 0, viewport_size.width(), viewport_size.height());

    // Sets camera transformations
    glMatrixMode(GL_PROJECTION);

    // Clears buffers for drawing
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);    // Sets camera transformations
    glLoadIdentity();              // Resets object transformations

    // Sets camera fov, aspect ratio, and near/far values
    gluPerspective(45.0, 1.0, 0.01f, 350.0);
    
    vector<double> u1Vec = normalizeUP();
    vector<double> f1Vec = normalizeLOS();
    vector<double> s1Vec = normalizeCross(f1Vec, u1Vec);
    float m1[16] = { s1Vec[0],  s1Vec[1],  s1Vec[2], 0,
                     u1Vec[0],  u1Vec[1],  u1Vec[2], 0,
                    -f1Vec[0], -f1Vec[1], -f1Vec[2], 0,
                            0,         0,         0, 1};
    glMultMatrixf(m1);

    double xDir = -m_lfxTrans - m_lfXAvg;             // Treats object as if
    double yDir = -m_lfyTrans - m_lfYAvg;             //   it is at the origin
    double zDir = -m_lfzTrans - m_lfZAvg;             //   and rotates around it

    glTranslated(-xDir, -yDir, -zDir);
    
    glRotatef(m_nxRot / 16.0, 1.0, 0.0, 0.0);          // Rotates camera
    glRotatef(m_nyRot / 16.0, 0.0, 1.0, 0.0);          //   around origin
    glRotatef(m_nzRot / 16.0, 0.0, 0.0, 1.0);

    glTranslated(-m_lfXAvg, -m_lfYAvg, -m_lfZAvg);     // Move the camera
                                                       //   relative to object
    // Origin axis
    if(m_bDrawAxis)
    {
        xDir = m_lfXMax - m_lfXMin;     // Shrink origin axis based on object
        if(m_lfYMax - m_lfYMin < xDir)  //   size
        {
            xDir = m_lfYMax - m_lfYMin;
        }
        if(m_lfZMax - m_lfZMin < xDir)
        {
            xDir = m_lfZMax - m_lfZMin;
        }
        glBegin(GL_LINES);
        glColor4f(1.0, 0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(xDir, 0.0, 0.0);
        glEnd();
        glBegin(GL_LINES);
        glColor4f(0.0, 1.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, xDir, 0.0);
        glEnd();
        glBegin(GL_LINES);
        glColor4f(0.0, 0.0, 1.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, xDir);
        glEnd();
    }

    size_t i = 0;

    // Draws m_vMeshes as GL_TRIANGLES
    for(vector<pcl::PolygonMesh>::iterator it = m_vMeshes.begin(); 
        it != m_vMeshes.end(); ++it)
    {
        bool bShow = false;
        try
        {
            bShow = checkShowMeshes(i);
        }
        catch(GLViewer::SmallDimension)
        {
            cout << "\033[1;31mException: Mesh[" << i << "] does not exist" 
                 << "\033[0m" << endl;
        }
        // Check whether current mesh is hidden or shown
        if(bShow)
        {
            // Draw the triangles for shown meshes
            drawGLTriangles(it);
        }
        ++i;
    }

    // Educated guess for spacing between points
    vector<double> oneRad;
    for(size_t i=0; i < m_vClouds.size(); ++i)
    {
        bool bShow = false;
        try
        {
            bShow = checkShowClouds(i);
        }
        catch(GLViewer::SmallDimension)
        {
            cout << "\033[1;31mException: Cloud[" << i << "] does not exist" 
                 << "\033[0m" << endl;
        }
        // Determine if the cloud is being shown or hidden
        if(bShow)
        {
            // Set a large distance between points to shrink
            oneRad.push_back(1e12);     
            for(int j=1; j < m_vClouds[i]->points.size(); ++j)
            {
                // Defines the current cloud point
                PointT p1;
                p1 = m_vClouds[i]->points[j];
                // Defines the first cloud point
                PointT p2;
                p2 = m_vClouds[i]->points[0];
                // The radius between the first cloud point and current point
                double thisRad = sqrt((p2.x - p1.x) * (p2.x - p1.x) + 
                                      (p2.y - p1.y) * (p2.y - p1.y) +
                                      (p2.z - p1.z) * (p2.z - p1.z));

                // The spacing between points is the minimum distance
                if(thisRad < oneRad[i])
                {
                    oneRad[i] = thisRad;
                }                                
            }
            // Cloud was too sparse, so multiplied by 10.0
            oneRad[i] = oneRad[i] * 10.0;
        }
    }

    // Draw point clouds
    for(size_t i=0; i < m_vClouds.size(); ++i)
    {
        bool bShow = false;
        try
        {
            bShow = checkShowClouds(i);
        }
        catch(GLViewer::SmallDimension)
        {
            cout << "\033[1;31mException: Cloud[" << i << "] does not exist" 
                 << "\033[0m" << endl;
        }
        if(bShow)
        {            
            glBegin(GL_LINES);
            for(int j=0; j < m_vClouds[i]->points.size(); ++j)
            {
                PointT p1;
                p1 = m_vClouds[i]->points[j];
                // Sets point color
                glColor4ub(p1.r, p1.g, p1.b, p1.a);
                // Draws the point
                glVertex3d(p1.x, p1.y, p1.z);
                // Draws a vector from the point
                glVertex3d(p1.x+oneRad[i], p1.y+oneRad[i], p1.z+oneRad[i]);
            }
            glEnd();
        }
    }

    // MIP for testing purposes
//     vector<double> cameraPos = getCameraLocation();
//     cout << "Camera Position: ";
//     cout << cameraPos[0] << ", " << cameraPos[1] << ", " << cameraPos[2] 
//          << endl;
//     vector<double> cameraLOS = getCameraLineOfSight();
//     cout << "Camera Line of Sight: ";
//     cout << cameraLOS[0] << ", " << cameraLOS[1] << ", " << cameraLOS[2] 
//          << endl;
//     vector<double> cameraUp = getCameraUp();
//     cout << "Camera Up Vector: ";
//     cout << cameraUp[0] << ", " << cameraUp[1] << ", " << cameraUp[2] 
//          << endl;

}

// Gets the camera location (x, y, z) as 
//                          (cameraPos[0], cameraPos[1], cameraPos[2])
vector<double> GLViewer::getCameraLocation()
{
    int viewport[4];           // Stores the viewing window size
    double matModelView[16];   // Stores the Modelview matrix
    double matProjection[16];  // Stores the Projection matrix
    vector<double> cameraPos;  // Stores the camera position
    cameraPos.push_back(0.0);  // Initialize the camera coordinates
    cameraPos.push_back(0.0);
    cameraPos.push_back(0.0);

    // get matrices and viewport:
    glGetDoublev( GL_MODELVIEW_MATRIX, matModelView ); 
    glGetDoublev( GL_PROJECTION_MATRIX, matProjection ); 
    glGetIntegerv( GL_VIEWPORT, viewport );  // Gets viewport dimensions
    
    // Unprojects the camera plane to determine its location in 3D space
    gluUnProject( (viewport[2]-viewport[0])/2 , (viewport[3]-viewport[1])/2, 
                  0.0, matModelView, matProjection, viewport,  
                  &cameraPos[0],&cameraPos[1],&cameraPos[2]);
    return cameraPos;
}

// Sets the camera location if it moved while adjusting point cloud data
void GLViewer::setCameraLocation(vector<double> vecLoc)
{
    // Only set the camera location if the input vector is well defined
    try
    {
        checkCameraDim(vecLoc.size());
        m_lfxTrans = vecLoc[0];
        m_lfyTrans = vecLoc[1];
        m_lfzTrans = vecLoc[2];
    }
    // If the camera location is defined in less than 3D space...
    catch(GLViewer::SmallDimension)
    {
        cout << "\033[1;31mException: Camera location dimension is too small." 
             << endl << 
                "           It should have 3 elements, not " << vecLoc.size() 
             << "\033[0m" << endl;
    }
    // If the camera location is defined in greater than 3D space...
    catch(GLViewer::LargeDimension)
    {
        cout << "\033[1;31mException: Camera location dimension is too large." 
             << endl << 
                "           It should have 3 elements, not " << vecLoc.size() 
             << "\033[0m" << endl;
    }
}

// Gets the camera line of sight in x, y, z coordinates
vector<double> GLViewer::getCameraLineOfSight()
{
    int viewport[4];           // Stores the viewing window size
    double matModelView[16];   // Stores the Modelview matrix
    double matProjection[16];  // Stores the Projection matrix
    vector<double> cameraPos;  // Stores the camera position
    cameraPos.push_back(0.0);  // Initialize camera coordinates
    cameraPos.push_back(0.0);
    cameraPos.push_back(0.0);
    vector<double> cameraLOS;  // Stores the camera line of sight
    cameraLOS.push_back(0.0);  // Initialize camera line of sight coordinates
    cameraLOS.push_back(0.0);
    cameraLOS.push_back(0.0);

    // get matrices and viewport for camera position:
    glGetDoublev( GL_MODELVIEW_MATRIX, matModelView ); 
    glGetDoublev( GL_PROJECTION_MATRIX, matProjection ); 
    glGetIntegerv( GL_VIEWPORT, viewport );  // Gets viewport dimensions
    gluUnProject( (viewport[2]-viewport[0])/2 , (viewport[3]-viewport[1])/2, 
                  0.0, matModelView, matProjection, viewport,  
                  &cameraPos[0],&cameraPos[1],&cameraPos[2]);

    // get matrices and viewport for camera viewing plane:
    glGetDoublev( GL_MODELVIEW_MATRIX, matModelView ); 
    glGetDoublev( GL_PROJECTION_MATRIX, matProjection ); 
    glGetIntegerv( GL_VIEWPORT, viewport );  // Gets viewport dimensions 
    gluUnProject( (viewport[2]-viewport[0])/2 , (viewport[3]-viewport[1])/2, 
                  1.0, matModelView, matProjection, viewport,  
                  &cameraLOS[0],&cameraLOS[1],&cameraLOS[2]);

    // get the vector from the camera to viewing plane
    cameraLOS[0] = cameraLOS[0] - cameraPos[0];
    cameraLOS[1] = cameraLOS[1] - cameraPos[1];
    cameraLOS[2] = cameraLOS[2] - cameraPos[2];

    // normalize the vector
    double denom = sqrt(cameraLOS[0] * cameraLOS[0] +
                        cameraLOS[1] * cameraLOS[1] +
                        cameraLOS[2] * cameraLOS[2]);
    cameraLOS[0] = cameraLOS[0]/denom;
    cameraLOS[1] = cameraLOS[1]/denom;
    cameraLOS[2] = cameraLOS[2]/denom;
    return cameraLOS;
}

// Sets the camera line of sight in x, y, z coordinates
void GLViewer::setCameraLineOfSight(std::vector<double> vecLOS)
{
    // Only set the camera line of sight if the input vector is well defined
    try
    {
        checkCameraDim(vecLOS.size());
        m_lfxLOS = vecLOS[0];
        m_lfyLOS = vecLOS[1];
        m_lfzLOS = vecLOS[2];
    }
    // If the camera line of sight is defined in less than 3D space...
    catch(GLViewer::SmallDimension)
    {
        cout << "\033[1;31mException: Camera line of sight dimension is "
             << "too small." << endl << 
                "           It should have 3 elements, not " << vecLOS.size() 
             << "\033[0m" << endl;
    }
    // If the camera line of sight is defined in greater than 3D space...
    catch(GLViewer::LargeDimension)
    {
        cout << "\033[1;31mException: Camera line of sight dimension is "
             << "too large." << endl << 
                "           It should have 3 elements, not " << vecLOS.size() 
             << "\033[0m" << endl;
    }
}

// Gets the camera's up vector for camera orientation
vector<double> GLViewer::getCameraUp()
{
    float pi = 3.14159265358979;
    float thetax = pi/180.0f * float(-m_nxRot) / 16.0f;    // convert to radians
    float thetay = pi/180.0f * float(-m_nyRot) / 16.0f;
    float thetaz = pi/180.0f * float(-m_nzRot) / 16.0f;
    float mx[16] = {1.0,          0.0,          0.0, 0.0,  // x-rotation matrix
                    0.0,  cos(thetax), -sin(thetax), 0.0,
                    0.0,  sin(thetax),  cos(thetax), 0.0,
                    0.0,          0.0,          0.0, 1.0};
    float my[16] = { cos(thetay), 0.0, sin(thetay), 0.0,  // y-rotation matrix
                             0.0, 1.0,         0.0, 0.0,
                    -sin(thetay), 0.0, cos(thetay), 0.0,
                             0.0, 0.0,         0.0, 1.0};
    float mz[16] = {cos(thetaz), -sin(thetaz), 0.0, 0.0,  // z-rotation matrix
                    sin(thetaz),  cos(thetaz), 0.0, 0.0,
                            0.0,          0.0, 1.0, 0.0,
                            0.0,          0.0, 0.0, 1.0};

    // rotate about the x-axis
    float xUp = m_lfxUp * mx[0] + m_lfyUp * mx[1] + m_lfzUp * mx[2];
    float yUp = m_lfxUp * mx[4] + m_lfyUp * mx[5] + m_lfzUp * mx[6];
    float zUp = m_lfxUp * mx[8] + m_lfyUp * mx[9] + m_lfzUp * mx[10];

    // Store values to maintain consistency
    float xUp1 = xUp;
    float yUp1 = yUp;
    float zUp1 = zUp;
    
    // rotate about the y-axis
    xUp = xUp1 * my[0] + yUp1 * my[1] + zUp1 * my[2];
    yUp = xUp1 * my[4] + yUp1 * my[5] + zUp1 * my[6];
    zUp = xUp1 * my[8] + yUp1 * my[9] + zUp1 * my[10];
    // Store values to maintain consistency
    xUp1 = xUp;
    yUp1 = yUp;
    zUp1 = zUp;
    
    // rotate about the z-axis
    xUp = xUp1 * mz[0] + yUp1 * mz[1] + zUp1 * mz[2];
    yUp = xUp1 * mz[4] + yUp1 * mz[5] + zUp1 * mz[6];
    zUp = xUp1 * mz[8] + yUp1 * mz[9] + zUp1 * mz[10];

    vector<double> cameraUp;  // Stores the camera up vector
    cameraUp.push_back(xUp);  // Initialize camera up vector
    cameraUp.push_back(yUp);
    cameraUp.push_back(zUp);
    return cameraUp;
}

// Sets the camera up vector in x, y, z coordinates
void GLViewer::setCameraUp(std::vector<double> vecUp)
{
    // Only set the camera up vector if the input vector is well defined
    try
    {
        checkCameraDim(vecUp.size());
        m_lfxUp = vecUp[0];
        m_lfyUp = vecUp[1];
        m_lfzUp = vecUp[2];
    }
    // If the camera up vector is defined in less than 3D space...
    catch(GLViewer::SmallDimension)
    {
        cout << "\033[1;31mException: Camera up vector dimension is "
             << "too small." << endl << 
                "           It should have 3 elements, not " << vecUp.size() 
             << "\033[0m" << endl;
    }
    // If the camera up vector is defined in greater than 3D space...
    catch(GLViewer::LargeDimension)
    {
        cout << "\033[1;31mException: Camera up vector dimension is "
             << "too large." << endl << 
                "           It should have 3 elements, not " << vecUp.size() 
             << "\033[0m" << endl;
    }
}

// Checks the dimension of the vector passed in
void GLViewer::checkCameraDim(size_t uDim)
{
    if(uDim < 3)
    {
        throw SmallDimension();    // throws exception for dimension too small
    }
    if(uDim > 3)
    {
        throw LargeDimension();    // throws exception for dimension too large
    }
}

// Toggles whether selecting a point provides index and coordinate information
void GLViewer::pointPicker()
{
    if(m_bPoint)
    {
        m_bPoint = false;
        cout << "We are not picking a point" << endl;
    }
    else
    {
        m_bPoint = true;
        cout << "We are picking a point" << endl;
    }
}

// Determines the 3D coordinates of the selected point
vector<double> GLViewer::getPointCoords(int mouseX, int mouseY)
{
    GLint viewport[4];
    double matModelView[16];   // Stores the Modelview matrix
    double matProjection[16];  // Stores the Projection matrix
    glGetDoublev( GL_MODELVIEW_MATRIX, matModelView ); 
    glGetDoublev( GL_PROJECTION_MATRIX, matProjection );

    vector<double> pointPos;
    pointPos.push_back(0.0);
    pointPos.push_back(0.0);
    pointPos.push_back(0.0);

    glGetIntegerv(GL_VIEWPORT, viewport);
    GLint width = viewport[2];
    GLint height = viewport[3];

    // Account for image plane y-axis being upside down
    mouseY = height - mouseY;

    GLfloat *pixels = new GLfloat[width * height];
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, pixels);
    gluUnProject( mouseX , mouseY, 
                  pixels[mouseY*width + mouseX], matModelView, matProjection, 
                  viewport, &pointPos[0],&pointPos[1],&pointPos[2]);
    return pointPos;
}

// Finds the cloud index and polygon it is attached to for point picking
void GLViewer::findIndex(int idx)
{
    int i = idx;
    PointCloudT *meshCloud = new PointCloudT;
    pcl::fromPCLPointCloud2(m_vMeshes[i].cloud, *meshCloud);
    size_t nPolygons = m_vMeshes[i].polygons.size();
    double dDist = 1e12;
    for(size_t j=0; j < nPolygons; ++j)
    {
        int v1, v2, v3;                         // Polygon vertices map
        v1 = m_vMeshes[i].polygons[j].vertices[0];
        v2 = m_vMeshes[i].polygons[j].vertices[1];
        v3 = m_vMeshes[i].polygons[j].vertices[2];
        PointT p1 = meshCloud->points[v1];      // Cloud points for vertices
        PointT p2 = meshCloud->points[v2];
        PointT p3 = meshCloud->points[v3];

        // Distance between point selected and cloud point
        double jDist = sqrt((m_vlfPoint[0] - p1.x) * (m_vlfPoint[0] - p1.x)
                         +  (m_vlfPoint[1] - p1.y) * (m_vlfPoint[1] - p1.y)
                         +  (m_vlfPoint[2] - p1.z) * (m_vlfPoint[2] - p1.z));
        if(jDist < dDist) // Find the closest cloud point to selected point
        {
           dDist = jDist;
           m_nIndex = v1;
           m_nPolygonIndex = j;
           m_sIndex = m_vQSMeshNames[i].toUtf8().constData();
        }
        jDist = sqrt((m_vlfPoint[0] - p2.x) * (m_vlfPoint[0] - p2.x)
                  +  (m_vlfPoint[1] - p2.y) * (m_vlfPoint[1] - p2.y)
                  +  (m_vlfPoint[2] - p2.z) * (m_vlfPoint[2] - p2.z));
        if(jDist < dDist)
        {
           dDist = jDist;
           m_nIndex = v2;
           m_nPolygonIndex = j;
           m_sIndex = m_vQSMeshNames[i].toUtf8().constData();
        }
        jDist = sqrt((m_vlfPoint[0] - p3.x) * (m_vlfPoint[0] - p3.x)
                  +  (m_vlfPoint[1] - p3.y) * (m_vlfPoint[1] - p3.y)
                  +  (m_vlfPoint[2] - p3.z) * (m_vlfPoint[2] - p3.z));
        if(jDist < dDist)
        {
           dDist = jDist;
           m_nIndex = v3;
           m_nPolygonIndex = j;
           m_sIndex = m_vQSMeshNames[i].toUtf8().constData();
        }
    }
    // MIP for testing
    PointT p1 = meshCloud->points[m_nIndex];
    cout << "Closest point: (" << p1.x << ", " << p1.y << ", " << p1.z 
         << ")" << endl;
    delete meshCloud;
}

// Determines the cloud index and polygon member for the selected point
void GLViewer::getPointIndex()
{
    m_nPolygonIndex = -1;  // Resets index information
    m_nIndex = -1;
    m_sIndex = "";
    for(size_t i=0; i < m_vMeshes.size(); ++i)
    {
        bool bShow = false;  // Make sure the mesh exists
        try
        {
            bShow = checkShowMeshes(i);
        }
        catch(GLViewer::SmallDimension)
        {
            cout << "\033[1;31mException: Mesh[" << i << "] does not exist" 
                 << "\033[0m" << endl;
        }
        if(bShow)
        {
            findIndex(i);
        }
    }
    for(size_t i=0; i < m_vClouds.size(); ++i)
    {
        bool bShow = false;  // Make sure the cloud exists
        try
        {
            bShow = checkShowClouds(i);
        }
        catch(GLViewer::SmallDimension)
        {
            cout << "\033[1;31mException: Cloud[" << i << "] does not exist" 
                 << "\033[0m" << endl;
        }
        if(bShow)
        {
            vector<double> oneRad;  // Find a good representation of the 
            oneRad.push_back(1e12); //   spacing between cloud points
			for(int j=1; j < m_vClouds[i]->points.size(); ++j)
            {
                PointT p1;
                p1 = m_vClouds[i]->points[j];
                PointT p2;
                p2 = m_vClouds[i]->points[0];

                // The distance between current points
                double thisRad = sqrt((p2.x - p1.x) * (p2.x - p1.x) + 
                                      (p2.y - p1.y) * (p2.y - p1.y) +
                                      (p2.z - p1.z) * (p2.z - p1.z));

                if(thisRad < oneRad[i])  // Get closest distance between points
                {
                    oneRad[i] = thisRad;
                }
            }                      
            oneRad[i] = oneRad[i] * 10.0; // Need cloud data to be visible
                                          //   so add scaling factor
            double dDist = 1e12;
            for(int j=1; j < m_vClouds[i]->points.size(); ++j)
            {
                PointT p1;
                p1 = m_vClouds[i]->points[j];
                PointT p2;
                p2.x = p1.x + oneRad[i];
                p2.y = p1.y + oneRad[i];
                p2.z = p1.z + oneRad[i];

                // how close is this cloud point to the point picked
                double jDist = 
                        sqrt((m_vlfPoint[0] - p1.x) * (m_vlfPoint[0] - p1.x)
                          +  (m_vlfPoint[1] - p1.y) * (m_vlfPoint[1] - p1.y)
                          +  (m_vlfPoint[2] - p1.z) * (m_vlfPoint[2] - p1.z));
                if(jDist < dDist)  // find closest cloud point to point picked
                {
                    dDist = jDist;
                    m_nIndex = j;
                    m_sIndex = m_vQSCloudNames[i].toUtf8().constData();
                }
                jDist = sqrt((m_vlfPoint[0] - p2.x) * (m_vlfPoint[0] - p2.x)
                          +  (m_vlfPoint[1] - p2.y) * (m_vlfPoint[1] - p2.y)
                          +  (m_vlfPoint[2] - p2.z) * (m_vlfPoint[2] - p2.z));
                if(jDist < dDist)
                {
                    dDist = jDist;
                    m_nIndex = j;
                    m_sIndex = m_vQSCloudNames[i].toUtf8().constData();
                }
            }
        }
    }
}

// Point picking coordinates for last selected point are returned
vector<double> GLViewer::getCloudPoint()
{
    return m_vlfPoint;
}

// Point picking cloud index for last selected point is returned 
int GLViewer::getCloudIndex()
{
    return m_nIndex;
}

// Point picking polygon if the point is part of a polygon
int GLViewer::getMeshPolygonIndex()
{
    return m_nPolygonIndex;
}

// Polygon mesh name or cloud name for the point picked
string GLViewer::getIndexName()
{
    return m_sIndex;
}
