cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (GLTest)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package (Qt5Widgets)
find_package (Qt5Core)
find_package (Qt5Gui)
find_package (PCL 1.7.2 REQUIRED)

include_directories (${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5GUI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${Qt5Widgets_DEFINITIONS} ${Qt5Core_DEFINITIONS} ${QtGUI_DEFINITIONS})

set  (project_SOURCES main.cpp glviewer.cpp mainwindow.cpp)
set  (project_HEADERS glviewer.h mainwindow.h)
set  (project_FORMS   mainwindow.ui)

#QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
#QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         ("${QT_USE_FILE}")
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (GLTest ${project_SOURCES}
                         ${project_FORMS_HEADERS}
                         ${project_HEADERS_MOC})
TARGET_LINK_LIBRARIES (GLTest Qt5::Widgets Qt5::Core Qt5::Gui ${PCL_LIBRARIES})

