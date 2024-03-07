cmake_minimum_required(VERSION 3.5)

project(Halcon VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(PROJECT_SOURCES
        main.cpp
        widget.h
        widget.cpp
        widget.ui
        workthread.h
        workthread.cpp
        mainwindow.h
        mainwindow.cpp
        mainwindow.ui
        mainwindow_2.h
        mainwindow_2.cpp
        mainwindow_2.ui
)

include_directories("$ENV{HALCONROOT}/include"
                    "$ENV{HALCONROOT}/include/halconcpp")
link_directories("$ENV{HALCONROOT}/lib/$ENV{HALCONARCH}")

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Halcon
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET Halcon APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(Halcon SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(Halcon
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(Halcon PRIVATE Qt${QT_VERSION_MAJOR}::Widgets halcon halconcpp)

set_target_properties(Halcon PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS Halcon
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(Halcon)
endif()
