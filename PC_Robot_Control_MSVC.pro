#-------------------------------------------------
#
# Project created by QtCreator 2024-08-06T10:26:07
#
#-------------------------------------------------

QT       += core gui
QT       += network
QT       += core
QT       += core gui network serialport
QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PC_Robot_Control_MSVC
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        caculatematrix.cpp \
        capture.cpp \
        connecttcp.cpp \
        datareceiver.cpp \
        main.cpp \
        mainwindow.cpp \
        serialport.cpp \
        udp.cpp \
        yrc1000micro_com.cpp \
        yrc1000micro_command.cpp

HEADERS += \
        caculatematrix.h \
        capture.h \
        connecttcp.h \
        datareceiver.h \
        mainwindow.h \
        serialport.h \
        udp.h \
        yrc1000micro_com.h \
        yrc1000micro_command.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#Add Realsense Camera Lib
INCLUDEPATH += 'C:/Program Files (x86)/Intel RealSense SDK 2.0/include'
DEPENDPATH  += 'C:/Program Files (x86)/Intel RealSense SDK 2.0/include'

##dùng cho kit mingw
# LIBS += -L'C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x86/realsense2.lib' #-realsense2
##kit msvc cần trỏ tới .lib file
LIBS += -L'C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64' -lrealsense2

##Add Opencv Lib
INCLUDEPATH += 'D:/Software/OpenCV2/OpenCV/build_MSVC_Module/install/include'
DEPENDPATH  += 'D:/Software/OpenCV2/OpenCV/build_MSVC_Module/install/include'

##Add library Opencv with Module for MSCV kit
LIBS += -L'D:/Software/OpenCV2/OpenCV/build_MSVC_Module/install/x64/vc17/lib'\
                -lopencv_aruco470 \
                -lopencv_core470 \
                -lopencv_imgcodecs470 \
                -lopencv_highgui470 \
                -lopencv_calib3d470 \
                -lopencv_dnn470 \
                -lopencv_dnn_objdetect470 \
                -lopencv_imgproc470 \
                -lopencv_video470 \
                -lopencv_videoio470 \
                -lopencv_features2d470
