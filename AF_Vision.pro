#-------------------------------------------------
#
# Project created by QtCreator 2018-06-21T20:41:43
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AF_Vision
TEMPLATE = app

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so  \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/local/lib/libopencv_calib3d.so

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
    calibrate.cpp \
    calibration.cpp \
    cvandui.cpp \
    mainwindow.cpp \
    ncc_match.cpp \
    nccmatchwidget.cpp

HEADERS += \
        mainwindow.h \
    calibrate.h \
    calibration.h \
    cvandui.h \
    ncc_match.h \
    nccmatchwidget.h \
    vision.h

FORMS += \
        mainwindow.ui \
    calibration.ui \
    nccmatchwidget.ui

#SUBDIRS += \
#    AF_Vision.pro

#DISTFILES += \
#    AF_Vision.pro.user \
#    nccParam.ini
