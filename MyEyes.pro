#-------------------------------------------------
#
# Project created by QtCreator 2018-04-30T19:13:10
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

INCLUDEPATH += $$quote(/usr/include/pcl-1.7/)\
               $$quote(/usr/include/vtk-6.2/)\

CONFIG += static

TARGET = MyEyes
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


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    userrecognizer_window.cpp \
    userrecognizer_thread.cpp \
    usercomm.cpp \
    usertracking.cpp \
    util.cpp \
    setposition_window.cpp

HEADERS += \
        mainwindow.h \
    commons.h \
    userrecognizer_window.h \
    userrecognizer_thread.h \
    usercomm.h \
    usertracking.h \
    util.h \
    setposition_window.h

FORMS += \
        mainwindow.ui \
    userRecognizer_window.ui \
    setposition_window.ui

DISTFILES += \
    CMakeLists.txt \
    misc/icons8-robot-100.ico

STATECHARTS += \
    AppStateChart.scxml
