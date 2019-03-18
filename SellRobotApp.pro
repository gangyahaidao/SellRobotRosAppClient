QT -= gui
QT += network serialport opengl

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

DESTDIR = $$PWD/bin

INCLUDEPATH += ./xfei_include \
                ./librealsense/third-party \
                /opt/ros/kinetic/include/opencv-3.3.1-dev

LIBS += $$PWD/libs/xfei/x64/libmsc.so \
        $$PWD/libs/xfei/x64/libaiui.so \
        /usr/lib/x86_64-linux-gnu/librealsense2.so \
        /usr/lib/x86_64-linux-gnu/libglfw.so   \
        /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    sellrobotclient.cpp \
    robotconfig.cpp \
    robotarm.cpp \
    movebase.cpp \
    robotpose.cpp \
    myvector4d.cpp \
    robotaction.cpp \
    realsensemulticam.cpp

HEADERS += \
    sellrobotclient.h \
    robotconfig.h \
    robotarm.h \
    movebase.h \
    robotpose.h \
    myvector4d.h \
    robotaction.h \
    example.hpp \
    realsensemulticam.h
