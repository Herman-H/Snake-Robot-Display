#-------------------------------------------------
#
# Project created by QtCreator 2015-02-05T17:07:32
#
#-------------------------------------------------

QT       += core gui
QT       += opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = display2Dmodel
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h \
    matlabinterface.h \
    dimensions.h \
    graphicsitems.h

FORMS    += mainwindow.ui
