#-------------------------------------------------
#
# Project created by QtCreator 2013-11-15T10:24:08
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IMUFilter
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    usbcan.cpp \
    IMUfilter.cpp \
    devicefilter.cpp \
    qcustomplot.cpp

HEADERS  += mainwindow.h \
    usbcan.h \
    IMUfilter.h \
    ControlCAN.h \
    macrodefs.h \
    devicefilter.h \
    qcustomplot.h

FORMS    += mainwindow.ui

unix|win32: LIBS += -L$$PWD/ -lControlCAN

INCLUDEPATH += $$PWD/
DEPENDPATH += $$PWD/
