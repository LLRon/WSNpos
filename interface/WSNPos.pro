#-------------------------------------------------
#
# Project created by QtCreator 2015-05-22T17:58:03
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = WSNPos
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    myimgtest.cpp

HEADERS  += mainwindow.h \
    myimgtest.h \
    Node.h

FORMS    += mainwindow.ui

DISTFILES +=

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../x64/release/ -lSensor_Locating
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../x64/debug/ -lSensor_Locating
else:unix: LIBS += -L$$PWD/../x64/ -lSensor_Locating

INCLUDEPATH += $$PWD/../master
DEPENDPATH += $$PWD/../master
