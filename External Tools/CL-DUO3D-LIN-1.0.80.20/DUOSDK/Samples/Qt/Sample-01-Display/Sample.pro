TARGET = Sample-01-Display
TEMPLATE = app

QT += core gui widgets

CONFIG += console c++11 warn_off

HEADERS += mainwindow.h
SOURCES += main.cpp mainwindow.cpp

INCLUDEPATH += $$PWD/../../../SDK/include C:/Dev/OpenCV/3.1.0/include
DEPENDPATH += $$PWD/../../../SDK/include C:/Dev/OpenCV/3.1.0/include

equals(QMAKE_HOST.os, Windows) {
    !contains(QT_ARCH, x86_64) {
        DESTDIR = $$PWD/../../bin/x86
        LIBS += -L$$PWD/../../../SDK/windows/x86 -LC:/Dev/OpenCV/3.1.0/x86/vc12/staticlib
    } else {
        DESTDIR = $$PWD/../../bin/x64
        LIBS += -L$$PWD/../../../SDK/windows/x64 -LC:/Dev/OpenCV/3.1.0/x64/vc12/staticlib
    }
    LIBS += -lDUOLib -lopencv_core310 -lopencv_imgproc310 -lippicvmt -lzlib
}
equals(QMAKE_HOST.os, Linux) {
    contains(QT_ARCH, x86_64) {
        DESTDIR = $$PWD/../../bin/x64
        LIBS += -L$$PWD/../../../SDK/linux/x64
    } else:contains(QT_ARCH, x86) {
        DESTDIR = $$PWD/../../bin/x86
        LIBS += -L$$PWD/../../../SDK/linux/x86
    } else:contains(QT_ARCH, arm) {
        DESTDIR = $$PWD/../../bin/arm
        LIBS += -L$$PWD/../../../SDK/linux/arm
    }
    LIBS += -lDUO -lopencv_core -lopencv_imgproc
}
