# -------------------------------------------------
# Project created by QtCreator 2012-12-30T14:57:59
# -------------------------------------------------
QT += core \
    gui \
    opengl
TARGET = QT_OpenGL
TEMPLATE = app
SOURCES += main.cpp \
    mainwindow.cpp \
    glwidget.cpp \
    cfinger.cpp \
    data_recv.cpp
HEADERS += mainwindow.h \
    glwidget.h \
    cfinger.h \
    data_recv.h
FORMS += mainwindow.ui
LIBS += -lglut \
    -lGL \
    -lGLU
