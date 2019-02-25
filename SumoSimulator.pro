TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
QT += opengl

SOURCES += main.cpp \
    SumoAIClass.cpp \
    SumoBotClass.cpp

SUBDIRS += \
    SumoSimulator.pro

DISTFILES +=

HEADERS += \
    SumoAIClass.h \
    SumoBotClass.h

LIBS += -lGL -lglut
