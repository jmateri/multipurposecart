TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    apriltags_demo.cpp \
    Edge.cc \
    FloatImage.cc \
    Gaussian.cc \
    GLine2D.cc \
    GLineSegment2D.cc \
    GrayModel.cc \
    Homography33.cc \
    MathUtil.cc \
    Quad.cc \
    Segment.cc \
    Serial.cpp \
    TagDetection.cc \
    TagDetector.cc \
    TagFamily.cc \
    UnionFindSimple.cc

HEADERS += \
    Edge.h \
    FloatImage.h \
    Gaussian.h \
    GLine2D.h \
    GLineSegment2D.h \
    GrayModel.h \
    Gridder.h \
    Homography33.h \
    MathUtil.h \
    pch.h \
    Quad.h \
    Segment.h \
    Serial.h \
    Tag16h5.h \
    Tag16h5_other.h \
    Tag25h7.h \
    Tag25h9.h \
    Tag36h9.h \
    Tag36h11.h \
    Tag36h11_other.h \
    TagDetection.h \
    TagDetector.h \
    TagFamily.h \
    UnionFindSimple.h \
    XYWeight.h

LIBS += -lpthread -lm \
        `pkg-config --libs opencv` \
        -lv4l1 -lv4l2

