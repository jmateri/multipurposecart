TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG += qt
QT += serialport
SOURCES += \
    opencv_demo.cc \
    apriltag.c \
    apriltag_quad_thresh.c \
    g2d.c \
    getopt.c \
    homography.c \
    image_f32.c \
    image_u8.c \
    image_u8x3.c \
    image_u8x4.c \
    matd.c \
    pam.c \
    pjpeg.c \
    pjpeg-idct.c \
    pnm.c \
    string_util.c \
    svd22.c \
    tag16h5.c \
    tag25h7.c \
    tag25h9.c \
    tag36artoolkit.c \
    tag36h10.c \
    tag36h11.c \
    time_util.c \
    unionfind.c \
    workerpool.c \
    zarray.c \
    zhash.c \
    zmaxheap.c

HEADERS += \
    apriltag.h \
    apriltag_math.h \
    doubles.h \
    doubles_floats_impl.h \
    floats.h \
    g2d.h \
    getopt.h \
    homography.h \
    image_f32.h \
    image_types.h \
    image_u8.h \
    image_u8x3.h \
    image_u8x4.h \
    matd.h \
    math_util.h \
    pam.h \
    pjpeg.h \
    pnm.h \
    postscript_utils.h \
    string_util.h \
    svd22.h \
    tag16h5.h \
    tag25h7.h \
    tag25h9.h \
    tag36artoolkit.h \
    tag36h10.h \
    tag36h11.h \
    thash_impl.h \
    timeprofile.h \
    time_util.h \
    unionfind.h \
    workerpool.h \
    zarray.h \
    zhash.h \
    zmaxheap.h

LIBS += -lpthread -lm \
        `pkg-config --libs opencv`

INCLUDEPATH += /usr/local/include/
