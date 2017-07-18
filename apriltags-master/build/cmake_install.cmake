# Install script for directory: /home/multipurpose/Design2/apriltags-master

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "applications" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/AprilTags" TYPE PROGRAM FILES
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Edge.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/FloatImage.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/GLine2D.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/GLineSegment2D.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Gaussian.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/GrayModel.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Gridder.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Homography33.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/MathUtil.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Quad.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Segment.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag16h5.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag16h5_other.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag25h7.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag25h9.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag36h11.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag36h11_other.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/Tag36h9.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/TagDetection.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/TagDetector.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/TagFamily.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/UnionFindSimple.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/XYWeight.h"
    "/home/multipurpose/Design2/apriltags-master/AprilTags/pch.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/multipurpose/Design2/apriltags-master/build/libapriltags.a")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/multipurpose/Design2/apriltags-master/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
