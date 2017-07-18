# CMake generated Testfile for 
# Source directory: /home/multipurpose/Design2/OpenCV/modules/ml
# Build directory: /home/multipurpose/Design2/OpenCV/build/modules/ml
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_ml "/home/multipurpose/Design2/OpenCV/build/bin/opencv_test_ml" "--gtest_output=xml:opencv_test_ml.xml")
set_tests_properties(opencv_test_ml PROPERTIES  LABELS "Main;opencv_ml;Accuracy" WORKING_DIRECTORY "/home/multipurpose/Design2/OpenCV/build/test-reports/accuracy")
