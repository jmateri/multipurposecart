/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

//Headers included by Apriltags creators
#include <iostream>
#include "opencv2/opencv.hpp"
#include <eigen3/Eigen/Dense>
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "getopt.h"
#include <math.h>
#include <cmath>

//Our included Headers
#include <QCoreApplication>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QTime>

using namespace std;
using namespace cv;


//Builtin function used for trig calculations
inline double standardRad(double t) {
    double PI = 3.14159265358979323846;
    double TWOPI = 2.0*PI;
    if (t >= 0.)
    {
        t = fmod(t+PI, TWOPI) - PI;
    }
    else
    {
        t = fmod(t-PI, -TWOPI) + PI;
    }
    return t;
}

int main(int argc, char *argv[])
{
    // Built in paramenters with the AprilTag Library
    getopt_t *getopt = getopt_create();
    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    //Parameters for debugging and testing purposes
    bool arduino = false;
    bool debugging = true; // displays camera view and draws lines on apriltags when detected
    bool showInfo = true; //Prints information to console
    bool showFps = true; //Prints the FPS that the camera is getting
    bool turn = false; //Allows the cart to turn

    //April tag calibration information
    double tagSize = 0.127; // April tag side length in meters of square black frame
    double fx = 203.718; // camera focal length in pixels
    double fy = 203.718;
    double px = 319.5; // camera principal point
    double py = 239.5;

    // Checks console for paramenters
    if (!getopt_parse(getopt, argc, argv, 1) ||  getopt_get_bool(getopt, "help"))
    {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    if (!cap.isOpened())
    {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
    {
        tf = tag36h11_create();
    }
    else if (!strcmp(famname, "tag36h10"))
    {
        tf = tag36h10_create();
    }
    else if (!strcmp(famname, "tag36artoolkit"))
    {
        tf = tag36artoolkit_create();
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        tf = tag25h9_create();
    }
    else if (!strcmp(famname, "tag25h7"))
    {
        tf = tag25h7_create();
    }
    else
    {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    // Adds the paraments to the td object
    tf->black_border = getopt_get_int(getopt, "border");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    // Initalize variables for sending data over serial port
    QSerialPort port;
    QByteArray outputArrayLeft;
    QByteArray outputArrayRight;

    //If true, data will be sent to the arduino
    if(arduino)
    {
        //Gets list of available serial ports
        QList<QSerialPortInfo> info = QSerialPortInfo::availablePorts();

        //Finds the port that the arduino is connected to automatically, if any
        int i;
        for(i=0; i<info.length(); i++)
        {
            if (info.at(i).productIdentifier() == 66 && info.at(i).vendorIdentifier() == 9025)
            {
                port.setPortName(info.at(i).portName());
                break;
            }
        }
        //If arduino is not connected, end the program
        if( i == info.length())
        {
            qDebug() << "No Arduino Device Found";
            return 1;
        }
        // Close the arduino port if it's already open
        if(port.isOpen())
        {
            port.close();
        }

        //Open arduino in Write only mode, set Baud to 9600, and configure other serial port parameters
        qDebug() << "Open: " << port.open(QIODevice::WriteOnly);
        qDebug() << "SetBaudRate: " << port.setBaudRate(QSerialPort::Baud9600);
        qDebug() << "SetDataBits: " << port.setDataBits(QSerialPort::Data8);
        qDebug() << "SetParity: " << port.setParity(QSerialPort::NoParity);
        qDebug() << "SetStopBits: " << port.setStopBits(QSerialPort::OneStop);
        qDebug() << "SetFlowControl: " << port.setFlowControl(QSerialPort::NoFlowControl);

    }
    Mat frame, gray;

    //Used for calculating the frames per second of the camera
    int frameCount;
    QTime timer;
    if (showFps)
    {
        timer.start();
    }

    //Infinite loop for program to run forever
    while (true)
    {
        //Takes the frame and changes it to grayscale
        cap >> frame;

        qDebug() << frame.rows << " Rows";
        qDebug() << frame.cols << " Cols";
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };

        //puts all the detected tags into a zarray
        zarray_t *detections = apriltag_detector_detect(td, &im);

        //Show frames per second, if desired
        if (showFps)
        {
            //Increment frame count with each passing while loop
            frameCount++;
            if (timer.elapsed() >= 1000)
            {
                cout << frameCount << " Fps"<< endl;
                timer.restart();
                frameCount = 0;
            }
        }

        //Show if tags are detected
        if (showInfo)
        {
            cout << zarray_size(detections) << " tags detected" << endl;
        }

        // Loops through all the detections
        for (int i = 0; i < zarray_size(detections); i++)
        {
            // Creates an apriltag detection pointer and gets a detection
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            std::vector<cv::Point3f> objPts;
            std::vector<cv::Point2f> imgPts;
            double size = tagSize/2.;
            objPts.push_back(cv::Point3f(-size,-size, 0));
            objPts.push_back(cv::Point3f( size,-size, 0));
            objPts.push_back(cv::Point3f( size, size, 0));
            objPts.push_back(cv::Point3f(-size, size, 0));
            imgPts.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
            imgPts.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
            imgPts.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
            imgPts.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));
            std::cout << "objPoints: "<< objPts << endl;
            std::cout << "imgPoints: "<< imgPts << endl;
            cv::Mat rvec, tvec;
            cv::Matx33f cameraMatrix( fx, 0, px, 0, fy, py, 0,  0,  1);
            cv::Vec4f distParam(0,0,0,0);
            cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
            std::cout << "rvec: "<< rvec << endl;
            std::cout << "tvec: "<< tvec << endl;
            cv::Matx33d r;
            cv::Rodrigues(rvec, r);
            std::cout << "R: "<< r << endl;
            Eigen::Matrix3d wRo;
            wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);
            std::cout << "wRo: "<< wRo << endl;
            Eigen::Matrix4d T;
            T.topLeftCorner(3,3) = wRo;
            T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
            T.row(3) << 0,0,0,1;
            // converting from camera frame (z forward, x right, y down) to
            // object frame (x forward, y left, z up)
            std::cout << "T: "<< T << endl;
            Eigen::Matrix4d M;
            M <<  0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
            std::cout << M << endl;
            Eigen::Matrix4d MT = M*T;
            std::cout << "MT: "<< MT << endl;
            // translation vector from camera to the April tag
            Eigen::Vector3d translation = MT.col(3).head(3);
            std::cout << "translation: "<< translation << endl;
            // orientation of April tag with respect to camera: the camera
            // convention makes more sense here, because yaw,pitch,roll then
            // naturally agree with the orientation of the object
            Eigen::Matrix3d rotation = T.block(0,0,3,3);
            Eigen::Matrix3d F;
            F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
            Eigen::Matrix3d fixedRotation = F*rotation;
            double yaw, pitch, roll, distance, c, s;
            yaw = standardRad(atan2(fixedRotation(1,0), fixedRotation(0,0)));
            c = cos(yaw);
            s = sin(yaw);
            pitch = standardRad(atan2(-fixedRotation(2,0), fixedRotation(0,0)*c + fixedRotation(1,0)*s));
            roll  = standardRad(atan2(fixedRotation(0,2)*s - fixedRotation(1,2)*c, -fixedRotation(0,1)*s + fixedRotation(1,1)*c));
            distance = translation.norm();

            //Used for sending data to the arduino
            if (arduino)
            {
                //Treat all distances greater than 4 the same (basically infinite distance)
                if(distance > 4)
                {
                    distance = 4;
                }

                //Bytes that will be sent of the serial port for each side. They are chars since
                //a char is one byte
                char outputRight; //Byte sent to the right side
                char outputLeft; //Byte sent to the left side
                char output; //Not actually sent, just used for calculation purposes

                //Clear Byte array each time so data is not resent
                outputArrayLeft.clear();
                outputArrayRight.clear();

                //Append 255 to the left size, and 0 to the right side. The arduino will know data
                //starting with 255 will be for the left motors, and data beginning with 0 will be
                //for the right side motors
                outputArrayLeft.append(255);
                outputArrayRight.append((char) 0);

                //If distance is greater than 1.2 meters, move forward
                if (distance > 1.2)
                {
                    //Formula for calculating speed based on distance
                    output = (char) 81.64 + 45.35 * distance;

                    //Cap the maximum output to be 254, since that's what the arduino can output for PWM
                    if(output == 255)
                    {
                        output = 254;
                    }
                    else
                    {
                        outputRight = output;
                        outputLeft = output;
                    }
                }

                //If distance is between .8 and 1.2, stop the cart
                else if(distance >= .8 && distance <= 1.2)
                {
                    //127 is stop
                    output = 127;
                    {
                        outputRight = output;
                        outputLeft = output;
                    }
                }
                //Otherwise, move the cart backward
                else
                {
                    //Formula for calculating backward distance
                    output = (char)(96.25 * distance + 50);
                    //Cap minimum output at 1, since that's what the arduino can output for PWM
                    if(output <= 0)
                    {
                        output = 1;
                    }
                    else
                    {
                        outputRight = output;
                        outputLeft = output;
                    }
                }

                //Get the angle of the horizontal angle of the april tag and convert it to degrees
                double angle = abs(atan(translation(1)/translation(0)) * (180 / 3.14159));
                qDebug() << angle << " Angle";

                //Formula for calculating the angle the cart should turn when it's near
                double turnDifferenceNear = angle * 1.6;

                //Formula for calculating the angle the cart should turn when it's far away
                double turnDifferenceFar =  (((-55.0 / 1369.0) * pow(angle - 40, 2)) + 60);

                //Offset so the cart doesn't turn as sharp the faster it's going
                double straightOffset = translation(0) * 15.0;

                //Turns the cart if set
                if(turn)
                {
                    //Turn cart sharply if tag is close and to the right
                    if(distance < 2 && (angle > 5) && (translation(1) < 0))
                    {
                        //Add turn difference to one side and subtract it to the other to create the turn
                        //Also cap it to not produce any overflow
                        outputRight = fmin(254,(int)((unsigned char)outputRight + turnDifferenceNear));
                        outputLeft = fmax(1,(int)((unsigned char)outputLeft - turnDifferenceNear));
                    }
                    //Turn cart sharply if tag is close and to the left
                    else if(distance < 2 && (angle > 5) && (translation(1) > 0))
                    {
                        //Add turn difference to one side and subtract it to the other to create the turn
                        //Also cap it to not produce any overflow
                        outputLeft = fmin(254,(int)((unsigned char)outputLeft + turnDifferenceNear));
                        outputRight =  fmax(1,(int)((unsigned char)outputRight - turnDifferenceNear));
                    }
                    //Turn cart less sharp if tag is close and to the right
                    else if((angle > 3) && (translation(1) < 0))
                    {
                        //Add turn difference to one side and subtract it to the other to create the turn
                        //Also cap it to not produce any overflow
                        outputRight = fmin(254,(int)((unsigned char)outputRight + turnDifferenceFar));
                        outputLeft = fmax(1,(int)((unsigned char)outputLeft - turnDifferenceFar));

                        if(turnDifferenceFar > straightOffset)
                        {
                            outputRight -= straightOffset;
                            outputLeft += straightOffset;
                        }
                    }
                    //turn cart less sharply if tag is close and to the left
                    else if((angle > 3) && (translation(1) > 0))
                    {
                        //Add turn difference to one side and subtract it to the other to create the turn
                        //Also cap it to not produce any overflow
                        outputLeft = fmin(254,(int)((unsigned char)outputLeft + turnDifferenceFar));
                        outputRight =  fmax(1,(int)((unsigned char)outputRight - turnDifferenceFar));
                        if(turnDifferenceFar > straightOffset)
                        {
                            outputRight -= straightOffset;
                            outputLeft += straightOffset;
                        }
                    }
                }

                //Append motor speeds to byte array
                outputArrayLeft.append(outputLeft);
                outputArrayRight.append(outputRight);

                //Write to the serial port and flush to ensure data is written
                qDebug() << port.write(outputArrayLeft);
                port.flush();
                qDebug() << port.write(outputArrayRight);
                port.flush();
            }
            if (showInfo)
            {
                cout << "  distance= " << distance  // norm of x,y,z
                     << " m, x= " << translation(0) // distance from camera
                     << ", y= " << translation(1)  // horizontal movement
                     << ", z= " << translation(2)  // vertical movement
                     << ", yaw= " << yaw  // rotation about the center
                     << ", pitch= " << pitch // rotation about a vertical axis
                     << ", roll= " << roll  // rotation about a horizontal axis
                     << endl;
            }
            if (debugging) // if true displays all current camera view and adds information to each frame
            {
                //Putting lines around each apriltag detection
                line(frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
                line(frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
                line(frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
                line(frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);
                // Labels each apriltag with
                stringstream ss;
                ss << det->id;
                String text = ss.str();
                int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
                putText(frame, text, Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2), fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
            }
        }
        zarray_destroy(detections);
        if (debugging)
        {
            namedWindow("Tag Detections");
            imshow("Tag Detections", frame);
            waitKey(1);
        }
    }
    apriltag_detector_destroy(td);
    if (!strcmp(famname, "tag36h11"))
        tag36h11_destroy(tf);
    else if (!strcmp(famname, "tag36h10"))
        tag36h10_destroy(tf);
    else if (!strcmp(famname, "tag36artoolkit"))
        tag36artoolkit_destroy(tf);
    else if (!strcmp(famname, "tag25h9"))
        tag25h9_destroy(tf);
    else if (!strcmp(famname, "tag25h7"))
        tag25h7_destroy(tf);
    getopt_destroy(getopt);
    return 0;
}
