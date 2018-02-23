#include "MyFreenectDevice.hpp"
#include <iostream>

#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>




using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    vpImage<unsigned char> I(480,640);
    vpDisplayX d(I, 0, 0, "Camera view");
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.startVideo();

    //*******************************************
    //*************TODO by STUDENTS**************
    //*******************************************
    vpDot2 vpD[4];
    vpD[0].setGraphics(true);
    vpD[1].setGraphics(true);
    vpD[2].setGraphics(true);
    vpD[3].setGraphics(true);
    bool flag = true;
    vpPoint tab_vpPoints[4];
    double au = 535;
    double av = 535;
    double u0 = 320;
    double v0 = 240;
    vpCameraParameters vpCp = vpCameraParameters(au, av, u0, v0);
    double x, y;
    vpPose sid_vpPose;
    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix rcMo;
    vpHomogeneousMatrix cdMo = vpHomogeneousMatrix(0,0,5,0,0,0);

    vpRobotCamera sid_vpRC;
    vpServo sid_vpServo;
    vpFeaturePoint sid_vpFP[4];
    vpColVector sid_vpCV;
    double Z  = 1;

    tab_vpPoints[0].setWorldCoordinates(10, 10, 0);
    tab_vpPoints[1].setWorldCoordinates(-10, 10, 0);
    tab_vpPoints[2].setWorldCoordinates(10, -10, 0);
    tab_vpPoints[3].setWorldCoordinates(-10, -10, 0);


    while(1){

        if(flag){
            device.getVideo(I);
            vpDisplay::display(I);
            vpDisplay::flush(I);

            if (vpDisplay::getClick(I, false)){
                   flag = false;
                   vpD[0].initTracking(I);
                   vpD[1].initTracking(I);
                   vpD[2].initTracking(I);
                   vpD[3].initTracking(I);
            }

        }

        else {

             device.getVideo(I);
             vpDisplay::display(I);
             for(int i=0; i<4; i++){
                vpD[i].track(I); // Tracking
                vpPixelMeterConversion::convertPoint(vpCp, vpD[i].getCog(), x, y);


                tab_vpPoints[i].set_x(x);
                tab_vpPoints[i].set_y(y);


                sid_vpPose.addPoint(tab_vpPoints[i]);

                //vpFeatureBuilder::create(sid_vpFP[i], vpCp, vpD[i]);
                //sid_vpFP[i].set_Z(Z);
             }

            sid_vpPose.computePose(vpPose::LAGRANGE, cMo);
            //sid_vpPose.display(I, cMo, vpCp, 0.05, vpColor::red);
            vpDisplay::displayFrame(I, cMo, vpCp, 0.05, vpColor::red, 2);


//             sid_vpRC.setPosition(cdMo);
//             sid_vpServo.setServo(vpServo::EYEINHAND_CAMERA);
//             sid_vpServo.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

////             for(int i=0; i<4; i++){
////                 vpFeatureBuilder::create(sid_vpFP[i], vpCp, vpD[i]);
////                 sid_vpFP[i].set_Z(Z);
////             }

//             for(int i=0; i<4; i++){
//                 tab_vpPoints[i].changeFrame(cMo);
//                 tab_vpPoints[i].projection();
//                 //vpFeatureBuilder::create(sid_vpFP[i], vpCp, vpD[i]);
//             }


//             sid_vpCV = sid_vpServo.computeControlLaw();
//             sid_vpRC.setVelocity(vpRobot::CAMERA_FRAME, sid_vpCV);
//             sid_vpRC.getPosition(cMo);

//             sid_vpPose.display(I, cMo, vpCp, 0.05, vpColor::green);


             vpDisplay::flush(I);
            vpTime::wait(300);
        }






    }


    //*******************************************
    //*******************************************
    //*******************************************


    device.stopVideo();

    return 0;
}
