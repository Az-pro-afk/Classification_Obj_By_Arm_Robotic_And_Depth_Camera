#ifndef CACULATEMATRIX_H
#define CACULATEMATRIX_H

#include <QObject>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ccalib.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/features2d.hpp"
#include <cmath>
#include <iostream>

#include <fstream>
#include <string>
#include <vector>
#include <cmath>

//#include "capture.h"

// #define AURO_LENGTH 64     // don vi la mm cho miếng 5x5
#define AURO_LENGTH 62     // don vi la mm cho miếng 7x7

using namespace std;
using namespace cv;
template <class T>
void displayVector(vector<T> vec){
    for(int i=0; i < vec.size(); i++){
        cout<<vec.at(i) <<endl;
    }
}

class caculateMatrix : public QObject
{
    Q_OBJECT
public:
    explicit caculateMatrix(QObject *parent = nullptr);
    ~caculateMatrix();



    void createWorlCoordinateAruco(float arcLength, std::vector<Point3f>& objp, std::string originPos = "center");
    void printMat(const cv::Mat& mat);
    void createTarget2Cam(string imgDir,int index, Mat cameraMatrix, Mat distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs);
    void loadCamera(Mat& cameraMatrix, Mat& distCoeffs, bool isOnline=false);

    cv::Mat Rx(double phi);
    cv::Mat Ry(double phi);
    cv::Mat Rz(double phi);

    void loadRobotPos(string fileName, vector<double>& values);
    void createGripper2Base(string folderDir,int index, std::vector<cv::Mat>& t_gripper2base,std::vector<cv::Mat>& R_gripper2base);
    void getInverseRT(Mat R1, Mat T1, Mat&	R_inv, Mat& T_inv);
    void createBase2Gripper(const std::vector<cv::Mat>& t_gripper2base,const std::vector<cv::Mat>& R_gripper2base,
                            std::vector<cv::Mat>& t_base2gripper,std::vector<cv::Mat>& R_base2gripper);
    double deg2rad(double deg);
    double rad2deg(double rad);
    void getRPYAngle(Mat R, Vec3d& vec1, Vec3d& vec2);

    void getMatrixFromCam2Base(int numSample);
signals:

public slots:

private:

};

#endif // CACULATEMATRIX_H
