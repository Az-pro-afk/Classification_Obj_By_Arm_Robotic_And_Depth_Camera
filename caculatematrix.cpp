#include "caculatematrix.h"
#include "QDebug"

caculateMatrix::caculateMatrix(QObject *parent) : QObject(parent)
{

}

caculateMatrix::~caculateMatrix()
{

}

void caculateMatrix::createWorlCoordinateAruco(float arcLength, std::vector<Point3f> &objp, string originPos)
{
    if (originPos == "center") {
        objp.push_back(Point3f(-arcLength / 2.f, arcLength / 2.f, 0.0f));
        objp.push_back(Point3f(arcLength / 2.f, arcLength / 2.f, 0.0f));
        objp.push_back(Point3f(arcLength / 2.f, -arcLength / 2.f, 0.0f));
        objp.push_back(Point3f(-arcLength / 2.f, -arcLength / 2.f, 0.0f));
    }
    else if (originPos == "corner")
    {
        objp.push_back(Point3f(0.0f, 0.0f, 0.0f));
        objp.push_back(Point3f(0, arcLength, 0.0f));

        objp.push_back(Point3f(arcLength, arcLength, 0.0f));
        objp.push_back(Point3f(arcLength, 0.0f, 0.0f));

    }
}

void caculateMatrix::printMat(const Mat &mat)
{
    std::stringstream ss;
    ss << mat; // Chuyển ma trận thành chuỗi
    qDebug().noquote() << QString::fromStdString(ss.str()); // In chuỗi bằng qDebug
}

void caculateMatrix::createTarget2Cam(string imgDir, int index, Mat cameraMatrix, Mat distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs)
{

    vector<Point3f> objp;
    createWorlCoordinateAruco(AURO_LENGTH, objp);
    //    cout<< " objp "<< objp <<endl;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);

    for(int i=1; i < index; i++)
    {
        std::string imgPath = imgDir + "image_(" + std::to_string(i) + ").jpg";

        Mat img = imread(imgPath);

        //cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, dictionary, markerCorners, ids);
        if(ids.size()<1){
            cout<< "=============================================";
            cout<< "KHONG CO ARRUCO";
            cout<< "=============================================";

        }
        //  cout<< "ids: " << ids[0]<< endl;
        //   cout << "maker: " << markerCorners[0] << endl;
        Mat rvec, tvec;

        solvePnP(objp, markerCorners.at(0), cameraMatrix, distCoeffs, rvec, tvec);
        Mat rotation_matrix;

        cv::Rodrigues(rvec, rotation_matrix);

        rvecs.push_back(rotation_matrix);
        tvecs.push_back(tvec);

    }
}

void caculateMatrix::loadCamera(Mat &cameraMatrix, Mat &distCoeffs, bool isOnline)
{
    if(isOnline){
        ;
    }
    else {
        cameraMatrix = cv::Mat::eye(3,3, CV_64F);
        distCoeffs = cv::Mat::zeros (5,1, CV_64F);
        cameraMatrix.at<double>(0, 0) = 606.281066894531; // Fx
        cameraMatrix.at<double>(1, 1) = 606.417541503906; // Fy
        cameraMatrix.at<double>(0, 2) = 327.089965820313; // PPX
        cameraMatrix.at<double>(1, 2) = 240.459075927734; //PPY
    }
}

Mat caculateMatrix::Rx(double phi)
{    // Initialize the rotation matrix
    cv::Mat Rx = cv::Mat::eye(3, 3, CV_64F);

    // Assign values to the rotation matrix
    double radians = phi * CV_PI / 180.0; // Convert degrees to radians
    Rx.at<double>(1, 1) = std::cos(radians);
    Rx.at<double>(1, 2) = -std::sin(radians);
    Rx.at<double>(2, 1) = std::sin(radians);
    Rx.at<double>(2, 2) = std::cos(radians);
    return Rx;

}

Mat caculateMatrix::Ry(double phi)
{    // Initialize the rotation matrix
    cv::Mat Ry = cv::Mat::eye(3, 3, CV_64F);

    // Assign values to the rotation matrix
    double radians = phi * CV_PI / 180.0; // Convert degrees to radians
    Ry.at<double>(0, 0) = std::cos(radians);
    Ry.at<double>(0, 2) = std::sin(radians);
    Ry.at<double>(2, 0) = -std::sin(radians);
    Ry.at<double>(2, 2) = std::cos(radians);

    return Ry;

}

Mat caculateMatrix::Rz(double phi)
{    // Initialize the rotation matrix
    cv::Mat Rz = cv::Mat::eye(3, 3, CV_64F);

    // Assign values to the rotation matrix
    double radians = phi * CV_PI / 180.0; // Convert degrees to radians
    Rz.at<double>(0, 0) = std::cos(radians);
    Rz.at<double>(0, 1) = -std::sin(radians);
    Rz.at<double>(1, 0) = std::sin(radians);
    Rz.at<double>(1, 1) = std::cos(radians);

    return Rz;

}

void caculateMatrix::loadRobotPos(string fileName, vector<double> &values)
{
    // Đọc từng dòng từ file và lưu các giá trị vào vector
    std::string line;
    std::ifstream file(fileName);
    while (std::getline(file, line)) {
        double value = std::stod(line); // Chuyển đổi dòng thành số dạng double
        values.push_back(value); // Thêm giá trị vào vector
    }

    // Đóng file sau khi đã đọc xong
    file.close();
}

void caculateMatrix::createGripper2Base(string folderDir, int index, std::vector<Mat> &t_gripper2base, std::vector<Mat> &R_gripper2base)
{
    for (int i = 1; i< index; i++){

        std::vector<cv::Mat> R_gripper2base_non_convert;
        std::string filePath = folderDir + "position_(" + std::to_string(i) + ").txt";

        // Tạo một đối tượng ifstream để mở file cho đọc

        // Tạo một vector để lưu các giá trị từ file

        std::vector<double> values;


        loadRobotPos(filePath, values);
        cv::Mat translation_vector_gripper2base = cv::Mat(3, 1, CV_64F);
        translation_vector_gripper2base.at<double>(0, 0) = values[2]; // Gán giá trị dịch chuyển x
        translation_vector_gripper2base.at<double>(1, 0) = values[3]; // Gán giá trị dịch chuyển y
        translation_vector_gripper2base.at<double>(2, 0) = values[4]; // Gán giá trị dịch chuyển z
        t_gripper2base.push_back(translation_vector_gripper2base);

        cv::Mat rotation_vector_gripper2base = cv::Mat(3, 1, CV_64F);
        rotation_vector_gripper2base.at<double>(0, 0) = values[5]; // Gán giá trị dịch chuyển Roll
        rotation_vector_gripper2base.at<double>(1, 0) = values[6]; // Gán giá trị dịch chuyển Pitch
        rotation_vector_gripper2base.at<double>(2, 0) = values[7]; // Gán giá trị dịch chuyển Yaw

        R_gripper2base_non_convert.push_back(rotation_vector_gripper2base);

        cv::Mat R_result = Rz(rotation_vector_gripper2base.at<double>(2, 0))*Ry(rotation_vector_gripper2base.at<double>(1, 0))*Rx(rotation_vector_gripper2base.at<double>(0, 0));
        R_gripper2base.push_back(R_result);
    }
}

void caculateMatrix::getInverseRT(Mat R1, Mat T1, Mat &R_inv, Mat &T_inv)
{
    R_inv = R1.t();
    T_inv = -R_inv * T1;
}

void caculateMatrix::createBase2Gripper(const std::vector<Mat> &t_gripper2base, const std::vector<Mat> &R_gripper2base, std::vector<Mat> &t_base2gripper, std::vector<Mat> &R_base2gripper)
{
    for (int i =0; i< t_gripper2base.size(); i++ )
    {
        Mat R_inv, T_inv;
        Mat R, T;
        R = R_gripper2base.at(i);
        T = t_gripper2base.at(i);
        getInverseRT(R, T, R_inv, T_inv);
        R_base2gripper.push_back(R_inv);
        t_base2gripper.push_back(T_inv);
    }
}

double caculateMatrix::deg2rad(double deg)
{
    return deg * CV_PI / 180.0;
}

double caculateMatrix::rad2deg(double rad)
{
    return rad * 180.0 / CV_PI;
}

void caculateMatrix::getRPYAngle(Mat R, Vec3d &vec1, Vec3d &vec2)
{
    double r11 = R.at<double>(0, 0);
    double r21 = R.at<double>(1, 0);
    double r31 = R.at<double>(2, 0);
    double r32 = R.at<double>(2, 1);
    double r33 = R.at<double>(2, 2);

    double pitch1, pitch2;
    pitch1 = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
    pitch2 = atan2(-r31, -sqrt(r32 * r32 + r33 * r33));

    double roll1, roll2;
    roll1 = atan2(r32 / cos(pitch1), r33 / cos(pitch1));
    roll2 = atan2(r32 / cos(pitch2), r33 / cos(pitch2));

    double yaw1, yaw2;
    yaw1 = atan2(r21 / cos(pitch1), r11 / cos(pitch1));
    yaw2 = atan2(r21 / cos(pitch2), r11 / cos(pitch2));
    /*vec1 = Vec3d(roll1, pitch1, yaw1);
    vec2 = Vec3d(roll2, pitch2, yaw2);*/

    vec1 = Vec3d(rad2deg(roll1), rad2deg(pitch1), rad2deg(yaw1));
    vec2 = Vec3d(rad2deg(roll2), rad2deg(pitch2), rad2deg(yaw2));
}

void caculateMatrix::getMatrixFromCam2Base(int numSample)
{
    Mat cameraMatrix, distCoeffs;
    loadCamera(cameraMatrix, distCoeffs);
    //    string imgDir = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/PC_Robot_Control/Image_Data_May_3/";
    string imgDir = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/Calib_Night/Calib_7x7/Image/";
    vector<Mat> tvecs, rvecs;
    createTarget2Cam(imgDir, numSample, cameraMatrix, distCoeffs,rvecs, tvecs);
    //    displayVector<Mat>(tvecs);
    //---------------------
    std::vector<cv::Mat> t_gripper2base, R_gripper2base;
    //    string folderDir = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/PC_Robot_Control/Image_Data/Position/";
    string folderDir = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/Calib_Night/Calib_7x7/Position/";

    createGripper2Base(folderDir, numSample, t_gripper2base, R_gripper2base);

    //displayVector<Mat>(R_gripper2base);
    cout<< "-------------------\n";
    std::vector<cv::Mat> t_base2gripper, R_base2gripper;
    createBase2Gripper(t_gripper2base, R_gripper2base, t_base2gripper, R_base2gripper);
    //displayVector<Mat>(R_base2gripper);

    cout << "rvec: " << rvecs.size() <<endl;
    cout << "tvecs: " << tvecs.size() <<endl;
    cout << "t_base2gripper: " << t_base2gripper.size()<<endl;
    cout << "R_base2gripper: " << R_base2gripper.size()<<endl;
    cout <<"-----------------------------"<<endl;
    cout << "rvec: " << rvecs[0]<<endl;
    cout << "tvecs: " << tvecs[0] <<endl;
    cout << "t_base2gripper: " << t_base2gripper[0]<<endl;
    cout << "R_base2gripper: " << R_base2gripper[0]<<endl;
    cv::Mat t_cam2base, R_cam2base;

    cv::calibrateHandEye(R_base2gripper, t_base2gripper, rvecs, tvecs,
                         R_cam2base, t_cam2base, CALIB_HAND_EYE_PARK);

    cout <<"------------ Result -----------------"<<endl;
    cout << "R_cam2base: " << R_cam2base<<endl;
    cout << "t_cam2base: " << t_cam2base <<endl;
    Vec3d vec1, vec2;
    getRPYAngle(R_cam2base,vec1, vec2);
    cout <<vec1;
}

