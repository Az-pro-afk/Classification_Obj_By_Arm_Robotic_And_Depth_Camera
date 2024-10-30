#ifndef CAPTURE_H
#define CAPTURE_H

// #include <QObject>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <QThread>
#include <QDebug>
#include <QPixmap>
#include <QElapsedTimer>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>         //__Có thể error nếu cùng khai báo .h này với TCP.h do bị trùng opencv.h
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include <limits>
#include <vector>
//--------------------------connect to Python-------------------------
// #include <TCP.h>
//-------------------------------------------------------------------

using namespace std;
//---------------------------------------------------------------------

using namespace rs2;
using namespace cv;

//---------------------------------------------------------------------
//----------------------------For Tracking testing---------------------
// Declare the structure for tracked objects
struct TrackedObject {
    cv::Point centroid;
    cv::Rect boundingBox;
    int id;
    int lostFrames;
    bool processed = false;  // Thêm thuộc tính để lưu trạng thái xử lý
};

//---------------------------------------------------------------------

class capture : public QThread
{
    Q_OBJECT
public:
    explicit capture(QObject *parent = nullptr);
    ~capture();

    //------------------for autoRunSystem------------------
    cv::Point closestPoint,farthestPoint, centroidObj;
    float center_pixel_distance, posObjZCoor;
    float pixel[2];
    float point_in_camera_space[3];
                //------caculate3D-------------------------
    cv::Mat translation_result;

    //-------------init Data Position For AutoRun-------

    cv::Mat getObjPos2Base(){
        qDebug() << "jump to getObjPos2Base():" ;

        if (!translation_result.empty()) {
            // Iterate through the matrix and print its values
            for (int i = 0; i < translation_result.rows; i++) {
                for (int j = 0; j < translation_result.cols; j++) {
                    qDebug() << "Value at (" << i << "," << j << "):" << translation_result.at<double>(i, j);
                }
            }
        } else {
            qDebug() << "translation_result is empty.";
        }
        return translation_result;
    }
    double rotationAngle()
    {
        return -1*rtAngle;
    }
    int getObjectColor() {
        qDebug() << "object color at capture: "<< objectColor;
        return objectColor;
    }
    bool resetObjInPos() {
        this->objectColor = -1;     // co sua them de chay code P003
        return objectInPos = false;
    }
    //-----------------Detect Object------------------------
    int objectColor = -1;
    bool prepareGrip = false;

    //------------------init camera state-------------------
    bool checkCameraState()
    {
        return cameraState;
    }
    bool changeStreamOpt(const uint8_t value)
    {
        return streamOpt = value;
    }
    void startCamera()
    {
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);

        // Get intrinsics
        auto stream = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        intrin = stream.get_intrinsics();

        cameraState = true;
    }
    void stopCamera()
    {
        cameraState = false;
        cfg.disable_all_streams();
        pipe.stop();
    }
    //------------------init camera state-------------------

    QPixmap cvMatToQPixmap(const Mat &inMat);
    QImage cvMatToQImage(const Mat &inMat);
    cv::Mat objDetect_1(const cv::Mat &frameInput, cv::Point &center);
    void caculatePosCam2Base(float point_in_camera_space[], cv::Mat &t_result);
    void toggleArucoDetection();
    void toggleAutoRunSystem();

    bool isFilter = false;
    // frame depth;
    //----------------------------------the code below is for model---------------------
    bool modelState = false;

    cv::Mat objDetect(const cv::Mat &frameInput, cv::Point &center);
    cv::Mat objDetect_2(const cv::Mat &frameInput, cv::Point &center);
    cv::Mat objDetect_3(const cv::Mat &frameInput, cv::Point &center);

    bool isObj = false;
    bool colorObject = false;

    std::vector<TrackedObject>& getTrackedObjects();
    //-----------------------------------------------
    QPixmap getPixmapColor() const
    {
        return pixmapColor;
    }


    void detectAndDrawArUco(cv::Mat &colorFrame);
    cv::Mat objDetect_4(const cv::Mat &frameInput, cv::Point &center);
    bool tcpSend = false;
signals:
    void newPixmapCaptured(QPixmap pixMap);
    void newPixmapCapturedForModel(QPixmap pixMap);
    // QThread interface
protected:
    void run();
private:
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::colorizer colorMap;

    QPixmap pixmapColor;
    QPixmap pixmapModel;
    //------------------init camera state-------------------
    uint8_t streamOpt = 0;
    bool cameraState = false;
    //------------------init camera state-------------------

    std::vector<cv::Vec3d> rvecs, tvecs;
    //------------------khởi tạo bộ lọc cho depth --------------------
    // Khởi tạo các bộ lọc
    rs2::decimation_filter decimation_filter;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    rs2::hole_filling_filter hole_filling_filter;
    rs2::colorizer color_map;
    rs2::threshold_filter threshold_filter;

    //------------------for calib-------------
    rs2_intrinsics intrin;

    cv::Mat cameraMatrix;
    cv::Mat distCoeeffs;

    cv::Mat R_cam2base = (cv::Mat_<double>(3, 3) <<
                                                    0.002642969286633374, 0.9992495771933076, -0.03864320370191021,
                                                    0.9994244662640771, -0.001332585808100424, 0.03389631908969071,
                                                    0.03381938713395011, -0.0387105501647997, -0.9986779973343874);
    cv::Mat t_cam2base = (cv::Mat_<double>(3, 1) << 298.8621674025867,
                                                    -150.900883162509,
                                                    248.3493787576531);
    //------------------for autoRunSystem---------------
    bool detectAruco = false;
    bool autoRunSystemBool = false;
    //---------------------for caculate 3D--------------
    cv::Mat homogeneous_matrix = cv::Mat::eye(4, 4, CV_64F);

    double distanceGripper;
    double rtAngle;
    bool objectInPos = false;

    //-------------------------For tracking object-------------------
    // Declare member variables for tracking
    std::vector<TrackedObject> trackedObjects;  // List of tracked objects
    int nextObjectId = 0;  // ID for the next new object
    const int MAX_LOST_FRAMES = 5;  // Max frames before an object is considered lost
};

#endif // CAPTURE_H
