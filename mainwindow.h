#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "iostream"
#include "queue"
#include <cmath>

#include <QCoreApplication>
#include <QEventLoop>
#include <QTime>
#include <QPixmap>
#include <QThread>
#include <QTimer>

#include <QFile>
#include <QTextStream>
#include <QString>

#include <opencv2/core.hpp>

#include <QMessageBox>
#include <QSerialPortInfo>
#include <QVector>
#include <QPair>

// #include "opencv2/core.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/aruco.hpp"
// #include "opencv2/calib3d.hpp"
// #include "opencv2/ccalib.hpp"

#include <udp.h>
#include <yrc1000micro_com.h>
#include <yrc1000micro_command.h>
#include <capture.h>
#include <serialport.h>
#include <connecttcp.h>

#include "caculatematrix.h"


#include <vector>
#include "algorithm"
// #include "fstream"

#include <datareceiver.h>

#define windowSize 10

// using namespace rs2;
using namespace std;

namespace Ui {
class MainWindow;
}

struct DetectionInfo {
    QPair<double, double> center;
    int trackerId;
    int classId;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void init_window();

    double speedAverageValue = 0.0;

    QVector<double> UpdatePos();
    void updateLabel(int value);
    void connectSliderData();

    void loadPorts();
    void readData(QString data);
    double calculateMovingAverage();
    void handleReceivedData(double newData);
    void display(QPixmap pixmap);
    void captureThePictureAndData(int imageIndex);
    void openGripper();
    void autoRunSystem();
    void delay(int n);
    int findMaxX(const std::vector<cv::Point> &points);
    void grippObj();

    void autoRun();
    void findCenterOfMinIdObject();

    //---------------------Test-------------
    void testAuto();
    void checkObj();
    void captureObjectImage();
    void processReceivedData(QByteArray data);
    void displaySpeed();
    QVector<int> UpdateJoint();
private slots:
    void on_btn_connect_clicked();

    void on_btn_servo_clicked();

    void on_btn_home_clicked();

    void on_btn_set_pos_clicked();

    void on_movj_btn_clicked();

    void on_movl_btn_clicked();

    void on_movi_btn_clicked();

    void on_btn_get_pos_clicked();

    void on_btn_sub_X_clicked();

    void on_btn_sub_Y_clicked();

    void on_btn_sub_Z_clicked();

    void on_btn_add_X_clicked();

    void on_btn_add_Y_clicked();

    void on_btn_add_Z_clicked();

    void on_btn_sub_Roll_clicked();

    void on_btn_sub_Ptch_clicked();

    void on_btn_sub_Yaw_clicked();

    void on_btn_add_Roll_clicked();

    void on_btn_add_Ptch_clicked();

    void on_btn_add_Yaw_clicked();

    void on_load_Job_butt_clicked();

    void on_start_Job_butt_clicked();

    void on_btn_open_port_clicked();

    void on_openCam_but_clicked();

    void on_selShowImage_activated(int index);

    void on_savepicObject_btn_clicked();

    void on_capture_btn_clicked();

    void on_btn_auto_clicked();

    void on_model_state_clicked();

    //----------------------------for receiving data from Python-------------------
    // New slot to handle sending QPixmap over TCP
    void sendCapturedPixmap(QPixmap pixmap);


    void on_test_fnc_btn_clicked();

    void on_reset_Sys_butt_clicked();

    void on_set_bg_btn_clicked();

    void on_dpth_filter_clicked();

    void on_btn_get_joints_clicked();

    void on_btn_cal_clicked();

    void on_detect_btn_clicked();

    void on_btnPortInf_clicked();

private:
    Ui::MainWindow *ui;
    YRC1000micro_com *yrc1000micro_com;
    serialPort _port;
    capture *videoCapture;


    QVector<double> robot_Pos;
    QVector<int> robot_joint;

    std::queue<double> receivedDataQueue;

    //--------------------autoRunSystem--------------------
    bool runSystem = false;
    QTimer *runTimer;
    QTimer *speedTimer;

    uint8_t orangeCount = 0;
    uint8_t greenCount = 0;
    uint8_t yellowCount = 0;
    uint8_t redCount = 0;
    bool done = false;

    //-----------------------for receiving data from Python------------
    connectTCP *tcpSocket;  // The TCP connection class
    DataReceiver *dataReceiver;
    std::vector<TrackedObject> objects;

    //-----------------------for auto run test ----------------------
    bool isDetect = false;
    int objType1 = -2;
    int objType2 = -2;

    float countNumber = 0;
    // int countNumer2 = 0;
    //-------------------------speedTimer------------------------------
    float rpm = 0.0;
    float speedObj = 0.0;

    //-------------------------Calibration-----------------------------
    caculateMatrix *maTrix;

    QTimer *posAuto;

};

#endif // MAINWINDOW_H
