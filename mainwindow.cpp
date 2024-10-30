#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    init_window();
    yrc1000micro_com = new YRC1000micro_com(this);
    videoCapture = new capture(this);
    maTrix = new caculateMatrix(this);

    runTimer = new QTimer(this);
    runTimer->setInterval(400);

    loadPorts();
    connect(&_port, &serialPort::dataReceive, this, &MainWindow::readData);
    connect(runTimer, &QTimer::timeout, this, &MainWindow::autoRunSystem);
    connectSliderData();

    //------------------------------speedTimer-------------------------------
    speedTimer = new QTimer(this);
    connect(speedTimer, &QTimer::timeout, this, &MainWindow::displaySpeed);
    speedTimer->start(100);  // Trigger every 500 ms (0.5 second)

    //-----------------------------------------------------------------------

}

MainWindow::~MainWindow()
{
    delete ui;
    delete yrc1000micro_com;
    delete videoCapture;
    delete &_port;
    delete maTrix;
    delete dataReceiver;
    delete tcpSocket;
    delete runTimer;
    delete speedTimer;
    delete posAuto;
}

void MainWindow::init_window()
{
    QImage image;
    image.load("D:/PC_Robot_MSVC/Image/No_signal.png");
    ui->Image->setPixmap(QPixmap::fromImage(image));
    QIcon icon;
    icon.addFile(QString::fromUtf8("D:/PC_Robot_MSVC/Image/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
    MainWindow::setWindowIcon(icon);
}
void MainWindow::on_btn_connect_clicked()
{
    if (ui->btn_connect->text() == "Connect"){
        QHostAddress udp_address;
        quint16 udp_port;
        QString ip_string = ui->txt_ip->text();
        QStringList ip_list = ip_string.split(".");
        quint32 ip_int32 = (ip_list.at(0).toUInt() << 24) | (ip_list.at(1).toUInt() << 16) |
                           (ip_list.at(2).toUInt() << 8) | ip_list.at(3).toUInt();
        udp_address.setAddress(ip_int32);
        udp_port = ui->txt_port->text().toUShort();
        yrc1000micro_com->YRC1000microSetConnection(udp_address,udp_port);
        ui->btn_connect->setText("Disconnect");
    }
    else if(ui->btn_connect->text() == "Disconnect"){
        ui->btn_connect->setText("Connect");
        yrc1000micro_com->YRC1000microDisConnect();
    }
}

//--------------------------------------------Basic Control Group----------------------------------------------------------------
void MainWindow::on_btn_servo_clicked()
{
    if(ui->btn_servo->text() == "Servo On"){
        yrc1000micro_com->YRC1000microOnServo();
        ui->btn_servo->setText("Servo Off");
    }
    else if(ui->btn_servo->text() == "Servo Off"){
        yrc1000micro_com->YRC1000microOffServo();
        ui->btn_servo->setText("Servo On");
    }
}

void MainWindow::on_btn_home_clicked()
{
    double set_speed = 50;
    QVector<double> set_position;

    set_position.append(185);
    set_position.append(0);
    set_position.append(25);
    set_position.append(180);
    set_position.append(0);
    set_position.append(0);

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);


}

//--------------------------------------------Set Position Group------------------------------------------------------------------

void MainWindow::on_btn_set_pos_clicked()
{
    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position;

    set_position.append(ui->txt_setX->text().toDouble());
    set_position.append(ui->txt_setY->text().toDouble());
    set_position.append(ui->txt_setZ->text().toDouble());
    set_position.append(ui->txt_setRoll->text().toDouble());
    set_position.append(ui->txt_setPitch->text().toDouble());
    set_position.append(ui->txt_setYaw->text().toDouble());

    if (ui->movj_btn->isChecked())
    {
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                    CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    }
    else if(ui->movi_btn->isChecked())
    {
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
    }
    else if(ui->movl_btn->isChecked())
    {
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_ABSOLUTE,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
    }

}
void MainWindow::on_movi_btn_clicked()
{
    ui->movj_btn->setChecked(false);
    ui->movl_btn->setChecked(false);
}
void MainWindow::on_movj_btn_clicked()
{
    ui->movi_btn->setChecked(false);
    ui->movl_btn->setChecked(false);
}
void MainWindow::on_movl_btn_clicked()
{
    ui->movj_btn->setChecked(false);
    ui->movi_btn->setChecked(false);
}

//--------------------------------------------Get Position Group------------------------------------------------------------------

QVector<double> MainWindow::UpdatePos()
{
    qDebug() << "Test UpdatePos";
    yrc1000micro_com->YRC1000ReadPosition();
    yrc1000micro_com->YRC1000microDataCallback();
    robot_Pos = yrc1000micro_com->updateRobotPosition();

    qDebug() << "here robot pos: " << robot_Pos;
    return robot_Pos;
}

QVector<int> MainWindow::UpdateJoint()
{
    yrc1000micro_com->YRC1000microReadPulse();
    yrc1000micro_com->YRC1000microDataCallback();
    robot_joint = yrc1000micro_com->updateRobotPulse();
    return robot_joint;
}
void MainWindow::on_btn_get_pos_clicked()
{
    UpdatePos();
    qDebug() << "test data of postion: robot_Pos" << robot_Pos;
    ui->txt_get_PosX    ->setText(QString::number(robot_Pos.at(0)));
    ui->txt_get_PosY    ->setText(QString::number(robot_Pos.at(1)));
    ui->txt_get_PosZ    ->setText(QString::number(robot_Pos.at(2)));
    ui->txt_get_Roll    ->setText(QString::number(robot_Pos.at(3)));
    ui->txt_get_Pitch   ->setText(QString::number(robot_Pos.at(4)));
    ui->txt_get_Yaw     ->setText(QString::number(robot_Pos.at(5)));
}


void MainWindow::on_btn_get_joints_clicked()
{
    UpdateJoint();
    qDebug() << "test data of postion: robot_Pos" << robot_joint;
    double degS = robot_joint.at(0) / (34816/30);
    double degL = robot_joint.at(1) / (102400/90);
    double degU = robot_joint.at(2) / (51200/90);
    double degR = robot_joint.at(3) / (10204/30);
    double degB = robot_joint.at(4) / (10204/30);
    double degT = robot_joint.at(5) / (10204/30);

    ui->txt_get_J1->setText(QString::number(degS,'f',3));
    ui->txt_get_J2->setText(QString::number(degL,'f',3));
    ui->txt_get_J3->setText(QString::number(degU,'f',3));
    ui->txt_get_J4->setText(QString::number(degR,'f',3));
    ui->txt_get_J5->setText(QString::number(degB,'f',3));
    ui->txt_get_J6->setText(QString::number(degT,'f',3));
}


//--------------------------------------------Incremental Position Group------------------------------------------------------------------
void MainWindow::updateLabel(int value) {

    QSlider *slider = qobject_cast<QSlider*>(sender());
    if (slider) {
        QString text = QString::number(value);
        if (slider == ui->horizontalSlider)
            ui->delta_dis_value->setText(text);
        else if (slider == ui->horizontalSlider_2)
            ui->delta_deg_value->setText(text);
        else if (slider == ui->horizontalSlider_4)
            ui->speed_dis_value->setText(text);
        else if (slider == ui->horizontalSlider_3)
            ui->speed_deg_value->setText(text);
    }
}

void MainWindow::connectSliderData(){
    connect(ui->horizontalSlider, &QSlider::valueChanged, this, &MainWindow::updateLabel);
    connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &MainWindow::updateLabel);
    connect(ui->horizontalSlider_4, &QSlider::valueChanged, this, &MainWindow::updateLabel);
    connect(ui->horizontalSlider_3, &QSlider::valueChanged, this, &MainWindow::updateLabel);

}

void MainWindow::on_btn_add_X_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_dis_value->text().toDouble();
        double deltaSpeed = ui->speed_dis_value->text().toDouble();

        double set_speed =deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[0] = deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_add_Y_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_dis_value->text().toDouble();
        double deltaSpeed = ui->speed_dis_value->text().toDouble();

        double set_speed =deltaSpeed;
        QVector<double> set_position(6, 0.0);
        set_position[1] = deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_add_Z_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_dis_value->text().toDouble();
        double deltaSpeed = ui->speed_dis_value->text().toDouble();

        double set_speed =deltaSpeed;
        QVector<double> set_position(6, 0.0);
        set_position[2] = deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_sub_X_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_dis_value->text().toDouble();
        double deltaSpeed = ui->speed_dis_value->text().toDouble();

        double set_speed =deltaSpeed;
        QVector<double> set_position(6, 0.0);
        set_position[0] = -deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_sub_Y_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_dis_value->text().toDouble();
        double deltaSpeed = ui->speed_dis_value->text().toDouble();

        double set_speed =deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[1] = -deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_sub_Z_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_dis_value->text().toDouble();
        double deltaSpeed = ui->speed_dis_value->text().toDouble();

        double set_speed =deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[2] = -deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_add_Roll_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_deg_value->text().toDouble();
        double deltaSpeed = ui->speed_deg_value->text().toDouble();

        double set_speed = deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[3] = deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_add_Ptch_clicked()
{

    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_deg_value->text().toDouble();
        double deltaSpeed = ui->speed_deg_value->text().toDouble();

        double set_speed = deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[4] = deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_add_Yaw_clicked()
{

    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_deg_value->text().toDouble();
        double deltaSpeed = ui->speed_deg_value->text().toDouble();

        double set_speed = deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[5] = deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_sub_Roll_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_deg_value->text().toDouble();
        double deltaSpeed = ui->speed_deg_value->text().toDouble();

        double set_speed = deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[3] = -deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }

}

void MainWindow::on_btn_sub_Ptch_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_deg_value->text().toDouble();
        double deltaSpeed = ui->speed_deg_value->text().toDouble();

        double set_speed = deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[4] = -deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}

void MainWindow::on_btn_sub_Yaw_clicked()
{
    if(ui->movi_btn->isChecked())
    {
        double deltaPos = ui->delta_deg_value->text().toDouble();
        double deltaSpeed = ui->speed_deg_value->text().toDouble();

        double set_speed = deltaSpeed;
        QVector<double> set_position(6, 0.0);

        set_position[5] = -deltaPos;
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
                                                    CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        on_btn_get_pos_clicked();

    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check move type at SetPosition !!!");
    }
}
//--------------------------------------------------------Job Select Function-------------------------------------------------------------------------------------------
void MainWindow::on_load_Job_butt_clicked()
{
    QString job = ui->jobFile_text->toPlainText();
    char jobName[12];
    strcpy(jobName, job.toStdString().c_str());
    qDebug() << "jobName: " << jobName;
    yrc1000micro_com->YRC1000microLoadJob(jobName);

}

void MainWindow::on_start_Job_butt_clicked()
{
    if( (ui->btn_connect->text() == "Disconnect") &&(ui->btn_servo->text() == "Servo Off"))
    {
        yrc1000micro_com->YRC1000microStartJob();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check Connect UDP or Servo State again");
    }
}

//--------------------------------------------------------Serial Port Function-------------------------------------------------------------------------------------------
void MainWindow::on_btn_open_port_clicked()
{
    qDebug() << "Here open port";
    if (ui -> btn_open_port->text() == "Open")
    {
        auto isConnected = _port.connectPort(ui->cmbPorts->currentText());

        if (!isConnected){
            QMessageBox::critical(this, "Error", "There is a problem connection");
        }
        connect(&_port, &serialPort::dataReceive, this, &MainWindow::processReceivedData);
        ui->btn_open_port->setText("Close");
        ui->btn_open_port->setStyleSheet("QPushButton {color: red;}");
    }
    else
    {
        disconnect(&_port, &serialPort::dataReceive, this, &MainWindow::processReceivedData);
        _port.closePort();
        ui->btn_open_port->setText("Open");
        ui->speed_txt->setText(QString::number(0.00, 'f', 2));
        ui->btn_open_port->setStyleSheet("QPushButton {color: green;}");

    }
}
void MainWindow::processReceivedData(QByteArray data)
{
    bool ok;
    rpm = data.toFloat(&ok);
}

void MainWindow::displaySpeed()
{
    if(rpm != 0.0)
    {
        float receive_speedObj = rpm * 100 / 60;
        QString speedStr = QString::number(receive_speedObj, 'f', 2);
        ui->speed_txt->setText(speedStr);

        readData(speedStr);
        QString averageStr = QString::number(speedAverageValue, 'f', 2); 
    }

}

void MainWindow::loadPorts()
{
    foreach(auto &port, QSerialPortInfo::availablePorts()){
        qDebug()<< port.portName();
        ui->cmbPorts->addItem(port.portName());
    }
}
void MainWindow::readData(QString data)
{
    double receivedValue = data.toDouble();
    handleReceivedData(receivedValue);
    speedAverageValue = calculateMovingAverage();
}

//-------------------------------------------Moving Average Speed--------------------------------------------------------------------------------------------------------
double MainWindow::calculateMovingAverage() {
    if (receivedDataQueue.empty()) {
        return 0;
    }

    double sum = 0;
    int count = 0;
    std::queue<double> temp(receivedDataQueue);

    while (!temp.empty()) {
        sum += temp.front();
        temp.pop();
        count++;
    }

    return sum / count;
}

void MainWindow::handleReceivedData(double newData) {
    receivedDataQueue.push(newData); 
    if (receivedDataQueue.size() > windowSize) {
        receivedDataQueue.pop();
    }
    double speedAverage = calculateMovingAverage();
}
//--------------------------------------------------------Camera Function-------------------------------------------------------------------------------------------

void MainWindow::display(QPixmap pixmap)
{
    if (videoCapture->checkCameraState() == false)
    {
        QImage image;
        image.load("No_signal.png");
        ui->Image->setPixmap(QPixmap::fromImage(image));
    }
    else
    {
        ui->Image->setPixmap(pixmap);
    }
}

void MainWindow::on_selShowImage_activated(int index)
{
    videoCapture->changeStreamOpt(index);
}

void MainWindow::on_openCam_but_clicked()
{
    std::cout << "Here open Camera " << std::endl;
    if (videoCapture->checkCameraState() == false)
    {
        connect(videoCapture, &capture::newPixmapCaptured, this, &MainWindow::display);
        ui->openCam_but->setText("Close Camera");
        videoCapture->startCamera();
        videoCapture->start(QThread::HighPriority);
    }
    else
    {
        disconnect(videoCapture, &capture::newPixmapCaptured, this, &MainWindow::display);
        videoCapture->stopCamera();
        QImage image;
        image.load("No_signal.png");
        ui->openCam_but->setText("Open Camera");
        ui->Image->setPixmap(QPixmap::fromImage(image));
    }
}

void MainWindow::on_capture_btn_clicked()
{
    if(ui->detect_btn->text() == "On Detecting")
    {
        static int imageIndex = 9;
        if (ui->detect_btn->text() == "On Detecting")
        {
            videoCapture->toggleArucoDetection();
            captureThePictureAndData(imageIndex);   
            videoCapture->toggleArucoDetection(); 
        }
        imageIndex++;
    }
    else QMessageBox::information(this,"Warning!!!","Detect Aruco is off!!!");
}

//-------------------------------Code save Data for Calibration---------------------------------
void MainWindow::on_savepicObject_btn_clicked()
{
    if(ui->savepicObject_btn->text() == "Capture Object")
    {
        ui->savepicObject_btn->setText("Capturing");
        captureObjectImage();
    }
    else if(ui->savepicObject_btn->text() == "Capturing")
    {
        ui->savepicObject_btn->setText("Capture Object");
        captureObjectImage();
    }
}

void MainWindow::captureObjectImage()
{
    while(ui->savepicObject_btn->text() == "Capturing")
    {
        static int imageIndex = 0;
        QPixmap pixmapColor = videoCapture->getPixmapColor();

        QImage currentImage = pixmapColor.toImage();

        if (!currentImage.isNull())
        {
            QString directory_pic = "D:/Tai_lieu_do_an/Oct"
                                    ""
                                    "";
            QString fileName_pic = QString("%1/%2.jpg").arg(directory_pic).arg(imageIndex);

            if (currentImage.save(fileName_pic))
            {
                imageIndex++;
            }
            else
            {
                QMessageBox::warning(this, tr("Error"), tr("Failed to save image."));
            }
        }
        else
        {
            QMessageBox::warning(this, tr("Error"), tr("No image to save."));
        }
        delay(200);
    }
}

void MainWindow::captureThePictureAndData(int imageIndex)
{
    QString path_picture_Data = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/Calib_Night/Calib_7x7/Image";
    QString path_data_robot = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/Calib_Night/Calib_7x7/Position";
    QPixmap pixmapColor = videoCapture->getPixmapColor();
    if (pixmapColor.isNull())
    {
        QMessageBox::warning(this, tr("Error"), tr("Pixmap is null or has no data."));
        return;
    }
    QString fileName_pic = QString("%1/image_(%2).jpg").arg(path_picture_Data).arg(imageIndex);
    if (!pixmapColor.save(fileName_pic, "JPG"))
    {
        QMessageBox::warning(this, tr("Error"), tr("Failed to save image as JPG."));
        return;
    }

    //----------------------------------save data into .txt file-------
    QString fileName_text = QString("%1/position_(%2).txt").arg(path_data_robot).arg(imageIndex);
    QFile file(fileName_text);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&file);

        out<< videoCapture->pixel[0] << Qt::endl;
        out<< videoCapture->pixel[1] << Qt::endl;
        out<< ui->txt_get_PosX->text().toStdString().c_str() << Qt::endl;
        out<< ui->txt_get_PosY->text().toStdString().c_str() << Qt::endl;
        out<< ui->txt_get_PosZ->text().toStdString().c_str() << Qt::endl;
        out<< ui->txt_get_Roll->text().toStdString().c_str() << Qt::endl;
        out<< ui->txt_get_Pitch->text().toStdString().c_str() << Qt::endl;
        out<< ui->txt_get_Yaw->text().toStdString().c_str() << Qt::endl;
        file.close();
    }
    else
    {
        QMessageBox::warning(this, tr("Error"), tr("Failed to open file for writing."));
    }
}

//---------------------------------------------------------------------------

//-------------------------------------------autoRunSystem--------------------------------------
void MainWindow::on_btn_auto_clicked()
{
    if( (ui->btn_connect->text() == "Disconnect") && (ui->btn_servo->text() == "Servo Off") || (ui->btn_auto->text() == "Stop Running") )
    {
        if(runSystem)
        {
            runTimer->stop();
            runSystem = false;
            ui->btn_auto->setText("Auto Run");
            ui->btn_auto->setStyleSheet("QPushButton {color: green;}");
            videoCapture->toggleAutoRunSystem();
        }
        else
        {
            runTimer->start();
            runSystem = true;
            ui->btn_auto->setText("Stop Running");
            ui->btn_auto->setStyleSheet("QPushButton {color: red;}");
            videoCapture->toggleAutoRunSystem();
        }
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check Connect UDP or Servo State again");
    }
}
void MainWindow::openGripper()
{
    qDebug() << "Calling Open Gripper";
    QString distanceStr = QString::number(880.0, 'f', 5);
    QByteArray byteArray = distanceStr.toUtf8();
    byteArray = byteArray.leftJustified(6, ' ', true);
    _port.writeData(byteArray);
}
void MainWindow::delay(int n)
{
    QTime dieTime= QTime::currentTime().addMSecs(n);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
};

void MainWindow::grippObj()
{
    QString distanceStr = QString::number(380.0, 'f', 5); 
    QByteArray byteArray = distanceStr.toUtf8();
    byteArray = byteArray.leftJustified(6, '0', true);
    _port.writeData(byteArray);
}

int MainWindow::findMaxX(const std::vector<cv::Point>& points) {
    if (points.empty()) {
        throw std::invalid_argument("Vector is empty!");
    }

    auto maxXPoint = std::max_element(points.begin(), points.end(),
                                      [](const cv::Point& p1, const cv::Point& p2) {
                                          return p1.x < p2.x;
                                      });
    return maxXPoint->x;
}
//-----------------------------------------------the code following is for testing Model Yolov5n-------------------------------
void MainWindow::on_model_state_clicked()
{
    if(ui->model_state->text() == "Model On")
    {
        connect(videoCapture, &capture::newPixmapCapturedForModel, this, &MainWindow::sendCapturedPixmap);
        dataReceiver = new DataReceiver(this);
        tcpSocket = new connectTCP(this); 
        videoCapture->tcpSend = true;
        dataReceiver->start();
        ui->model_state->setText("Model Off");
        videoCapture->modelState = true;
    }
    else if (ui->model_state->text() == "Model Off")
    {
        disconnect(videoCapture, &capture::newPixmapCapturedForModel, this, &MainWindow::sendCapturedPixmap);
        videoCapture->tcpSend = false;
        tcpSocket->stop();
        dataReceiver->quit();
        delete dataReceiver;
        delete tcpSocket;
        ui->model_state->setText("Model On");
        videoCapture->modelState = false;
    }
}
void MainWindow::sendCapturedPixmap(QPixmap pixmap)
{
    tcpSocket->setPixmap(pixmap);
    tcpSocket->start();
}

void MainWindow::on_test_fnc_btn_clicked()
{
    grippObj();
    yrc1000micro_com->YRC1000microWriteByte(8,01);
    videoCapture->getObjPos2Base();
    qDebug() << "center of Min IDs at MainWindow: (" << videoCapture->centroidObj.x << ", " << videoCapture->centroidObj.y << ")";
}

void MainWindow::findCenterOfMinIdObject() {

    objects = videoCapture->getTrackedObjects();
    if (objects.empty()) {
        std::cout << "No objects are being tracked currently." << std::endl;
        return;
    }
    int minId = std::numeric_limits<int>::max();
    cv::Point minIdCenter;
    for (const auto& obj : objects) {
        if (obj.id < minId) {
            minId = obj.id;
            minIdCenter = obj.centroid;
        }
    }
    if (minId != std::numeric_limits<int>::max()) {
        std::cout << "Tracked Object with smallest ID: " << minId << std::endl;
        std::cout << "Centroid: (" << minIdCenter.x << ", " << minIdCenter.y << ")" << std::endl;
    } else {
        std::cout << "No valid objects found." << std::endl;
    }

    qDebug() << "Center Model:" << dataReceiver->centerModel.first << dataReceiver->centerModel.second;
    double offset = 5.0;
    bool isObj = false;
    double difference = std::abs(minIdCenter.x - dataReceiver->centerModel.first);
    qDebug() << "Difference between minIdCenter.x and centerModel.first:" << difference;

    if (difference < offset) {
        isObj = true;
    }
    if (isObj) {
        qDebug() << "Object is within the offset range of centerModel.";
    } else {
        qDebug() << "Object is outside the offset range of centerModel.";
    }
}

void MainWindow::on_reset_Sys_butt_clicked()
{
    on_btn_home_clicked();
    qDebug() << "reset cac co, re-run program";
    delay(100);                     
    openGripper();
    yrc1000micro_com->YRC1000microWriteByte(7, 0);   
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(8, 0);    
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(9, 0);    
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(10, 0);  
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(11, 0);    
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(12, 0);   
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(13, 0);     
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(14, 0); 
    delay(20);
    done = true;
    ui->rsl_obj_dct->setText("");
    ui->rotation_angle->setText("");
    ui->speed_txt->setText("");
    objType1 = objType2 = -2;
}

void MainWindow::checkObj()
{
    const int lowerThreshold = 330;
    const int upperThreshold = 350;
    if ( (videoCapture->centroidObj.x >= lowerThreshold && videoCapture->centroidObj.x <= upperThreshold) && dataReceiver->classObj() != -1)
    {
        objType1 = dataReceiver->classObj();
        qDebug() << "objType1: "  << objType1;
        isDetect = true;
    }
}

void MainWindow::autoRunSystem()
{
    qDebug() << "Calling Auto Run";
    openGripper();
    delay(10);
    yrc1000micro_com->YRC1000microGetByte(7);
    while (yrc1000micro_com->byte_val == 1)
    {
        int centroidObjAtX = videoCapture->centroidObj.x;
        double yawAngle;
        delay(20);
        qDebug() << "centroidObj:"<< videoCapture->centroidObj.x;
        delay(100);
        bool inProcess = false;
        if( videoCapture->centroidObj.x >250)
        {
            objType1 = dataReceiver->classId;
            inProcess = true;
            objType2 = objType1;
            delay(100);
        }
        while(inProcess )
        {
            delay(100);
            switch (objType2) {
            case 0:
            {
                ui->rsl_obj_dct->setText("Duck");
            }
                break;
            case 1:
            {
                ui->rsl_obj_dct->setText("Fish");
            }
                break;
            case 2:
                ui->rsl_obj_dct->setText("Frog");
                break;
            case 3:
                ui->rsl_obj_dct->setText("Octopus");
                break;
            default:
                ui->rsl_obj_dct->setText("Other objects!!!");
                break;
            }
            if(videoCapture->centroidObj.x > 270)
            {
                yawAngle = videoCapture->rotationAngle();
                yrc1000micro_com->YRC1000microWriteByte(8,01);
                delay(100);

                yrc1000micro_com->YRC1000microGetByte(9);
                delay(100);
                while(yrc1000micro_com->byte_val == 0 )
                {
                    qDebug() << "Byte 9 not set to 1";
                    yrc1000micro_com->YRC1000microGetByte(9);
                    delay(100);
                    if(yrc1000micro_com->byte_val == 1)
                    {
                        qDebug() << "Byte 9 set to 1";
                        break;
                    }
                    if(done == true) break;
                    if(ui->btn_auto->text() == "Auto Run") break;
                }

                while(yrc1000micro_com->byte_val == 1 )
                {
                    delay(50);
                    centroidObjAtX = videoCapture->centroidObj.x;
                    delay(100);
                    QVector<int32_t> posPick, posPlace0, posMiddle, yawPos;
                    cv::Mat objPos2Base;
                    //---------Get the position from objPos2Base x,y,z----------
                    objPos2Base = videoCapture->getObjPos2Base();
                    for (int i = 0; i < objPos2Base.rows; i++) {
                        for (int j = 0; j < objPos2Base.cols; j++) {
                            qDebug() << objPos2Base.at<float>(i, j);
                        }
                    }
                    posPick.resize(objPos2Base.rows);
                    for (int i = 0; i < objPos2Base.rows; ++i)
                    {
                        posPick[i] = static_cast<int32_t>(std::round(objPos2Base.at<double>(i, 0)*1000));
                    }
                    //---------------------------------posPick---------------------------
                    int32_t yValue = 17*1000; 
                    int32_t zValue = 5*1000;   

                    posPick[1] = posPick[1] + yValue; 
                    posPick[2] = posPick[2] - 3*zValue;
                    posPick.push_back(180*10000);
                    posPick.push_back(0);
                    posPick.push_back(yawAngle*10000);

                    //-------------Adjust offset ----------
                    switch (objType2) {
                    case 0:     //Duck
                    {
                            posPick[0] = posPick[0] - 6*1000;
                            posPick[1] = posPick[1] + 15*1000;
                            posPick[2] = -102*1000;
                    }
                        break;
                    case 1:     //Fish

                        posPick[0] = posPick[0] - 3*1000;
                        posPick[1] = posPick[1] + 11*1000;
                        posPick[2] = -103000;
                        // posPick[5] = posPick[5] + 5*10000;
                        break;
                    case 2:     //Frog
                        posPick[0] = posPick[0] - 7*1000;
                        posPick[1] = posPick[1] + yValue - 4*1000;
                        posPick[2] = posPick[2] - 17*1000;

                        break;
                    case 3:     //Oct
                        posPick[0] = posPick[0] - 4*1000;
                        posPick[1] = posPick[1] + yValue + 2*1000;
                        posPick[2] = posPick[2] - 22*1000;
                        break;
                    case -1:
                        posPick[0] = posPick[0] - 3*1000;
                        posPick[1] = posPick[1] + 13*1000;
                        posPick[2] = -103000;
                        break;
                    default:
                        break;
                    }
                    //--------------------- Height Limited ---------------
                    if( posPick[2] < -110000)
                    {
                        posPick[2] = -100000;
                    }

                    if(posPick[0] <190*1000 || posPick[0] > 280*1000 || posPick[1] < -207*1000 || posPick[1] > 55*1000 )
                    {
                        posPick[0] = 185*1000;
                        posPick[1] = 0*  10000;
                        posPick[2] = 25* 1000;
                        posPick[3] = 180*10000;
                        posPick[4] = 0*  10000;
                        posPick[5] = 90* 10000;
                    }

                    //-----------------------------------
                    delay(20);
                    yrc1000micro_com->YRC1000microWriteVarPosition(002, posPick);
                    qDebug() << "posPick:" << posPick;
                    delay(100);
                    //-------------------------------------------------------------------
                    yawPos.resize(6);
                    yawPos[0] = posPick[0] - 3*1000;
                    yawPos[1] = posPick[1] + 5*1000;

                    yawPos[2] =  25 * 1000;
                    yawPos[3] = 180 * 10000;
                    yawPos[4] = 0 * 10000;
                    yawPos[5] = posPick[5];
                    yrc1000micro_com->YRC1000microWriteVarPosition(006, yawPos);
                    qDebug() << "yawPos:" << yawPos;
                    delay(100);
                    ui->rotation_angle->setText(QString::number(yawAngle));
                    //-------------------------------------------------------------------
                    if(objType2 != -1)
                    {
                        QVector<int32_t> posP004;
                        posP004.resize(6); 
                        posP004[0] = 100 * 1000;
                        posP004[1] = -167 * 1000;
                        posP004[2] = 24 * 1000;
                        posP004[3] = 180 * 10000;
                        posP004[4] = 0 * 10000;
                        posP004[5] = -90*10000;
                        if(objType2 == 2 || objType2 == 3) posP004[0] = 20 * 1000;

                        yrc1000micro_com->YRC1000microWriteVarPosition(004, posP004);
                        qDebug() << "posP004:" << posP004;
                        delay(100);
                    }
                    else if(objType2 == -1)
                    {
                        QVector<int32_t> posP007;
                        posP007.resize(6);  
                        posP007[0] = 185.088 * 1000;
                        posP007[1] = -0.004 * 1000;
                        posP007[2] = -10 * 1000;
                        posP007[3] = 180 * 10000;
                        posP007[4] = 0 * 10000;
                        posP007[5] = posPick[5];
                        yrc1000micro_com->YRC1000microWriteVarPosition(004, posP007);
                        qDebug() << "posP007:" << posP007;
                        delay(100);
                    }
                    //-------------------------------------------------------------------
                    //-------------------------------------------------------------------
                    yrc1000micro_com->YRC1000microWriteByte(10,1);
                    delay(100);

                    yrc1000micro_com->YRC1000microGetByte(11);
                    delay(100);
                    while(yrc1000micro_com->byte_val == 0)
                    {
                        qDebug() << "Byte 11 not set to 1";
                        delay(50);
                        yrc1000micro_com->YRC1000microGetByte(11);
                        delay(100);
                        if(yrc1000micro_com->byte_val == 1)
                        {
                            qDebug() << "Byte 11 set to 1";
                            delay(50);
                            break;
                        }
                        if(done == true) break;
                        if(ui->btn_auto->text() == "Auto Run") break;
                    }
                    while(yrc1000micro_com->byte_val == 1 /*|| ui->txt_get_PosZ->text() == "1"*/)
                    {
                        qDebug() << "Byte 11 = 1";
                        delay(50);
                        grippObj();
                        qDebug() << "objType2 when place:" <<objType2;
                        switch (objType2)               // dựa vào thông tin màu sắc để di chuyển
                        {
                        case 0:         //Duck object
                        {
                            posPlace0.push_back(120*1000 - 35*yellowCount*1000);
                            posPlace0.push_back(-140*1000);
                            posPlace0.push_back(-45*1000);
                            posPlace0.push_back(-180*10000);
                            posPlace0.push_back(0*10000);
                            posPlace0.push_back(-90*10000);
                            yellowCount += 1;
                            if(yellowCount == 3)
                            {
                                yellowCount =0;
                            }
                            qDebug() << "Duck object detected";
                        }
                        break;
                        case 2:         //Frog object, move to vi tri thả
                        {
                            //                vị trí thả
                            posPlace0.push_back(-130*1000 + 42*greenCount*1000);
                            posPlace0.push_back(-160*1000);
                            posPlace0.push_back(-50*1000);
                            posPlace0.push_back(-180*10000);
                            posPlace0.push_back(0*10000);
                            posPlace0.push_back(-90*10000);
                            greenCount += 1;

                            if(greenCount == 3)
                            {
                                greenCount =0;
                            }
                            qDebug() << "Frog object detected";
                        }
                        break;
                        case 1:         //Fish object
                        {

                            posPlace0.push_back(120*1000 - 35*orangeCount*1000);
                            posPlace0.push_back(-260*1000);
                            posPlace0.push_back(-50*1000);
                            posPlace0.push_back(-180*10000);
                            posPlace0.push_back(-0*10000);
                            posPlace0.push_back(-90*10000);
                            orangeCount += 1;
                            if(orangeCount == 3)
                            {
                                orangeCount =0;
                            }
                            qDebug() << "Fish object detected";
                        }
                        break;
                        case 3:         //Red object
                        {
                            posPlace0.push_back(-130*1000 + 45*redCount*1000);
                            posPlace0.push_back(-265*1000);
                            posPlace0.push_back(-50.443*1000);
                            posPlace0.push_back(-180*10000);
                            posPlace0.push_back(0*10000);
                            posPlace0.push_back(-90*10000);
                            redCount += 1;

                            if(redCount == 3)
                            {
                                redCount =0;
                            }
                            qDebug() << "Oct object detected";
                        }
                        break;
                        default:
                            posPlace0.push_back(47.146*1000);
                            posPlace0.push_back(178.864*1000);
                            posPlace0.push_back(-75.003*1000);
                            posPlace0.push_back(-180*10000);
                            posPlace0.push_back(0);
                            posPlace0.push_back(75.2335*10000);
                            qDebug() << "No object has been detect";


                            break;
                        }

                        //---------------------Nâng lên sau khi thả
                        {
                            QVector<int32_t> posP005;
                            posP005.resize(6);
                            posP005 = posPlace0;
                            posP005[2] = 15.438*1000;
                            yrc1000micro_com->YRC1000microWriteVarPosition(005, posP005);
                            delay(20);
                        }

                        yrc1000micro_com->YRC1000microWriteVarPosition(003, posPlace0);
                        delay(100);
                        yrc1000micro_com->YRC1000microWriteByte(12,1);
                        delay(100);
                        yrc1000micro_com->YRC1000microGetByte(13);
                        delay(100);

                        while(yrc1000micro_com->byte_val == 0)
                        {
                            qDebug() << "Byte 13 not set to 1";
                            delay(50);
                            yrc1000micro_com->YRC1000microGetByte(13);
                            delay(100);
                            if(yrc1000micro_com->byte_val == 1 )
                            {
                                qDebug() << "Byte 13 set to 1";
                                delay(50);
                                break;
                            }
                            if(done == true) break;
                            if(ui->btn_auto->text() == "Auto Run") break;
                        }

                        while(yrc1000micro_com-> byte_val == 1)
                        {
                            qDebug() << "Byte 13 = 1";
                            delay(50);
                            openGripper();
                            yrc1000micro_com->YRC1000microGetByte(15);
                            while(yrc1000micro_com-> byte_val == 0 )
                            {
                                yrc1000micro_com->YRC1000microGetByte(15);
                                delay(100);
                                if(done == true) break;
                                if(ui->btn_auto->text() == "Auto Run") break;
                            }

                            while(yrc1000micro_com->byte_val == 1 )
                            {
                                delay(100);
                                yrc1000micro_com->YRC1000microWriteByte(7, 0);    
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(8, 0);     
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(9, 0);     
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(10, 0);    
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(11, 0);   
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(12, 0);     
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(13, 0); 
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(14, 0);  
                                delay(20);

                                centroidObjAtX = 0;
                                videoCapture->centroidObj.x =0;
                                objType2 = -2;
                                ui->rsl_obj_dct->setText("");
                                ui->rotation_angle->setText("");
                                done = true;
                                inProcess = false;
                                if(done == true) break;
                                if(ui->btn_auto->text() == "Auto Run") break;
                            }
                            if(done == true) break;
                            if(ui->btn_auto->text() == "Auto Run") break;
                        }
                        if(done == true) break;
                        if(ui->btn_auto->text() == "Auto Run") break;
                    }
                    if(done == true) break;
                    if(ui->btn_auto->text() == "Auto Run") break;
                }
            }
            if(done == true) break;
            if(ui->btn_auto->text() == "Auto Run") break;
        }
        if(done == true) break;
        if(ui->btn_auto->text() == "Auto Run") break;
    }
    done = false;
    qDebug() << "---------------Waiting at Get byte 7---------------------";
}

void MainWindow::on_set_bg_btn_clicked()
{
    QPixmap pixmapColor = videoCapture->getPixmapColor();

    if (!pixmapColor.isNull()) {
        QString filePath = "D:/03_08_GUI_Python/captured_frame.jpg";
        bool success = pixmapColor.save(filePath, "jpg");

        if (success) {
            qDebug() << "Image saved successfully at:" << filePath;
        } else {
            qDebug() << "Error: Failed to save the image.";
        }
    } else {
        qDebug() << "pixmapColor is null, nothing to save.";
    }
}
void MainWindow::on_dpth_filter_clicked()
{
    if(ui->dpth_filter->text() == "Depth Filter")
    {
        ui->dpth_filter->setText("Stop Filter");
        videoCapture->isFilter = true;
    }
    else if(ui->dpth_filter->text() == "Stop Filter")
    {
        ui->dpth_filter->setText("Depth Filter");
        videoCapture->isFilter = false;
    }
}

void MainWindow::on_btn_cal_clicked()
{
    if(!ui->numberSampleCalib->text().isEmpty())
    {
        int numberOfSample = ui->numberSampleCalib->text().toInt();
        maTrix->getMatrixFromCam2Base(numberOfSample);
    }
    else QMessageBox::information(this,"Warning!!!","Enter number of photos !!!");
}


void MainWindow::on_detect_btn_clicked()
{
    if(ui->detect_btn->text() == "Detect ArUco")
    {
        videoCapture->toggleArucoDetection();
        ui->detect_btn->setText("On Detecting");

        posAuto = new QTimer(this);
        connect(posAuto, &QTimer::timeout, this, &MainWindow::on_btn_get_pos_clicked);
        connect(posAuto, &QTimer::timeout, this, &MainWindow::on_btn_get_joints_clicked);
        posAuto->start(200);

    }else if (ui->detect_btn->text() == "On Detecting")
    {
        videoCapture->toggleArucoDetection();
        ui->detect_btn->setText("Detect ArUco");

        disconnect(posAuto, &QTimer::timeout, this, &MainWindow::on_btn_get_pos_clicked);
        disconnect(posAuto, &QTimer::timeout, this, &MainWindow::on_btn_get_joints_clicked);
        delete posAuto;
    }
}
void MainWindow::on_btnPortInf_clicked()
{
    ui->cmbPorts->clear();
    loadPorts();
}

