#include "datareceiver.h"
#include <iostream>

DataReceiver::DataReceiver(QObject *parent) : QThread(parent), stopFlag(false)
{

}

DataReceiver::~DataReceiver()
{
    stopReceiving();
    wait();  // Ensure the thread is finished before destroying the object
    socket->close();
    delete socket;
}

void DataReceiver::stopReceiving()
{
    QMutexLocker locker(&stopMutex);
    stopFlag = true;
}

void DataReceiver::run()
{
    socket = new QTcpSocket();
    connectToServer();

    while (true) {
        {
            QMutexLocker locker(&stopMutex);
            if (stopFlag) {
                break;
            }
        }

        if (socket->waitForReadyRead(50)) {
            QByteArray data = socket->readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            // qDebug() << "data receive from TCP: " << doc;

            if (!doc.isNull()) {
                QJsonArray jsonArray = doc.array();
                // qDebug() << "jsonArray:" <<jsonArray;
                processData(jsonArray);
                // qDebug() << "classId in run():" << classId;
               // int newClassId = getLatestClassId(jsonArray);
               //  {
               //      QMutexLocker locker(&classIdMutex);
               //      *classId = newClassId;  // Update the value at the address of classId
               //  }
            }
        }
        msleep(50);  // Chờ thêm 50 milliseconds giữa các lần lặp
    }

    socket->close();
}

void DataReceiver::connectToServer()
{
    socket->connectToHost("127.0.0.1", 12002);
    if (!socket->waitForConnected(5000)) {
        std::cerr << "Connection to server failed!" << std::endl;
    } else {
        std::cout << "Connected to server." << std::endl;
    }
}

void DataReceiver::processData(QJsonArray &data)
{
    // Process the received data
    for (const QJsonValue &value : data) {
        QJsonArray arr = value.toArray();

        if (arr.size() == 3) {
            QJsonArray bboxArray = arr[0].toArray();  // The bounding box is the first element
            trackerId = arr[1].toInt();           // The tracker ID is the second element
            classId = arr[2].toInt();             // The class ID is the third element

            // dataSplit(bboxArray, trackerId, classId, detections);
            // qDebug() << "Processed data: "
            //          << "bbox: " << bboxArray << ", "
            //          << "tracker_id: " << trackerId << ", "
            //          << "class_id: " << classId;
        } else {
            qDebug() << "Unexpected data format: " << value;
        }
    }
}

void DataReceiver::dataSplit(QJsonArray bboxArray, int trackerId, int classId, QVector<std::tuple<QPair<double, double>, int, int>> &detections)
{
    if (bboxArray.size() != 4) {
        // qWarning() << "Khong co vat the nao detect duoc o Model.";

        return;
    }

    // Extract bounding box coordinates
    int left = bboxArray[0].toDouble();
    int top = bboxArray[1].toDouble();
    int right = bboxArray[2].toDouble();
    int bottom = bboxArray[3].toDouble();

    // Calculate the center
    double centerX = (left + right) / 2.0;
    double centerY = (top + bottom) / 2.0;
    // Khởi tạo QPair với giá trị centerX và centerY
    centerModel.first = centerX;
    centerModel.second = centerY;

    // Store the center, tracker ID, and class ID in the detections array
    detections.append(std::make_tuple(QPair<double, double>(centerX, centerY), trackerId, classId));
}
int DataReceiver::getLatestClassId(QJsonArray &data)
{
    int latestClassId = -1;  // Default value in case no valid data is found

    for (const QJsonValue &value : data) {
        QJsonArray arr = value.toArray();

        if (arr.size() == 3) {
            latestClassId = arr[2].toInt();  // Extract and update the classId
        } else {
            qDebug() << "Unexpected data format: " << value;
        }
    }

    // qDebug() << "Latest classId:" << latestClassId;
    return latestClassId;
}
