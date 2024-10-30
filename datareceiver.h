#ifndef DATARECEIVER_H
#define DATARECEIVER_H

#include <QThread>
#include <QTcpSocket>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QMutex>

class DataReceiver : public QThread
{
    Q_OBJECT
public:
    explicit DataReceiver(QObject *parent = nullptr);
    ~DataReceiver();

    int classObj()
    {
        qDebug() << "classId in classObj(): " << classId;
        return classId;  // Return the value at the address of classId
    }

    void stopReceiving();   // Method to stop the receiving thread

    void processData(QJsonArray &data);
    void dataSplit(QJsonArray bboxArray, int trackerId, int classId, QVector<std::tuple<QPair<double, double>, int, int> > &detections);
    QVector<std::tuple<QPair<double, double>, int, int>> detections;
    int trackerId,classId;
    QPair<double, double> centerModel;

    int getLatestClassId(QJsonArray &data);
signals:
    void dataReceived(const QJsonArray &data);  // Signal emitted when data is received

protected:
    void run() override;  // Override the run method to perform the thread's work

private:
    QTcpSocket *socket;
    QMutex stopMutex;
    bool stopFlag;
    QMutex classIdMutex;  // Mutex for protecting access to classId

    void connectToServer();  // Method to handle server connection

};

#endif // DATARECEIVER_H
