#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QObject>
#include <QThread>
#include <QSerialPort>

class serialPort : public QThread
{
    Q_OBJECT
public:
    explicit serialPort(QObject *parent = nullptr);
    ~serialPort();
    bool connectPort(QString portName);

    qint64 writeData(QByteArray data);

    void closePort();
signals:
    void dataReceive(QByteArray b);

public slots:

private slots:
    void dataReady();

private:
    QSerialPort *_serialPort;
};

#endif // SERIALPORT_H
