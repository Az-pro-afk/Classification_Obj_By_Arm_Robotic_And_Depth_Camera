#include "serialport.h"
#include "QDebug"

serialPort::serialPort(QObject *parent) : QThread(parent), _serialPort(nullptr)
{
}

serialPort::~serialPort()
{
    if(_serialPort != nullptr){
        _serialPort ->close();
        delete  _serialPort;
    }
}

bool serialPort::connectPort(QString portName)
{
    if(_serialPort != nullptr){
        _serialPort ->close();
        delete  _serialPort;
    }
    _serialPort = new QSerialPort(this);
    _serialPort->setPortName(portName);
    _serialPort->setBaudRate(QSerialPort::Baud115200);
    _serialPort->setDataBits(QSerialPort::Data8);
    _serialPort->setParity(QSerialPort::NoParity);
    _serialPort->setStopBits(QSerialPort::OneStop);

    if(_serialPort->open(QIODevice::ReadWrite))
    {
        QObject::connect(_serialPort, &QSerialPort::readyRead, this, &serialPort::dataReady);
    }
    return _serialPort->isOpen();
}

qint64 serialPort::writeData(QByteArray data)
{
    if (_serialPort == nullptr || !_serialPort->isOpen()) {
        return -1;
    }
    return _serialPort->write(data);

}

void serialPort::dataReady()
{
    static QByteArray buffer; // Bộ đệm để lưu trữ dữ liệu nhận được
    if (_serialPort->isOpen())
    {
        buffer.append(_serialPort->readAll()); // Đọc tất cả dữ liệu hiện có vào bộ đệm
        int startIdx = buffer.indexOf('\n');
        while (startIdx != -1)
        {
            int endIdx = buffer.indexOf('\n', startIdx + 1);
            if (endIdx != -1)
            {
                QByteArray dataToSend = buffer.mid(startIdx + 1, endIdx - startIdx - 1);
                emit dataReceive(dataToSend);
                // Xóa phần đã xử lý khỏi bộ đệm
                buffer.remove(0, endIdx + 1);
                startIdx = buffer.indexOf('\n');
            }
            else
            {
                break;
            }
        }
    }
}

void serialPort::closePort()
{
    if (_serialPort != nullptr) {
        if (_serialPort->isOpen()) {
            _serialPort->close();
        }
        delete _serialPort;
        _serialPort = nullptr;
    }
}
