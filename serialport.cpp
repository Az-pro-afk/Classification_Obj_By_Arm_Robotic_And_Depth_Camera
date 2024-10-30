#include "serialport.h"
#include "QDebug"

serialPort::serialPort(QObject *parent) : QThread(parent), _serialPort(nullptr)
{
    //    _serialPort = new QSerialPort(this);
}

serialPort::~serialPort()
{
    if(_serialPort != nullptr){
        _serialPort ->close();
        delete  _serialPort;
    }
    //    _serialPort ->close();
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
        //        qDebug() << "data nhan ve la: " << buffer;
        // Tìm vị trí của các dấu '\n'
        int startIdx = buffer.indexOf('\n');
        while (startIdx != -1)
        {
            int endIdx = buffer.indexOf('\n', startIdx + 1);
            // Nếu tìm thấy cả dấu '\n' bắt đầu và kết thúc
            if (endIdx != -1)
            {
                //                qDebug()  <<" co data nhan ve";
                // Trích xuất dữ liệu nằm giữa hai dấu '\n'
                QByteArray dataToSend = buffer.mid(startIdx + 1, endIdx - startIdx - 1);

                // Phát tín hiệu với dữ liệu trích xuất được
                emit dataReceive(dataToSend);

                // Xóa phần đã xử lý khỏi bộ đệm
                buffer.remove(0, endIdx + 1);

                // Tìm lại dấu '\n' tiếp theo
                startIdx = buffer.indexOf('\n');
            }
            else
            {
                // Nếu không tìm thấy dấu '\n' kết thúc, thoát vòng lặp
                //                qDebug() << "Khong co data nhan ve";
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
