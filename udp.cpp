#include "udp.h"

UDP::UDP(QObject *parent) : QThread(parent)
{
    socket = new QUdpSocket(this);
}

UDP::~UDP(){
    socket->close();
}

bool UDP::udpConnect(QHostAddress address, quint16 port){
    bool status = socket->bind(address,port);
    qDebug()<<"Connect status: " << status;
    connect (socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
    //    connect (socket, &UDP::dataReceiveSignal, this, &UDP::dataReceiveSignal);
    return status;
}

void UDP::udpDisConnect(){
    socket->close();
}

void UDP::sendData(QHostAddress address, quint16 port, QByteArray data){
    socket->writeDatagram(data, address, port);
}

void UDP::readyRead()
{
    rx_buffer.resize(socket->pendingDatagramSize());

    QHostAddress sender;
    quint16 senderPort;
    socket->readDatagram(rx_buffer.data(), rx_buffer.size(), &sender, &senderPort);

    //    emit dataReceiveSignal();
    emit dataReceiveSignal(rx_buffer);
}


QByteArray UDP::getUdpData(){
    return rx_buffer;
}
