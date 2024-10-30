#include "connectTCP.h"
#include <QDebug>

connectTCP::connectTCP(QObject *parent)
    : QThread(parent), running(false), host("127.0.0.1"), port(12001)
{
}
connectTCP::~connectTCP()
{
    stop();
    wait();  // Wait for the thread to finish
    if (tcpSocket) {
        tcpSocket->close();
        delete tcpSocket;
    }
}
void connectTCP::setPixmap(const QPixmap &pixmap)
{
    pixmapToSend = pixmap;
}
void connectTCP::stop()
{
    running = false;
    if (tcpSocket && tcpSocket->isOpen()) {
        QMetaObject::invokeMethod(tcpSocket, "close", Qt::QueuedConnection);
    }

    quit();
    wait();
    tcpSocket->close();
}
void connectTCP::run()
{
    running = true;
    tcpSocket = new QTcpSocket();
    tcpSocket->connectToHost(host, port);
    if (!tcpSocket->waitForConnected(3000)) {
        emit errorOccurred("Could not connect to server");
        running = false;
        return;
    }
    emit connectionStatusChanged(true);
    while (running) {
        if (!pixmapToSend.isNull()) {
            sendPixmap();
        }
        QThread::msleep(120); 
    }

    emit connectionStatusChanged(false);
}

void connectTCP::sendPixmap()
{
    QByteArray pngData = convertPixmapToPNG(pixmapToSend);

    if (pngData.isEmpty()) {
        emit errorOccurred("Failed to convert QPixmap to PNG");
        return;
    }
    qint64 bytesWritten = tcpSocket->write(pngData);

    if (bytesWritten == -1) {
        emit errorOccurred("Failed to send QPixmap");
        return;
    }
    tcpSocket->flush();
}
QByteArray connectTCP::convertPixmapToPNG(const QPixmap &pixmap)
{
    QByteArray byteArray;
    QBuffer buffer(&byteArray);
    buffer.open(QIODevice::WriteOnly);

    if (!pixmap.save(&buffer, "PNG")) {
        emit errorOccurred("Failed to convert QPixmap to PNG");
    }
    return byteArray;
}
