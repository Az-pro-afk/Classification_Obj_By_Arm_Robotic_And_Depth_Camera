#ifndef UDP_H
#define UDP_H

#include <QObject>
#include <QUdpSocket>
#include <QThread>

class UDP : public QThread
{
    Q_OBJECT
public:
    explicit UDP(QObject *parent = nullptr);
    ~UDP();

    void sendData(QHostAddress address, quint16 port, QByteArray data);
    bool udpConnect(QHostAddress address, quint16 port);
    void udpDisConnect();
    QByteArray getUdpData();

signals:
    void dataReceiveSignal(QByteArray data);

public slots:
    void readyRead();

private:
    QUdpSocket *socket;
    QByteArray buffer;
    QByteArray rx_buffer;

    // QThread interface
protected:
};

#endif // UDP_H

