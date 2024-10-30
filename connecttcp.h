#ifndef CONNECTTCP_H
#define CONNECTTCP_H

#include <QThread>
#include <QTcpSocket>
#include <QPixmap>
#include <QBuffer>

class connectTCP : public QThread
{
    Q_OBJECT

public:
    explicit connectTCP(QObject *parent = nullptr);
    ~connectTCP();

    void setPixmap(const QPixmap &pixmap);
    void stop();

signals:
    void errorOccurred(const QString &message);
    void connectionStatusChanged(bool connected);

protected:
    void run() override;

private:
    QTcpSocket *tcpSocket;
    bool running;
    QString host;
    int port;
    QPixmap pixmapToSend;

    void sendPixmap();
    QByteArray convertPixmapToPNG(const QPixmap &pixmap);
};

#endif // CONNECTTCP_H
