#ifndef YRC1000MICRO_COM_H
#define YRC1000MICRO_COM_H

#include <QObject>
#include <QWidget>
#include <QFile>
#include "udp.h"
#include "yrc1000micro_command.h"

class YRC1000micro_com : public QObject
{
    Q_OBJECT
public:
    explicit YRC1000micro_com(QObject *parent = nullptr);
    ~YRC1000micro_com();


    uint8_t byte_val;

    bool YRC1000microConnect();
    void YRC1000microDisConnect();
    void YRC1000microSetConnection(QHostAddress address,quint16 port);


    void YRC1000microOnServo();
    void YRC1000microOffServo();


    void YRC1000microMoveCartesian(quint8 coordinate, quint8 move_type, quint8 speed_type,
                                   double speed, QVector<double>* position);

    void YRC1000microWritePulse(quint8 coordinate, quint8 move_type, quint8 speed_type,
                                double speed, QVector<double>* position);

    void YRC1000microHomePos();

    void YRC1000ReadPosition();
    void YRC1000microDataCallback();
    void YRC1000microReadPositionResponse(QByteArray data);
    void YRC1000microReadPulseResponse(QByteArray data);

    QVector<double> updateRobotPosition();
    void YRC1000microLoadJob(char* jobName);
    void YRC1000microStartJob();
    void YRC1000microGetVarPosition(uint16_t index);
    void YRC1000microWriteVarPosition(uint16_t index, QVector<int32_t> pos);
    void YRC1000microGetByte(uint16_t index);
    void YRC1000microWriteByte(uint16_t index, uint8_t buffer);
    void YRC1000microWriteVariablePositionAgain(uint16_t index, QVector<double> *pos);
    void YRC1000microReadPulse();
    QVector<int> updateRobotPulse();
signals:
    void dataUIRecieveSiUIgnal();
public slots:

private:
    UDP udp_server;
    QHostAddress udp_address;
    quint16 udp_port;

    UDP udp_file_control;
    YRC1000micro_command yrc1000micro_command;

    quint8 request_id_index;
    quint8 response_id_index;


    QByteArray header_to_send;
    QByteArray data_to_send;
    QVector<double> robot_position;
    QVector<int> robot_pulse;
    quint32 load_file_block_num;

    int read_position_type;
};

#endif // YRC1000MICRO_COM_H
